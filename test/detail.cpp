// Copyright (c)  Zubax Robotics  <zubax.com>
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include <ramen.hpp>
#include <doctest.h>
#include <cstdint>
#include <string_view>

namespace
{
struct FunX2 final
{
    static inline std::uint32_t count_dtor;
    static inline std::uint32_t count_move;

    int operator()(const int x) const { return x * 2; }

    FunX2()             = default;
    FunX2(const FunX2&) = delete;
    FunX2(FunX2&&) noexcept { count_move++; }
    FunX2& operator=(const FunX2&) = delete;
    FunX2& operator=(FunX2&&)      = delete;
    ~FunX2() { count_dtor++; }

    static void reset()
    {
        count_dtor = 0;
        count_move = 0;
    }
};

// NOLINTBEGIN(bugprone-use-after-move,hicpp-invalid-access-moved,clang-diagnostic-self-move,clang-analyzer-cplusplus.Move)
TEST_CASE("function")
{
    FunX2::reset();
    using ramen::Function;
    {
        int                              state = 0;
        Function<int(int), sizeof(int*)> fun_a = [&state](const auto x) { return state += x; };
        REQUIRE(1 == fun_a(1));
        REQUIRE(3 == fun_a(2));
        fun_a = FunX2{};
        REQUIRE(2 == fun_a(1));
        REQUIRE(4 == fun_a(2));
        REQUIRE(1 == FunX2::count_dtor);
        REQUIRE(1 == FunX2::count_move);
        fun_a = [](const int x) { return x * 3; };
        REQUIRE(3 == fun_a(1));
        REQUIRE(6 == fun_a(2));
        REQUIRE(2 == FunX2::count_dtor);
        REQUIRE(1 == FunX2::count_move);
        fun_a = FunX2{};
        REQUIRE(3 == FunX2::count_dtor);
        REQUIRE(2 == FunX2::count_move);
        {
            Function<int(int), sizeof(int*)> fun_b = std::move(fun_a);
            REQUIRE(6 == fun_b(3));
            REQUIRE(8 == fun_b(4));
            REQUIRE(3 == FunX2::count_dtor);
            REQUIRE(3 == FunX2::count_move);
            fun_a = [](const int x) { return x * -1; };
            REQUIRE(4 == FunX2::count_dtor);
            REQUIRE(3 == FunX2::count_move);
            REQUIRE(-3 == fun_a(3));
            fun_a = std::move(fun_b);
            REQUIRE(4 == FunX2::count_dtor);
            REQUIRE(4 == FunX2::count_move);
            fun_b = std::move(fun_a);
            REQUIRE(5 == FunX2::count_dtor);  // Old in B destroyed.
            REQUIRE(5 == FunX2::count_move);
            (void) fun_b;
        }
        REQUIRE(6 == FunX2::count_dtor);
        REQUIRE(5 == FunX2::count_move);
        {
            const Function<int(int), sizeof(int*)> fun_c = std::move(fun_a);
            REQUIRE(6 == FunX2::count_dtor);
            REQUIRE(6 == FunX2::count_move);
            REQUIRE(10 == fun_c(5));
        }
        REQUIRE(7 == FunX2::count_dtor);
        REQUIRE(6 == FunX2::count_move);
        // Self-assignment has no effect.
        // NOLINTBEGIN(clang-diagnostic-self-assign-overloaded)
        Function<int(int), sizeof(int*)>* const volatile fu = &fun_a;
        fun_a                                               = static_cast<Function<int(int), sizeof(int*)>&&>(*fu);
        // NOLINTEND(clang-diagnostic-self-assign-overloaded)
        REQUIRE(7 == FunX2::count_dtor);
        REQUIRE(6 == FunX2::count_move);
    }
    REQUIRE(8 == FunX2::count_dtor);
    REQUIRE(6 == FunX2::count_move);
    FunX2::reset();
    {
        Function<int(int), 16, 16> fun_a = FunX2{};
        REQUIRE(1 == FunX2::count_dtor);
        REQUIRE(1 == FunX2::count_move);
        Function<int(int), 32, 32> fun_b = std::move(fun_a);  // Reverse won't compile.
        REQUIRE(1 == FunX2::count_dtor);
        REQUIRE(2 == FunX2::count_move);
        fun_b = std::move(fun_a);  // Reverse won't compile.
        REQUIRE(2 == FunX2::count_dtor);
        REQUIRE(3 == FunX2::count_move);
        (void) fun_b;
    }
    REQUIRE(4 == FunX2::count_dtor);
    REQUIRE(3 == FunX2::count_move);
    FunX2::reset();
}
// NOLINTEND(bugprone-use-after-move,hicpp-invalid-access-moved,clang-diagnostic-self-move,clang-analyzer-cplusplus.Move)

TEST_CASE("list_node")
{
    using ramen::detail::ListNode;
    struct Value
    {
        char val;
        explicit Value(const char v) : val(v) {}
    };
    struct Node final : ListNode<Value>
    {
        explicit Node(const char x) : ListNode<Value>(x) {}
    };
    Node a{'a'};
    Node b{'b'};
    Node c{'c'};
    Node d{'d'};
    Node e{'e'};
    Node f{'f'};

    // First sublist: a -> b -> f
    REQUIRE(!a.linked());
    REQUIRE(!b.linked());
    a.merge(&a);       // no effect
    a.merge(nullptr);  // no effect
    REQUIRE(!a.linked());
    REQUIRE(!b.linked());
    a.merge(&b);  // a,b
    a.merge(&f);  // a,b,f
    b.merge(&f);  // no effect -- already in the list
    REQUIRE(a.linked());
    REQUIRE(b.linked());

    // Second sublist: c -> d -> e
    c.merge(&d);  // c,d
    c.merge(&e);  // c,d,e
    e.merge(&d);  // no effect -- already in the list
    REQUIRE(c.linked());
    REQUIRE(d.linked());
    REQUIRE(e.linked());

    const auto unroll_onwards = [](const Node* const n)
    {
        std::array<char, 100>  result{};
        std::size_t            i = 0;
        const ListNode<Value>* p = n;
        while (p != nullptr)
        {
            result.at(i++) = p->val;
            p              = p->next();
        }
        return result;
    };
    REQUIRE(std::string_view(std::string_view("abf")) == unroll_onwards(&a).data());
    REQUIRE(std::string_view(std::string_view("cde")) == unroll_onwards(&c).data());

    // Merge two lists
    a.merge(&c);
    REQUIRE(std::string_view(std::string_view("abfcde")) == unroll_onwards(&a).data());

    // Remove nodes from the beginning, end, and middle
    a.remove();
    e.remove();
    f.remove();
    REQUIRE(!a.linked());
    REQUIRE(b.linked());
    REQUIRE(c.linked());
    REQUIRE(d.linked());
    REQUIRE(!e.linked());
    REQUIRE(!f.linked());
    REQUIRE(std::string_view("a") == unroll_onwards(&a).data());
    REQUIRE(std::string_view("bcd") == unroll_onwards(&b).data());
    REQUIRE(std::string_view("e") == unroll_onwards(&e).data());
    REQUIRE(std::string_view("f") == unroll_onwards(&f).data());
    REQUIRE(a.head() == &a);
    REQUIRE(a.tail() == &a);
    REQUIRE(e.head() == &e);
    REQUIRE(e.tail() == &e);
    REQUIRE(d.head() == &b);
    REQUIRE(b.tail() == &d);

    // Insert the nodes back again.
    a.merge(&c);  // Note how we're linking against c here! It will be rewound.
    REQUIRE(std::string_view("abcd") == unroll_onwards(&a).data());
    e.merge(&f);
    REQUIRE(std::string_view("ef") == unroll_onwards(&e).data());
    b.merge(&f);
    REQUIRE(std::string_view("abcdef") == unroll_onwards(&a).data());
    REQUIRE(std::string_view("ef") == unroll_onwards(&e).data());

    static const auto is_vowel = [](const char z) { return (z == 'a') || (z == 'e'); };

    // Clusterize vowels first, then consonants.
    c.clusterize<2>([](const ListNode<Value>& n) { return !is_vowel(n.val); });
    REQUIRE(a.head() == &a);  // a is the head of the list
    REQUIRE(a.tail() == &f);
    REQUIRE(std::string_view("aebcdf") == unroll_onwards(&a).data());

    c.clusterize<2>([](const ListNode<Value>& n) { return is_vowel(n.val); });
    REQUIRE(b.head() == &b);
    REQUIRE(b.tail() == &e);
    REQUIRE(std::string_view("bcdfae") == unroll_onwards(&b).data());

    // Also clusterization but with some clusters unused: first, middle, and last.
    c.clusterize<5>([](const ListNode<Value>& n) -> std::size_t { return is_vowel(n.val) ? 1 : 3; });
    REQUIRE(a.head() == &a);  // a is the head of the list
    REQUIRE(a.tail() == &f);
    REQUIRE(std::string_view("aebcdf") == unroll_onwards(&a).data());

    // Simple sorting -- each letter gets its own category.
    c.clusterize<10>([](const ListNode<Value>& n) -> std::size_t { return static_cast<std::size_t>(n.val - 'a'); });
    REQUIRE(a.head() == &a);  // a is the head of the list
    REQUIRE(a.tail() == &f);
    REQUIRE(std::string_view("abcdef") == unroll_onwards(&a).data());

    // Degenerate clusterization -- all elements go to the same cluster. Ordering not affected.
    c.clusterize<3>([](const ListNode<Value>&) -> std::size_t { return 0; });
    REQUIRE(a.head() == &a);  // a is the head of the list
    REQUIRE(a.tail() == &f);
    REQUIRE(std::string_view("abcdef") == unroll_onwards(&a).data());

    // Same but only one cluster.
    c.clusterize<1>([](const ListNode<Value>&) -> std::size_t { return 0; });
    REQUIRE(a.head() == &a);  // a is the head of the list
    REQUIRE(a.tail() == &f);
    REQUIRE(std::string_view("abcdef") == unroll_onwards(&a).data());
}

}  // namespace
