// Copyright (c)  Zubax Robotics  <zubax.com>
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include <ramen.hpp>
#include <doctest.h>

namespace ramen
{
namespace
{
struct FunX2 final
{
    static inline std::uint32_t count_dtor;
    static inline std::uint32_t count_move;

    int operator()(int x) const { return x * 2; }

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
TEST_CASE(function)
{
    FunX2::reset();
    {
        int                              state = 0;
        Function<int(int), sizeof(int*)> fun_a = [&state](const auto x) { return state += x; };
        TEST_ASSERT_EQUAL(1, fun_a(1));
        TEST_ASSERT_EQUAL(3, fun_a(2));
        fun_a = FunX2{};
        TEST_ASSERT_EQUAL(2, fun_a(1));
        TEST_ASSERT_EQUAL(4, fun_a(2));
        TEST_ASSERT_EQUAL(1, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(1, FunX2::count_move);
        fun_a = [](int x) { return x * 3; };
        TEST_ASSERT_EQUAL(3, fun_a(1));
        TEST_ASSERT_EQUAL(6, fun_a(2));
        TEST_ASSERT_EQUAL(2, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(1, FunX2::count_move);
        fun_a = FunX2{};
        TEST_ASSERT_EQUAL(3, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(2, FunX2::count_move);
        {
            Function<int(int), sizeof(int*)> fun_b = std::move(fun_a);
            TEST_ASSERT_EQUAL(6, fun_b(3));
            TEST_ASSERT_EQUAL(8, fun_b(4));
            TEST_ASSERT_EQUAL(3, FunX2::count_dtor);
            TEST_ASSERT_EQUAL(3, FunX2::count_move);
            fun_a = [](int x) { return x * -1; };
            TEST_ASSERT_EQUAL(4, FunX2::count_dtor);
            TEST_ASSERT_EQUAL(3, FunX2::count_move);
            TEST_ASSERT_EQUAL(-3, fun_a(3));
            fun_a = std::move(fun_b);
            TEST_ASSERT_EQUAL(4, FunX2::count_dtor);
            TEST_ASSERT_EQUAL(4, FunX2::count_move);
            fun_b = std::move(fun_a);
            TEST_ASSERT_EQUAL(5, FunX2::count_dtor);  // Old in B destroyed.
            TEST_ASSERT_EQUAL(5, FunX2::count_move);
        }
        TEST_ASSERT_EQUAL(6, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(5, FunX2::count_move);
        {
            const Function<int(int), sizeof(int*)> fun_c = std::move(fun_a);
            TEST_ASSERT_EQUAL(6, FunX2::count_dtor);
            TEST_ASSERT_EQUAL(6, FunX2::count_move);
            TEST_ASSERT_EQUAL(10, fun_c(5));
        }
        TEST_ASSERT_EQUAL(7, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(6, FunX2::count_move);
        // Self-assignment has no effect.
        // NOLINTBEGIN(clang-diagnostic-self-assign-overloaded)
        Function<int(int), sizeof(int*)>* const volatile fu = &fun_a;
        fun_a                                               = static_cast<Function<int(int), sizeof(int*)>&&>(*fu);
        // NOLINTEND(clang-diagnostic-self-assign-overloaded)
        TEST_ASSERT_EQUAL(7, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(6, FunX2::count_move);
    }
    TEST_ASSERT_EQUAL(8, FunX2::count_dtor);
    TEST_ASSERT_EQUAL(6, FunX2::count_move);
    FunX2::reset();
    {
        Function<int(int), 16, 16> fun_a = FunX2{};
        TEST_ASSERT_EQUAL(1, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(1, FunX2::count_move);
        Function<int(int), 32, 32> fun_b = std::move(fun_a);  // Reverse won't compile.
        TEST_ASSERT_EQUAL(1, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(2, FunX2::count_move);
        fun_b = std::move(fun_a);  // Reverse won't compile.
        TEST_ASSERT_EQUAL(2, FunX2::count_dtor);
        TEST_ASSERT_EQUAL(3, FunX2::count_move);
    }
    TEST_ASSERT_EQUAL(4, FunX2::count_dtor);
    TEST_ASSERT_EQUAL(3, FunX2::count_move);
    FunX2::reset();
}
// NOLINTEND(bugprone-use-after-move,hicpp-invalid-access-moved,clang-diagnostic-self-move,clang-analyzer-cplusplus.Move)

TEST_CASE(list_node)
{
    using dyshlo::detail::ListNode;
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
    TEST_ASSERT_FALSE(a.linked());
    TEST_ASSERT_FALSE(b.linked());
    a.merge(&a);       // no effect
    a.merge(nullptr);  // no effect
    TEST_ASSERT_FALSE(a.linked());
    TEST_ASSERT_FALSE(b.linked());
    a.merge(&b);  // a,b
    a.merge(&f);  // a,b,f
    b.merge(&f);  // no effect -- already in the list
    TEST_ASSERT(a.linked());
    TEST_ASSERT(b.linked());

    // Second sublist: c -> d -> e
    c.merge(&d);  // c,d
    c.merge(&e);  // c,d,e
    e.merge(&d);  // no effect -- already in the list
    TEST_ASSERT(c.linked());
    TEST_ASSERT(d.linked());
    TEST_ASSERT(e.linked());

    const auto unroll_onwards = [](Node* const n)
    {
        std::array<char, 100> result{};
        std::size_t           i = 0;
        ListNode<Value>*      p = n;
        while (p != nullptr)
        {
            result.at(i++) = p->val;
            p              = p->next();
        }
        return result;
    };
    TEST_ASSERT_EQUAL_STRING("abf", unroll_onwards(&a).data());
    TEST_ASSERT_EQUAL_STRING("cde", unroll_onwards(&c).data());

    // Merge two lists
    a.merge(&c);
    TEST_ASSERT_EQUAL_STRING("abfcde", unroll_onwards(&a).data());

    // Remove nodes from the beginning, end, and middle
    a.remove();
    e.remove();
    f.remove();
    TEST_ASSERT_FALSE(a.linked());
    TEST_ASSERT(b.linked());
    TEST_ASSERT(c.linked());
    TEST_ASSERT(d.linked());
    TEST_ASSERT_FALSE(e.linked());
    TEST_ASSERT_FALSE(f.linked());
    TEST_ASSERT_EQUAL_STRING("a", unroll_onwards(&a).data());
    TEST_ASSERT_EQUAL_STRING("bcd", unroll_onwards(&b).data());
    TEST_ASSERT_EQUAL_STRING("e", unroll_onwards(&e).data());
    TEST_ASSERT_EQUAL_STRING("f", unroll_onwards(&f).data());
    TEST_ASSERT_EQUAL(a.head(), &a);
    TEST_ASSERT_EQUAL(a.tail(), &a);
    TEST_ASSERT_EQUAL(e.head(), &e);
    TEST_ASSERT_EQUAL(e.tail(), &e);
    TEST_ASSERT_EQUAL(d.head(), &b);
    TEST_ASSERT_EQUAL(b.tail(), &d);

    // Insert the nodes back again.
    a.merge(&c);  // Note how we're linking against c here! It will be rewound.
    TEST_ASSERT_EQUAL_STRING("abcd", unroll_onwards(&a).data());
    e.merge(&f);
    TEST_ASSERT_EQUAL_STRING("ef", unroll_onwards(&e).data());
    b.merge(&f);
    TEST_ASSERT_EQUAL_STRING("abcdef", unroll_onwards(&a).data());
    TEST_ASSERT_EQUAL_STRING("ef", unroll_onwards(&e).data());

    static const auto is_vowel = [](const char c) { return (c == 'a') || (c == 'e'); };

    // Clusterize vowels first, then consonants.
    c.clusterize<2>([](const ListNode<Value>& n) { return !is_vowel(n.val); });
    TEST_ASSERT_EQUAL(a.head(), &a);  // a is the head of the list
    TEST_ASSERT_EQUAL(a.tail(), &f);
    TEST_ASSERT_EQUAL_STRING("aebcdf", unroll_onwards(&a).data());

    c.clusterize<2>([](const ListNode<Value>& n) { return is_vowel(n.val); });
    TEST_ASSERT_EQUAL(b.head(), &b);
    TEST_ASSERT_EQUAL(b.tail(), &e);
    TEST_ASSERT_EQUAL_STRING("bcdfae", unroll_onwards(&b).data());

    // Also clusterization but with some clusters unused: first, middle, and last.
    c.clusterize<5>([](const ListNode<Value>& n) -> std::size_t { return is_vowel(n.val) ? 1 : 3; });
    TEST_ASSERT_EQUAL(a.head(), &a);  // a is the head of the list
    TEST_ASSERT_EQUAL(a.tail(), &f);
    TEST_ASSERT_EQUAL_STRING("aebcdf", unroll_onwards(&a).data());

    // Simple sorting -- each letter gets its own category.
    c.clusterize<10>([](const ListNode<Value>& n) -> std::size_t { return static_cast<std::size_t>(n.val - 'a'); });
    TEST_ASSERT_EQUAL(a.head(), &a);  // a is the head of the list
    TEST_ASSERT_EQUAL(a.tail(), &f);
    TEST_ASSERT_EQUAL_STRING("abcdef", unroll_onwards(&a).data());

    // Degenerate clusterization -- all elements go to the same cluster. Ordering not affected.
    c.clusterize<3>([](const ListNode<Value>&) -> std::size_t { return 0; });
    TEST_ASSERT_EQUAL(a.head(), &a);  // a is the head of the list
    TEST_ASSERT_EQUAL(a.tail(), &f);
    TEST_ASSERT_EQUAL_STRING("abcdef", unroll_onwards(&a).data());

    // Same but only one cluster.
    c.clusterize<1>([](const ListNode<Value>&) -> std::size_t { return 0; });
    TEST_ASSERT_EQUAL(a.head(), &a);  // a is the head of the list
    TEST_ASSERT_EQUAL(a.tail(), &f);
    TEST_ASSERT_EQUAL_STRING("abcdef", unroll_onwards(&a).data());
}

}  // namespace
}  // namespace dyshlo::verification
