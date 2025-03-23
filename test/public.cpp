// Copyright (c)  Zubax Robotics  <zubax.com>
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include <ramen.hpp>
#include <doctest.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <iostream>

namespace
{

TEST_CASE("pull")
{
    using ramen::Pullable;
    using ramen::Puller;

    Pullable<char>                out_bhv_a     = [](char& x) { x = 'a'; };
    std::optional<Pullable<char>> opt_out_bhv_b = [](char& x) { x = 'b'; };
    auto&                         out_bhv_b     = opt_out_bhv_b.value();
    Puller<char>                  in_evt_a;
    std::optional<Puller<char>>   opt_in_evt_b;
    auto&                         in_evt_b = opt_in_evt_b.emplace();

    REQUIRE(false == static_cast<bool>(in_evt_a));
    REQUIRE(false == static_cast<bool>(in_evt_b));
    REQUIRE(!in_evt_a);
    REQUIRE(!in_evt_b);
    REQUIRE(false == static_cast<bool>(out_bhv_a));
    REQUIRE(false == static_cast<bool>(out_bhv_b));
    REQUIRE(!out_bhv_a);
    REQUIRE(!out_bhv_b);

    REQUIRE('\0' == *in_evt_a);  // Not linked yet
    in_evt_a >> out_bhv_a;
    REQUIRE('a' == *in_evt_a);
    in_evt_b >> in_evt_a;
    REQUIRE('a' == *in_evt_a);
    REQUIRE('a' == *in_evt_b);
    in_evt_b >> out_bhv_b;  // Later-linked behavior overwrites the value.
    REQUIRE('b' == *in_evt_a);
    REQUIRE('b' == *in_evt_b);

    REQUIRE(in_evt_a);
    REQUIRE(in_evt_b);
    REQUIRE(false == (!in_evt_a));
    REQUIRE(false == (!in_evt_b));
    REQUIRE(out_bhv_a);
    REQUIRE(out_bhv_b);
    REQUIRE(false == (!out_bhv_a));
    REQUIRE(false == (!out_bhv_b));

    in_evt_a.detach();
    REQUIRE(false == static_cast<bool>(in_evt_a));
    opt_out_bhv_b.reset();
    REQUIRE(in_evt_b);
    REQUIRE(out_bhv_a);
    opt_in_evt_b.reset();
    REQUIRE(!out_bhv_a);  // All detached.
}

TEST_CASE("push")
{
    using ramen::Pushable;
    using ramen::Pusher;

    char                          a            = '\0';
    char                          b            = '\0';
    Pushable<char>                in_bhv_a     = [&](const char& x) { a = x; };
    std::optional<Pushable<char>> opt_in_bhv_b = [&](const char& x) { b = x; };
    auto&                         in_bhv_b     = opt_in_bhv_b.value();
    Pusher<char>                  out_evt_a;
    std::optional<Pusher<char>>   opt_out_evt_b;
    auto&                         out_evt_b = opt_out_evt_b.emplace();

    REQUIRE(false == static_cast<bool>(out_evt_a));
    REQUIRE(false == static_cast<bool>(out_evt_b));
    REQUIRE(!out_evt_a);
    REQUIRE(!out_evt_b);
    REQUIRE(false == static_cast<bool>(in_bhv_a));
    REQUIRE(false == static_cast<bool>(in_bhv_b));
    REQUIRE(!in_bhv_a);
    REQUIRE(!in_bhv_b);

    out_evt_a('z');  // No effect, not linked yet.
    REQUIRE('\0' == a);
    REQUIRE('\0' == b);
    out_evt_a >> in_bhv_a;
    REQUIRE(out_evt_a);
    REQUIRE(false == (!out_evt_a));
    REQUIRE(in_bhv_a);
    REQUIRE(false == (!in_bhv_a));
    out_evt_a('z');
    REQUIRE('z' == a);
    REQUIRE('\0' == b);
    out_evt_b >> out_evt_a;
    out_evt_b('y');
    REQUIRE('y' == a);
    REQUIRE('\0' == b);
    out_evt_b >> in_bhv_b;
    out_evt_b('x');
    REQUIRE('x' == a);
    REQUIRE('x' == b);

    REQUIRE(out_evt_a);
    REQUIRE(out_evt_b);
    REQUIRE(false == (!out_evt_a));
    REQUIRE(false == (!out_evt_b));
    REQUIRE(in_bhv_a);
    REQUIRE(in_bhv_b);
    REQUIRE(false == (!in_bhv_a));
    REQUIRE(false == (!in_bhv_b));

    out_evt_a.detach();
    REQUIRE(false == static_cast<bool>(out_evt_a));
    opt_in_bhv_b.reset();
    REQUIRE(out_evt_b);
    REQUIRE(in_bhv_a);
    opt_out_evt_b.reset();
    REQUIRE(!in_bhv_a);  // All detached.
}

TEST_CASE("performance")
{
    using ramen::Callable;
    using ramen::Pullable;
    using ramen::Puller;

    const bool              choice     = (std::rand() % 2) == 0;
    constexpr std::uint32_t iterations = 30'000'000;
    static constexpr float  magic      = 1e-5F;
    float                   y          = 0;  // This is an artificial data dependency to constrain the optimization.

    const auto reset_y = [&y] { y = 1.0F / static_cast<float>((std::rand() % 0xFF'FF'FF) + 1); };

    // OOP reference for comparison. Different implementations and dummy data to avoid devirtualization and inlining.
    using Base = Callable<void(float&)>;
    struct Impl1 : public Base  // NOLINT(cppcoreguidelines-special-member-functions,hicpp-special-member-functions)
    {
        float& y;
        explicit Impl1(float& y) : y(y) {}
        void operator()(float& x) const override
        {
            x += y;
            y += magic;
        }
        virtual ~Impl1() = default;
    } const oop_1{y};
    struct Impl2 : public Base  // NOLINT(cppcoreguidelines-special-member-functions,hicpp-special-member-functions)
    {
        float& y;
        explicit Impl2(float& y) : y(y) {}
        void operator()(float& x) const override
        {
            x -= y;
            y += magic;
        }
        virtual ~Impl2() = default;
    } const oop_2{y};
    const Base& oop = choice ? static_cast<const Base&>(oop_1) : static_cast<const Base&>(oop_2);

    // Actor model alternative that does the same.
    Pullable<float> out_bhv_1 = [&y](float& x)
    {
        x += y;
        y += magic;
    };
    Pullable<float> out_bhv_2 = [&y](float& x)
    {
        x -= y;
        y += magic;
    };
    Puller<float> in_evt;
    in_evt ^ (choice ? out_bhv_1 : out_bhv_2);

    // Establish the baseline to subtract it later.
    reset_y();
    std::chrono::steady_clock::duration elapsed_baseline{};
    {
        float      x     = 0;
        const auto start = std::chrono::steady_clock::now();
        for (std::uint32_t i = 0; i < iterations; i++)
        {
            x += y;  // There is an intentional data dependency between sequential loop executions.
            y += magic;
        }
        elapsed_baseline = std::chrono::steady_clock::now() - start;
        std::cout << "Final value: " << x << "\n";  // Explicit data dependency to constrain optimization
    }
    // Benchmark the OOP version.
    reset_y();
    std::chrono::steady_clock::duration elapsed_oop{};
    {
        float      x     = 0;
        const auto start = std::chrono::steady_clock::now();
        for (std::uint32_t i = 0; i < iterations; i++)
        {
            oop(x);
        }
        elapsed_oop = std::chrono::steady_clock::now() - start;
        std::cout << "Final value: " << x << "\n";  // Explicit data dependency to constrain optimization
    }
    // Benchmark the actor model version.
    reset_y();
    std::chrono::steady_clock::duration elapsed_actor{};
    {
        float      x     = 0;
        const auto start = std::chrono::steady_clock::now();
        for (std::uint32_t i = 0; i < iterations; i++)
        {
            in_evt(x);
        }
        elapsed_actor = std::chrono::steady_clock::now() - start;
        std::cout << "Final value: " << x << "\n";  // Explicit data dependency to constrain optimization
    }

    std::cout << "Baseline: elapsed: " << elapsed_baseline << "; per iteration: " << elapsed_baseline / iterations
              << "\n";
    std::cout << "OOP:      elapsed: " << elapsed_oop << " s; per iteration: " << elapsed_oop / iterations << "\n";
    std::cout << "Actor:    elapsed: " << elapsed_actor << " s; per iteration: " << elapsed_actor / iterations << "\n";
    using D                            = std::chrono::duration<float>;
    const float slowdown_uncompensated = std::chrono::duration_cast<D>(elapsed_actor) /  //
                                         std::chrono::duration_cast<D>(elapsed_oop);
    const float slowdown_compensated = std::chrono::duration_cast<D>(elapsed_actor - elapsed_baseline) /
                                       std::chrono::duration_cast<D>(elapsed_oop - elapsed_baseline);
    std::cout << "Slowdown factor uncompensated: " << slowdown_uncompensated
              << "; compensated: " << slowdown_compensated << "\n";
}

TEST_CASE("latch")
{
    using ramen::Pushable;
    using ramen::Pusher;
    using ramen::Puller;
    using ramen::Latch;
    Pusher<float>             out_evt;
    Puller<double>            in_evt;
    Latch<int, float, double> latch{0};
    out_evt >> latch.in;
    in_evt >> latch.out;
    REQUIRE(0 == *in_evt);
    out_evt(123.0F);
    REQUIRE(123 == *in_evt);
}

TEST_CASE("lift")
{
    using ramen::Pushable;
    using ramen::Pusher;
    using ramen::Pullable;
    using ramen::Lift;
    std::int16_t                      input   = 0;
    std::uint64_t                     output  = 0;
    Pullable<std::int16_t>            out_bhv = [&input](std::int16_t& x) { x = input; };
    Pushable<std::uint64_t>           in_bhv  = [&output](const std::uint64_t& x) { output = x; };
    Pusher<>                          trigger;
    Lift<std::int16_t, std::uint64_t> lift;
    lift.in >> out_bhv;
    lift.out >> in_bhv;
    trigger >> lift.trigger;

    input = 12345;
    trigger();
    REQUIRE(12345 == output);
    input = 4321;
    trigger();
    REQUIRE(4321 == output);
}

TEST_CASE("push_unary")
{
    using ramen::PushUnary;
    using ramen::Pusher;
    using ramen::Latch;
    Latch<int>            output;
    PushUnary<int, float> obj = [](const float x) { return static_cast<int>(x * 2); };
    Pusher<float>         trigger;
    trigger >> obj.in;
    obj.out >> output.in;
    trigger(1.5F);
    REQUIRE(3 == output.value);
}

TEST_CASE("push_unary_void")
{
    using ramen::PushUnary;
    using ramen::Pusher;
    using ramen::Pushable;
    using ramen::Footprint;
    std::uint64_t                                 counter  = 0;
    int                                           argument = 0;
    Pushable<>                                    in_bhv   = [&] { counter++; };
    PushUnary<Footprint<sizeof(int*)>, void, int> obj      = [&argument](const int x) { argument = x; };
    Pusher<int>                                   trigger;
    trigger >> obj.in;
    obj.out >> in_bhv;
    trigger(3);
    REQUIRE(3 == argument);
    REQUIRE(1 == counter);
    trigger(9);
    REQUIRE(9 == argument);
    REQUIRE(2 == counter);
}

TEST_CASE("pull_unary_0")
{
    using ramen::PullUnary;
    using ramen::Puller;
    PullUnary<int> obj = [] { return 9; };
    Puller<int>    trigger;
    trigger >> obj.out;
    REQUIRE(9 == *trigger);
}

TEST_CASE("pull_unary_1")
{
    using ramen::Latch;
    using ramen::PullUnary;
    using ramen::Puller;
    using ramen::Footprint;
    Latch<float>                        input{1.5F};
    PullUnary<Footprint<1>, int, float> obj = [](const float x) { return static_cast<int>(x * 2); };
    Puller<int>                         trigger;
    trigger >> obj.out;
    obj.in >> input.out;
    REQUIRE(3 == *trigger);
}

TEST_CASE("pull_unary_2_init")
{
    using ramen::Pullable;
    using ramen::PullUnary;
    using ramen::Puller;
    using ramen::Footprint;
    Pullable<float, double> input = [](float& out_a, double& out_b)
    {
        out_a = 1.7F;
        out_b = 2.5;
    };
    PullUnary<Footprint<1>, int, float, double> obj{
        [](const float x, const double y) { return static_cast<int>(static_cast<double>(x) * y); },
        456.0F,
        789.0,
    };
    Puller<int> trigger;
    trigger >> obj.out;
    obj.in >> input;
    REQUIRE(4 == *trigger);
}

TEST_CASE("pull_unary_2_default")
{
    using ramen::Pullable;
    using ramen::PullUnary;
    using ramen::Puller;
    using ramen::Footprint;
    Pullable<float, double> input = [](float& out_a, double& out_b)
    {
        out_a = 1.7F;
        out_b = 2.5;
    };
    PullUnary<Footprint<1>, int, float, double> obj = [](const float x, const double y)
    { return static_cast<int>(static_cast<double>(x) * y); };
    Puller<int> trigger;
    trigger >> obj.out;
    obj.in >> input;
    REQUIRE(4 == *trigger);
}

TEST_CASE("pull_nary_0")
{
    using ramen::PullNary;
    using ramen::Puller;
    PullNary<int> obj = [] { return 123; };
    Puller<int>   trigger;
    obj.out ^ trigger;
    REQUIRE(123 == *trigger);
}

TEST_CASE("pull_nary_2")
{
    using ramen::PullNary;
    using ramen::Puller;
    using ramen::Latch;
    Latch<int>                   a{123};
    Latch<double>                b{456.0};
    PullNary<float, int, double> obj = [](const int x, const double y)
    { return static_cast<float>(static_cast<double>(x) * y); };
    Puller<float> trigger;
    obj.out ^ trigger;
    std::get<0>(obj.in) ^ a.out;
    std::get<1>(obj.in) ^ b.out;
    REQUIRE(*trigger == doctest::Approx(56088));
}

TEST_CASE("cast")
{
    using ramen::PushCast;
    using ramen::PullCast;
    using ramen::Pusher;
    using ramen::Puller;
    using ramen::Latch;
    Latch<int>            latch{123};
    PushCast<int, float>  push_cast;
    PullCast<double, int> pull_cast;
    Pusher<float>         push_trigger;
    Puller<double>        pull_trigger;
    push_cast.out >> latch.in;
    pull_cast.in >> latch.out;
    push_trigger >> push_cast.in;
    pull_trigger >> pull_cast.out;
    push_trigger(3.14F);
    REQUIRE(3 == latch.value);
    REQUIRE(3.0 == *pull_trigger);
}

TEST_CASE("ctor")
{
    using ramen::Ctor;
    int        value = 0;
    const Ctor ctor  = [&value] { value = 123; };
    (void) ctor;
    REQUIRE(123 == value);
}

TEST_CASE("finalizer")
{
    using ramen::Finalizer;
    int value = 0;
    {
        const Finalizer fin1 = [&value] { value += 1; };
        (void) fin1;
    }
    {
        Finalizer moved_into;
        REQUIRE(1 == value);
        {
            Finalizer fin2 = [&value] { value += 10; };
            moved_into     = std::move(fin2);
        }
        (void) moved_into;
        REQUIRE(1 == value);
    }
    REQUIRE(11 == value);
    {
        Finalizer fin3 = [&value] { value += 222; };
        fin3.disarm();  // Will not be invoked.
    }
    REQUIRE(11 == value);
}

}  // namespace
