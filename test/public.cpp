// Copyright (c)  Zubax Robotics  <zubax.com>
// Author: Pavel Kirienko <pavel.kirienko@zubax.com>

#include <ramen.hpp>
#include <doctest.h>

namespace ramen
{
namespace
{

TEST_CASE(pull)
{
    Pullable<char>                out_bhv_a     = [](char& x) { x = 'a'; };
    std::optional<Pullable<char>> opt_out_bhv_b = [](char& x) { x = 'b'; };
    auto&                         out_bhv_b     = opt_out_bhv_b.value();
    Puller<char>                  in_evt_a;
    std::optional<Puller<char>>   opt_in_evt_b;
    auto&                         in_evt_b = opt_in_evt_b.emplace();

    TEST_ASSERT_FALSE(in_evt_a);
    TEST_ASSERT_FALSE(in_evt_b);
    TEST_ASSERT(!in_evt_a);
    TEST_ASSERT(!in_evt_b);
    TEST_ASSERT_FALSE(out_bhv_a);
    TEST_ASSERT_FALSE(out_bhv_b);
    TEST_ASSERT(!out_bhv_a);
    TEST_ASSERT(!out_bhv_b);

    TEST_ASSERT_EQUAL('\0', *in_evt_a);  // Not linked yet
    in_evt_a >> out_bhv_a;
    TEST_ASSERT_EQUAL('a', *in_evt_a);
    in_evt_b >> in_evt_a;
    TEST_ASSERT_EQUAL('a', *in_evt_a);
    TEST_ASSERT_EQUAL('a', *in_evt_b);
    in_evt_b >> out_bhv_b;  // Later-linked behavior overwrites the value.
    TEST_ASSERT_EQUAL('b', *in_evt_a);
    TEST_ASSERT_EQUAL('b', *in_evt_b);

    TEST_ASSERT(in_evt_a);
    TEST_ASSERT(in_evt_b);
    TEST_ASSERT_FALSE(!in_evt_a);
    TEST_ASSERT_FALSE(!in_evt_b);
    TEST_ASSERT(out_bhv_a);
    TEST_ASSERT(out_bhv_b);
    TEST_ASSERT_FALSE(!out_bhv_a);
    TEST_ASSERT_FALSE(!out_bhv_b);

    in_evt_a.detach();
    TEST_ASSERT_FALSE(in_evt_a);
    opt_out_bhv_b.reset();
    TEST_ASSERT(in_evt_b);
    TEST_ASSERT(out_bhv_a);
    opt_in_evt_b.reset();
    TEST_ASSERT(!out_bhv_a);  // All detached.
}

TEST_CASE(push)
{
    char                          a            = '\0';
    char                          b            = '\0';
    Pushable<char>                in_bhv_a     = [&](const char& x) { a = x; };
    std::optional<Pushable<char>> opt_in_bhv_b = [&](const char& x) { b = x; };
    auto&                         in_bhv_b     = opt_in_bhv_b.value();
    Pusher<char>                  out_evt_a;
    std::optional<Pusher<char>>   opt_out_evt_b;
    auto&                         out_evt_b = opt_out_evt_b.emplace();

    TEST_ASSERT_FALSE(out_evt_a);
    TEST_ASSERT_FALSE(out_evt_b);
    TEST_ASSERT(!out_evt_a);
    TEST_ASSERT(!out_evt_b);
    TEST_ASSERT_FALSE(in_bhv_a);
    TEST_ASSERT_FALSE(in_bhv_b);
    TEST_ASSERT(!in_bhv_a);
    TEST_ASSERT(!in_bhv_b);

    out_evt_a('z');  // No effect, not linked yet.
    TEST_ASSERT_EQUAL('\0', a);
    TEST_ASSERT_EQUAL('\0', b);
    out_evt_a >> in_bhv_a;
    TEST_ASSERT(out_evt_a);
    TEST_ASSERT_FALSE(!out_evt_a);
    TEST_ASSERT(in_bhv_a);
    TEST_ASSERT_FALSE(!in_bhv_a);
    out_evt_a('z');
    TEST_ASSERT_EQUAL('z', a);
    TEST_ASSERT_EQUAL('\0', b);
    out_evt_b >> out_evt_a;
    out_evt_b('y');
    TEST_ASSERT_EQUAL('y', a);
    TEST_ASSERT_EQUAL('\0', b);
    out_evt_b >> in_bhv_b;
    out_evt_b('x');
    TEST_ASSERT_EQUAL('x', a);
    TEST_ASSERT_EQUAL('x', b);

    TEST_ASSERT(out_evt_a);
    TEST_ASSERT(out_evt_b);
    TEST_ASSERT_FALSE(!out_evt_a);
    TEST_ASSERT_FALSE(!out_evt_b);
    TEST_ASSERT(in_bhv_a);
    TEST_ASSERT(in_bhv_b);
    TEST_ASSERT_FALSE(!in_bhv_a);
    TEST_ASSERT_FALSE(!in_bhv_b);

    out_evt_a.detach();
    TEST_ASSERT_FALSE(out_evt_a);
    opt_in_bhv_b.reset();
    TEST_ASSERT(out_evt_b);
    TEST_ASSERT(in_bhv_a);
    opt_out_evt_b.reset();
    TEST_ASSERT(!in_bhv_a);  // All detached.
}

TEST_CASE(performance)
{
    const bool              choice     = (pseudorandom_u32() % 2) == 0;
    constexpr std::uint32_t iterations = 30'000'000;
    static constexpr F32    magic      = 1e-5F;
    F32                     y          = 0;  // This is an artificial data dependency to constrain the optimization.

    const auto reset_y = [&y] { y = 1.0F / static_cast<F32>((pseudorandom_u32() % 0xFF'FF'FFU) + 1U); };

    // OOP reference for comparison. Different implementations and dummy data to avoid devirtualization and inlining.
    using Base = Callable<void(F32&)>;
    struct Impl1 : public Base  // NOLINT(cppcoreguidelines-special-member-functions,hicpp-special-member-functions)
    {
        F32& y;
        explicit Impl1(F32& y) : y(y) {}
        void operator()(F32& x) const override
        {
            x += y;
            y += magic;
        }
        virtual ~Impl1() = default;
    } const oop_1{y};
    struct Impl2 : public Base  // NOLINT(cppcoreguidelines-special-member-functions,hicpp-special-member-functions)
    {
        F32& y;
        explicit Impl2(F32& y) : y(y) {}
        void operator()(F32& x) const override
        {
            x -= y;
            y += magic;
        }
        virtual ~Impl2() = default;
    } const oop_2{y};
    const Base& oop = choice ? static_cast<const Base&>(oop_1) : static_cast<const Base&>(oop_2);

    // Actor model alternative that does the same.
    Pullable<F32> out_bhv_1 = [&y](F32& x)
    {
        x += y;
        y += magic;
    };
    Pullable<F32> out_bhv_2 = [&y](F32& x)
    {
        x -= y;
        y += magic;
    };
    Puller<F32> in_evt;
    in_evt ^ (choice ? out_bhv_1 : out_bhv_2);

    // Establish the baseline to subtract it later.
    reset_y();
    Duration elapsed_baseline{};
    {
        F32        x     = 0;
        const auto start = get_host().get_ticks();
        for (std::uint32_t i = 0; i < iterations; i++)
        {
            x += y;  // There is an intentional data dependency between sequential loop executions.
            y += magic;
        }
        elapsed_baseline = get_host().convert_ticks_to_duration(get_host().get_ticks() - start);
        logln("Final value:", x);  // Create an explicit data dependency to constrain the optimization.
    }
    // Benchmark the OOP version.
    reset_y();
    Duration elapsed_oop{};
    {
        F32        x     = 0;
        const auto start = get_host().get_ticks();
        for (std::uint32_t i = 0; i < iterations; i++)
        {
            oop(x);
        }
        elapsed_oop = get_host().convert_ticks_to_duration(get_host().get_ticks() - start);
        logln("Final value:", x);  // Create an explicit data dependency to constrain the optimization.
    }
    // Benchmark the actor model version.
    reset_y();
    Duration elapsed_actor{};
    {
        F32        x     = 0;
        const auto start = get_host().get_ticks();
        for (std::uint32_t i = 0; i < iterations; i++)
        {
            in_evt(x);
        }
        elapsed_actor = get_host().convert_ticks_to_duration(get_host().get_ticks() - start);
        logln("Final value:", x);  // Create an explicit data dependency to constrain the optimization.
    }

    logln("Baseline: elapsed: ", elapsed_baseline, " s; per iteration: ", elapsed_baseline / iterations, " s");
    logln("OOP:      elapsed: ", elapsed_oop, " s; per iteration: ", elapsed_oop / iterations, " s");
    logln("Actor:    elapsed: ", elapsed_actor, " s; per iteration: ", elapsed_actor / iterations, " s");
    using D                          = std::chrono::duration<F32>;
    const F32 slowdown_uncompensated = std::chrono::duration_cast<D>(elapsed_actor) /  //
                                       std::chrono::duration_cast<D>(elapsed_oop);
    const F32 slowdown_compensated = std::chrono::duration_cast<D>(elapsed_actor - elapsed_baseline) /
                                     std::chrono::duration_cast<D>(elapsed_oop - elapsed_baseline);
    logln("Slowdown factor uncompensated: ", slowdown_uncompensated, "; compensated: ", slowdown_compensated);
}

TEST_CASE(latch)
{
    Pusher<float>             out_evt;
    Puller<double>            in_evt;
    Latch<int, float, double> latch{0};
    out_evt >> latch.in;
    in_evt >> latch.out;
    TEST_ASSERT_EQUAL(0, *in_evt);
    out_evt(123.0F);
    TEST_ASSERT_EQUAL(123, *in_evt);
}

TEST_CASE(lift)
{
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
    TEST_ASSERT_EQUAL(12345, output);
    input = 4321;
    trigger();
    TEST_ASSERT_EQUAL(4321, output);
}

TEST_CASE(push_unary)
{
    Latch<int>            output;
    PushUnary<int, float> obj = [](const float x) { return static_cast<int>(x * 2); };
    Pusher<float>         trigger;
    trigger >> obj.in;
    obj.out >> output.in;
    trigger(1.5F);
    TEST_ASSERT_EQUAL(3, output.value);
}

TEST_CASE(push_unary_void)
{
    std::uint64_t                                 counter  = 0;
    int                                           argument = 0;
    Pushable<>                                    in_bhv   = [&] { counter++; };
    PushUnary<Footprint<sizeof(int*)>, void, int> obj      = [&argument](const int x) { argument = x; };
    Pusher<int>                                   trigger;
    trigger >> obj.in;
    obj.out >> in_bhv;
    trigger(3);
    TEST_ASSERT_EQUAL(3, argument);
    TEST_ASSERT_EQUAL(1, counter);
    trigger(9);
    TEST_ASSERT_EQUAL(9, argument);
    TEST_ASSERT_EQUAL(2, counter);
}

TEST_CASE(pull_unary_0)
{
    PullUnary<int> obj = [] { return 9; };
    Puller<int>    trigger;
    trigger >> obj.out;
    TEST_ASSERT_EQUAL(9, *trigger);
}

TEST_CASE(pull_unary_1)
{
    Latch<float>                        input{1.5F};
    PullUnary<Footprint<1>, int, float> obj = [](const float x) { return static_cast<int>(x * 2); };
    Puller<int>                         trigger;
    trigger >> obj.out;
    obj.in >> input.out;
    TEST_ASSERT_EQUAL(3, *trigger);
}

TEST_CASE(pull_unary_2_init)
{
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
    TEST_ASSERT_EQUAL(4, *trigger);
}

TEST_CASE(pull_unary_2_default)
{
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
    TEST_ASSERT_EQUAL(4, *trigger);
}

TEST_CASE(pull_nary_0)
{
    PullNary<int> obj = [] { return 123; };
    Puller<int>   trigger;
    obj.out ^ trigger;
    TEST_ASSERT_EQUAL(123, *trigger);
}

TEST_CASE(pull_nary_2)
{
    Latch<int>                   a{123};
    Latch<double>                b{456.0};
    PullNary<float, int, double> obj = [](const int x, const double y)
    { return static_cast<float>(static_cast<double>(x) * y); };
    Puller<float> trigger;
    obj.out ^ trigger;
    std::get<0>(obj.in) ^ a.out;
    std::get<1>(obj.in) ^ b.out;
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 56088, *trigger);
}

TEST_CASE(cast)
{
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
    TEST_ASSERT_EQUAL(3, latch.value);
    TEST_ASSERT_EQUAL(3.0, *pull_trigger);
}

TEST_CASE(ctor)
{
    int        value = 0;
    const Ctor ctor  = [&value] { value = 123; };
    (void) ctor;
    TEST_ASSERT_EQUAL(123, value);
}

TEST_CASE(finalizer)
{
    int value = 0;
    {
        const Finalizer fin1 = [&value] { value += 1; };
        (void) fin1;
    }
    {
        Finalizer moved_into;
        TEST_ASSERT_EQUAL(1, value);
        {
            Finalizer fin2 = [&value] { value += 10; };
            moved_into     = std::move(fin2);
        }
        TEST_ASSERT_EQUAL(1, value);
    }
    TEST_ASSERT_EQUAL(11, value);
    {
        Finalizer fin3 = [&value] { value += 222; };
        fin3.disarm();  // Will not be invoked.
    }
    TEST_ASSERT_EQUAL(11, value);
}

}  // namespace
}  // namespace dyshlo::verification
