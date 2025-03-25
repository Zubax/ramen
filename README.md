# Real-time Actor-based Message Exchange Network 🍜

[![Verification](https://github.com/Zubax/ramen/actions/workflows/verification.yml/badge.svg)](https://github.com/Zubax/ramen/actions/workflows/verification.yml)
[![Forum](https://img.shields.io/discourse/https/forum.zubax.com/users.svg?color=e00000)](https://forum.zubax.com)

RAMEN is a very compact, unopinionated, single-header C++20+ dependency-free library that implements
message-passing/[flow-based](https://en.wikipedia.org/wiki/Flow-based_programming)
programming for hard real-time mission-critical embedded systems, as well as general-purpose applications.
It is designed to be very low-overhead, efficient, and easy to use.

**To use the library in your project**, simply copy `ramen/ramen.hpp` into your project tree, `#include <ramen.hpp>`,
and you're ready to roll. Alternatively, add this repository as a submodule, and add `ramen/` to the include paths.

It should work on any conventional platform, from 8-bit to 64-bit; if you find this to be untrue, please open a ticket.

## Why though?

There exists a class of problems in software engineering that are hard to model efficiently using more
conventional paradigms, such as OOP, but are easy to describe using the dataflow model.
Problems of that class are particularly often encountered in embedded real-time control systems and digital signal 
processing (DSP) pipelines.

One approach there is to use model-based design with automatic code generation using LabView, Simulink, etc.
The disadvantage of this approach is that it may be difficult to couple autogenerated code with the rest of the
system (such as the higher-level control logic), as the resulting code tends to be opinionated,
strongly affecting the rest of the codebase.

Another approach is to use a full-scale event framework like QP/C++ et al. These are probably great tools for some 
projects, but importantly, RAMEN is not a framework but a very lightweight library that is designed to work on any
platform out of the box.

RAMEN allows one to apply dataflow programming in an extremely unopinionated way, and it can be coupled with
conventional C/C++ programs ad-hoc. It is implemented in only a few hundred lines of straightforward C++, plus a couple
more hundred lines for some useful utilities that are nice to have in a dataflow program.

RAMEN is typesafe, has no runtime error states, requires no heap, no exceptions, no RTTI, and adds no nontrivial
computational complexity on top of the user logic.

## Notation

There are two way to arrange a dataflow:

- **Push model:** an actor receives data together with control flow and performs evaluations *eagerly*, updating its
  outputs immediately, triggering dependent computations *downstream* from itself.

- **Pull model:** an actor receives control when its outputs are needed, and fetches the data it needs *lazily*,
  triggering dependent computations *upstream*.

Any problem can be modeled using either approach, but some problems may be easier to model using only one or the
other. RAMEN supports both and allows mixing them if necessary (which should be done with caution, though).

RAMEN is built using only two primitive entities:

- **Behaviors** implement business logic, like methods in an OOP program. They contain completely arbitrary user
  code, no strings attached. In the pull model, behaviors produce data, while in the push model they accept data.

- **Events** are used to notify other actors when new data is available (push aka eager model) or new data is
  required for some computation (pull aka lazy model).

Events and behaviors are linked into *topics* using `operator>>`. When an event is triggered, all behaviors on the
topic are executed. In the pull model, there should be only one behavior per topic to avoid ambiguity (otherwise,
behavior executed later will overwrite the output computed by the earlier behaviors, which may not be what you need).
The push model allows mixing an arbitrary number of events and behaviors per topic; there, either of the connected
events will trigger all behaviors (fanout).

Given that behaviors and ports can either sink or source data, we end up with four combinations:

| Port kind    | Control | Data | Alias      |
|--------------|---------|------|------------|
| in-behavior  | in      | in   | `Pushable` |
| out-event    | out     | out  | `Pusher`   |
| out-behavior | in      | out  | `Pullable` |
| in-event     | out     | in   | `Puller`   |

On a diagram, data inputs go on the left, data outputs on the right, and the direction of the control flow is shown
with an arrow:

```
                    ┌───────┐
 (output data type) │ Actor │ (output data type)
  pushable ────────►│       ├────────► pusher
                    │       │
  (input data type) │       │ (output data type)
    puller ◄────────┤       │◄──────── pullable
                    └───────┘
```

## Dazzle me 🤯

Let's make a simple summation node using the pull model (lazy computation):

```
               ┌────────┐
       (float) │ Summer │ (float)
 in_a ◄────────┤        │◄──────── out_sum
               │        │
       (float) │        │
 in_b ◄────────┤        │
               └────────┘
```

```c++
struct Summer
{
    ramen::Puller<float> in_a;
    ramen::Puller<float> in_b;
    ramen::Pullable<float> out_sum = [this](float& out) { out = *in_a + *in_b; };
};
```

We can see that the `Puller` entities are used to source the arguments from whatever external entity this summer is
connected to. Crucially, the summer itself has no idea where the data is coming from.

The data is read using `operator*` for convenience, but this will only work if the data type is default-constructible.
Sometimes it is not (e.g., `Eigen::MatrixRef` has to be bound to the storage matrix upon construction, so it is not
default-constructible; this use case is very common in DSP), in which case the data is obtained using the ordinary
function call syntax:

```c++
float a = 0;
in_a(a);

float b = 0;
in_b(b);

out = a + b;
```

This is also why we return the result via the out-parameter.

The summer can be linked to other actors using `operator>>`; doing so does not allocate dynamic memory and cannot fail,
but it involves a linked list traversal, so it has a linear complexity on the number of ports on the topic.
The direction of the operator arrow follows the direction of the control flow (not data flow).

```c++
Summer sum;

// Create the top-level ports (these could be ports of another actor, but in this example we only have one).
ramen::Pullable<float> ingest_a = [](float& out) { std::cin >> out; };
ramen::Pullable<float> ingest_b = [](float& out) { std::cin >> out; };
ramen::Puller<float> final_answer;

// Link them up. An unconnected behavior is never executed. An unconnected event is computationally free.
sum.in_a >> ingest_a;
sum.in_b >> ingest_b;
final_answer >> sum.out_sum;

// Run the network.
while (true) { std::cout << *final_answer << std::endl; }
```

Some logic will be easier to implement using push model instead. It could be said to be more flexible in certain ways.
Below we have another summer implemented using the push model. One matter that is immediately apparent is that this
naive implementation will update the output whenever either of the inputs are updated; this is rarely the desired
behavior.

```
               ┌────────┐
       (float) │ Summer │ (float)
 in_a ────────►│        ├────────► out_sum
               │        │
       (float) │        │
 in_b ────────►│        │
               └────────┘
```

```c++
struct Summer
{
    float a;
    float b;
    ramen::Pusher<float> out_sum;
    ramen::Pushable<float> in_a = [this](const float x) { a = x; out_sum(a + b); };
    ramen::Pushable<float> in_b = [this](const float x) { b = x; out_sum(a + b); };
};
```

There are many ways to address the update rate issue. The optimal choice depends on the specifics of the problem at
hand. One such approach is to introduce an explicit trigger; in control systems, it is convenient to carry some
shared context via the trigger inputs, such as the update time step $\Delta{}t$.

```
                  ┌────────┐
          (float) │ Summer │ (float)
    in_a ────────►│        ├────────► output
                  │        │
          (float) │        │
    in_b ────────►│        │
                  │        │
               () │        │
 in_tick ────────►│        │
                  └────────┘
```

```c++
struct Summer
{
    float a;
    float b;
    ramen::Pusher<float> out_sum;
    ramen::Pushable<float> in_a = [this](const float x) { a = x; };
    ramen::Pushable<float> in_b = [this](const float x) { b = x; };
    ramen::Pushable<>   in_tick = [this](const float x) { out_sum(a + b); };  // often accepts time delta
};
```

Another way is to use implicit synchronization at the rate of the slowest input like this:

```c++
struct Summer
{
    std::optional<float> a;
    std::optional<float> b;
    
    ramen::Pusher<float> out_sum;
    ramen::Pushable<float> in_a = [this](const float x) { a = x; poll(); };
    ramen::Pushable<float> in_b = [this](const float x) { b = x; poll(); };

    void poll() {
        if (a && b) {
            out_sum(*a + *b);  // Only emit output when both inputs are updated.
            a.reset();         // The output is throttled at the rate of the slowest input.
            b.reset();
        }
    }
};
```

Connection example for the last one:

```c++
Summer sum;

// Create the top-level ports (these could be ports of another actor, but in this example we only have one).
ramen::Pusher<float> ingest_a;
ramen::Pusher<float> ingest_b;
ramen::Pushable<float> print_float = [](const float x) { std::cout << x << std::endl; };

// Link them up. An unconnected behavior is never executed. An unconnected event is computationally free.
ingest_a >> sum.in_a;  //>> print_float; // You can also connect multiple behaviors to be triggered simultaneously.
ingest_b >> sum.in_b;
sum.out_sum >> print_float;

// Run the network.
while (true) {
    // Get the inputs.
    float a, b;
    std::cin >> a >> b;
    // Push the data to the network, triggering computation and eventually printing the final output.
    ingest_a(a);
    ingest_b(b);
}
```

Many practical actors will have a single master input that is updated at a high rate and also drives the main 
computation process, while the other inputs only quickly mutate some internal states.
For example, in a PID controller, the process variable input would normally trigger the computation of a
new control effort output, while the setpoint input would just quickly update the internal state.

Actors are usually implemented as structs with all data fields public, although it is not a requirement.
Public data does not hinder encapsulation because actors are unable to affect or even see each other's members directly,
as all interation is done through message passing.
Hence, the encapsulation mechanisms provided by C++ become redundant.
This point only holds for pure actors, however; there may exist mixed classes that work both as actors and as
regular OOP objects, in which case this consideration may not apply.

Recursive dependencies are common in flow-based programs, especially when they implement control systems.
When implementing an actor, keep in mind that triggering any event to push or pull data can
cause the control flow to eventually loop back to the current actor through a possibly very long chain of interactions.
Proper design should prevent the possibility of descending into an infinite recursion and also serving data
from an actor whose internal state is inconsistent.
To avoid this class of errors, state updates should always be performed in a transactional manner:
first, all inputs are read, then the state is updated, and only then the outputs are written.
This is a general rule of thumb for designing such systems and is not specific to this library.
Improper design can cause an infinite recursion with a subsequent stack overflow.

Defining ports of highly specialized types is possible but rarely useful because specialized types impair composability.
For example, suppose there is a specialized configuration struct for some actor.
The actor could accept the configuration via an in-port and it would work, but the utility of this choice is limited
because in order to make use of this port, the other actors would need to have access to its specific type,
at which point the message passing aspect becomes redundant, as it would be easier to just pass/alter the configuration
struct directly (e.g., by mutating the state of the first actor).
Instead, if the first actor were to accept configuration via more granular in-ports of more generic types,
like vectors, matrices, or whatever is common in the application, then the composability would not be compromised.

Flow-based programming enables a new approach to policy-based design. It is possible to ship predefined policies with
an actor by defining several behaviors, each implementing its own policy while sharing the same type.
The client will then choose which particular behavior to link with their own events at the time when the network is 
linked.

The library ships a few higher-level entities that aid in building practical applications.
Some of the more important ones are reviewed here; for the rest, please check the code -- it is well-documented.

The `Latch` bridges a push-model output with a pull-model input, acting as a one-element-deep queue.
It has behaviors on either side.
The input and output types may be different, but it must be possible to `static_cast` between them.

```
                              ┌───────┐
                        (In)  │ Latch │  (Out)
 (input behavior) in ────────►│       │◄──────── out (output behavior)
                              │       │
                              └───────┘
```

The `Lift` is the counterpart of `Latch` --- it bridges a pull-model output with a push-model input,
meaning that it has events on either side. Events have to be triggered,
which implies that it has an additional empty `trigger` input that fires the events: first `in`, then `out`.

```
                                   ┌───────┐
                             (In)  │ Lift  │  (Out)
         (input event) in ◄────────│       ├────────► out (output event)
                                   │       │
                               ()  │       │
 (input behavior) trigger ────────►│       │
                                   └───────┘
```

Often, messages exchanged over a topic need to be transformed in some way. As this is a common use case, several 
helpers are introduced:

- `PushUnary`/`PullUnary` which apply an unary function to the message;
- `PullNary` applies an N-ary function to N messages, each arriving from a separate input pull port;
- `PushCast`/`PullCast` which are convenience helpers like `PushUnary`/`PullUnary` applying `static_cast`;
- and a few others --- refer to `ramen.hpp` for details.

Here is an actual usage example from a production application --- this is a very simple DQ-frame synchronous machine 
simulator from the Dyshlo library:

```c++
struct DQFrameSynchronousMachine final
{
    static constexpr std::size_t max_phase_count = 15;

    struct Params
    {
        F32                  resistance;
        Vector<2>            inductance_dq;
        F32                  flux_linkage;
        F32                  pole_count;
        clarke::BalancedPair clarke;
    };
    Params param;

    struct
    {
        Vector<2>               i_dq{0, 0};
        F32                     mpos{};
        F32                     mvel{};
        F64                     mpos_contiguous = static_cast<F64>(mpos);
        Vector<max_phase_count> u_ac_storage    = Vector<max_phase_count>::Zero();
    } st{};

    Puller<VectorRef<>> in_ac_voltage_pull{};  ///< Sources the AC phase voltages. Required.
    Puller<F32>         in_velocity{};         ///< Sources the mechanical velocity. Required.

    Pullable<ElectroKinematics> out_electro_kinematics = [this](ElectroKinematics& out)
    {
        out = {.position = normalize_angle(st.mpos * (param.pole_count * 0.5F)),
               .velocity = st.mvel * (param.pole_count * 0.5F)};
    };
    Pullable<F32>      out_force    = [this](F32& out) { out = compute_force(); };
    Pullable<Dynamics> out_dynamics = [this](Dynamics& out)
    {
        out.position = st.mpos_contiguous;
        out.velocity = st.mvel;
        out.force    = compute_force();
    };
    Pullable<ACVoltageCurrent<>> out_ac_voltage_current = [this](ACVoltageCurrent<>& out)
    {
        const auto n = param.clarke.fwd.cols();
        assert((out.voltage.rows() == n) && (out.current.rows() == n));
        out.voltage = st.u_ac_storage.head(n);
        park::inverse(param.clarke.inv, st.i_dq, sincos(st.mpos * (param.pole_count * 0.5F)), out.current);
    };
    Pushable<Tick> in_tick = [this](const Tick& tick)
    {
        const F32 dt = tick.dt * 1.0F;
        if (dt < std::numeric_limits<F32>::epsilon()) { [[unlikely]] return; }
        // Integrate the position first because the AC voltage source may loop back the kinematics behavior,
        // which will be making use of the position estimate.
        in_velocity(st.mvel);
        st.mpos = normalize_angle_fast(st.mpos + (st.mvel * dt));
        unroll_normalized_angle(st.mpos_contiguous, st.mpos);
        // in_ac_voltage may loop back to out_electro_kinematics.
        VectorRef<> u_ac(st.u_ac_storage.head(param.clarke.fwd.cols()));
        in_ac_voltage_pull(u_ac);
        const auto l_d   = param.inductance_dq[0];
        const auto l_q   = param.inductance_dq[1];
        const auto omega = st.mvel * (param.pole_count * 0.5F);
        const auto sc    = sincos(st.mpos * (param.pole_count * 0.5F));
        if (u_ac.allFinite())  // External voltage applied.
        {
            ...
        }
        else  // Terminals disconnected, compute back-EMF.
        {
            ...
        }
    };

    F32 compute_force() const noexcept
    {
        return (3.0F / 4.0F) * param.pole_count * st.i_dq[1] *
               (param.flux_linkage + (st.i_dq[0] * (param.inductance_dq[0] - param.inductance_dq[1])));
    }

    bool all_linked() const noexcept
    {
        return in_ac_voltage_pull && in_velocity && out_electro_kinematics && out_force && out_dynamics &&
               out_ac_voltage_current && in_tick;
    }
};
```

**🎓 For complete examples, refer to the `examples/` directory 📖**

## Development 🧑‍💻

The `CMakeLists.txt` recipe is for development purposes only. Do not use in production.
