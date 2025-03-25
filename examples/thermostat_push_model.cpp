/// Public domain dedication: RAMEN usage example by Pavel Kirienko <pavel.kirienko@zubax.com> is marked with CC0 1.0.
///
/// In this example we simulate a simple thermostat system with a PID controller that tracks a sinewave setpoint.
/// Here, we are using the push model, meaning that the outputs are pushed when inputs are updated; this can also be
/// referred to as the eager model. Compare it to the pull model counterpart.

#include <ramen.hpp>
#include <chrono>
#include <cmath>
#include <numbers>
#include <iostream>
#include <thread>

using Time        = std::chrono::nanoseconds;
constexpr auto pi = std::numbers::pi_v<float>;
using namespace std::chrono_literals;

/// An ordinary parallel PID controller using the push model (eager evaluation).
///
/// Here, we implement explicitly synchronized computation, where the actor only triggers its output events upon
/// receiving an explicit tick signal (which conveniently carries the current time delta). Other approaches are
/// possible; for example, we could have the controller update the outputs upon arrival of a certain specific input,
/// or any input. If necessary, the controller can also pull needed data via additional pull input ports.
///
/// The conventional diagram notation is as follows: data inputs on the left, data outputs on the right;
/// arrows represent the control flow direction.
///
///                                  ┌───────────────┐
///                          (float) │ PidController │ (float)
///             in_setpoint ────────►│               ├────────► out_effort
///                                  │               │
///                          (float) │               │
///     in_process_variable ────────►│               │
///                                  │               │
///                           (Time) │               │
///                 in_tick ────────►│               │
///                                  └───────────────┘
struct PidController
{
    // PRIVATE ACTOR STATES
    // It is convenient to define them near the top to make them accessible for the behavior definitions below.
    //
    // A normal OOP program would make them private fields of the class. Here, in the flow-based programming model,
    // items can be kept public because actors do not see each other directly; instead, they interact strictly via
    // events and behaviors. This is a key feature of flow-based programming that enables the composability of actors.
    //
    // As a positive side effect, public fields enable aggregate initialization.
    std::array<float, 2> gain_pi{0, 0};
    std::array<float, 2> integration_min_max{0, 0};
    float                integral{0};
    float                setpoint{0};
    float                process_variable{0};

    // EVENTS
    // These ports allow the actor to trigger behaviors in other actors that it is connected to.
    // Crucially, this actor has no idea how those behaviors are implemented; it only knows their data types.
    // Note: the empty {} braces are not really required, but they silence the warning about the missing initializers.
    ramen::Pusher<float> out_effort{};

    // BEHAVIORS
    // This is where the actual logic of the actor is implemented.
    ramen::Pushable<Time> in_tick = [this](const Time time_delta)
    {
        const float dt    = std::chrono::duration_cast<std::chrono::duration<float>>(time_delta).count();
        const float error = setpoint - process_variable;
        integral = std::clamp(integral + (error * dt * gain_pi[1]), integration_min_max[0], integration_min_max[1]);
        out_effort((error * gain_pi[0]) + integral);
    };
    ramen::Pushable<float> in_setpoint         = [this](const float x) { setpoint = x; };
    ramen::Pushable<float> in_process_variable = [this](const float x) { process_variable = x; };
};

/// This node will be used as the setpoint source. It generates a sinewave with the specified parameters.
///
///                            ┌──────────┐
///                    (float) │ Sinewave │ (float)
///             input ────────►│          ├────────► output
///                            └──────────┘
struct Sinewave
{
    float freq{1};
    float amp{1};
    float phase{0};

    ramen::Pusher<float> output{};

    ramen::Pushable<float> input = [this](const float x) { output(amp * std::sin((2 * pi * freq * x) + phase)); };
};

/// A very basic thermal system with a heat source and an environment that sinks the heat.
///
///                                         ┌──────────────┐
///                                 (float) │ ThermalModel │ (float)
///                 in_heat_source ────────►│              ├────────► out_temperature
///                                         │              │
///                                 (float) │              │
///     in_environment_temperature ────────►│              │
///                                         │              │
///                                  (Time) │              │
///                        in_tick ────────►│              │
///                                         └──────────────┘
struct ThermalModel
{
    // INTERNAL STATES
    float temperature{0};
    float heating_factor{1};
    float dissipation_factor{1};
    float heat_source{0};
    float environment_temperature{0};

    // EVENTS
    ramen::Pusher<float> out_temperature{};

    // BEHAVIORS
    ramen::Pushable<Time> in_tick = [this](const Time time_delta)
    {
        const float dt = std::chrono::duration_cast<std::chrono::duration<float>>(time_delta).count();
        temperature += heat_source * heating_factor * dt;                                  // Energy input
        temperature -= (temperature - environment_temperature) * dissipation_factor * dt;  // Dissipation
        out_temperature(temperature);
    };
    ramen::Pushable<float> in_heat_source             = [this](const float x) { heat_source = x; };
    ramen::Pushable<float> in_environment_temperature = [this](const float x) { environment_temperature = x; };
};

int main()  // NOLINT(bugprone-exception-escape)
{
    // Instantiate the actors.
    ThermalModel  plant{.temperature = 0, .heating_factor = 0.1F, .dissipation_factor = 0.1F};
    Sinewave      commander{.freq = 1.0F, .amp = 10};
    PidController controller{.gain_pi = {100, 10}, .integration_min_max = {-100, 100}};

    // ----------------------------------------------------------------------------------------------------------------

    // Set up some additional ports we're going to need. We could create dedicated actors for this, too.
    // Time delta pusher.
    ramen::Pusher<Time> ticker;

    // The final result of the simulation.
    ramen::Pushable<float> float_printer = [](const float x) { std::cout << x << "\n"; };

    // Pushers for ingesting data into the network.
    ramen::Pusher<float> environment_temperature;
    ramen::Pusher<float> commander_time;

    // ----------------------------------------------------------------------------------------------------------------

    // Link the network. The arrows in the operator>> represent the direction of the control flow, not data flow;
    // in the push model they are the same, but in the pull model they are the opposite.
    ticker >> controller.in_tick >> plant.in_tick;
    commander_time >> commander.input;
    commander.output >> controller.in_setpoint;  // We can also connect it to the float_printer for visualization.
    controller.out_effort >> plant.in_heat_source;
    plant.out_temperature >> controller.in_process_variable >> float_printer;
    environment_temperature >> plant.in_environment_temperature;

    // ----------------------------------------------------------------------------------------------------------------

    // Run the simulation by simply pulling the final output_temperature port.
    // Every time it is pulled, the entire network will be evaluated.
    auto time = std::chrono::steady_clock::now();
    for (int i = 0; i < 200; i++)
    {
        const auto t  = std::chrono::steady_clock::now();
        const auto dt = t - time;
        time          = t;
        // Update inputs in this tick.
        commander_time(std::chrono::duration_cast<std::chrono::duration<float>>(t.time_since_epoch()).count());
        environment_temperature(3);
        // Explicitly trigger the network with a new tick message.
        ticker(dt);
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}
