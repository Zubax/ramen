/// Public domain dedication: RAMEN usage example by Pavel Kirienko <pavel.kirienko@zubax.com> is marked with CC0 1.0.
///
/// In this example we simulate a simple thermostat system with a PID controller that tracks a sinewave setpoint.

#include <ramen.hpp>
#include <chrono>
#include <cmath>
#include <numbers>
#include <iostream>
#include <thread>

using Time        = std::chrono::nanoseconds;
constexpr auto pi = std::numbers::pi_v<float>;
using namespace std::chrono_literals;

/// An ordinary parallel PID controller using the pull model (lazy evaluation).
/// The result is computed when the output is pulled. The inputs are pulled from the connected actors when needed.
struct PidController
{
    // PRIVATE ACTOR STATES
    // It is convenient to define them near the top to make them accessible for the behavior definitions below.
    //
    // A normal OOP program would make them private fields of the class. Here, in the flow-based programming model,
    // items can be kept public because actors do not see each other directly; instead, they interact strictly via
    // events and behaviors. This is a key feature of flow-based programming that enables the composability of actors.
    //
    // As a positive side effect, the fact that the fields are public enables the use of aggregate initialization.
    std::array<float, 2> gain_pi{0, 0};
    std::array<float, 2> integration_min_max{0, 0};
    float                integral{0};
    std::optional<Time>  prev_time{};

    // EVENTS
    // These ports allow the actor to trigger behaviors in other actors that it is connected to.
    // Crucially, this actor has no idea how those behaviors are implemented; it only knows their data types.
    // Note: the empty {} braces are not really required, but they silence the warning about the missing initializers.
    ramen::Puller<float> in_setpoint{};
    ramen::Puller<float> in_process_variable{};
    ramen::Puller<Time>  in_time{};

    // BEHAVIORS
    // This is where the actual logic of the actor is implemented.
    ramen::Pullable<float> out_effort = [this](float& out)
    {
        // First, compute the time delta by subtracting the previous time from the current time.
        // Homework: if the I term is zero, no need to compute this, just set dt to zero!
        const Time  t  = *in_time;
        const float dt = prev_time  //
                             ? std::chrono::duration_cast<std::chrono::duration<float>>(t - *prev_time).count()
                             : 0;
        prev_time      = t;
        // Compute the error and the integral term.
        const float error = *in_setpoint - *in_process_variable;
        integral = std::clamp(integral + (error * dt * gain_pi[1]), integration_min_max[0], integration_min_max[1]);
        out      = error * gain_pi[0] + integral;
    };
};

/// This node will be used as the setpoint source. It generates a sinewave with the specified parameters.
struct Sinewave
{
    float freq{1};
    float amp{1};
    float phase{0};

    ramen::Puller<float> input{};

    ramen::Pullable<float> output = [this](float& out) { out = amp * std::sin((2 * pi * freq * *input) + phase); };
};

/// A very basic thermal system with a heat source and an environment that sinks the heat.
struct ThermalModel
{
    // INTERNAL STATES
    float               temperature{0};
    float               heating_factor{1};
    float               dissipation_factor{1};
    std::optional<Time> prev_time{};

    // EVENTS
    ramen::Puller<float> in_heat_source{};
    ramen::Puller<float> in_environment_temperature{};
    ramen::Puller<Time>  in_time{};

    // BEHAVIORS
    ramen::Pullable<float> out_temperature = [this](float& out)
    {
        // Compute the time delta.
        const Time  t = *in_time;
        const float dt =
            prev_time ? std::chrono::duration_cast<std::chrono::duration<float>>(t - *prev_time).count() : 0;
        prev_time = t;
        // Compute the temperature change.
        temperature += *in_heat_source * heating_factor * dt;                                  // Energy input
        temperature -= (temperature - *in_environment_temperature) * dissipation_factor * dt;  // Dissipation
        // Output the current temperature.
        out = temperature;
    };
};

int main()
{
    // Instantiate the actors.
    ThermalModel  plant{.temperature = 10, .heating_factor = 0.1F, .dissipation_factor = 0.05F};
    Sinewave      commander{.freq = 0.1F, .amp = 10};
    PidController controller{.gain_pi = {0.1F, 0.01F}, .integration_min_max = {-10, 10}};

    // Set up some additional ports we're going to need. We could create dedicated actors for this, too.
    // Source of the current time.
    ramen::Pullable<Time> now = [](Time& out)
    { out = std::chrono::duration_cast<Time>(std::chrono::steady_clock::now().time_since_epoch()); };

    // The final result of the simulation.
    ramen::Puller<float> output_temperature;

    // An auxiliary node for converting the rich Time type into a simple float.
    ramen::PullUnary<float, Time> time_to_seconds = [](const Time t)
    { return std::chrono::duration_cast<std::chrono::duration<float>>(t).count(); };

    // A simple constant value.
    ramen::Latch<float> environment_temperature{3};

    // Build the network. The arrows in the operator>> represent the direction of the control flow, not data flow.
    time_to_seconds.in >> controller.in_time >> plant.in_time >> now;
    commander.input >> time_to_seconds.out;
    controller.in_setpoint >> commander.output;
    output_temperature >> controller.in_process_variable >> plant.out_temperature;
    plant.in_heat_source >> controller.out_effort;
    plant.in_environment_temperature >> environment_temperature.out;

    // Run the simulation by simply pulling the final output_temperature port.
    // Every time it is pulled, the entire network will be evaluated.
    for (int i = 0; i < 100; i++)
    {
        std::cout << "Temperature: " << *output_temperature << "°C\n";
        std::this_thread::sleep_for(10ms);
    }

    return 0;
}
