/// Public domain dedication: RAMEN usage example by Pavel Kirienko <pavel.kirienko@zubax.com> is marked with CC0 1.0.
///
/// A demo implementation of Peter Naur's "telegram problem" using RAMEN push model (eager computation).
/// The task is to accept blocks of text and generate output lines, each containing as many words as possible,
/// where the number of characters in each line does not exceed a certain limit.

#include <ramen.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <string_view>
#include <variant>

/// The end tag is used to indicate that the input stream has ended.
struct End final
{
};

/// The decomposer is the input of the network. It accepts text, and pushes out individual words one by one.
///
///                            ┌────────────┐
///          (string_view|End) │ Decomposer │ (string_view|End)
///           in_text ────────►│            ├────────► out_word
///                            └────────────┘
struct Decomposer
{
    ramen::Pusher<std::variant<std::string_view, End>> out_word{};

    ramen::Pushable<std::variant<std::string_view, End>> in_text = [this](const std::variant<std::string_view, End>& in)
    {
        if (const auto* const txt = std::get_if<std::string_view>(&in))
        {
            std::string_view text = *txt;
            while (!text.empty())
            {
                const auto pos = text.find_first_of(" \t\n");
                if (pos == std::string_view::npos)
                {
                    out_word(text);
                    break;
                }
                out_word(text.substr(0, pos));
                text.remove_prefix(pos + 1);
            }
        }
        else
        {
            out_word(std::get<End>(in));
        }
    };
};

/// The aggregator accepts words one by one, and builds lines of text from them such that each line is as long as
/// possible but does not exceed the specified limit.
///
///                            ┌────────────────┐
///          (string_view|End) │ LineAggregator │ (string_view)
///           in_word ────────►│                ├────────► out_line
///                            └────────────────┘
struct LineAggregator
{
    std::size_t line_length_limit{0};
    std::string current_line{};

    ramen::Pusher<std::string_view> out_line{};

    ramen::Pushable<std::variant<std::string_view, End>> in_word = [this](const std::variant<std::string_view, End>& in)
    {
        if (const auto* const word = std::get_if<std::string_view>(&in))
        {
            if (current_line.size() + word->size() >= line_length_limit)
            {
                out_line(current_line);
                current_line.clear();
            }
            if (!current_line.empty())
            {
                current_line += ' ';
            }
            current_line += *word;
        }
        else
        {
            out_line(current_line);
            current_line.clear();
        }
    };
};

constexpr std::string_view sample_text_1 = R"===(
Software engineers should look to their hardware counterparts for approaches to managing complexity. Before the advent
of Integrated Circuits, electronic circuits were generally complex creatures with many interconnected discrete
components. The complexity of the circuit was visible, difficult to manage and adversely affected the cost of products.
With the advent of ICs much of this complexity was hidden inside the chips themselves. The job of designing complex
functionality into products became much easier.
)===";

constexpr std::string_view sample_text_2 = R"===(
Despite many attempts over the years, software design has never been able to replicate this IC design paradigm.
The advent of object-oriented programming languages and tools was supposed to address some of these issues.
While object-oriented design offers some significant improvements in areas such as GUI programming it doesn't
always do a great job of hiding complexity. In fact, they often simply shift the complexity into other areas in
the software development chain such as testing, toolsets, class library design or the learning curve.
)===";

constexpr std::string_view sample_text_3 = R"===(
Modern hardware design requires a complex skillset. What the advent of ICs did, however, was allow the hardware
designer to use a given chip in a product design without having to understand the exact physics and layout of the
circuits contained within the chip itself. As long as the designer conformed to the specs at the external interface
to the chip (pins) the chip will react in a very predictable manner. This is what encapsulation of complexity offers:
complexity hiding and predictable/reproducible behavior. Objects in software design offer partial complexity hiding,
but the software designer often still has to know and master a complex language (e.g., C++) in order to be able to
wire objects together into a product. Objects are very poor at providing predictable and reproducible software behavior.
Furthermore, the language of the objects themselves often dictates the language used to “wire them together”.
In our opinion, true encapsulation of complexity behind a universally simple and extendable API is what is required
to produce a software IC. The software designer should not have to master a complex object-oriented language and
toolset in order to be able to “wire together” these software ICs. At the very least the software designer should be
able to choose the wiring language independent of the chip language.
)===";

int main()
{
    // Instantiate the actors.
    Decomposer     decomposer;
    LineAggregator line_aggregator{80};

    // Top-level ports: input text and output lines.
    ramen::Pusher<std::variant<std::string_view, End>> ingest_text;
    ramen::Pushable<std::string_view> sink_line = [](const std::string_view& line) { std::cout << line << '\n'; };

    // Link up the network.
    ingest_text >> decomposer.in_text;
    decomposer.out_word >> line_aggregator.in_word;
    line_aggregator.out_line >> sink_line;

    // Push some text into the network.
    ingest_text(sample_text_1);
    ingest_text(sample_text_2);
    ingest_text(sample_text_3);
    ingest_text(End{});

    return 0;
}
