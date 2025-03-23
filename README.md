# Real-time Actor-based Message Exchange Network 🍜

RAMEN is a very compact single-header C++20+ dependency-free library that implements message-passing computation semantics for hard real-time mission-critical embedded systems, as well as general-purpose applications. It is designed to be very low-overhead, efficient, and easy to use.

**To use the library in your project**, simply copy `ramen/ramen.hpp` into your project tree, `#include <ramen.hpp>`, and you're ready to roll. Alternatively, add this repository as a submodule, and add `ramen/` to the include paths.

In theory, it should work on any conventional platform, from 8-bit to 64-bit; if you find this to be untrue, please open a ticket.

## Dazzle me

...


## Trade-offs

Internally, each message passing invocation goes through two virtual method calls.
