# Real-time Actor-based Message Exchange Network 🍜

[![Verification](https://github.com/Zubax/ramen/actions/workflows/verification.yml/badge.svg)](https://github.com/Zubax/ramen/actions/workflows/verification.yml)
[![Forum](https://img.shields.io/discourse/https/forum.zubax.com/users.svg?color=e00000)](https://forum.zubax.com)

RAMEN is a very compact single-header C++20+ dependency-free library that implements
message-passing/[flow-based](https://en.wikipedia.org/wiki/Flow-based_programming)
programming for hard real-time mission-critical embedded systems, as well as general-purpose applications.
It is designed to be very low-overhead, efficient, and easy to use.

**To use the library in your project**, simply copy `ramen/ramen.hpp` into your project tree, `#include <ramen.hpp>`,
and you're ready to roll. Alternatively, add this repository as a submodule, and add `ramen/` to the include paths.

It should work on any conventional platform, from 8-bit to 64-bit; if you find this to be untrue, please open a ticket.

## Dazzle me 🤯

...

`std::variant` can be used to pass distinct message types via the same link, which may occasionally be useful.

## Development 🧑‍💻

The `CMakeLists.txt` recipe is for development purposes only. Do not use in production.
