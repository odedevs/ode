The Open Dynamics Engine (ODE)
==============================

[![Windows Build](https://github.com/odedevs/ode/actions/workflows/build-windows.yml/badge.svg)](https://github.com/odedevs/ode/actions/workflows/build-windows.yml)
[![MSYS2 Build](https://github.com/odedevs/ode/actions/workflows/build-msys2.yml/badge.svg)](https://github.com/odedevs/ode/actions/workflows/build-msys2.yml)
[![Ubuntu Build](https://github.com/odedevs/ode/actions/workflows/build-ubuntu.yml/badge.svg)](https://github.com/odedevs/ode/actions/workflows/build-ubuntu.yml)
[![Arch Linux Build](https://github.com/odedevs/ode/actions/workflows/build-archlinux.yml/badge.svg)](https://github.com/odedevs/ode/actions/workflows/build-archlinux.yml)
[![macOS Build](https://github.com/odedevs/ode/actions/workflows/build-macos.yml/badge.svg)](https://github.com/odedevs/ode/actions/workflows/build-macos.yml)

![ODE logo](https://raw.githubusercontent.com/odedevs/ode/master/web/ODElogo.png)

Copyright (C) 2001-2007 Russell L. Smith.
Copyright (C) 2008-2026 Open Dynamics Engine contributors.


ODE is a free, industrial quality library for simulating articulated
rigid body dynamics - for example ground vehicles, legged creatures,
and moving objects in VR environments. It is fast, flexible, robust
and platform independent, with advanced joints, contact with friction,
and built-in collision detection.

This library is free software; you can redistribute it and/or
modify it under the terms of EITHER:

- The GNU Lesser General Public License version 2.1 or any later.
- The BSD-style License.

See the [COPYING](https://raw.githubusercontent.com/odedevs/ode/master/COPYING) file for more details.

Links
-----

- The ODE web pages are at [ode.org](https://www.ode.org/).
- An online manual is at [the Wiki](https://ode.org/wiki/index.php?title=Manual).
- API documentation is in the file ode/docs/index.html, or you
  can view it on the web at [opende.sf.net/docs/index.html](http://opende.sf.net/docs/index.html).
- Coding style requirements can be found in the [CSR.txt](https://raw.githubusercontent.com/odedevs/ode/master/CSR.txt) file.


Building
--------

ODE uses CMake as its build system. CMake can generate project files for many
platforms and IDEs including Unix Makefiles, Ninja, Visual Studio, Xcode, and
more.

### Prerequisites

- [CMake](https://cmake.org/) 3.10 or later
- A C/C++ compiler (GCC, Clang, MSVC, MinGW, etc.)
- OpenGL and GLU (only if building demos with `-DODE_WITH_DEMOS=ON`)

### Quick Start

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
cmake --install build
```

To run the test suite:

```sh
ctest --test-dir build --output-on-failure
```

A specific generator can be selected with `-G`:

```sh
cmake -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake -B build -G "Visual Studio 17 2022"
cmake -B build -G "MSYS Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake -B build -G "Xcode"
```

Most modern IDEs (Visual Studio, CLion, Qt Creator, VS Code) can open the
source directory with the `CMakeLists.txt` file directly.

### CMake Options

Options can be set with `-D` on the cmake command line:

```sh
cmake -B build -DODE_DOUBLE_PRECISION=ON -DODE_WITH_DEMOS=OFF
```

| Option | Description | Default |
|--------|-------------|---------|
| `ODE_DOUBLE_PRECISION` | Use double-precision math | ON (64-bit), OFF (32-bit) |
| `ODE_WITH_DEMOS` | Build demo applications and DrawStuff library (requires OpenGL) | ON |
| `ODE_WITH_TESTS` | Build the unit test application | ON |
| `ODE_WITH_OPCODE` | Use OPCODE for trimesh collisions | ON |
| `ODE_WITH_GIMPACT` | Use GIMPACT for trimesh collisions (experimental) | OFF |
| `ODE_WITH_LIBCCD` | Use libccd for additional collision tests | OFF |
| `ODE_WITH_OU` | Use TLS for global caches, allows threaded collision checks | OFF |
| `ODE_16BIT_INDICES` | Use 16-bit trimesh indices instead of 32-bit | OFF |
| `ODE_NO_THREADING_INTF` | Disable threading interface support | OFF |
| `BUILD_SHARED_LIBS` | Build shared library instead of static | ON |

When `ODE_WITH_LIBCCD` is ON, additional sub-options become available for
specific collision pairs (box-cylinder, capsule-cylinder, convex-convex, etc.).
See `CMakeLists.txt` for the full list.

### Platform Notes

#### Windows (Visual Studio)

```bat
cmake -B build
cmake --build build --config Release --parallel
```

CMake will auto-detect Visual Studio. The library output name includes the
precision by default (`ode_double` or `ode_single`).

#### Windows (MSYS2)

From an MSYS2 shell (mingw64, ucrt64, or clang64):

```sh
cmake -G "MSYS Makefiles" -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
```

#### Linux

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
sudo cmake --install build
```

Demos require OpenGL development packages. On Debian/Ubuntu:

```sh
sudo apt-get install libglu1-mesa-dev freeglut3-dev
```

#### macOS

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(sysctl -n hw.ncpu)
```

Xcode projects can be generated with `-G Xcode`.
