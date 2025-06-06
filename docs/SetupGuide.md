# HF-TMC9660 Setup Guide

This friendly guide walks you through getting the library onto your machine
and compiling it for a quick test build – or for integration into a larger
project. It assumes a desktop environment with the GNU toolchain available –
any C++20 compliant compiler will do.

---

## 1. Clone the Repository

Begin by cloning the code from GitHub. If you plan to integrate the driver
into another repository you can instead add it as a Git submodule. Install
`git` if needed and run:

```bash
git clone https://github.com/<your-account>/HF-TMC9660.git
cd HF-TMC9660
```
Or as a submodule:
```bash
git submodule add https://github.com/<your-account>/HF-TMC9660.git external/HF-TMC9660
cd external/HF-TMC9660
```

The remainder of this guide assumes the working directory is the root of
the cloned repository.

---

## 2. Install a C++20 Toolchain

The driver requires a modern compiler with C++20 support.

* **Linux** – `g++` 10+ or Clang 11+.
* **Windows** – MSYS2/MinGW packages or Visual Studio 2019 with the
  `-std:c++20` switch.

Verify your compiler with:

```bash
g++ --version
```

If you get an error or the reported version is older than 10 then update
or install a newer compiler before proceeding.

---

## 3. Build the Library

No special build system is provided so you can slot the files directly into
your own build. For a quick standalone test, compile the sources with a single
command:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp -c -o TMC9660.o
```

This creates an object file which you can link into your program.  If you are
using the bootloader helper add `src/TMC9660Bootloader.cpp` to the command line
as well. When integrating with a larger project you may instead compile the
sources as part of your existing build system or turn them into a static
library.

---

## 4. Verify the Setup with an Example

To confirm that everything is working try building one of the provided
examples.  The following compiles the Hall sensor BLDC demo:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp examples/BLDC_with_HALL.cpp -o hall_demo
```

Running the resulting executable should print a short message to the console.
If you see it – 🎉 congratulations! The library is now built correctly and is
ready to drop into your own project.

---

## 5. Next Steps

* Implement a custom communication interface as described in
  [ImplementingCommInterface.md](ImplementingCommInterface.md).
* Explore the full examples in [HardwareAgnosticExamples.md](HardwareAgnosticExamples.md).
* Integrate the object files into your build system or create a small
  static library to reuse across multiple projects.


---

[⬅️ Prev](index.md) | [⬆️ Back to Index](index.md) | [Next ➡️](ImplementingCommInterface.md)
