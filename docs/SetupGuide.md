# HF-TMC9660 Setup Guide

This guide walks through getting the library onto your machine and
building it from source.  It assumes a standard desktop environment
with the GNU toolchain available.

---

## 1. Clone the Repository

Begin by cloning the code from GitHub.  If you have not already done so
install `git` and run:

```bash
git clone https://github.com/<your-account>/HF-TMC9660.git
cd HF-TMC9660
```

The remainder of this guide assumes the working directory is the root of
the cloned repository.

---

## 2. Install a C++20 Toolchain

The driver requires a C++20 compliant compiler.  On Linux this typically
means `g++` 10 or later.  On Windows you can use the MSYS2 or MinGW
packages.  Verify your compiler with:

```bash
g++ --version
```

If you get an error or the reported version is older than 10 then update
or install a newer compiler before proceeding.

---

## 3. Build the Library

No special build system is required.  Simply compile the library source
along with any example or application code.  The minimum command line is
shown below:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp -c -o TMC9660.o
```

This creates an object file which you can link into your program.  If
you are using the bootloader helper add `src/TMC9660Bootloader.cpp` to
the command line as well.

---

## 4. Verify the Setup with an Example

To confirm that everything is working try building one of the provided
examples.  The following compiles the Hall sensor BLDC demo:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp examples/BLDC_with_HALL.cpp -o hall_demo
```

Running the resulting executable should print a short message to the
console.  At this point the library is built correctly and ready for
integration into your own project.

---

## 5. Next Steps

* Implement a custom communication interface as described in
  [ImplementingCommInterface.md](ImplementingCommInterface.md).
* Explore the full examples in [HardwareAgnosticExamples.md](HardwareAgnosticExamples.md).

