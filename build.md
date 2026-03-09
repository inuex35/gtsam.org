---
layout: page
title: Build
permalink: /build/
---

To build GTSAM from source, clone or download the latest release from the [GTSAM GitHub repository](https://github.com/borglab/gtsam). This page folds in the most useful material from the stalled docs refresh while keeping the current site structure.

## Quick Start

From the repository root, use an out-of-source build:

```sh
mkdir build
cd build
cmake ..
make check
make install
```

`make check` is optional, but recommended when you are validating a local build.

## Supported Configurations

| Tested operating systems | Tested compilers |
| --- | --- |
| Ubuntu 16.04 - 18.04 | GCC 4.2 - 7.3 |
| macOS 10.6 - 10.14, 15.2 - 15.3 | OS X Clang 2.9 - 10.0 |
| Windows 7, 8, 8.1, 10 | OS X GCC 4.2 |
|  | MSVC 2017 |

## Required Dependencies

Install these first:

1. [Boost](https://www.boost.org/users/download/) 1.43 or newer
2. [CMake](https://cmake.org/download/) 3.0 or newer

On macOS, support for Xcode 4.3 command line tools requires CMake 2.8.8 or newer.

## Optional Dependencies

### Intel TBB

If TBB is installed and detectable by CMake, GTSAM will use it automatically. Confirm that CMake prints `Use Intel TBB : Yes`.

- Disable it with `GTSAM_WITH_TBB=OFF`.
- On Ubuntu, install it from the package manager.
- On other platforms, see [oneTBB](https://github.com/uxlfoundation/oneTBB).

### Intel MKL

GTSAM can be configured to use MKL with `GTSAM_WITH_EIGEN_MKL` and `GTSAM_WITH_EIGEN_MKL_OPENMP`, but it does not always improve performance. Benchmark your workload before enabling it.

To use MKL on Linux, Intel provides installation guidance through its package repositories. If you are building the Python wrapper, you may also need:

```sh
source /opt/intel/mkl/bin/mklvars.sh intel64
export LD_PRELOAD="$LD_PRELOAD:/opt/intel/mkl/lib/intel64/libmkl_core.so:/opt/intel/mkl/lib/intel64/libmkl_sequential.so"
```

Then pass:

```sh
cmake -DGTSAM_WITH_EIGEN_MKL=ON ..
```

## Ubuntu Packages

Install the basic dependencies with:

```sh
sudo apt-get install libboost-all-dev
sudo apt-get install cmake
```

Optional packages:

- TBB: `sudo apt-get install libtbb-dev`
- MKL: install through Intel's APT repositories if needed

## Ubuntu PPA Packages

GTSAM is also available through the [BorgLab Launchpad PPAs](https://launchpad.net/~borglab).

Latest 4.x stable release:

```sh
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

Nightly builds:

```sh
sudo add-apt-repository ppa:borglab/gtsam-develop
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

## Arch Linux

GTSAM is also available in the [AUR](https://aur.archlinux.org/packages/gtsam/).

```sh
yay -S gtsam
```

For Intel-accelerated builds:

```sh
yay -S gtsam-mkl
```

## Running Tests

`make check` builds and runs all tests. Tests are only built for the `check` targets so that `make install` does not build them unnecessarily.

Examples:

- Run all tests: `make check`
- Run one module: `make check.geometry`
- Run one test: `make testMatrix.run`
- Build timing targets: `make timing`

## Important CMake Options

### `CMAKE_BUILD_TYPE`

```sh
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

Supported values:

- `Debug`: full error checking, no optimization
- `Release`: optimized, no debug symbols
- `Timing`: enables timing statistics
- `Profiling`: intended for profiling runs
- `RelWithDebInfo`: release build with debug symbols

### `CMAKE_INSTALL_PREFIX`

Set the install location:

```sh
cmake -DCMAKE_INSTALL_PREFIX:PATH=$HOME ..
```

### `GTSAM_TOOLBOX_INSTALL_PATH`

Set the MATLAB toolbox install path:

```sh
cmake -DGTSAM_TOOLBOX_INSTALL_PATH:PATH=$HOME/toolbox ..
```

### `GTSAM_BUILD_CONVENIENCE_LIBRARIES`

```sh
cmake -DGTSAM_BUILD_CONVENIENCE_LIBRARIES:OPTION=ON ..
```

- `ON` (default): faster for developers iterating on tests
- `OFF`: avoids rebuilding the full library twice, better for most users

### `GTSAM_BUILD_UNSTABLE`

```sh
cmake -DGTSAM_BUILD_UNSTABLE:OPTION=ON ..
```

- `ON` (default): builds and installs `libgtsam_unstable`
- `OFF`: excludes unstable code from build and install

### `MEX_COMMAND`

Path to the MATLAB `mex` compiler. If `mex` is not already in `PATH`, point it at `$MATLABROOT/bin/mex`.

## Debugging Tips

GTSAM makes extensive use of debug assertions, so development work should usually happen in `Debug` mode. Switch back to `Release` when benchmarking or running finished code.

Another useful option is `_GLIBCXX_DEBUG`, which enables additional standard library checks. If you use it to compile GTSAM, anything linking against GTSAM must also use it.

## Performance Tips

1. Use `Release` mode for production workloads.
2. Enable TBB on multi-core systems and benchmark with and without it.
3. Consider `-march=native` in `GTSAM_CMAKE_CXX_FLAGS` if portability is not a concern.
4. Only enable MKL if you have measured a real benefit.

## API Documentation

For API documentation, see the [generated Doxygen site](/doxygen/).
