# Apartment Navigation

A Gazebo Harmonic simulation of a robot navigating an apartment, built as a Gazebo Sim plugin.

## Dependencies

Install via Homebrew:

```bash
brew install gz-harmonic   # pulls in gz-sim8, gz-plugin2, gz-cmake3, and the rest of the stack
brew install cmake
brew install doxygen
brew install cppcheck
brew install pre-commit
```

> `gz-harmonic` is the meta-package — it installs the full Gazebo Harmonic suite as a unit.

## Build

```bash
cmake -B build
cmake --build build
```

**Available build options** (pass with `-D<OPTION>=ON`):

| Option | Default | Description |
|---|---|---|
| `ENABLE_UBSAN` | `ON` | UndefinedBehaviorSanitizer — catches UB at runtime |
| `ENABLE_ASAN` | `OFF` | AddressSanitizer — catches memory errors (2× memory overhead) |
| `ENABLE_TSAN` | `OFF` | ThreadSanitizer — catches data races (5–15× overhead) |
| `ENABLE_CLANG_TIDY` | `OFF` | Run clang-tidy static analysis on every file |
| `ENABLE_CPPCHECK` | `OFF` | Run cppcheck static analysis on every file |

> ASan and TSan are mutually exclusive — enabling both will abort the configure step.

Example — full debug build with address sanitizer and static analysis:

```bash
cmake -B build -DENABLE_ASAN=ON -DENABLE_CLANG_TIDY=ON
cmake --build build
```

## Documentation

Generate API docs with Doxygen:

```bash
doxygen
open docs/doxygen/html/index.html
```

Documentation coverage is enforced — Doxygen exits non-zero if any public symbol is undocumented.

## Run

```bash
gz sim world/myworld.sdf
```

Make sure the build output is on the plugin path:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
gz sim world/myworld.sdf
```
