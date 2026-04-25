# Apartment Navigation (MacOS)

A Gazebo Harmonic simulation of a robot navigating an apartment, built as a Gazebo Sim plugin.

## Dependencies

```zsh
brew tap osrf/simulation
brew install gz-harmonic cmake doxygen cppcheck pre-commit
```

## Build

```zsh
cmake -B build
cmake --build build
```

**Optional build flags** (pass with `-D<FLAG>=ON`):

| Flag | Default | Description |
|---|---|---|
| `ENABLE_UBSAN` | `ON` | UndefinedBehaviorSanitizer |
| `ENABLE_ASAN` | `OFF` | AddressSanitizer — mutually exclusive with TSan |
| `ENABLE_TSAN` | `OFF` | ThreadSanitizer — mutually exclusive with ASan |
| `ENABLE_CLANG_TIDY` | `OFF` | clang-tidy static analysis |
| `ENABLE_CPPCHECK` | `OFF` | cppcheck static analysis |

## Run

On macOS, the server and GUI must be launched in separate terminals.

**Terminal 1 — server:**

```zsh
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
export GZ_SIM_RESOURCE_PATH=$PWD/model

gz sim -s world/myworld.sdf
```

**Terminal 2 — GUI:**

```zsh
export OPENSSL_ROOT_DIR=/opt/homebrew/opt/openssl@3
export CFLAGS=-I/opt/homebrew/include
export LDFLAGS=-L/opt/homebrew/lib
export CMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5

gz sim -g
```

## Development Setup

Install the pre-commit hooks so linting and formatting run automatically before every commit:

```zsh
pre-commit install
```

If you use VS Code, the following tasks are available out of the box (`⌘⇧B` to run):

| Task | Description |
|---|---|
| **Build (MacOS)** | Default build with UBSan off |
| **Build (ASan)** | Build with AddressSanitizer |
| **Build (TSan)** | Build with ThreadSanitizer |
| **Analyse** | Build with clang-tidy and cppcheck enabled |
| **Format Check** | Dry-run clang-format across all sources |
| **Docs** | Generate Doxygen docs and open in browser |

## Documentation

```zsh
doxygen
open docs/doxygen/html/index.html
```

Or browse the hosted docs at [bkekule.github.io/apartment-navigation](https://bkekule.github.io/apartment-navigation/).
