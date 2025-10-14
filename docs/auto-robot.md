# Developing Robot Programs with Arduino, Neovim, and PlatformIO

## Prerequisites

Before you start, make sure you have the following installed:

- **Neovim** (v0.9+ recommended)
- **PlatformIO CLI**
- **Git**
- **Python 3**
- **C/C++ toolchain** (depends on your OS)

You can install PlatformIO globally using pip:

```bash
pip install platformio
```

______________________________________________________________________

## Neovim Setup

I use a custom Neovim configuration optimized for embedded development and C/C++ projects.

**Config repository:**\
[gfm-neovim-config](https://github.com/Chayanon-Ninyawee/gfm-neovim-config)

This configuration includes:

- **LSP (clangd)** for C/C++ autocompletion and diagnostics
- **Telescope** for fuzzy finding
- **Treesitter** for syntax highlighting
- **Mason + nvim-lspconfig** for managing language servers
- **Git integration** via `gitsigns.nvim`
- **Formatter/Linter support** via `null-ls.nvim`

To use it, clone the repo into your Neovim config directory:

```bash
git clone https://github.com/Chayanon-Ninyawee/gfm-neovim-config ~/.config/nvim
```

Then run Neovim once to install all plugins.

______________________________________________________________________

## Project Structure

A typical robot project using Arduino and PlatformIO might look like this (Not finalize yet):

```
robot-project/
├── include/
│   └── robot_config.h
├── lib/
│   └── motor_driver/
│       └── motor_driver.cpp
├── src/
│   └── main.cpp
├── platformio.ini
└── README.md
```

### Example `platformio.ini`

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
monitor_speed = 115200
```

______________________________________________________________________

## Building and Uploading with PlatformIO CLI

### Initialize a new project

```bash
pio project init --board uno
```

### Build your code

```bash
pio run
```

### Upload to your Arduino board

```bash
pio run --target upload
```

### Open Serial Monitor

```bash
pio device monitor
```

______________________________________________________________________

## Development Workflow

1. **Build & upload** using PlatformIO

   - Run `pio run` to build.
   - Run `pio run --target upload` to flash your board.

1. **Monitor serial output**\
   Use `pio device monitor` to debug sensor readings or logs.

______________________________________________________________________

## Useful PlatformIO Commands

| Command | Description |
|----------|-------------|
| `pio project init` | Create a new PlatformIO project |
| `pio run` | Build the project |
| `pio run -t upload` | Upload to board |
| `pio device list` | Show available serial ports |
| `pio device monitor` | Open serial monitor |
| `pio check` | Run static code analysis |

______________________________________________________________________

## Integrating with Git

Create a `.gitignore` file:

```
.pio
.vscode
.gitignore
```

______________________________________________________________________

**Author:** [Chayanon Ninyawee](https://github.com/Chayanon-Ninyawee)\
**Neovim Config:** [gfm-neovim-config](https://github.com/Chayanon-Ninyawee/gfm-neovim-config)
