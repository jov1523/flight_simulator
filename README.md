# FlightSimulator

A Cessna 172S flight simulator written in C++17 with OpenGL (GLEW + FreeGLUT).
It models a NACA 2412 wing and Lycoming IO-360-L2A engine, renders a runway,
terrain, sky and windsock, and draws a glass-cockpit PFD (attitude indicator,
airspeed/altitude tapes, VSI, heading bar, slip/skid ball) plus an instructor
panel for weather and failure scenarios.

## Requirements

- CMake 3.20 or newer
- A C++17 compiler — MinGW-w64 (g++) for the bundled MinGW import libraries
- Windows (the build links `opengl32`, `glu32`, `winmm`, `gdi32`)

GLEW 2.2.0 and FreeGLUT are vendored: headers in `GL/`, import libraries in the
project root. The two runtime DLLs (`glew32.dll`, `libfreeglut.dll`) are copied
next to the executable automatically after each build.

## Build and run

```sh
cmake -S . -B build
cmake --build build
./build/FlightSimulator.exe
```

## Controls

| Keys        | Action                             |
| ----------- | ---------------------------------- |
| `W` / `S`   | Pitch down / up                    |
| `A` / `D`   | Roll left / right                  |
| `Q` / `E`   | Rudder left / right                |
| `+` / `-`   | Throttle up / down                 |
| `[` / `]`   | Flaps up / down                    |
| `T` `Y` `U` | Autopilot / heading / altitude     |
| `5` `6` `7` | Training scenarios                 |
| `1` / `2`   | Wind speed down / up               |
| `3` / `4`   | Toggle engine failure / turbulence |
| `B`         | Toggle brakes                      |
| `V`         | Toggle chase / cockpit camera      |
| `H`         | Toggle HUD                         |
| `I`         | Toggle instructor panel            |
| `O`         | Toggle fog                         |
| `P`         | Pause                              |
| `R`         | Reset aircraft                     |
| `Esc`       | Exit                               |
