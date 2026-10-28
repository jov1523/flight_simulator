cmake_minimum_required(VERSION 3.20)
project(FlightSimulator)

set(CMAKE_CXX_STANDARD 17)

# ── Headers ──────────────────────────────────────────────────────────────
# Look for GL/ subfolder (glew.h, freeglut.h) in the project root
include_directories(${CMAKE_SOURCE_DIR})

# ── Libraries ────────────────────────────────────────────────────────────
# Look for .a / .dll.a import libraries in the project root
link_directories(${CMAKE_SOURCE_DIR})

# ── Executable ───────────────────────────────────────────────────────────
add_executable(FlightSimulator main.cpp)

# Link order matters for MinGW: specific libs first, system libs last
# libfreeglut.a  → FreeGLUT dynamic import lib  (requires libfreeglut.dll at runtime)
# libglew32.dll.a → GLEW dynamic import lib     (requires glew32.dll at runtime)
# opengl32, glu32, winmm → Windows system OpenGL libraries
target_link_libraries(FlightSimulator
    freeglut
    glew32.dll
    opengl32
    glu32
    winmm
    gdi32
)

# ── Copy DLLs to build output ────────────────────────────────────────────
add_custom_command(TARGET FlightSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/libfreeglut.dll"
        $<TARGET_FILE_DIR:FlightSimulator>
)

add_custom_command(TARGET FlightSimulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/glew32.dll"
        $<TARGET_FILE_DIR:FlightSimulator>
)
