#!/usr/bin/env python
import os
import sys

from SCons.Script import ARGUMENTS

target_path = ARGUMENTS.pop("target_path", "demo/addons/hexmap/lib")
target_name = ARGUMENTS.pop("target_name", "libgdhexmap")

env = SConscript("godot-cpp/SConstruct")

# we use designated initializers, and MSVC requires c++20 to support them
if env.get("is_msvc", False):
    env["CXXFLAGS"].remove("/std:c++17")
    env["CXXFLAGS"].append("/std:c++20")

env.Append(CPPPATH=["src/"])
sources = [
    Glob("src/*.cpp"),
    Glob("src/core/*.cpp"),
    Glob("src/core/editor/*.cpp"),
    Glob("src/tiled_node/*.cpp"),
    Glob("src/tiled_node/editor/*.cpp"),
    Glob("src/int_node/*.cpp"),
    Glob("src/int_node/editor/*.cpp"),
]

if env["platform"] == "macos":
    target = "{}/{}.{}.{}.framework/{}.{}.{}".format(
            target_path,
            target_name,
            env["platform"],
            env["target"],
            target_name,
            env["platform"],
            env["target"],
        )
else:
    target = "{}/{}{}{}".format(
            target_path,
            target_name,
            env["suffix"],
            env["SHLIBSUFFIX"],
    )

library = env.SharedLibrary(target=target, source=sources)
Default(library)

# ANNOYING
# * by default, godot-cpp scons makes shared library symbol visibility to hidden
# * by default linking allows for undefined symbols that just panic
#
# So when we built the dll, the symbols we're testing were hidden.  When we
# linked the binary, the linker couldn't find the symbols in the shared library,
# but instead of erroring, it redirects those to an __abort_with_payload() call.
# So the binary links, you execut it and it crashes.
#
# This is a crappy workaround, just recompile the sources and link them
# directly into the test binary.
godot_cpp = File(f"godot-cpp/bin/libgodot-cpp{env['suffix']}.a")
tests = env.Program(
    target='tests/tests',
    source=Glob("tests/*cpp") + sources,
    LIBS=[godot_cpp]
)

# import pdb
# pdb.set_trace()

run_tests = Command("run_tests", None, "tests/tests" )
Depends(run_tests, tests)
