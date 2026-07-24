# Bundled simulator interfaces

This directory contains the smallest subset of `dv_interfaces` required by
LEM Simulator. The message definitions were selected from the supplied
`dv_interfaces.zip`; controller/debug-only messages are intentionally not
bundled.

When a workspace already provides `dv_interfaces`, CMake uses that package so
the rest of the Driverless stack keeps a single interface owner. Otherwise
LEM Simulator generates these compatible message headers as part of its own
build.
