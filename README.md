# ardumaster-cpp

ArduPilot vehicle firmware ported to modern C++ (Plane, Copter, and QuadPlane/VTOL),
pinned to `Plane-4.7.0` / matching Copter tag — same baseline as
[`ardumaster-rust`](../ardumaster-rust). See the `fw-cpp`, `copter-cpp`, and `vtol-cpp`
efforts in the tracker (`../../tracker/efforts/`) for charters, and `ADR-0012` for
conventions.

This is a second, independent implementation, not a fork of `ardumaster-rust` and not a
translation of it. It reads only from the shared, read-only `upstream/plane-4.7.0`
worktree and does not depend on anything in `ports/ardumaster-rust`.

## Building

```
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build
```

## Layout

```
modules/    one directory per ported module, mirrors ardumaster-rust's crates/
sitl/       Plane / Copter / QuadPlane SITL entrypoints
tests/      parity and unit tests, one target per module under test
```

## Status

Check `python3 ../../tracker/tracker.py status --effort fw-cpp -v` from the tracker
directory, or see `../../tracker/STATUS.md`.
