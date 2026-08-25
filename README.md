# plane-fw-cpp

ArduPlane fixed-wing, ported to modern C++. Pinned to `Plane-4.7.0`, same tag as
[`plane-fw-rust`](../plane-fw-rust). See the `fw-cpp` effort in the tracker
(`../../tracker/efforts/fw-cpp.md`) for the charter, and `ADR-0012` for the
conventions this port follows.

This is a second, independent implementation, not a fork of `plane-fw-rust` and not a
translation of it. It reads only from the shared, read-only `upstream/plane-4.7.0`
worktree and does not depend on anything in `ports/plane-fw-rust`.

## Building

```
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build
```

## Layout

```
modules/    one directory per ported module, mirrors plane-fw-rust's crates/
tests/      parity and unit tests, one target per module under test
```

## Status

Check `python3 ../../tracker/tracker.py status --effort fw-cpp -v` from the tracker
directory, or see `../../tracker/STATUS.md`.
