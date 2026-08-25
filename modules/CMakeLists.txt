# Modules land here as they are ported, one directory per module, mirroring
# fw-rust's crates/ layout (module names deliberately parallel: ap-math here
# is the same upstream scope as ap-math in ports/plane-fw-rust/crates/).
#
# add_subdirectory(ap-common)
# add_subdirectory(ap-math)
# ...
#
# Empty until CPP-002 lands the first module. An empty add_subdirectory tree
# is intentional, not a placeholder to fill blindly - see ADR-0007 and the
# project's no-stubbing rule: nothing here compiles a stub, it simply isn't
# started yet.
