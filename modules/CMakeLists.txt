# Modules land here as they are ported, one directory per module, mirroring
# fw-rust's crates/ layout (module names deliberately parallel: ap-math here
# is the same upstream scope as ap-math in ports/plane-fw-rust/crates/).

add_subdirectory(fwcpp-result)
add_subdirectory(ap-internal-error)
add_subdirectory(ap-math)
add_subdirectory(ap-common)
add_subdirectory(ap-filter)
add_subdirectory(ap-pid)
add_subdirectory(ap-nav)
add_subdirectory(ap-param)
add_subdirectory(ap-scheduler)
add_subdirectory(ap-hal)
add_subdirectory(ap-rc-channel)
