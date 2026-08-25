add_library(ap-math STATIC
    src/scalar.cpp
    src/matrix_alg.cpp
    src/vector2.cpp
    src/vector3.cpp
    src/quaternion.cpp
)
target_include_directories(ap-math PUBLIC include)
target_compile_features(ap-math PUBLIC cxx_std_20)

# ap-internal-error is PUBLIC: scalar.hpp (constrain_value) and
# quaternion.hpp (normalize) both reference fwcpp::InternalError in their
# public interfaces, so consumers need its include path too.
target_link_libraries(ap-math PUBLIC ap-internal-error)

# See scalar.hpp's file banner: only this module's own .cpp sources get the
# upstream-matching, literal-affecting flags (PRIVATE - not propagated to
# consumers, whose own literals must keep ordinary C++ typing).
target_link_libraries(ap-math PRIVATE fwcpp_upstream_flags)
