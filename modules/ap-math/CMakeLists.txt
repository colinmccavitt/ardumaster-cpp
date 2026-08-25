add_library(ap-math STATIC
    src/scalar.cpp
    src/matrix_alg.cpp
)
target_include_directories(ap-math PUBLIC include)
target_compile_features(ap-math PUBLIC cxx_std_20)

# See scalar.hpp's file banner: only this module's own .cpp sources get the
# upstream-matching, literal-affecting flags (PRIVATE - not propagated to
# consumers, whose own literals must keep ordinary C++ typing).
target_link_libraries(ap-math PRIVATE fwcpp_upstream_flags)
