cmake_minimum_required (VERSION 3.11)

project (fuzzy-bools)

enable_testing ()

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)
set_property (GLOBAL PROPERTY USE_FOLDERS ON)

file (GLOB CoreFiles src/*.h)
file (GLOB SourceFiles src/main.cpp)

set (
	AllFiles
	${CoreFiles}
	${SourceFiles}
)

source_group ("core" FILES ${CoreFiles})
source_group ("sources" FILES ${SourceFiles})

add_executable (fuzzy-bools ${AllFiles})
target_include_directories(fuzzy-bools PUBLIC ./deps/glm/glm/ ./deps/cdt/)
set_target_properties(fuzzy-bools PROPERTIES LINK_FLAGS "-O3 --bind -flto --define-macro=REAL_T_IS_DOUBLE -s ALLOW_MEMORY_GROWTH=1 -s MAXIMUM_MEMORY=4GB -sSTACK_SIZE=5MB -s EXPORT_NAME=fuzzybools -s EXPORT_ES6=1 -s MODULARIZE=1")

file (GLOB TestSourceFiles test/*)
set (TestFiles ${TestSourceFiles})
add_executable (fuzzy-bools-test ${TestFiles})
source_group ("Tests" FILES ${TestSourceFiles})
add_test (fuzzy-bools-test fuzzy-bools-test)
