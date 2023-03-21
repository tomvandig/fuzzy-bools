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
target_include_directories(fuzzy-bools PUBLIC ./deps/glm/ ./deps/cdt/)

file (GLOB TestSourceFiles test/*)
set (TestFiles ${TestSourceFiles})
add_executable (fuzzy-bools-test ${TestFiles})
source_group ("Tests" FILES ${TestSourceFiles})
add_test (fuzzy-bools-test fuzzy-bools-test)
