#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 3.0.0)

# The name of the project. (EDIT THIS)
PROJECT(RIC_portfolio)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})


# Set executable output dir to build  dir
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#Wanted to add more error flags, but Image.cpp throw warnings with -pedantic and -Wunused-variable
SET( CMAKE_CXX_FLAGS  "	-std=c++11 		\
		 	-Wall 			\
			-Wextra 		\
			-Wshadow 		\
			-Wpointer-arith 	\
			-ffunction-sections 	\
			-fdata-sections		\
			-Wcast-qual		\
			-Werror			\
			-Wno-unused-variable 	\
			-pedantic		\
			-Wunused-variable	\
			-g			\
" )

set (HEADER_DIR "${ROOT}/headers")
set (HEADER_FILES ${HEADER_DIR}/Image.hpp)
set (HEADER_FILES ${HEADER_FILES} ${HEADER_DIR}/CupCollector.hpp)
set (HEADER_FILES ${HEADER_FILES} ${HEADER_DIR}/macros.hpp)
set (HEADER_FILES ${HEADER_FILES} ${HEADER_DIR}/Map.hpp)
set (HEADER_FILES ${HEADER_FILES} ${HEADER_DIR}/point.hpp)
set (HEADER_FILES ${HEADER_FILES} ${HEADER_DIR}/PPMLoader.hpp)
set (HEADER_FILES ${HEADER_FILES} ${HEADER_DIR}/vector2D.hpp)

include_directories(${HEADER_DIR})

add_subdirectory("sources")
