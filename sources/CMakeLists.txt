#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)



# And now we add any targets that we want
add_executable(CupCollector main.cpp Image.cpp PPMLoader.cpp CupCollector.cpp point.cpp ${HEADER_FILES})



