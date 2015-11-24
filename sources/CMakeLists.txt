#
# Test CMake version
#
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)



# And now we add any targets that we want
add_executable(main.cpp Image.cpp PPMLoader.cpp ${HEADER_FILES})



