cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-7")
set(CMAKE_C_COMPILER "/usr/bin/gcc-7")
project(test LANGUAGES CXX VERSION 0.1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")
find_package(Boost REQUIRED COMPONENTS
    atomic
    chrono
    date_time
    regex
    system
    thread
    random
    )
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED) 
# find_package(DLIB REQUIRED)
add_executable(main main.cpp)

target_compile_options(main
PRIVATE -lbase64
)
target_link_libraries(main 
base64
curl
crypto
jsoncpp
ssl
Boost::atomic
Boost::chrono
Boost::date_time
Boost::regex
Boost::system
Boost::thread
Boost::random
)