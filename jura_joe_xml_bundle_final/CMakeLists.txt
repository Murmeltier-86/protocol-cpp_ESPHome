
cmake_minimum_required(VERSION 3.10)
project(jura_proto_example C)
set(CMAKE_C_STANDARD 99)
add_executable(query_example examples/query_example.c C/jura_proto.c)
