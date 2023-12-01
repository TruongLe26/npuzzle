cmake_minimum_required(VERSION 3.27)
project(npuzzle)

set(CMAKE_CXX_STANDARD 17)

add_executable(npuzzle main.cpp
        Node.h
        AStarSearch.h
        FocalSearch.h)
