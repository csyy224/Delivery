cmake_minimum_required(VERSION 3.20)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Delivery)
add_executable(Delivery Delivery/src/main.cpp
						Delivery/src/DataReader.cpp
						Delivery/src/DeliveryScheduler.cpp
						Delivery/src/Graph.cpp
						Delivery/src/Task1.cpp
						Delivery/src/Task2.cpp
						Delivery/src/Task3.cpp
						Delivery/src/Task4.cpp
						Delivery/src/Task5.cpp
						Delivery/src/Task6.cpp
						Delivery/src/TSP.cpp
						Delivery/src/Visualizer.cpp)
target_include_directories(Delivery PUBLIC Delivery/include)