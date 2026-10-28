cmake_minimum_required(VERSION 3.16)
project(ishitva_waste_sorter_hmi VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Werror -O2)

include_directories(include)

# Core CLI Simulator & Unit Test Binary (Zero external GUI dependencies for headless CI)
add_executable(waste_sorter_cli src/ishitva_suka_waste_sorter.cpp)
find_package(Threads REQUIRED)
target_link_libraries(waste_sorter_cli PRIVATE Threads::Threads)

# Enable CTest unit testing for GitLab CI / GitHub Actions
enable_testing()
add_test(NAME SortingControllerUnitTests COMMAND waste_sorter_cli --test)

# Optional Qt5 / Qt6 Industrial HMI GUI Build (enabled via -DBUILD_QT_HMI=ON)
option(BUILD_QT_HMI "Build the Qt Widgets Industrial HMI Dashboard" OFF)
if(BUILD_QT_HMI)
    set(CMAKE_AUTOMOC ON)
    find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
    add_executable(waste_sorter_hmi
        src/qt_hmi/hmi_mainwindow.cpp
        include/sorting_engine.hpp
    )
    target_link_libraries(waste_sorter_hmi PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Threads::Threads)
endif()
