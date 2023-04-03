cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# Above line indicates to Cmake that minimum version 2.8 is required.
# As you may have noted all lines beginning with hash symbol are treated as comments by Cmake.
project(k-robust-ccbs)
# In the above line we declared a variable sampleName and assigned it a value MyApp. We will make use of the same later.project(PrjMyApp)
# here we define the project or solution (for visual studio) name
# In following lines we try to find out the packages of additional libraries, if reqd. Here we are trying to locate PCL and mrpt library.
# u may replace the names by yours. Each such package should have a Cmake file in them, if not then we may have to manually define the paths.
# we Show that later.
find_package(Boost COMPONENTS multi_index)
include_directories( ${Boost_INCLUDE_DIRS} )
include_directories( "." )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -Wall -Wextra" )
set( CMAKE_EXE_LINKER_FLAGS "-static -static-libgcc -static-libstdc++" )

#set only the cpp files
set( SOURCE_FILES
    map.cpp
    tinyxml2.cpp
    xml_logger.cpp
    main.cpp
    config.cpp
    task.cpp
    heuristic.cpp
    sipp.cpp
    cbs.cpp
    reservation_table.cpp)

#set only the header files
set( HEADER_FILES
    map.h
    tinyxml2.h
    xml_logger.h
    structs.h
    const.h
    config.h
    task.h
    heuristic.h
    sipp.h
    cbs.h
    reservation_table.h)
# End of CMakeLists.Txt file. Save it as a plain text file and the name should be exactly CMakeLists.Txt
add_executable( k-robust-ccbs ${SOURCE_FILES} ${HEADER_FILES} )