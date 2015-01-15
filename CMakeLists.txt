CMAKE_MINIMUM_REQUIRED (VERSION 2.8.11 FATAL_ERROR)
PROJECT (P4)

option(BUILD_QT "Build Project with Qt Gui." OFF)
option(USE_OPENMP "Build Project with OpenMP support." OFF)
option(PRETTY_OUT "Build with pretty out" OFF)

set(EXEC praktikum4)
set(CMAKE_VERBOSE_MAKEFILE on)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

set( CMAKE_BUILD_TYPE Release )

if(BUILD_QT)
	add_definitions(-DBUILD_QT)

	find_package(Qt5Widgets REQUIRED)
	find_package(Qt5Core REQUIRED)
	find_package(Qt5Gui REQUIRED)

	message("Qt5 Gui support activated.")
	
	include_directories(
	  ${Qt5Widgets_INCLUDE_DIRS} 
	  ${Qt5Core_INCLUDE_DIRS} 
	  ${Qt5Gui_INCLUDE_DIRS}
	)

	set(CMAKE_AUTOMOC ON)

	set(QT_UI_nbody
	include/gui/nbody.ui
	)

    QT5_WRAP_UI(UIS_HDRS
        ${QT_UI_nbody}
    )

	set(QT_SOURCES_MOC
	include/gui/SimulationGUI.h
    ${UIS_HDRS}
	)

	set(QT_SOURCES
	src/gui/SimulationGUI.cpp
	)

else(BUILD_QT)
	message("Qt Gui support NOT requested.")	
endif(BUILD_QT)

if(USE_OPENMP)
	FIND_PACKAGE(OpenMP REQUIRED)
	if(OPENMP_FOUND)
		message("OpenMP support activated.")
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
		set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
	endif(OPENMP_FOUND)
else(USE_OPENMP)
	message("OpenMP support NOT requested.")
endif(USE_OPENMP)

if(PRETTY_OUT)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPRETTY_OUT")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPRETTY_OUT")
endif(PRETTY_OUT)

add_executable(${EXEC}
    src/Praktikum4.cpp
    src/Simulation.cpp
    src/Config.cpp
    ${QT_SOURCES}
    ${QT_SOURCES_MOC}
)

if(UNIX)
	# on some plattforms needed for PTime to work
	target_link_libraries(${EXEC})
endif(UNIX)

if(BUILD_QT)
	qt5_use_modules(${EXEC} Core Gui Widgets)
endif(BUILD_QT)
