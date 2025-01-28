# CMakeLists.txt for the muorb module

# This tells cmake to process the msg files under src/modules/muorb/msg/
set(MUORB_MSG_FILES
    msg/loadcell_data.msg
)

# Add custom PX4 topics
muorb_add_topic(loadcell_data)

# Include the necessary directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/msg
    ${PX4_SOURCE_DIR}/src/modules/muorb
    ${PX4_SOURCE_DIR}/src/modules/muorb/topics
)

# List of source files to compile for the muorb module
set(MUORB_SOURCES
    muorb.cpp
    # Add other source files if needed
)

# This adds the module to the PX4 build system
px4_add_module(
    MODULE muorb
    MAIN muorb
    SRCS ${MUORB_SOURCES}
    INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/msg
    DEPENDS uORB
)

# If you have any specific tests for the muorb module, add them here
if(PX4_TESTING)
    add_subdirectory(tests)
endif()
