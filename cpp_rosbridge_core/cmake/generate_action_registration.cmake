# CMake script to auto-discover actions from interface packages
# and generate C++ registration code for generic actions.
#
# This script:
# 1. Finds all .action files in workspace interface packages
# 2. Reads specialized_actions.yaml to exclude specialized actions
# 3. Generates C++ code to register all non-specialized actions generically

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/helpers.cmake)

# Find Python3 executable
find_python_executable(python_exe)
if(NOT python_exe)
    create_empty_registration("Python3 not found")
    return()
endif()

# Find interface packages that have action definitions
discover_interface_action_packages(package_specs)
if(NOT package_specs)
    create_empty_registration("No interface packages with actions found")
    set(AUTO_ACTION_INTERFACE_PACKAGES "")
    return()
endif()

set(interface_packages "")
foreach(pkg_spec ${package_specs})
    string(REPLACE "|" ";" parts "${pkg_spec}")
    list(GET parts 0 pkg_name)
    list(APPEND interface_packages "${pkg_name}")
endforeach()
list(REMOVE_DUPLICATES interface_packages)
list(SORT interface_packages)
set(AUTO_ACTION_INTERFACE_PACKAGES "${interface_packages}")

# Discover action specs ("<pkg>|<action>")
discover_action_specs("${package_specs}" all_action_specs)
if(NOT all_action_specs)
    create_empty_registration("No .action files found")
    set(AUTO_ACTION_INTERFACE_PACKAGES "")
    return()
endif()

message(STATUS "Discovered interface packages: ${AUTO_ACTION_INTERFACE_PACKAGES}")
message(STATUS "Discovered action specs: ${all_action_specs}")

# Read specialized actions from config
set(config_file "${CMAKE_CURRENT_SOURCE_DIR}/config/specialized_actions.yaml")
read_specialized_actions(${config_file} specialized_actions)
if(specialized_actions)
    message(STATUS "Specialized actions (excluded): ${specialized_actions}")
endif()

# Filter to get generic action specs only.
# specialized_actions.yaml keeps excluding by action type name.
set(generic_action_specs "")
foreach(spec ${all_action_specs})
    string(REPLACE "|" ";" parts "${spec}")
    list(GET parts 1 action_name)
    list(FIND specialized_actions "${action_name}" idx)
    if(idx EQUAL -1)
        list(APPEND generic_action_specs "${spec}")
    endif()
endforeach()

if(NOT generic_action_specs)
    create_empty_registration("All actions are specialized")
    set(AUTO_ACTION_INTERFACE_PACKAGES "")
    return()
endif()

message(STATUS "Action specs to auto-register: ${generic_action_specs}")

# Generate C++ code components
generate_includes("${generic_action_specs}" ACTION_INCLUDES)
generate_registrations("${generic_action_specs}" ACTION_REGISTRATIONS)

# Configure generated file
set(output_file "${CMAKE_CURRENT_BINARY_DIR}/generated/action_registration.cpp")
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/action_registration.cpp.in"
    "${output_file}"
    @ONLY
)

message(STATUS "Generated action registration code at: ${output_file}")
