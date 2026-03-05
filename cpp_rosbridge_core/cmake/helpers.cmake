# Helper functions for action auto-discovery CMake script

# Constants
set(EMPTY_REGISTRATION_TEMPLATE
    "#include \"cpp_rosbridge_core/action_bridge.hpp\"\n// No actions auto-discovered\nnamespace cpp_rosbridge_core {\nvoid register_generated_actions(action::ActionBridgeRegistry&) {}\n}\n")
set(YAML_NAME_REGEX "name: ([A-Za-z0-9_]+)")
set(SNAKE_CASE_REGEX1 "(.)([A-Z][a-z]+)")
set(SNAKE_CASE_REGEX2 "([a-z0-9])([A-Z])")

# Create empty registration file with given reason
function(create_empty_registration reason)
    set(output_file "${CMAKE_CURRENT_BINARY_DIR}/generated/action_registration.cpp")
    file(WRITE "${output_file}" "// ${reason}\n${EMPTY_REGISTRATION_TEMPLATE}")
    message(STATUS "Action auto-discovery: ${reason}")
endfunction()

# Find Python3 executable
function(find_python_executable out_var)
    find_program(PYTHON_EXE python3)
    if(NOT PYTHON_EXE)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()
    set(${out_var} "${PYTHON_EXE}" PARENT_SCOPE)
endfunction()

# Discover workspace interface packages that contain action definitions.
# Returns list entries in the form "<package_name>|<absolute_action_dir>".
function(discover_interface_action_packages out_var)
    get_filename_component(ws_src_dir "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
    file(GLOB package_xmls "${ws_src_dir}/*/package.xml")

    set(pkg_specs "")
    foreach(pkg_xml ${package_xmls})
        get_filename_component(pkg_dir "${pkg_xml}" DIRECTORY)
        set(action_dir "${pkg_dir}/action")
        if(NOT EXISTS "${action_dir}")
            continue()
        endif()

        file(GLOB action_files "${action_dir}/*.action")
        if(NOT action_files)
            continue()
        endif()

        file(READ "${pkg_xml}" pkg_xml_content)
        string(REGEX MATCH "<name>[ \t\r\n]*([^< \t\r\n]+)[ \t\r\n]*</name>" name_match "${pkg_xml_content}")
        if(NOT name_match)
            continue()
        endif()
        set(pkg_name "${CMAKE_MATCH_1}")
        list(APPEND pkg_specs "${pkg_name}|${action_dir}")
    endforeach()

    list(SORT pkg_specs)
    set(${out_var} "${pkg_specs}" PARENT_SCOPE)
endfunction()

# Discover action specs from package specs.
# package_specs: "<pkg>|<action_dir>"
# out_var: entries "<pkg>|<action_name>"
function(discover_action_specs package_specs out_var)
    set(specs "")
    foreach(pkg_spec ${package_specs})
        string(REPLACE "|" ";" parts "${pkg_spec}")
        list(GET parts 0 pkg_name)
        list(GET parts 1 action_dir)

        file(GLOB action_files "${action_dir}/*.action")
        foreach(file ${action_files})
            get_filename_component(action_name "${file}" NAME_WE)
            list(APPEND specs "${pkg_name}|${action_name}")
        endforeach()
    endforeach()
    list(SORT specs)
    set(${out_var} "${specs}" PARENT_SCOPE)
endfunction()

# Read specialized actions from YAML config
function(read_specialized_actions config_file out_var)
    if(NOT EXISTS "${config_file}")
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    file(STRINGS "${config_file}" lines)
    set(specialized "")
    foreach(line ${lines})
        # Skip lines that start with # (comments)
        string(REGEX MATCH "^[ \t]*#" is_comment "${line}")
        if(NOT is_comment)
            # Check if line contains "name: <ActionName>"
            string(REGEX MATCH "${YAML_NAME_REGEX}" match "${line}")
            if(match)
                string(REGEX REPLACE "name: " "" name "${match}")
                list(APPEND specialized ${name})
            endif()
        endif()
    endforeach()
    set(${out_var} "${specialized}" PARENT_SCOPE)
endfunction()

# Filter out specialized actions
function(filter_generic_actions all_actions specialized_actions out_var)
    set(generic "")
    foreach(action ${all_actions})
        list(FIND specialized_actions ${action} idx)
        if(idx EQUAL -1)
            list(APPEND generic ${action})
        endif()
    endforeach()
    set(${out_var} "${generic}" PARENT_SCOPE)
endfunction()

# Convert CamelCase to snake_case
function(to_snake_case input out_var)
    string(REGEX REPLACE "${SNAKE_CASE_REGEX1}" "\\1_\\2" result "${input}")
    string(REGEX REPLACE "${SNAKE_CASE_REGEX2}" "\\1_\\2" result "${result}")
    string(TOLOWER "${result}" result)
    set(${out_var} "${result}" PARENT_SCOPE)
endfunction()

# Generate C++ includes for action types
function(generate_includes action_specs out_var)
    set(includes "")
    foreach(spec ${action_specs})
        string(REPLACE "|" ";" parts "${spec}")
        list(GET parts 0 pkg_name)
        list(GET parts 1 action_name)
        to_snake_case("${action_name}" header_name)
        set(includes "${includes}#include <${pkg_name}/action/${header_name}.hpp>\n")
    endforeach()
    set(${out_var} "${includes}" PARENT_SCOPE)
endfunction()

# Generate C++ registration calls
function(generate_registrations action_specs out_var)
    set(registrations "")
    foreach(spec ${action_specs})
        string(REPLACE "|" ";" parts "${spec}")
        list(GET parts 0 pkg_name)
        list(GET parts 1 action_name)
        set(registrations "${registrations}  action::register_generic<${pkg_name}::action::${action_name}>(\n")
        set(registrations "${registrations}      registry, \"${pkg_name}/action/${action_name}\");\n")
    endforeach()
    set(${out_var} "${registrations}" PARENT_SCOPE)
endfunction()
