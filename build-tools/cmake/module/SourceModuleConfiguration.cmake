
#
# SourceModuleConfiguration.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

include(Assert)
include(DirectoryConfiguration)

function(project_name_from_directory PROJECT_NAME)
	if("${PROJECT_NAME}" MATCHES "None")
		get_filename_component(MODULE_TOP_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
		get_filename_component(DEFAULT_PROJECT_NAME ${MODULE_TOP_DIRECTORY} NAME_WE)

		set(NEW_PROJECT_NAME ${DEFAULT_PROJECT_NAME} CACHE STRING "Project name" FORCE)

		buildtool_message("Update project name ${NEW_PROJECT_NAME}")

		set(PROJECT_NAME ${NEW_PROJECT_NAME} PARENT_SCOPE)

	endif()
endfunction(project_name_from_directory)

function(get_executable_output_configuration OUTPUT_CONFIG_PATH)
	set(DEFAULT_EXECUTABLE_OUTPUT_PATH_CONFIG_PATH ../../data/build-config/executable_output_config.txt)

	set(EXECUTABLE_OUTPUT_PATH_CONFIG_PATH ${DEFAULT_EXECUTABLE_OUTPUT_PATH_CONFIG_PATH} CACHE PATH "Executable output configuration file path seen by this source module" FORCE)
	set(EXECUTABLE_OUTPUT_PATH_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${EXECUTABLE_OUTPUT_PATH_CONFIG_PATH})

	set(${OUTPUT_CONFIG_PATH} ${EXECUTABLE_OUTPUT_PATH_CONFIG_PATH} PARENT_SCOPE)
endfunction(get_executable_output_configuration)

function(get_library_output_configuration OUTPUT_CONFIG_PATH)
	set(DEFAULT_LIBRARY_OUTPUT_PATH_CONFIG_PATH ../../data/build-config/library_output_config.txt)

	set(LIBRARY_OUTPUT_PATH_CONFIG_PATH ${DEFAULT_LIBRARY_OUTPUT_PATH_CONFIG_PATH} CACHE PATH "Library output configuration file path seen by this source module" FORCE)
	set(LIBRARY_OUTPUT_PATH_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${LIBRARY_OUTPUT_PATH_CONFIG_PATH})

	set(${OUTPUT_CONFIG_PATH} ${LIBRARY_OUTPUT_PATH_CONFIG_PATH} PARENT_SCOPE)
endfunction(get_library_output_configuration)

function(read_once_line_from_text_file RESULT_FILE_COMPONENT CONFIG_FILE_PATH)
	assert_exist_path(${CONFIG_FILE_PATH})

	file(SIZE ${CONFIG_FILE_PATH} FILE_SIZE)
	file(READ ${CONFIG_FILE_PATH} RESULT_RAW_COMPONENT)

	math(EXPR COMPONENT_SIZE "${FILE_SIZE} - 1")
	string(SUBSTRING "${RESULT_RAW_COMPONENT}" 0 ${COMPONENT_SIZE} RESULT_CONVERTED_COMPONENT)

	buildtool_message("Read once line result is ${RESULT_CONVERTED_COMPONENT}")

	set(${RESULT_FILE_COMPONENT} ${RESULT_CONVERTED_COMPONENT} PARENT_SCOPE)
endfunction(read_once_line_from_text_file)

function(get_output_path_from_config OUTPUT_PATH OUTPUT_CONFIGURATION_PATH)
	read_once_line_from_text_file(OUTPUT_BASE_PATH ${OUTPUT_CONFIGURATION_PATH})
	get_filename_component(OUTPUT_FULL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${OUTPUT_BASE_PATH} ABSOLUTE)

	set(${OUTPUT_PATH} ${OUTPUT_FULL_PATH} PARENT_SCOPE)
endfunction(get_output_path_from_config)

function(output_direcoty_preparation OUTPUT_DIRECTORY_MOTHER_PATH OUTPUT_DIRECTORY_PATH OUTPUT_CONFIGURATION_PATH MODULE_NAME)
	get_output_path_from_config(OUTPUT_FULL_PATH ${OUTPUT_CONFIGURATION_PATH})
	buildtool_message("Output mother path ${OUTPUT_FULL_PATH}")

	set(OUTPUT_DIRECTORY_PATH_FOR_MODULE ${OUTPUT_FULL_PATH}/${MODULE_NAME})
	buildtool_message("Output path ${OUTPUT_DIRECTORY_PATH_FOR_MODULE}")

	create_directory(${OUTPUT_DIRECTORY_PATH_FOR_MODULE})

	set(${OUTPUT_DIRECTORY_MOTHER_PATH} ${OUTPUT_FULL_PATH} PARENT_SCOPE)
	set(${OUTPUT_DIRECTORY_PATH} ${OUTPUT_DIRECTORY_PATH_FOR_MODULE} PARENT_SCOPE)
endfunction(output_direcoty_preparation)

function(executable_output_directory EXECUTABLE_OUTPUT_MOTHER_DIRECTORY_PATH EXECUTABLE_OUTPUT_DIRECTORY_PATH MODULE_NAME)
	get_executable_output_configuration(EXECUTABLE_OUTPUT_PATH_CONFIG_PATH)
	output_direcoty_preparation(EXECUTABLE_OUTPUT_MOTHER_PATH EXECUTABLE_OUTPUT_MODULE_PATH ${EXECUTABLE_OUTPUT_PATH_CONFIG_PATH} ${MODULE_NAME})

	set(${EXECUTABLE_OUTPUT_MOTHER_DIRECTORY_PATH} ${EXECUTABLE_OUTPUT_MOTHER_PATH} PARENT_SCOPE)
	set(${EXECUTABLE_OUTPUT_DIRECTORY_PATH} ${EXECUTABLE_OUTPUT_MODULE_PATH} PARENT_SCOPE)
endfunction(executable_output_directory)

function(library_output_directory LIBRARY_OUTPUT_MOTHER_DIRECTORY_PATH LIBRARY_OUTPUT_DIRECTORY_PATH MODULE_NAME)
	get_library_output_configuration(LIBRARY_OUTPUT_PATH_CONFIG_PATH)
	output_direcoty_preparation(LIBRARY_OUTPUT_MOTHER_PATH LIBRARY_OUTPUT_MODULE_PATH ${LIBRARY_OUTPUT_PATH_CONFIG_PATH} ${MODULE_NAME})

	set(${LIBRARY_OUTPUT_MOTHER_DIRECTORY_PATH} ${LIBRARY_OUTPUT_MOTHER_PATH} PARENT_SCOPE)
	set(${LIBRARY_OUTPUT_DIRECTORY_PATH} ${LIBRARY_OUTPUT_MODULE_PATH} PARENT_SCOPE)
endfunction(library_output_directory)

