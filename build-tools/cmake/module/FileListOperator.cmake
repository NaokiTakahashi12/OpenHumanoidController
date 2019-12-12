#
# FileListOperator.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

function(CreateLibrarySourceFromSourceList library_files source_files main_file main_filename)
	foreach(source_file ${source_files})
		if(NOT "${source_file}" MATCHES "${main_filename}")
			if("${source_file}" MATCHES ".cpp" OR "${source_file}" MATCHES ".c")
				if(NOT "${source_file}" MATCHES "CMakeCXXCompilerId.cpp" AND NOT "${source_file}" MATCHES "feature_tests.cxx")
					set(library_source_files ${library_source_files} ${source_file})
				endif()
			endif()
		else()
			set(${main_file} ${source_file} PARENT_SCOPE)
		endif()
	endforeach()
	set(${library_files} ${library_source_files} PARENT_SCOPE)
endfunction()

function(GetFileNames file_names files)
	foreach(file ${files})
		get_filename_component(name ${file} NAME_WE)
		set(names ${names} ${name})
	endforeach()
	set(${file_names} ${names} PARENT_SCOPE)
endfunction()

function(LinkModuleLibrary library_name library_source link_library other_module_library_paths)
	foreach(other_module_library_path ${other_module_library_paths})
		if(EXISTS ${other_module_library_path})
			get_filename_component(absolute_other_module_library_path ${other_module_library_path} ABSOLUTE)
			file(GLOB absolute_module_library "${absolute_other_module_library_path}/lib*.*")

			if(${absolute_module_library})
				message(FATAL_ERROR " ---- Not found module library")
			endif()
			set(absolute_module_libraries ${absolute_module_libraries} ${absolute_module_library})

		else()
			message(FATAL_ERROR " ---- Not found module library path ${other_module_library_path}")
		endif()
	endforeach()
	message(" ---- Create library ${library_name}")
	add_library(${library_name} SHARED ${library_source})
	target_link_libraries(${library_name} ${absolute_module_libraries} ${link_library})
endfunction()

function(CreateExecutable executable_name executable_source dependencies_files link_library)
	if(executable_source)
		message(" ---- Create executable file ${executable_name}")
		add_executable(${executable_name} ${executable_source})

		if(LIBRARY_SOURCES)
			add_dependencies(${executable_name} ${dependencies_files})
		else()
			message(" ---- Not found library source")
		endif()

		target_link_libraries(${executable_name} ${link_library})

	else()
		message(" ---- Not found main source")
	endif()
endfunction()

