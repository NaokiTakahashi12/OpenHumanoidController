#
# Finder.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

include(MessageProtocol)

function(FindFiles result_files top_directories exts)
	set(directory_pattern /*.)

	foreach(top_directory ${top_directories})
		set(directory_pattern_path ${directory_pattern})

		while(TRUE)
			foreach(ext ${exts})
				file(GLOB result_file "${top_directory}${directory_pattern_path}${ext}")
				set(result_files_handler ${result_files_handler} ${result_file})

			endforeach()
			file(GLOB found_file "${top_directory}${directory_pattern_path}*")

			if(found_file)
				set(directory_pattern_path /*${directory_pattern_path})
			else()
				break()
			endif()
		endwhile()
	endforeach()
	set(${result_files} ${result_files_handler} PARENT_SCOPE)
endfunction()

function(CreateDirectoryPathListFromFile result_directories input_files)
	foreach(input_file ${input_files})
		get_filename_component(input_directory ${input_file} DIRECTORY)
		set(match_directory false)

		foreach(result_directory_handler ${result_directories_handler})
			if("${result_directory_handler}" MATCHES "${input_directory}")
				set(match_directory true)
			endif()
		endforeach()
		if(NOT ${match_directory})
				set(result_directories_handler ${result_directories_handler} ${input_directory})
		endif()
	endforeach()
	set(${result_directories} ${result_directories_handler} PARENT_SCOPE)
endfunction()

function(UsingPython2 includes libraries)
	find_package(PythonLibs 2.7 REQUIRED COMPONENTS numpy)
	if(PythonLibs_FOUND)
		message(" ---- Found PythonLibs2 ${PYTHON_INCLUDE_DIRS}")
		set(${includes} ${PYTHON_INCLUDE_DIRS} PARENT_SCOPE)
		set(${libraries} ${PYTHON_LIBRARIES} PARENT_SCOPE)
	else()
		message(FATAL_ERROR "Not found PythonLibs2")
	endif()
endfunction()

function(UsingPython3 includes libraires)
	find_package(PythonLibs 3.8 REQUIRED COMPONENTS numpy)
	if(PythonLibs_FOUND)
		message(" ---- Found PythonLibs3 ${PYTHON_INCLUDE_DIRS}")
		set(${includes} ${PYTHON_INCLUDE_DIRS} PARENT_SCOPE)
		set(${libraries} ${PYTHON_LIBRARIES} PARENT_SCOPE)
	else()
		message(FATAL_ERROR "Npt found PythonLibs3")
	endif()
endfunction()

function(UsingRBDL includes libraries)
	set(RBDL_INCLUDE_DIR $ENV{RBDL_INCLUDE_PATH})
	set(RBDL_LIBRARY_DIR $ENV{RBDL_LIBRARY_PATH})

	if(EXISTS ${RBDL_INCLUDE_DIR} AND EXISTS ${RBDL_LIBRARY_DIR})
		message(" ---- Found RBDL ${RBDL_INCLUDE_DIR}")

		if(APPLE)
			file(GLOB RBDL_LIBRARY "${RBDL_LIBRARY_DIR}/librbdl*.dylib")
		elseif(UNIX)
			file(GLOB RBDL_LIBRARY "${RBDL_LIBRARY_DIR}/librbdl*.so")
		endif()

		set(${includes} ${RBDL_INCLUDE_DIR} PARENT_SCOPE)
		set(${libraries} ${RBDL_LIBRARY} PARENT_SCOPE)
	else()
		message(FATAL_ERROR "Not found RBDL")
	endif()
endfunction()

function(UsingQT includes libraries)
	find_package(Qt5Core REQUIRED)
	find_package(Qt5Widgets REQUIRED)
	if(Qt5Core_FOUND AND Qt5Widgets_FOUND)
		set(CMAKE_AUTOMOC ON)
		set(CMAKE_AUTOUIC ON)
		set(${includes} ${Qt5Core_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} PARENT_SCOPE)
		set(${libraries} ${Qt5Core_LIBRARIES} ${Qt5Widgets_LIBRARIES} PARENT_SCOPE)
	else()
		message(FATAL_ERROR "Not found qt")
	endif()
endfunction()

function(UsingSPDLOG includes libraries)
	if(APPLE)
		set(SPDLOG_INCLUDE_DIR /usr/local/include/)
	elseif(UNIX)
		set(SPDLOG_INCLUDE_DIR NotFound)
	endif(APPLE)

	message("Found spdlog ${SPDLOG_INCLUDE_DIR}")
	set(${includes} ${SPDLOG_INCLUDE_DIR} PARENT_SCOPE)
endfunction()

function(UsingGL includes libraries)
	find_package(OpenGL REQUIRED)

	if(OpenGL_FOUND)
		message(" ---- Found OpenGL ${OpenGL_INCLUDE_DIRS}")
		set(${includes} ${OpenGL_INCLUDE_DIRS} PARENT_SCOPE)

	else()
		message(FATAL_ERROR " ---- Not Found OpenGL")
	endif()

	if(APPLE)
		set(${libraries} ${OpenGL_LIBRARIES} glfw "-framework OpenGL" PARENT_SCOPE)
	elseif(UNIX)
		set(${libraries} ${OpenGL_LIBRARIES} glfw PARENT_SCOPE)
	endif()
endfunction()

function(UsingSSH includes libraries)
	if(APPLE)
		set(LIBSSH_TOP_DIR /usr/local)
	elseif(UNIX)
		set(LIBSSH_TOP_DIR /usr)
	endif()

	set(libssh_INCLUDE_DIRS ${LIBSSH_TOP_DIR}/include)
	set(libssh_LIBRARIES_DIR ${LIBSSH_TOP_DIR}/lib)

	if(EXISTS ${libssh_INCLUDE_DIRS} AND EXISTS ${libssh_LIBRARIES_DIR})
		message(" ---- Found libssh ${libssh_INCLUDE_DIRS}")
		set(${includes} ${libssh_INCLUDE_DIRS} PARENT_SCOPE)
		file(GLOB libssh_LIBRARIES "${libssh_LIBRARIES_DIR}/libssh*.dylib")

	else()
		message(FATAL_ERROR " ---- Not Found libssh Top Directories")
	endif()

	set(${libraries} ${libssh_LIBRARIES} PARENT_SCOPE)
endfunction()

function(UsingODE includes libraries)
	if(APPLE)
		set(ODE_TOP_DIRECTORIES /usr/local/Cellar/ode-drawstuff/0.13/)
	elseif(UNIX)
		set(ODE_TOP_DIRECTORIES /usr/local)
	endif()

	set(ODE_INCLUDE_DIR ${ODE_TOP_DIRECTORIES}/include)
	set(ODE_LIBRARIES_DIR ${ODE_TOP_DIRECTORIES}/lib)

	if(EXISTS ${ODE_INCLUDE_DIR} AND EXISTS ${ODE_LIBRARIES_DIR})
		message(" ---- Found Ode ${ODE_INCLUDE_DIR}")
		set(${includes} ${ODE_INCLUDE_DIR} PARENT_SCOPE)
		file(GLOB ODE_LIBRARIES "${ODE_LIBRARIES_DIR}/lib*.a")

	else()
		message(FATAL_ERROR " ---- Not Found Ode Top Directories")
	endif()

	if(APPLE)
		set(${libraries} ${ODE_LIBRARIES} "-framework AGL -framework OpenGL -framework GLUT" PARENT_SCOPE)
	elseif(UNIX)
		set(${libraries} ${ODE_LIBRARIES} drawstuff GL GLU glut X11 "ode" PARENT_SCOPE)
	endif()
endfunction()

function(UsingCV includes libraries)
	find_package(OpenCV REQUIRED)

	if(OpenCV_FOUND)
		message(" ---- Found OpenCV ${OpenCV_INCLUDE_DIRS}")
		set(${includes} ${OpenCV_INCLUDE_DIRS} PARENT_SCOPE)

	else(OpenCV_FOUND)
		message(FATAL_ERROR " ---- Not Found OpenCV")
	endif(OpenCV_FOUND)

	set(${libraries} ${OpenCV_LIBRARIES} PARENT_SCOPE)
endfunction()

function(UsingBoost includes libraries)
	find_package(Boost	REQUIRED COMPONENTS thread)

	if(Boost_FOUND)
		message(" ---- Found Boost ${Boost_INCLUDE_DIRS}")
		set(${includes} ${Boost_INCLUDE_DIRS} PARENT_SCOPE)

	else(Boost_FOUND)
		message(FATAL_ERROR " ---- Not Found Boost")
	endif(Boost_FOUND)

	set(${libraries} ${Boost_LIBRARIES} Boost::thread PARENT_SCOPE)
endfunction()

function(UsingEigen3 includes libraries)
	if(APPLE)
		find_package(Eigen3	REQUIRED COMPONENTS Dense Core Geometry)

		if(Eigen3_FOUND)
			message(" ---- Found Eigen3 ${EIGEN3_INCLUDE_DIRS}")
			set(${includes} ${EIGEN3_INCLUDE_DIRS} PARENT_SCOPE)

		else(Eigen3_FOUND)
			message(FATAL_ERROR " ---- Not Found Eigen3")
		endif(Eigen3_FOUND)
	elseif(UNIX)
		if(EXISTS $ENV{EIGEN3_INCLUDE_DIR})
			message(" ---- Used Unix Eigen3 Dir $ENV{EIGEN3_INCLUDE_DIR}")
			set(${includes} $ENV{EIGEN3_INCLUDE_DIR} PARENT_SCOPE)

		else()
			find_package(Eigen3 REQUIRED)
			if(Eigen3_FOUND)
				message(" ---- Found Eigen3 ${EIGEN3_INCLUDE_DIR}")
				set(${includes} ${EIGEN3_INCLUDE_DIR} PARENT_SCOPE)

			else()
				message(" ---- Not Defind EIGEN3_INCLUDE_DIRS")
			endif(Eigen3_FOUND)
		endif()
	endif()
endfunction()

function(UsingIce includes libraries)
	find_package(Ice REQUIRED COMPONENTS Ice++11 IceBox++11 IceSSL++11 IceGrid++11 IceDiscovery++11)

	if(Ice_FOUND)
		message(" ---- Found Ice ${Ice_INCLUDE_DIRS}")
		set(${includes} ${Ice_INCLUDE_DIRS} PARENT_SCOPE)
		add_definitions("-D ICE_CPP11_MAPPING")
		
		if(NOT ICE_DIR)
			set(ICE_DIR ice CACHE PATH "find .ice file" FORCE)
		endif()

		if(EXISTS ${ICE_DIR})
			file(GLOB ICE_FILES "${ICE_DIR}/*.ice")
			if(NOT ICE_OUTPUT_DIRECTORY)
				set(ICE_OUTPUT_DIRECTORY ice_create_files CACHE PATH "Ice create file out-put directory" FORCE)
			endif()
			
			if(NOT EXISTS ${Ice_SLICE2CPP_EXECUTABLE})
				message(FATAL_ERROR " ---- Not found ice slice")
			endif()

			if(NOT DEFINED ICE_FILES)
				message(FATAL_ERROR " ---- Not found ice files")
			endif()

			if(NOT EXISTS ${ICE_OUTPUT_DIRECTORY})
				message(FATAL_ERROR " ---- Not found ice output directory")
			endif()

			get_filename_component(ICE_OUTPUT_DIRECTORY ${ICE_OUTPUT_DIRECTORY} ABSOLUTE)

			foreach(ICE_FILE ${ICE_FILES})
				get_filename_component(ICE_CREATE_FILENAME ${ICE_FILE} NAME_WE)
				set(ICE_OUTPUT_FILES ${ICE_OUTPUT_FILES} ${ICE_OUTPUT_DIRECTORY}/${ICE_CREATE_FILENAME}.cpp ${ICE_OUTPUT_DIRECTORY}/${ICE_CREATE_FILENAME}.h)

			endforeach(ICE_FILE)

			add_custom_command(OUTPUT ${ICE_OUTPUT_FILES} DEPENDS ${ICE_FILES} COMMAND "${Ice_SLICE2CPP_EXECUTABLE}" ARGS ${ICE_FILES} --output-dir ${ICE_OUTPUT_DIRECTORY})

			foreach(ICE_OUTPUT_FILE ${ICE_OUTPUT_FILES})
				if("${ICE_OUTPUT_FILE}" MATCHES ".cpp")
					set(SOURCE_FILES ${SOURCE_FILES} ${ICE_OUTPUT_FILE})
				endif()
			endforeach(ICE_OUTPUT_FILE)

			set(${includes} ${ICE_OUTPUT_DIRECTORY} ${Ice_INCLUDE_DIRS} PARENT_SCOPE)

		else()
			message(" ---- Not found ice directory")

		endif()
	else(Ice_FOUND)
		message(FATAL_ERROR " ---- Not Found Ice")

	endif(Ice_FOUND)

	set(${libraries} ${Ice_LIBRARIES} PARENT_SCOPE)
endfunction()

