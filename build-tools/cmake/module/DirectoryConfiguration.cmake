
#
# DirectoryConfiguration.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

include(MessageProtocol)

function(create_directory DIRECTORY_PATH)
	if(NOT EXISTS ${DIRECTORY_PATH})
		file(MAKE_DIRECTORY ${DIRECTORY_PATH})
		buildtool_message("Create ${DIRECTORY_PATH}")
	else()
		buildtool_message("Skip create directory ${DIRECTORY_PATH}")
	endif()
endfunction(create_directory)

