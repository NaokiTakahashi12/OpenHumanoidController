
#
# Assert.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

include(MessageProtocol)

function(assert_exist_path CONFIRM_PATH)
	if(EXISTS ${CONFIRM_PATH})
		buildtool_message("Exist confirm path ${CONFIRM_PATH}")
	else()
		buildtool_fatal_message("Not Exist confirm path ${CONFIRM_PATH}")
	endif()
endfunction(assert_exist_path)

function(assert_exist_paths CONFIRM_PATHS)
	foreach(CONFIRM_PATH ${CONFIRM_PATHS})
		assert_exist_path(${CONFIRM_PATH})
	endforeach()
endfunction(assert_exist_paths)

