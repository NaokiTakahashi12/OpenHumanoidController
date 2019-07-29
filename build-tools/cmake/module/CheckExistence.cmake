#
# CheckExistence.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

function(CreateDirectory directory_path)
	if(NOT EXISTS ${directory_path})
		message("Create ${directory_path}")
		file(MAKE_DIRECTORY ${directory_path})
	endif()
endfunction()

function(RequiredDirectory directory_path)
	if(NOT EXISTS ${directory_path})
		message(FATAL_ERROR "Not found required ${directory_path}")
	endif()
endfunction()



