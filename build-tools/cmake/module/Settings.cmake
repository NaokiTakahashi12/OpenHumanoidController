#
# Settings.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

function(Flag flag init_flag flag_message)
	if(NOT DEFINED ${flag})
		set(${flag} ${init_flag} CACHE BOOL "${flag_message} : true or false" FORCE)
	endif()
endfunction(Flag)

function(Path path init_path path_message)
	if(NOT DEFINED ${path})
		set(${path} ${init_path} CACHE PATH "${path_message}" FORCE)
	endif()
endfunction(Path)

