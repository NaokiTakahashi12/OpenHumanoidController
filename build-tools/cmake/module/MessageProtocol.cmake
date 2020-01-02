
#
# MessageProtocol.cmake
#
# Author Naoki Takahashi
# Email s16c1077hq@s.chibakoudai.jp
#

function(buildtool_message PRINTOUT_STRING)
	message(" ---- ${PRINTOUT_STRING}")
endfunction(buildtool_message)

function(buildtool_fatal_message PRINTOUT_STRING)
	message(FATAL_ERROR " ---- ${PRINTOUT_STRING}")
endfunction(buildtool_fatal_message)

