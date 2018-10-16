cmake_minimum_required(VERSION 3.1.3)

message("picam360-server generating Makefile")
project(picam360-server)

message("check node path")
execute_process(COMMAND which node OUTPUT_VARIABLE NODE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT NODE_PATH)
	message(FATAL_ERROR "can not find node. please install node")
endif()

add_custom_target(git_submodule_update ALL
	COMMAND git submodule update -i
	COMMENT "git submodule update -i"
)

add_custom_target(npm_install ALL
	COMMAND npm -dd install
	COMMENT "npm install - it takes few minutes"
)
	
#install
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)
configure_file( setup/picam360-server.in setup/picam360-server )
configure_file( setup/picam360-server.service.in setup/picam360-server.service )
configure_file( setup/install.sh.in setup/install.sh )
install(CODE "execute_process(COMMAND bash -x setup/install.sh)")