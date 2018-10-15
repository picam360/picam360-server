cmake_minimum_required(VERSION 3.1.3)

message("picam360-server generating Makefile")
project(picam360-server)

execute_process(COMMAND git submodule update -i)

execute_process(COMMAND which node OUTPUT_VARIABLE NODE_PATH OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND npm install)
	
#install
configure_file( setup/picam360-server.in setup/picam360-server )
configure_file( setup/picam360-server.service.in setup/picam360-server.service )
configure_file( setup/install.sh.in setup/install.sh )
install(CODE "execute_process(COMMAND bash -x setup/install.sh)")