cmake_minimum_required(VERSION 3.28)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rpi-electic-winch)

target_sources(app PRIVATE src/main.c src/winch_shell.c src/as5600.c)