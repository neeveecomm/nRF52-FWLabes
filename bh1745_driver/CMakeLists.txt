cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(MCP4725_driver)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/mcp4725.c)
