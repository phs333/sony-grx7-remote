cmake_minimum_required(VERSION 3.16)

# Ativa o ccache automaticamente em qualquer ambiente (CLI ou VS Code)
set(CCACHE_ENABLE ON CACHE BOOL "Enable ccache for faster incremental builds" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Compila APENAS os componentes necessarios declarados em main/CMakeLists.txt (ignora LCD, SPIFFS, FATFS, Bluetooth, etc.)
idf_build_set_property(MINIMAL_BUILD ON)
project(sony_grx7_remote)
