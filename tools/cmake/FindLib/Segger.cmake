# reference to https://kb.segger.com/FreeRTOS_with_SystemView
if(NOT EXISTS "${THIRD_PARTY_DIR}/RTT")
  FetchContent_Declare(
    RTT
    URL C:/Users/sky/PinyCore_cache/RTT-main.tar.gz
    SOURCE_DIR "${THIRD_PARTY_DIR}/RTT")
  FetchContent_MakeAvailable(RTT)
endif()

if(NOT EXISTS "${THIRD_PARTY_DIR}/systemview")
  FetchContent_Declare(
    systemview
    URL C:/Users/sky/PinyCore_cache/SystemView-main.tar.gz
    SOURCE_DIR "${THIRD_PARTY_DIR}/systemview")
  FetchContent_MakeAvailable(systemview)
endif()

file(GLOB_RECURSE SOURCES CMAKE_CONFIGURE_DEPENDS 
  "${THIRD_PARTY_DIR}/RTT/RTT/*.c"
  "${THIRD_PARTY_DIR}/RTT/RTT/*.S"
  "${THIRD_PARTY_DIR}/RTT/Syscalls/SEGGER_RTT_Syscalls_GCC.c")

add_library(segger STATIC ${SOURCES})

target_include_directories(segger PUBLIC
  "${THIRD_PARTY_DIR}/RTT/Config"
  "${THIRD_PARTY_DIR}/RTT/RTT")
