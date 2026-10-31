FetchContent_Declare(
  cmsis_dsp
  URL C:/Users/sky/PinyCore_cache/CMSIS-DSP-v1.17.1.tar.gz
  SOURCE_DIR "${THIRD_PARTY_DIR}/CMSIS-DSP")

set(CMSISCORE "${soc_folder}/hal/Drivers/CMSIS")

FetchContent_MakeAvailable(cmsis_dsp)

target_compile_options(CMSISDSP PRIVATE -O3  -g3 -ffast-math)
