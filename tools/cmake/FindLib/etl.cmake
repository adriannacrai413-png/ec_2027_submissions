FetchContent_Declare(
  etl
  URL C:/Users/sky/PinyCore_cache/etl-20.45.0.tar.gz
  SOURCE_DIR "${THIRD_PARTY_DIR}/etl")

FetchContent_MakeAvailable(etl)

set(ETL_CONFIG
    ETL_CPP23_SUPPORTED
    ETL_NO_STL
    ETL_NO_INITIALIZER_LIST
    ETL_NO_64BIT_TYPES
)

target_compile_definitions(etl INTERFACE ${ETL_CONFIG})
