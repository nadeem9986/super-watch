idf_component_register(SRCS "main.cpp" "ble_phone.c" INCLUDE_DIRS "." REQUIRES framework-arduinoespressif32 bt nvs_flash)
# Preserve the existing compact UI source; keep normal warnings on native BLE.
set_source_files_properties(main.cpp PROPERTIES COMPILE_OPTIONS "-Wno-misleading-indentation")
