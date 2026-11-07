# Target picks its UI/Display source; original C3 source stays untouched.
if(CONFIG_IDF_TARGET_ESP32)
    set(NWATCH_MAIN "main_esp32.cpp")
else()
    set(NWATCH_MAIN "main.cpp")
endif()
idf_component_register(SRCS ${NWATCH_MAIN} "ble_phone.c" INCLUDE_DIRS "." REQUIRES framework-arduinoespressif32 bt nvs_flash)
# Preserve the existing compact UI source; keep normal warnings on native BLE.
set_source_files_properties(${NWATCH_MAIN} PROPERTIES COMPILE_OPTIONS "-Wno-misleading-indentation")
