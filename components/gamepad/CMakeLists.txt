idf_component_register(
    SRCS "gamepad.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES usb
)

# Debug builds: RETRO_DEBUG_USB=1 leaves the native USB port as a serial log port (no USB pads)
if($ENV{RETRO_DEBUG_USB})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE GAMEPAD_NO_USB_HOST)
endif()
