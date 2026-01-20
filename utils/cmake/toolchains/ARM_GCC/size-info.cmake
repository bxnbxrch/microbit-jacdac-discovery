add_custom_command(
    TARGET ${device.device}
    COMMAND "${ARM_NONE_EABI_SIZE}" "$<TARGET_FILE:${device.device}>"
    DEPENDS  ${device.device}
    COMMENT "Print total size info:"
)
