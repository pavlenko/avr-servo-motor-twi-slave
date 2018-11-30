Import("env")

# Override upload FLASH command
#
# If target PLATFORMIO_UPLOAD not exists in CMakeLists.txt add code below:
# add_custom_target(
#     PLATFORMIO_UPLOAD ALL
#     COMMAND ${PLATFORMIO_CMD} -f -c clion run --target upload
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )
env.Replace(UPLOADHEXCMD="$UPLOADER -v -p m32 -c usbasp -P usb -U flash:w:$SOURCES:i")

# Override upload EEPROM command
#
# If target PLATFORMIO_EEPROM not exists in CMakeLists.txt add code below:
# add_custom_target(
#     PLATFORMIO_EEPROM ALL
#     COMMAND ${PLATFORMIO_CMD} -f -c clion run --target uploadeep
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )
env.Replace(UPLOADEEPCMD="$UPLOADER -v -p m32 -c usbasp -P usb -U eeprom:w:$SOURCES:i")

# Override command for FUSES target
#
# If target PLATFORMIO_FUSES not exists in CMakeLists.txt add code below:
# add_custom_target(
#     PLATFORMIO_FUSES ALL
#     COMMAND ${PLATFORMIO_CMD} -f -c clion run --target fuses
#     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )
env.Replace(FUSESCMD="$UPLOADER -v -p m32 -c usbasp -P usb -Uhfuse:w:0x99:m -Ulfuse:w:0xef:m")
