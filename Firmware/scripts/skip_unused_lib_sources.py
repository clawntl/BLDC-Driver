# Leaves library sources the firmware never uses out of the build.
#
# lib_archive = false (platformio.ini) links every object file of every
# library, and some of them stay in the image even though nothing calls them,
# because they create global objects at start-up. SimpleFOCDrivers' comms/
# folder (register access, telemetry, I2C commander, which also pulls in the
# I2C driver) was ~2.9 KB of a flash budget that is nearly used up (the
# settings page starts at 126 KB, see docs/firmware.md#flash-size-limit).
Import("env")


def skip(node):
    return None  # returning None drops the file from the build


env.AddBuildMiddleware(skip, "*/SimpleFOCDrivers/src/comms/*")
