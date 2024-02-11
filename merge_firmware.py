Import("env")

APP_BIN = "$BUILD_DIR/${PROGNAME}.bin"
MERGED_BIN = "$BUILD_DIR/${PROGNAME}_0x00.bin"
BOARD_CONFIG = env.BoardConfig()


def merge_bin(source, target, env):
    # The list contains all extra images (bootloader, partitions, eboot) and
    # the final application binary
    flash_images = env.Flatten(env.get("FLASH_EXTRA_IMAGES", [])) + ["$ESP32_APP_OFFSET", APP_BIN]

    # Run esptool to merge images into a single binary
    env.Execute(
        " ".join(
            [
                "$PYTHONEXE",
                "$OBJCOPY",
                "--chip",
                BOARD_CONFIG.get("build.mcu", "esp32"),
                "merge_bin",
                "--flash_size",
                BOARD_CONFIG.get("upload.flash_size", "4MB"),
                "-o",
                MERGED_BIN,
            ]
            + flash_images
        )
    )

# Add a post action that runs esptoolpy to merge available flash images
env.AddPostAction(APP_BIN , merge_bin)

# Patch the upload command to flash the merged binary at address 0x0
#env.Replace(
#    UPLOADERFLAGS=[
#            f
#            for f in env.get("UPLOADERFLAGS")
#            if f not in env.Flatten(env.get("FLASH_EXTRA_IMAGES"))
#        ]
#        + ["0x0", MERGED_BIN],
#    UPLOADCMD='"$PYTHONEXE" "$UPLOADER" $UPLOADERFLAGS',
#)