# QorTroller biometric pipeline module
# Conditionally compiled when CONFIG_QORTROLLER=y in sdkconfig / idf_component.yml.
# Zero impact on upstream joypad-os builds that don't set this flag.

if(CONFIG_QORTROLLER)
    idf_component_register(
        SRCS
            "qortroller_hook.c"
            "poac_builder.c"
            # sense_task.c      — add when Core-0 FreeRTOS task is implemented
            # uwb_presence.c    — add when QM35825DK-05 eval kit is connected
            # atca_signer.c     — add when ATECC608B breakout is wired (Path A Arc 2)
            # bridge_transport.c — add with BLE GATT characteristic implementation
        INCLUDE_DIRS
            "."
        REQUIRES
            driver
            freertos
            esp_timer
        PRIV_REQUIRES
            # cryptoauthlib  — uncomment when atca_signer.c is added
            # esp_wifi       — uncomment when bridge_transport.c WiFi path is added
    )

    # Compile-time assertion: PoAC wire format must be exactly 228 bytes.
    # This fires at firmware build time — catches any accidental struct change
    # before the binary reaches a device.
    target_compile_options(${COMPONENT_LIB} PRIVATE
        -DCONFIG_QORTROLLER=1
        -DPOAC_ASSERT_SIZE=1
    )

    message(STATUS "[QorTroller] biometric pipeline module enabled (CONFIG_QORTROLLER=y)")
else()
    message(STATUS "[QorTroller] biometric pipeline disabled — set CONFIG_QORTROLLER=y to enable")
endif()
