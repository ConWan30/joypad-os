# QorTroller biometric pipeline module
# Conditionally compiled when CONFIG_QORTROLLER=y in sdkconfig / idf_component.yml.
# Zero impact on upstream joypad-os builds that don't set this flag.

if(CONFIG_QORTROLLER)
    idf_component_register(
        SRCS
            "qortroller_hook.c"
            "poac_builder.c"
            "sense_task.c"
            "atca_signer.c"
            "bridge_transport.c"
            "uwb_presence.c"
        INCLUDE_DIRS
            "."
        REQUIRES
            driver
            freertos
            esp_timer
        PRIV_REQUIRES
            # cryptoauthlib  — define CRYPTOAUTHLIB_PRESENT when ATECC608B is wired (Arc 2)
            # esp_wifi       — add when WiFi fallback path is enabled in bridge_transport.c
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
