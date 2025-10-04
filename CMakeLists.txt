idf_component_register(SRCS "DShotRMT.cpp" "dshot_rmt_encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       PRIV_REQUIRES esp_timer)
