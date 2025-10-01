idf_component_register(SRCS "roll_pitch.c" "mpu6050.c" "quaternions.c" "gps.c" "main.c"

                    PRIV_REQUIRES spi_flash driver
                    INCLUDE_DIRS ".")

