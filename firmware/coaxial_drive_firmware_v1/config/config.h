#ifndef CONFIG_H
#define CONFIG_H

    #ifdef ESP32
        #include "esp32_hardware_pin.h"
    #else
        #include "teensy_hardware_pin.h"
    #endif
    #include "PIDF_config.h"

#endif