#pragma once

#include "Module.h"
#include "Pin.h"

#include <stdint.h>

#include "DACs/MCP4922/MCP4922.h"

namespace mbed {
    class SPI;
}


class GalvoControl : public Module {
    public:
        GalvoControl(uint8_t id);
        virtual ~GalvoControl();

        void on_module_loaded();

    private:
        bool config_module(uint16_t cs);

        int sendSPI(uint8_t *b, int cnt, uint8_t *r);

        MCP4922 *dac;
        Pin spi_cs_pin;
        mbed::SPI *spi;
        uint8_t id;

};
