#include "GalvoControl.h"

#include "libs/Kernel.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "ConfigValue.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "Robot.h"
#include "StepperMotor.h"
#include "PublicDataRequest.h"

#include "Gcode.h"
#include "Config.h"
#include "checksumm.h"

#include "mbed.h" // for SPI

#include <string>
#include <vector>

#define galvo_controller_checksum      CHECKSUM("galvo_controller")
#define enable_checksum                CHECKSUM("enable")
#define spi_channel_checksum           CHECKSUM("spi_channel")
#define spi_cs_pin_checksum            CHECKSUM("spi_cs_pin")
#define spi_frequency_checksum         CHECKSUM("spi_frequency")

GalvoControl::GalvoControl(uint8_t id) : id(id){
    
}

GalvoControl::~GalvoControl(){

}

void GalvoControl::on_module_loaded(){
    vector<uint16_t> modules;
    THEKERNEL->config->get_module_list( &modules, galvo_controller_checksum );
    uint8_t cnt = 1;
    for( auto cs : modules ) {
        // If module is enabled create an instance and initialize it
        if( THEKERNEL->config->value(galvo_controller_checksum, cs, enable_checksum )->as_bool() ) {
            GalvoControl *controller = new GalvoControl(cnt++);
            if(!controller->config_module(cs)) delete controller;
        }
    }

    // we don't need this instance anymore
    delete this;
}

bool GalvoControl::config_module(uint16_t cs){

    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;

    dac = new MCP4922(std::bind( &GalvoControl::sendSPI, this, _1, _2, _3));

    // select which SPI channel to use
    int spi_channel = THEKERNEL->config->value(galvo_controller_checksum, cs, spi_channel_checksum)->by_default(1)->as_number();
    int spi_frequency = THEKERNEL->config->value(galvo_controller_checksum, cs, spi_frequency_checksum)->by_default(1000000)->as_number();

    // select SPI channel to use
    PinName mosi, miso, sclk;
    if(spi_channel == 0) {
        mosi = P0_18; miso = P0_17; sclk = P0_15;
    } else if(spi_channel == 1) {
        mosi = P0_9; miso = P0_8; sclk = P0_7;
    } else {
        THEKERNEL->streams->printf("GalvoControl ERROR: Unknown SPI Channel: %d\n", spi_channel);
        return false;
    }

    this->spi = new mbed::SPI(mosi, miso, sclk);
    this->spi->frequency(spi_frequency);
    this->spi->format(8, 0); // 8bit, mode0

    return true;
}

int GalvoControl::sendSPI(uint8_t *b, int cnt, uint8_t *r){
    spi_cs_pin.set(0);
    for (int i = 0; i < cnt; ++i) {
        r[i]= spi->write(b[i]);
    }
    spi_cs_pin.set(1);
    return cnt;
}