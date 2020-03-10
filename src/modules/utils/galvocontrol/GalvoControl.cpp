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

#define galvo_controller_checksum      CHECKSUM("galvo_control")
#define enable_checksum                CHECKSUM("enable")
#define dac_checksum                   CHECKSUM("dac")
#define spi_channel_checksum           CHECKSUM("spi_channel")
#define spi_cs_pin_checksum            CHECKSUM("spi_cs_pin")
#define spi_latch_pin_checksum         CHECKSUM("spi_latch_pin")
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

    std::string str = THEKERNEL->config->value(galvo_controller_checksum, cs, dac_checksum)->by_default("")->as_string();
    if(str.empty()) {
        THEKERNEL->streams->printf("GalvoControl ERROR: dac type not defined\n");
        return false; // dac type required
    }

    using std::placeholders::_1;
    using std::placeholders::_2;

    spi_latch_pin.from_string(THEKERNEL->config->value( galvo_controller_checksum, cs, spi_latch_pin_checksum)->by_default("nc")->as_string())->as_output();
    if(!spi_latch_pin.connected()) {
        THEKERNEL->streams->printf("GalvoControl ERROR: dac latch pin not defined\n");
        return false; // if not defined then we can't use this instance
    }

    //Unlatch the output
    spi_latch_pin.set(1);

    if(str == "MCP4922"){
        dac = new MCP4922(std::bind( &GalvoControl::sendSPI, this, _1, _2), spi_latch_pin);
    }
    else{
        THEKERNEL->streams->printf("GalvoControl ERROR: Unknown DAC type: %s\n", str.c_str());
        return false;
    }

    spi_cs_pin.from_string(THEKERNEL->config->value( galvo_controller_checksum, cs, spi_cs_pin_checksum)->by_default("nc")->as_string())->as_output();
    if(!spi_cs_pin.connected()) {
        THEKERNEL->streams->printf("GalvoControl ERROR: chip select not defined\n");
        return false; // if not defined then we can't use this instance
    }

    //Unselect the DAC
    spi_cs_pin.set(1);
    
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

    this->register_for_event(ON_GCODE_RECEIVED);

    return true;
}

void GalvoControl::on_gcode_received(void *argument){
    Gcode *gcode = static_cast<Gcode*>(argument);

    if (gcode->has_m){
        if(gcode->m == 888){
            gcode->stream->printf("Running galvo test\n");

            const unsigned short maxval = (1 << 12) - 1;

            const unsigned short line[] = {0, maxval/2, maxval};

            int idx = 0;
            int output_every = 50;
            
            gcode->stream->printf("X channel\n");
            for (int i = 0; i < 100000; i++){
                if (i % output_every == 0){
                    dac->output(MCP4922::Channel::X, line[idx]);
                    idx = idx == 2 ? 0 : idx + 1 ;
                }
            }

            gcode->stream->printf("Y channels\n");
            for (int i = 0; i < 100000; i++){
                if (i % output_every == 0){
                    dac->output(MCP4922::Channel::Y, line[idx]);
                    idx = idx == 2 ? 0 : idx + 1 ;
                }
            }


            gcode->stream->printf("Both channels\n");
            for (int i = 0; i < 100000; i++){
                if (i % output_every == 0){
                    dac->output2(line[idx], line[idx]);
                    idx = idx == 2 ? 0 : idx + 1 ;
                }
            }

        }
    }
}

int GalvoControl::sendSPI(uint8_t *b, int cnt){
    spi_cs_pin.set(0);
    for (int i = 0; i < cnt; ++i) {
        spi->write(b[i]);
    }
    spi_cs_pin.set(1);
    return cnt;
}