#include "MCP4922.h"

MCP4922::MCP4922(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi) : spi(spi){
    regs[0] = MCP4X_DEFAULTS;
    regs[1] = MCP4X_WRITE_B | MCP4X_DEFAULTS;
}

MCP4922::~MCP4922(){

}

void MCP4922::setGain2x(Channel chan, bool gain2x){
	if (gain2x)
		regs[chan] &= ~MCP4X_GAIN_1X;
	else
		regs[chan] |= MCP4X_GAIN_1X;
}

void MCP4922::output(Channel chan, unsigned short data){

    const unsigned short maxval = (1 << 12) - 1;
	
    if (data > maxval)
		data = maxval;

	// clear value bits
	regs[chan] &= 0xF000;
	regs[chan] |= data;
}

void MCP4922::output2(unsigned short _outX, unsigned short _outY){
    output(X, _outX);
    output(Y, _outY);

    if(autoLatch)
		latch();
}

void MCP4922::latch(){

}