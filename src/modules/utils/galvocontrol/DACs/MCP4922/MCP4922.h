#pragma once

#define MCP4X_WRITE_B			(1 << 15)
#define MCP4X_BUFFERED			(1 << 14)
#define MCP4X_GAIN_1X			(1 << 13)
#define MCP4X_ACTIVE			(1 << 12)

#define MCP4X_DEFAULTS			(MCP4X_BUFFERED | MCP4X_GAIN_1X | MCP4X_ACTIVE)

#include <functional>
#include "Pin.h"

class MCP4922{

public:

    enum Channel{
        X = 1,
        Y = 0
    };

    MCP4922(std::function<int(uint8_t *b, int cnt)> spi, Pin latchPin);
    ~MCP4922();

    void setGain2x(Channel chan, bool gain2x = 1);
	void setAutoLatch(bool enabled = 1)	                { autoLatch = enabled;			}

    void output(Channel chan, unsigned short data);
    void output(unsigned short data) 					{ output(X, data); 	}
	void outputX(unsigned short data)					{ output(X, data); 	}
	void outputY(unsigned short data) 					{ output(Y, data);	}
	void output2(unsigned short _outX, unsigned short _outY);

    void setMirrorX(bool mirror)                        {mirrorX = mirror;  }
    void setMirrorY(bool mirror)                        {mirrorY = mirror;  }

    void write(unsigned int data);
    void latch();


private:

    std::function<int(uint8_t *b, int cnt)> spi;

    unsigned int vrefs[2];
    unsigned int regs[2];
    bool autoLatch;
    bool mirrorX;
    bool mirrorY;
    Pin latchPin;

};