/*
Copyright Lucian Copeland, 2016
Rewritten by Tom Moxon, 2017

MIT License
*/

#include "HL_TCA9548a.h"

uint8_t TCA9548a::TCA_ADDR[] = {0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77};

TCA9548a::TCA9548a(PinName sda, PinName scl): i2c(sda, scl)  
{
    _internalCallback.attach(this, &TCA9548a::cbHandler);
}

void TCA9548a::openChannel(uint8_t tca_id, uint8_t channel) {
    char tchannel = 0x01<<channel; 
    writeByte(tca_id, tchannel);
}

void TCA9548a::writeChannel(uint8_t tca_id, char channel_byte) {
    writeByte(tca_id, channel_byte);
}

void TCA9548a::closeChannel(uint8_t tca_id) {
    writeByte(tca_id, 0x00);
}

void TCA9548a::writeByte(uint8_t addr, char byte) {
    i2c.write(TCA_ADDR[addr]<<1, &byte, 1);
}

void TCA9548a::cbHandler(int events) {

}