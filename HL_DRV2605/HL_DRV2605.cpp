/*
DRV2605.h
Based off the Arduino DRV2605 library by LadyAda
Copyright (c) LadyAda
Licensed for free use under MIT License

Rewritten for mbed & non blocking functionality by Lucian Copeland, 2016
Rewritten again by Tom Moxon, 2017
*/

#include "HL_DRV2605.h"

DRV2605::DRV2605(PinName sda, PinName scl): i2c(sda, scl)
{
    _internalCallback.attach(this, &DRV2605::cbHandler);
}

bool DRV2605::begin() {
  writeRegister8(DRV2605_REG_MODE, 0x00);    //! DRV2605 - out of standby  
  writeRegister8(DRV2605_REG_RTPIN, 0x00);   //! DRV2605 - no real-time-playback
  writeRegister8(DRV2605_REG_WAVESEQ1, 1);   //! DRV2605 - strong click
  writeRegister8(DRV2605_REG_WAVESEQ2, 0);   //! DRV2605 - strong click
  writeRegister8(DRV2605_REG_OVERDRIVE, 0);  //! DRV2605 - no overdrive
  writeRegister8(DRV2605_REG_SUSTAINPOS, 0); //! DRV2605 - Sustain Time Offset
  writeRegister8(DRV2605_REG_SUSTAINNEG, 0); //! DRV2605 - Sustain Time Offset
  writeRegister8(DRV2605_REG_BRAKE, 0);      //! DRV2605 - Braking Time Offset
  
  writeRegister8(DRV2605_REG_AUDIOVIBECTRL, 0x00);  //! audio to vibe control   (0x00) 10ms, 100hz
  writeRegister8(DRV2605_REG_AUDIOMINLVL,   0x00);  //! audio-to-vibe min level (0x19)
  writeRegister8(DRV2605_REG_AUDIOMAXLVL,   0x40);  //! audio-to-vibe max level (0xFF) (0x64?)
  writeRegister8(DRV2605_REG_AUDIOMINDRV,   0x20);  //! audio-to-vibe min drive (0x19)
  writeRegister8(DRV2605_REG_AUDIOMAXDRV,   0x64);  //! audio-to-vibe max drive (0xFF)
  
  if ((readRegister8(DRV2605_REG_STATUS) == 0xE0) || (readRegister8(DRV2605_REG_STATUS)== 0x60)) {
    return true;
  } else {
    return false;  
  }
}

void DRV2605::setWaveform(uint8_t slot, uint8_t w) {
  writeRegister8(DRV2605_REG_WAVESEQ1+slot, w);
}

void DRV2605::selectLibrary(uint8_t lib) {
  writeRegister8(DRV2605_REG_LIBRARY, lib);
}

void DRV2605::go() {
  writeRegister8(DRV2605_REG_GO, 1);
}

void DRV2605::halt() {
  writeRegister8(DRV2605_REG_GO, 0);
}

void DRV2605::setMode(uint8_t mode) {
  writeRegister8(DRV2605_REG_MODE, mode);
}

//non blocking write register
void DRV2605::writeRegister8_nb(uint8_t reg, uint8_t val) {
    // use i2c
    _tx_buf[0] = reg;
    _tx_buf[1] = val;
    
//    if(i2c.transfer(DRV2605_ADDR<<1, (char*)_tx_buf, 2, (char*)_rx_buf, 0, _internalCallback, I2C_EVENT_ALL)==0) {
//        return;
//    }
}

//blocking write register
void DRV2605::writeRegister8(char reg, char value){
    char buff[2] = {reg, value};
    i2c.write(DRV2605_ADDR<<1, buff, 2);
}

char DRV2605::readRegister8(char reg) {
    char result;
    i2c.write(DRV2605_ADDR<<1, &reg, 1);  
    i2c.read(DRV2605_ADDR<<1, &result, 1);
    return result;
}

//full effect play with blocking i2c
void DRV2605::playEffect(int effect) {
    setMode(DRV2605_MODE_INTTRIG);
    setWaveform(0, effect);  // play effect 
    setWaveform(1, 0);  // end waveform
    go();
}

//audio play with blocking i2c
void DRV2605::playAudio(uint8_t vibectrl, uint8_t minlevel, uint8_t maxlevel, uint8_t mindrv, uint8_t maxdrv) {
    setMode(DRV2605_MODE_AUDIOVIBE);
    writeRegister8(DRV2605_REG_CONTROL1, 0xB3);           //! set to ac coupled input, with a 0.9V bias (was 0x20)
    writeRegister8(DRV2605_REG_CONTROL3, 0xA3);           //! set to 4%, supply compensation disabled, analog input (was 0xA3)
    writeRegister8(DRV2605_REG_AUDIOVIBECTRL, vibectrl);  //! audio to vibe control   (0x00) 10ms, 100hz
    writeRegister8(DRV2605_REG_AUDIOMINLVL, minlevel);    //! audio-to-vibe min level (0x19)
    writeRegister8(DRV2605_REG_AUDIOMAXLVL, maxlevel);    //! audio-to-vibe max level (0xFF) (0x64?)
    writeRegister8(DRV2605_REG_AUDIOMINDRV, mindrv);      //! audio-to-vibe min drive (0x19)
    writeRegister8(DRV2605_REG_AUDIOMAXDRV, maxdrv);      //! audio-to-vibe max drive (0xFF)
    go();
}

//stop audio play with blocking i2c
void DRV2605::stopAudio(void) {
    setMode(DRV2605_MODE_INTTRIG);
    // Restore power up defaults 
    writeRegister8(DRV2605_REG_CONTROL1,      0x93);      //! set to ac coupled input, with a 0.9V bias
    writeRegister8(DRV2605_REG_CONTROL3,      0xA0);      //! set to 4%, supply compensation disabled, analog input
    writeRegister8(DRV2605_REG_AUDIOVIBECTRL, 0x05);      //! audio to vibe control   (0x03) 10ms, 100hz
    writeRegister8(DRV2605_REG_AUDIOMINLVL,   0x19);      //! audio-to-vibe min level (0x19)
    writeRegister8(DRV2605_REG_AUDIOMAXLVL,   0xFF);      //! audio-to-vibe max level (0xFF) (0x64?)
    writeRegister8(DRV2605_REG_AUDIOMINDRV,   0x19);      //! audio-to-vibe min drive (0x19)
    writeRegister8(DRV2605_REG_AUDIOMAXDRV,   0xFF);      //! audio-to-vibe max drive (0xFF)
    go();
}
//non blocking Go call
void DRV2605::go_nb() {
    writeRegister8_nb(DRV2605_REG_GO, 1);
}

void DRV2605::cbHandler(int events) {
// cb interrupts not implemented
}
