/*
DRV2605.cpp
Based off the Arduino DRV2605 library by LadyAda
Copyright (c) LadyAda
Licensed for free use under MIT License

Rewritten for mbed & non blocking functionality by Lucian Copeland, 2016
Rewritten again by Tom Moxon, 2017
*/

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/

#include "mbed.h"

#define DRV2605_ADDR              0x5A

#define DRV2605_REG_STATUS        0x00
#define DRV2605_REG_MODE          0x01

#define DRV2605_MODE_INTTRIG      0x00
#define DRV2605_MODE_EXTTRIGEDGE  0x01
#define DRV2605_MODE_EXTTRIGLVL   0x02
#define DRV2605_MODE_PWMANALOG    0x03
#define DRV2605_MODE_AUDIOVIBE    0x04
#define DRV2605_MODE_REALTIME     0x05
#define DRV2605_MODE_DIAGNOS      0x06
#define DRV2605_MODE_AUTOCAL      0x07

#define DRV2605_REG_RTPIN         0x02
#define DRV2605_REG_LIBRARY       0x03
#define DRV2605_REG_WAVESEQ1      0x04
#define DRV2605_REG_WAVESEQ2      0x05
#define DRV2605_REG_WAVESEQ3      0x06
#define DRV2605_REG_WAVESEQ4      0x07
#define DRV2605_REG_WAVESEQ5      0x08
#define DRV2605_REG_WAVESEQ6      0x09
#define DRV2605_REG_WAVESEQ7      0x0A
#define DRV2605_REG_WAVESEQ8      0x0B

#define DRV2605_REG_GO            0x0C
#define DRV2605_REG_OVERDRIVE     0x0D
#define DRV2605_REG_SUSTAINPOS    0x0E
#define DRV2605_REG_SUSTAINNEG    0x0F
#define DRV2605_REG_BRAKE         0x10
#define DRV2605_REG_AUDIOVIBECTRL 0x11
#define DRV2605_REG_AUDIOMINLVL   0x12
#define DRV2605_REG_AUDIOMAXLVL   0x13
#define DRV2605_REG_AUDIOMINDRV   0x14
#define DRV2605_REG_AUDIOMAXDRV   0x15
#define DRV2605_REG_RATEDV        0x16
#define DRV2605_REG_CLAMPV        0x17
#define DRV2605_REG_AUTOCALCOMP   0x18
#define DRV2605_REG_AUTOCALEMP    0x19
#define DRV2605_REG_FEEDBACK      0x1A
#define DRV2605_REG_CONTROL1      0x1B
#define DRV2605_REG_CONTROL2      0x1C
#define DRV2605_REG_CONTROL3      0x1D
#define DRV2605_REG_CONTROL4      0x1E
#define DRV2605_REG_RFU1          0x1F
#define DRV2605_REG_RFU2          0x20
#define DRV2605_REG_VBAT          0x21
#define DRV2605_REG_LRARESON      0x22

/** DRV2605 Class
 *  Library for controlling the DRV2605 motor driver
 */
class DRV2605 {
    public:
        DRV2605(PinName sda, PinName scl);
        
        void writeRegister8(char reg, char val);
        void writeRegister8_nb(uint8_t reg, uint8_t val);
        //uint8_t readRegister8(uint8_t reg);
        
        //blocking
        bool begin(void);
        void setWaveform(uint8_t slot, uint8_t w);
        void selectLibrary(uint8_t lib);
        void go(void);
        void halt(void);
        void setMode(uint8_t mode);
        void playEffect(int effect);
        void playAudio(uint8_t vibectrl, uint8_t minlevel, uint8_t maxlevel, uint8_t mindrv, uint8_t maxdrv);
        void stopAudio(void);
        char readRegister8(char reg);
               
        //non blocking
        void go_nb(void);
        void cbHandler(int events);
    protected:
        event_callback_t _internalCallback;
        uint8_t  _address;
        uint8_t  _rx_buf[8];
        uint8_t  _tx_buf[2];
    private:
        I2C i2c;
        
};