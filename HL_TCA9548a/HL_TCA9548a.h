/*
Copyright Lucian Copeland, 2016
Rewritten by Tom Moxon, 2017

MIT License
*/

#include "mbed.h"

//#define TCA_ADDR_1 0x70

/** TCA9548a Class
 *  Library for the TCA9548a 8 channel I2C multiplexer
 */
class TCA9548a {
    public:
        TCA9548a(PinName sda, PinName scl);
        
        //uint8_t readRegister8(uint8_t reg);
        void openChannel(uint8_t tca_id, uint8_t channel);
        void writeChannel(uint8_t tca_id, char channel_byte);
        void closeChannel(uint8_t tca_id);
        void cbHandler(int events);
        
    private:
        I2C i2c;
        void writeByte(uint8_t addr, char byte);
        //declare all splitter addresses (7 possible, 0x77 used for MPUs)
        static uint8_t TCA_ADDR[8];
        event_callback_t _internalCallback;
        uint8_t  _rx_buf[8];
        uint8_t  _tx_buf[2];
};