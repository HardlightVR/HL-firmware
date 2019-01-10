/*
Copyright Lucian Copeland, 2016
Rewritten by Tom Moxon, 2017

MIT License
*/


/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef HL_INSTRUCTION_H_
#define HL_INSTRUCTION_H_

/** @defgroup API The Instruction API */
/** 
   @par
    \n          The Hardlight Instruction dispatcher is I/O independent
    \n          and written to use only buffers(pointers) for input and output.
    \n          This allows for easy testing, and to change the backend more
    \n          easily (i.e. to support bluetooth, Wi-Fi, native USB, etc)
    \n           
    \n @param       *inData  Input Instruction Packet to be decoded
    \n @param       *packet  Return Replay Packet, preformatted by the calling function
   \n
   \n                    On entry *packet points to a 16 byte buffer that has
   \n                    been pre-initialized to "success reply" packet.
   \n                    If the instruction modifies the packet (i.e. failure)
   \n                    or returns data, it will set packet[0] to zero 
   \n                    to indicate that the buffer is dirty and needs to be flushed (sent).
   \n                    Caller will test packet[0] and if equal to zero,
   \n                    the caller will restore the header (packet[0]) to 0x24,
   \n                    and then send the packet through whatever port is selected
   \n                    at the time, either PC or BLTE (or native USB, Wi-Fi, etc.)
   \n                    
   @par                 
   \n       Host Packet
   \n       inData[00]   : 0x24 (header) '$'
   \n       inData[01]   : 0x02 (header)
   \n       inData[02]   : Instruction Code
   \n       inData[03]   : PACKETID
   \n       inData[4-12] : (parameters)
   \n       inData[13]   : 0XFF
   \n       inData[14]   : 0x0D '\r'
   \n       inData[15]   : 0x0A '\n'
   \n
   \n       Reply Packet
   \n       packet[0] : 0x24
   \n       
   \n       packet[0]   :  0x24  (set to zero to indicate dirty/changed)
   \n       packet[1]   :  0x02 (header)
   \n       packet[2]   :  Return Code           
   \n       packet[3]   :  parameter
   \n       packet[4]   :  parameter
   \n       packet[5]   :  parameter
   \n       packet[6]   :  parameter
   \n       packet[7]   :  parameter
   \n       packet[8]   :  parameter
   \n       packet[9]   :  parameter
   \n       packet[10]  :  parameter
   \n       packet[11]  :  parameter
   \n       packet[12]  :  PACKETID
   \n       packet[13]  :  0x00
   \n       packet[14]  :  0x0D '\r'
   \n       packet[15]  :  0x0A '\n'          \n
    @par
   \n       For more information refer to :
   \n       https://os.mbed.com/teams/NullSpace-VR/code/HL_MKIII_Firmware_Dev/wiki/Introduction-to-Packets
   \n
   \n
         
  ******************************************************************************
  */

//* Instruction Return Codes                                                    */
#define HL_INS_RET_FLUSH        0x00
#define HL_INS_RET_VERSION      0x01
#define HL_INS_RET_PING         0x02
#define HL_INS_RET_INITMSG      0x03
#define HL_INS_RET_DRV_DATA     0x15
#define HL_INS_RET_IMU_DATA     0x33
#define HL_INS_RET_OVERFLOW     0x34

/**  
 *  Instruction Class
 *  Assigns instructions to the relevant tasks or variables.
 *  Converts the input array into meaningful parameters per instruction.
 *  Returns a pre-formated "ready packet" where required. 
 *
 *  @see API 
 */
class Instruction {
    public:
        Instruction();
    /**
        @ingroup API
        @param *inData pointer to an input character buffer
        @param *packet pointer to an output return buffer
    */               
    void DISPATCH(char *inData, char *packet);
        
        // Instruction List
        // ================================================================
        // DEVICE INSTRUCTIONS -- "DEV" -- 0x00-0x0F
        // ================================================================
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void NOP(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_VERSION(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_PING(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_UUID(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_CONFIG(char *inData, char *packet);        
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_LED(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_STATUS_LED(char *inData, char *packet);
        
        // ================================================================
        // MOTOR INSTRUCTIONS -- "MTR" -- 0x10-0x1F
        // ================================================================
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_MTR_STATUS(char *inData, char *packet);            //0x10
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_INIT(char *inData, char *packet);              //0x11
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_INIT_ALL(char *inData, char *packet);          //0x12
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_PLAY_EFFECT(char *inData, char *packet);       //0x13
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_DATA(char *inData, char *packet);              //0x14
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_MTR_DATA(char *inData, char *packet);              //0x15
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_LOAD_CONTINUOUS(char *inData, char *packet);   //0x16
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_HALT_SINGLE(char *inData, char *packet);       //0x17
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_STREAM_CONTINUOUS(char *inData, char *packet); //0x18
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_PLAY_CONTINUOUS(char *inData, char *packet);   //0x19
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_PLAY_AUDIO(char *inData, char *packet);        //0x1A
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_INTRIGMODE(char *inData, char *packet);        //0x1B  
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_RTPMODE(char *inData, char *packet);           //0x1C
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_PLAY_RTP(char *inData, char *packet);          //0x1D
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_WAVEFORM(char *inData, char *packet);          //0x1E             
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_GO(char *inData, char *packet);                //0x1E
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_MTR_STOP_AUDIO(char *inData, char *packet);        //0x22 
                
        // ================================================================
        //TRACKING INSTRUCTIONS
        // ================================================================
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_TRK_STATUS(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_TRK_INIT(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_TRK_INIT_ALL(char *inData, char *packet);  
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_TRK_ENABLE(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void SET_TRK_DISABLE(char *inData, char *packet);      
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_TRK_DATA(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_TRK_GRAVITY(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_TRK_COMPASS(char *inData, char *packet);
        /**
            @ingroup API
            @param *inData pointer to an input character buffer
            @param *packet pointer to an output return buffer
        */               
        void GET_TRK_UUID(char *inData, char *packet);
         
    protected:      
    private:

};


#endif