/**
  ******************************************************************************
   @defgroup API The Instruction API
   
   @file        HL_Instruction.cpp
   @author      Lucian Copeland <lucian@nullspacevr.com>, December 2016
   @author      Tom Moxon <tom@moxon.com>, September 2017
   @version     1.0.0
   @brief       Define all the MKIII Suit Instructions
   MIT License
  
   @section     HL_Instruction HL_Instruction
   @par
    \n          The Hardlight Instruction dispatcher is I/O independent
    \n          and written to use only buffers(pointers) for input and output.
    \n          This allows for easy testing, and to change the backend more
    \n          easily (i.e. to support bluetooth, Wi-Fi, native USB, etc)
                
   @param       *inData  Input Instruction Packet to be decoded
   @param       *packet  Return Replay Packet, preformatted by the calling function
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

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "HL_Platform.h" 
#include "HL_Instruction.h"

Instruction::Instruction() {
}

//! Dispatch the instruction
void Instruction::DISPATCH(char *inData, char *packet) {
     //! Test the Instruction Header (footer?)
     if ( (inData[0] == 0x24) && (inData[1] == 0x02)) {   
            char instruction = inData[2];
            switch (instruction) {
            case 0x00 :
                NOP(inData, packet);
                break;    
            case 0x01 :
                GET_VERSION(inData, packet);
                break;
            case 0x02:
                GET_PING(inData, packet);
                break;
            case 0x03:
                GET_UUID(inData, packet);
                break;
            case 0x04:
                GET_CONFIG(inData, packet);   
            case 0x05:
                SET_LED(inData, packet);
                break;
            case 0x06: 
                SET_STATUS_LED(inData, packet);
                break;
            case 0x10:
                GET_MTR_STATUS(inData, packet);
                break;
            case 0x11: 
                SET_MTR_INIT(inData, packet);
                break;
            case 0x12:
                SET_MTR_INIT_ALL(inData, packet);                                 
            case 0x13:
                SET_MTR_PLAY_EFFECT(inData, packet);
                break;
            case 0x14:
                SET_MTR_DATA(inData, packet);
                break;
            case 0x15:
                GET_MTR_DATA(inData, packet);
                break;
            case 0x16:
                SET_MTR_LOAD_CONTINUOUS(inData, packet);
                break;   
            case 0x17:
                SET_MTR_HALT_SINGLE(inData, packet);
                break;
            case 0x18:
                SET_MTR_STREAM_CONTINUOUS(inData, packet);
                break;    
            case 0x19:
                SET_MTR_PLAY_CONTINUOUS(inData, packet);
                break;
            case 0x1A:
                SET_MTR_PLAY_AUDIO(inData, packet);
                break;
            case 0x1B:
                SET_MTR_INTRIGMODE(inData, packet);
                break;
            case 0x1C:
                SET_MTR_RTPMODE(inData, packet);
                break;
            case 0x1D:
                SET_MTR_PLAY_RTP(inData, packet);
                break;
            case 0x1E:
                SET_MTR_WAVEFORM(inData, packet);
                break;
            case 0x1F:
                SET_MTR_GO(inData, packet);
                break;
            case 0x20:
                    //deprecated see 0x10
                break;        
            case 0x21:
                    //deprecated see 0x10
                break;
            case 0x22:
                SET_MTR_STOP_AUDIO(inData, packet);
                break;                               
            case 0x30:
                GET_TRK_STATUS(inData, packet);
                break;    
            case 0x31:   
                SET_TRK_INIT(inData, packet);
                break;
            case 0x32:
                SET_TRK_INIT_ALL(inData, packet);
                break;  
            case 0x33:
                 GET_TRK_DATA(inData, packet);
                break;
            case 0x34:
                SET_TRK_ENABLE(inData, packet);
                break;
            case 0x35:
                SET_TRK_DISABLE(inData, packet);
                break;
            case 0x36:
                 GET_TRK_GRAVITY(inData, packet); 
                break;
            case 0x37:
                GET_TRK_COMPASS(inData, packet);
                break;
            case 0x38:
                GET_TRK_UUID(inData, packet);
                break;
            default: 
                break;
            }    
        }  // else { ignore bad instruction packet }
}
//*---------------------------------------------------------------*/ 
//! 0x00 No Operation - No Return  (Reserved) \n
void Instruction::NOP(char *inData, char *packet) {
}
 
//! 0x01 Get the Version of the suit \n
void Instruction::GET_VERSION(char *inData, char *packet) { 
    
    packet[2] = 0x01;                      // Return type = Version
    packet[3] = HL_SUIT_VERSION;           // HL Suit  Revision
    packet[4] = HL_PLATFORM_MAJOR_VERSION; // Platform Major
    packet[5] = HL_PLATFORM_MINOR_VERSION; // Platform Minor  
    packet[6] = HL_PLATFORM_REVISION;      // Platform sub-revision
    packet[12] = inData[3];                // PACKET ID
    //flush the return packet buffer  
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x02 Get the Status of the suit   \n
void Instruction::GET_PING(char *inData, char *packet) {
    packet[2] = 0x02;                       // Return type = Ping
    //! TODO : Add suit general failure return
    packet[12] = inData[3];                // PACKET ID     
    //flush the return packet buffer    
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x03  Get the Unique ID of the Suit \n
void Instruction::GET_UUID(char *inData, char *packet) {
    packet[2]  = 0x03;                      // Return type = InitStatus?
    packet[3]  = Platform_UniqueID[0];
    packet[4]  = Platform_UniqueID[1];
    packet[5]  = Platform_UniqueID[2];
    packet[6]  = Platform_UniqueID[3];
    packet[7]  = Platform_UniqueID[4];
    packet[8]  = Platform_UniqueID[5];
    packet[9]  = Platform_UniqueID[6];
    packet[10] = Platform_UniqueID[7];
    packet[11] = Platform_UniqueID[8];  
    packet[12] = inData[3];                // PACKET ID
    //flush the return packet buffer    
    packet[0] = HL_INS_RET_FLUSH;
}
//! 0x04 Get a Suit Configuration Table Entry \n
void Instruction::GET_CONFIG(char *inData, char *packet){
    // return zero for now
    packet[2]  = 0x04;               // Return type = Get Config
    packet[12] = inData[3];                // PACKET ID
    //flush the return packet buffer    
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x05 Control the Chest IMU LEDS \n
void Instruction::SET_LED(char *inData, char *packet) {
    float r = ((inData[4] << 8) | inData[5]) / (float)16384.0;
    float g = ((inData[6] << 8) | inData[7]) / (float)16384.0;
    float b = ((inData[8] << 8) | inData[9]) / (float)16384.0;
    Platform_LED_control(r,b,g);
}

//! 0x06 Control the Control Board Status LEDs \n
//! Status LED's are generally under "Platform" control, and will be overridden... \n
void Instruction::SET_STATUS_LED(char *inData, char *packet) {
    float r = ((inData[4] << 8) | inData[5]) / (float)16384.0;
    float g = ((inData[6] << 8) | inData[7]) / (float)16384.0;
    float b = ((inData[8] << 8) | inData[9]) / (float)16384.0;
    Platform_Status_LED_control(r,b,g);
}

// ================================================================
// MOTOR INSTRUCTIONS -- "MTR" -- 0x10-0x1F
// ================================================================
//! 0x10 - GET_MTR_STATUS \n
void Instruction::GET_MTR_STATUS(char *inData, char *packet) {
    packet[2] = 0x10;
    packet[3] = platform_motor_table[inData[4]]; 
    packet[4] = inData[4];
    packet[12] = inData[3];                // PACKET ID
    // flush the packet
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x11 - SET_MTR_INIT \n
void Instruction::SET_MTR_INIT(char *inData, char *packet) {
    Platform_Motor_Init(inData[4]);
}

//! 0x12 - SET_MTR_INIT_ALL \n
void Instruction::SET_MTR_INIT_ALL(char *inData, char *packet) {
    Platform_Motor_Init_All();
}

//! 0x13 - SET_MTR_PLAY_EFFECT \n
void Instruction::SET_MTR_PLAY_EFFECT(char *inData, char *packet) {
    Platform_Motor_Play_Effect(inData[4], inData[5]);
}

//! 0x14 - SET_MTR_DATA \n
void Instruction::SET_MTR_DATA(char *inData, char *packet) {
    Platform_Motor_Set_Register(inData[4], inData[5], &inData[3]);  // drv/motor data byte
}

//! 0x15 - GET_MTR_DATA \n
void Instruction::GET_MTR_DATA(char *inData, char *packet){
    packet[2] = 0x15;           // packet return code
    Platform_Motor_Get_Register(inData[4], inData[5], &packet[3]);  // drv/motor data byte
    packet[4] = inData[4];      // motor-drv:unit number
    packet[5] = inData[5];      //
    packet[12] = inData[3];                // PACKET ID
     
    // flush the packet
    packet[0] = HL_INS_RET_FLUSH;      
}

//! 0x16 - MTR_LOAD_CONTINUOUS \n
void Instruction::SET_MTR_LOAD_CONTINUOUS(char *inData, char *packet) {
}

//! 0x17 - SET_MTR_HALT_SINGLE \n
void Instruction::SET_MTR_HALT_SINGLE(char *inData, char *packet) {
   Platform_Motor_Stop(inData[4]);
}

//! 0x18 - SET_MTR_STREAM_CONTINUOUS \n
void Instruction::SET_MTR_STREAM_CONTINUOUS(char *inData, char *packet) {
}

//! 0x19 -SET_MTR_PLAY_CONTINUOUS \n
void Instruction::SET_MTR_PLAY_CONTINUOUS(char *inData, char *packet) {
    Platform_Motor_Play_Continuous(inData[4],inData[5]);
}

//! 0x1A - SET_MTR_PLAY_AUDIO \n
void Instruction::SET_MTR_PLAY_AUDIO(char *inData, char *packet) {
    Platform_Motor_Play_Audio(inData[4], inData[5], inData[6], inData[7], inData[8], inData[9]);
}

//! 0x1B - SET_MTR_INTRIGMODE \n
void Instruction::SET_MTR_INTRIGMODE(char *inData, char *packet) {
    Platform_Motor_Set_INTTRIGMODE(inData[4]);
}

//! 0x1C - SET_MTR_RTPMODE \n
void Instruction::SET_MTR_RTPMODE(char *inData, char *packet) {
    Platform_Motor_Set_RTPMODE(inData[4]);
}

//! 0x1D - SET_MTR_PLAY_RTP \n
void Instruction::SET_MTR_PLAY_RTP(char *inData, char *packet) {
    Platform_Motor_Play_RTP(inData[4], inData[5]);
}

//! 0x1E - SET_MTR_WAVEFORM \n
void Instruction::SET_MTR_WAVEFORM(char *inData, char *packet) {
    Platform_Motor_Set_Waveform(inData[4], inData[5]);
}

//! 0x1F - SET_MTR_GO \n
void Instruction::SET_MTR_GO(char *inData, char *packet) {
    Platform_Motor_Set_Go(inData[4]);
}


//! 0x22 - SET_MTR_STOP_AUDIO \n
void Instruction::SET_MTR_STOP_AUDIO(char *inData, char *packet) {
    Platform_Motor_Stop_Audio(inData[4]);
}

// ================================================================
//TRACKING INSTRUCTIONS -- "TRK" -- 0x30-0xFF
// ================================================================

//!0x30 - GET_TRK_STATUS
void Instruction::GET_TRK_STATUS(char *inData, char *packet) {
    packet[2] = 0x30;
    packet[3] = platform_sensor_table[inData[4]]; 
    packet[4] = inData[4];
    packet[12] = inData[3];                // PACKET ID
    // flush the packet
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x31 - SET_TRK_INIT \n
void Instruction::SET_TRK_INIT(char *inData, char *packet) {
    Platform_IMU_Init(inData[4]);
}

//! 0x32 - SET_TRK_INIT_ALL \n
void Instruction::SET_TRK_INIT_ALL(char *inData, char *packet) {
    if (Platform_IMU_Init_All() != PLATFORM_SUCCESS) {
        packet[2] = 0x32 | 0x80; // failure
    } 
}

//! 0x33 - GET_TRK_DATA \n
void Instruction::GET_TRK_DATA(char *inData, char *packet) {
    char readBuffer[8];
    memset(readBuffer,0,8); //buffer must be empty
    Platform_IMU_Get_Data(inData[4], readBuffer);
    
    // IMU data has to be rearranged a bit
    packet[2] = 0x33;
    packet[3] = readBuffer[1];
    packet[4] = readBuffer[0];
    packet[5] = readBuffer[3];
    packet[6] = readBuffer[2];
    packet[7] = readBuffer[5];
    packet[8] = readBuffer[4];
    packet[9] = readBuffer[7];
    packet[10] = readBuffer[6];
    packet[11] = inData[4];     //last data byte returns the IMU ID
    packet[12] = inData[3];                // PACKET ID
    // flush the packet
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x34 - SET_TRK_ENABLE \n
void Instruction::SET_TRK_ENABLE(char *inData, char *packet) {
    Platform_Tracker_Flag = true;
}

//! 0x35 - SET_TRK_DISABLE \n
void Instruction::SET_TRK_DISABLE(char *inData, char *packet) {
    Platform_Tracker_Flag = false;
}

//! 0x36 - GET_TRK_GRAVITY \n
void Instruction::GET_TRK_GRAVITY(char *inData, char *packet) {
    char readBuffer[8];
    memset(readBuffer,0,8); //buffer must be empty
    Platform_IMU_Get_Gravity(inData[4], readBuffer);
    
    // IMU data has to be rearranged a bit
    packet[2] = 0x36;
    packet[3] = readBuffer[1];
    packet[4] = readBuffer[0];
    packet[5] = readBuffer[3];
    packet[6] = readBuffer[2];
    packet[7] = readBuffer[5];
    packet[8] = readBuffer[4];
    packet[9] = readBuffer[7];
    packet[10] = readBuffer[6];
    packet[11] = inData[4];     //last data byte returns the IMU ID
    packet[12] = inData[3];                // PACKET ID
    // flush the packet
    packet[0] = HL_INS_RET_FLUSH;
}
//! 0x37 - GET_TRK_COMPASS \n
void Instruction::GET_TRK_COMPASS(char *inData, char *packet) {
    char readBuffer[8];
    memset(readBuffer,0,8); //buffer must be empty
    Platform_IMU_Get_Compass(inData[4], readBuffer);
    
    // IMU data has to be rearranged a bit
    packet[2] = 0x37;
    packet[3] = readBuffer[1];
    packet[4] = readBuffer[0];
    packet[5] = readBuffer[3];
    packet[6] = readBuffer[2];
    packet[7] = readBuffer[5];
    packet[8] = readBuffer[4];
    packet[9] = readBuffer[7];
    packet[10] = readBuffer[6];
    packet[11] = inData[4];     //last data byte returns the IMU ID
    packet[12] = inData[3];                // PACKET ID
    // flush the packet
    packet[0] = HL_INS_RET_FLUSH;
}

//! 0x38 - GET_TRK_UUID \n
void Instruction::GET_TRK_UUID(char *inData, char *packet) {
  Platform_IMU_Get_UniqueID(inData[4], &packet[3]);
  packet[2] = 0x38;
  // flush the packet
  packet[0] = HL_INS_RET_FLUSH;  
}


