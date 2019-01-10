/*
Copyright Lucian Copeland, 2016
Rewritten by Tom Moxon, 2017

MIT License
*/

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "HL_Platform.h"
#include "HL_Instruction.h"

/*-----------------------------------------------------------------------------*/
/** @defgroup HL_platform HL_platform
  * @{
  */
//! Define that Hardlight Hardware Platform
//! Serial ports are buffered using MODSERIAL library
//
MODSERIAL pc(HL_USB_TXD_PIN, HL_USB_RXD_PIN);
MODSERIAL btle(HL_BTLE_TXD_PIN, HL_BTLE_RXD_PIN);
Serial    dbg(HL_DBG_TXD_PIN,   HL_DBG_RXD_PIN);

//! I2C Multiplexors for Motor Driver Bus and Tracking Sensor Bus
TCA9548a d_tca(HL_I2C2_SDA_PIN,HL_I2C2_SCL_PIN); // motor driver tca
TCA9548a t_tca(HL_I2C1_SDA_PIN,HL_I2C1_SCL_PIN); // tracking sensor tca

//! Bosch BNO055 9DOF IMU Unit Driver
BNO055   bno(HL_I2C1_SDA_PIN,HL_I2C1_SCL_PIN);

//! DRV2605 Haptic Motor Unit Driver
DRV2605 drv(HL_I2C2_SDA_PIN,HL_I2C2_SCL_PIN);
 
//! Control Board Status LEDs
//! Nota Bene : mbed didn't like these as PWMOut, failed to function
DigitalOut led_red(PC_13);
DigitalOut led_green(PC_14);
DigitalOut led_blue(PC_15);

//! Suit Chest Sensor LED's
PwmOut  pwm_red(PA_6); 
PwmOut  pwm_green(PA_7);
PwmOut  pwm_blue(PA_5);

//! FTDI USB-UART Power Good Monitor \n
//! A rising edge on this signal indicates that USB has been disconnected. \n
//! A low leve indicates that the FTDI USB-UART is connected to the host. \n
InterruptIn pcUSB_Disconnect(HL_USB_CON_PIN);       /** Low = USB is connected */

//! Timeout counter for I/O Operations
//! We use a "side-effect" of the Timeout function,
//! as attaching the handler function resets Timeout timers.
//! so we "reattach" to keep the timer from timing out
Timeout ioWatchdog; /* we'll set a two second timeout */

//! Instructions Class
Instruction inst;

//! Tracker Timer 
Ticker Tracker; 

/*-----------------------------------------------------------------------------*/
/* Global variables                                                            */
/*-----------------------------------------------------------------------------*/
Platform_RetCode_t  Platform_RetVal = PLATFORM_SUCCESS;             // return status
Platform_Mode_t     Platform_Mode   = PLATFORM_START;               // current operating mode
Platform_RetCode_t  Platform_Status = PLATFORM_SUCCESS;             // current status
uint8_t             Platform_Host_Port = HL_PLATFORM_IO_TEST;       // start with host interface as internal
char                Platform_UniqueID[33];                          // ARM Cortex Serial Number Register String (32 + null)
char                Platform_TestLevel = HL_TEST;                   // self-test/diagnostic level
uint8_t             Platform_TestCount;                             // self-test/diagnostic counter
// Time/RTC
time_t              Platform_Time;                                  // Time stamp holder (int32 on m4 - Y2038 issue)
float               Platform_Frac_Secs;                             // Fractional Seconds
RTC_HandleTypeDef   Platform_RTC;                                   // RTC handler
bool                Platform_Tracker_Flag = false;                  // True if Tracker is enabled/running
bool                Platform_Tracker_Status = false;                // True if Tracker is ready
uint16_t            Platform_Tracker_Count = 0;                     // Tracker Running Count
uint16_t            Platform_Tracker_Unit = 0;                      // Tracker Running Unit

bool                Platform_Blink_Flag = false;
float               Platform_Blink_Red = 0;
float               Platform_Blink_Green = 1;
float               Platform_Blink_Blue = 0;

/* Suit Instruction Protocol Buffers and Flags                      */
bool                platform_pc_newline   = false;                  // newline-packet ready flag
uint16_t            platform_pc_charCnt   = 0;                      // bytes received count (from RX IRQ)
uint16_t            platform_pc_buf_flag = 0;                       // ping-pong buffer select
char                platform_pc_cmd_packet[16];                     // ping-pong packet buffer

bool                platform_btle_newline = false;
uint16_t            platform_btle_charCnt   = 0;
uint16_t            platform_btle_buf_flag = 0;
char                platform_btle_cmd_packet[16];

bool                platform_dbg_newline  = false;
uint16_t            platform_dbg_charCnt   = 0;
uint16_t            platform_dbg_buf_flag = 0;
char                platform_dbg_cmd_packet[16];

/* scorepad(s) for populated motors and sensors, with status */
char platform_motor_table[HL_MOTOR_TOTAL];
char platform_motor_unit;
char platform_sensor_table[HL_IMU_TOTAL];
char platform_sensor_unit;

char cmdTRK_packet[] = {0x24,0x02,0x33,0,0,0,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'};
char cmdRET_packet[] = {0x24,0x02,0x00,0,0,0,0,0,0,0,0,0,0,0,'\r','\n'};
char suit_cmd_packet[16];
char suit_ret_packet[16];

/*-----------------------------------------------------------------------------*/
/* Platform Test Packets                                                       */
/*-----------------------------------------------------------------------------*/
/* Production mode Tests */
#if (HL_DEBUG == 0) 
    #define NUM_TESTS 2
    //!  test_packets[NUM_TESTS][PACKET_SIZE]; \n
    //!  this is a bit brute force in that every packet requires a return, 
    //!  even it is default packet, add a flag for that later, etc. to reduce size
    char test_packets[NUM_TESTS][16] =  {
                    {0x24,0x02,0x05,0,0xFF,0xFF,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'},
                    {0x24,0x02,0x05,0,0,0,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'}
                };
    char test_returns[NUM_TESTS][16] =  {
                    {0x24,0x02,0x00,0,0,0,0,0,0,0,0,0,0,0,'\r','\n'},
                    {0x24,0x02,0x00,0,0,0,0,0,0,0,0,0,0,0,'\r','\n'}
                };                
#endif
/* Debug mode Tests */
#if (HL_DEBUG != 0) 
    #define NUM_TESTS 4
    char test_packets[NUM_TESTS][16] =  {
                    // cmd 0x01
                    {0x24,0x02,0x00,0,0,0,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'},
                    {0x24,0x02,0x01,0,0,0,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'},
                    {0x24,0x02,0x02,0,0,0,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'},
                    {0x24,0x02,0x03,0,0,0,0,0,0,0,0,0,0xFF,0xFF,'\r','\n'}
                };
    char test_returns[NUM_TESTS][16] =  {
                    {0x24,0x02,0x00,0,0,0,0,0,0,0,0,0,0,0,'\r','\n'},
                    {0x24,0x02,0x01,3,0,0,1,0,0,0,0,0,0,0,'\r','\n'},
                    {0x24,0x02,0x02,0,0,0,0,0,0,0,0,0,0,0,'\r','\n'},
                    {0x24,0x02,0x03,0x34,0x32,0x39,0x34,0x39,0x36,0x37,0x32,0x39,0,0,'\r','\n'}
                };                
#endif

/*-----------------------------------------------------------------------------*/
/* Platform Mode Functions                                                     */
/*-----------------------------------------------------------------------------*/
Platform_RetCode_t Platform_Start() {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;

    //! Start up procedure for Suit Platform components \n
    //! Start with all LED's On at first for an LED Test \n
    Platform_Status_LED_control(1,1,1);
    Platform_LED_control(1,1,1);

    //! Configure Motor and Status tables
    Platform_Configure();
    
    //! Set the pcUSB_Disconnect Interrupt for rising edge \n
    pcUSB_Disconnect.rise(&Platform_USB_Disconnect);
    //! Enable the pcUSB_Disconnect Interrupt \n
    pcUSB_Disconnect.enable_irq();

    // Get the processor serial number to use as the Suit UniqueID (UUID)
    Platform_Get_UniqueID(Platform_UniqueID);
    
    //! Initialize the Serial Port Components \n
    //! 1) "pc" is the host interface port (FTDI) \n
    pc.baud(HL_PC_BAUD);
    pc.set_flow_control ( SerialBase::RTSCTS, HL_USB_RTS_PIN, HL_USB_CTS_PIN); 
    pc.attach(&Platform_pc_rxCallback, MODSERIAL::RxIrq);
    //! 2) "btle" is the BMD-300 Bluetooth interface port (BTLE) \n
    btle.baud(HL_BTLE_BAUD);
    btle.set_flow_control ( SerialBase::RTSCTS, HL_BTLE_RTS_PIN, HL_BTLE_CTS_PIN); 
    btle.attach(&Platform_btle_rxCallback, MODSERIAL::RxIrq);
    //! 3) "dbg" is the debug interface port (UART6) \n
    //! In order not to impact performance or production code size, \n
    //! all debug messages should be guarded. \n
    #if (HL_DEBUG != 0) 
      dbg.baud(HL_DBG_BAUD);
      wait_ms(HL_MINWAIT);
      dbg.printf("\r\n------------------------------------\r\n");
      dbg.printf("MKIII : Platform : Version  : %d.%d.%d\r\n", HL_PLATFORM_MAJOR_VERSION, HL_PLATFORM_MINOR_VERSION, HL_PLATFORM_REVISION);      
      dbg.printf("MKIII : Platform : UniqueID : %s\r\n", Platform_UniqueID);      
      dbg.printf("MKIII : Platform : MODE : Start\r\n");
    #endif
    
    //! Initialize all the Suit Motors
    if ((Platform_RetVal = Platform_Motor_Init_All()) != PLATFORM_SUCCESS) {
      #if (HL_DEBUG != 0) 
       dbg.printf("MKIII : Platform : DRV  : Initialize All Motors failed!\r\n");
      #endif
   }

    //! Initialize all the Suit Tracking Sensors
    if ((Platform_RetVal = Platform_IMU_Init_All()) != PLATFORM_SUCCESS) {
      #if (HL_DEBUG != 0) 
       dbg.printf("MKIII : Platform : IMU  : Initialize All IMU Trackers failed!\r\n");
      #endif
   }
 
    // Initialize the RTC input clock, there is no 32.768khz Watch Crystal, so use LSI osc
    Platform_Start_RTC();
        
        // Nota Bene : STM32/mbed time functions hang the CPU!
        // Don't use this - it hangs...
        // Platform_Time = time(NULL);
        // No seriously, I had to write my own RTC api's to get them to work...
        
    // set the build time
    time_t Build_Time = Platform_Set_Build_Time(__DATE__,__TIME__);
    // Initialize the RTC to the build time  
    Platform_Set_RTC(Build_Time); 
    // see that it "took"        
        Platform_Get_RTC(&Platform_Time, &Platform_Frac_Secs);
   #if (HL_DEBUG != 0) 
        dbg.printf("MKIII : Platform : Build : Time   : %s \r", ctime(&Build_Time));
        dbg.printf("MKIII : Platform : RTC   : Time   : %s \r", ctime(&Platform_Time));   
        // see that the seconds "count"...
        wait(1);
        Platform_Get_RTC(&Platform_Time, &Platform_Frac_Secs);
        dbg.printf("MKIII : Platform : RTC   : Time   : %s \r", ctime(&Platform_Time)); 
        dbg.printf("MKIII : Platform : MBED  : Sysclk : %d Hz\r\n", SystemCoreClock);     
  #endif
  
    //! Initialize Tracker Interrupt (10Hz)
    Tracker.attach(&Platform_Tracker_Tick, HL_PLATFORM_TRK_PERIOD);
    
    //! Turn off all LEDS after Start mode is finished \n
    //! "Run/Test" Modes will use them as appropriate... \n
    Platform_Status_LED_control(0,0,0);  
    Platform_LED_control(0,0,0);

    //! Platform started successfully, change mode to run, test otherwise \n
    if (HL_TEST == 0) {
        Platform_Mode = PLATFORM_RUN;
        #if (HL_DEBUG != 0) 
          dbg.printf("MKIII : Platform : MODE : Run\r\n");
        #endif
    } else {
        Platform_Mode = PLATFORM_TEST;
    } 
  //! Return to "run" the next state                  
  return(RetCode);
}

//! run the platform 
Platform_RetCode_t Platform_Run(Platform_Mode_t mode) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  
  //! Timeout counter for I/O Operations
  //! We use a "side-effect" of the Timeout function,
  //! as attaching the handler function resets Timeout timers.
  //! so we "reattach" to keep the timer from timing out  
  //! (re)setup ioWatchdog to call ioWatchdogServiceService after HL_TIMEOUT seconds
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  
  //* dispatch the run mode */
  switch(mode) {
    case PLATFORM_START :
      RetCode = Platform_Start();
    break;
    case PLATFORM_RESTART :
      RetCode = Platform_Restart(Platform_RetVal);
    break;
    case PLATFORM_TEST :
      RetCode = Platform_Test();
    break;
    case PLATFORM_RUN :
      RetCode = Platform_Run_Task();
    break;
    case PLATFORM_SLEEP :
      RetCode = Platform_Sleep();
    break;
    case PLATFORM_WAKE :
      RetCode = Platform_Wake();
    break;
    case PLATFORM_STOP :
    default :
      RetCode = Platform_Stop();
    break;
  }     
  return(RetCode);
}

//! The main platform run task
//! @TODO refactor this after resolving why multiple MODSERIAL fail in heirarchy
//! @TODO no time to debug that now, flatten the hierarchy to get it working...
//
Platform_RetCode_t Platform_Run_Task(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
   
  //Platform_Host_Port = HL_PLATFORM_IO_BTLE; 
  // check for BTLE packet and switch
  // BTLE link will asseble packets even when not selected
  if (platform_btle_newline) {
      Platform_Host_Port = HL_PLATFORM_IO_BTLE;
  } else {
      // check timeout and switch back to HL_PLATFORM_IO_PC...
  }
  
  // Process Instructions first
  // it's either a complete packet of 16 or nothing now...
  if (Platform_io_Get_Packet(Platform_Host_Port, suit_cmd_packet, sizeof(suit_cmd_packet)) == PKT_SIZE ) {
            
            //! blink blue when processing packets
            Platform_Blink_Blue = 1;
            
            //! create a return packet for the instruction
            memcpy( suit_ret_packet, cmdRET_packet, 16);    
            
            //! dispatch the instruction based on command code
            inst.DISPATCH( suit_cmd_packet, suit_ret_packet );
            
            //! check return packet for error or buffer flush
            if (suit_ret_packet[0] == 0x00) {
                // send the return packet
                // reset the header type
                suit_ret_packet[0] = 0x24;
                // Send the packet
                Platform_io_Set_Packet(Platform_Host_Port, suit_ret_packet, sizeof(suit_ret_packet));
            } else {
                if (suit_ret_packet[0] == 0xFF) {
                    // the instruction failed, what now?
                    // indicate failed packet somehow?
                    // turn off status LEDs for now...
                    Platform_Status_LED_control(0,0,0); 
                }
            }  
  } 
  //! Now, if the Tracker is Enabled and Ready, send Tracker data
  if (Platform_Tracker_Flag && Platform_Tracker_Status){

      //! Find the first available tracker
      //! platform_sensor_table[#Sensors] is sparse, so we
      //! need to look for the next tracker that is ready
       char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
       bool tracker_found = false;
       while(!tracker_found){
          char sensor_status = platform_sensor_table[Platform_Tracker_Unit] & status;
          if (sensor_status == status) {
            tracker_found = true;
          } else {
              Platform_Tracker_Unit++;
              if (Platform_Tracker_Unit >= HL_IMU_TOTAL) Platform_Tracker_Unit = 0;
          }
       } 
       
        //! create an instruction to get the data from Platform_Tracker_Unit
        memcpy( suit_cmd_packet, cmdTRK_packet, 16);  
        //! time-stamp the packet as a packet ID
        Platform_Get_RTC(&Platform_Time, &Platform_Frac_Secs);
        cmdTRK_packet[3] = (char)Platform_Frac_Secs; 
        cmdTRK_packet[4] = Platform_Tracker_Unit;
        
        //! create a return packet for the instruction
        memcpy( suit_ret_packet, cmdRET_packet, 16);    
            
        //! dispatch the instruction based on command code
        inst.DISPATCH( suit_cmd_packet, suit_ret_packet );
            
            //! check return packet for error or buffer flush
            if (suit_ret_packet[0] == 0x00) {
                // send the return packet
                // reset the header type
                suit_ret_packet[0] = 0x24;
                // Send the packet
                Platform_io_Set_Packet(Platform_Host_Port, suit_ret_packet, sizeof(suit_ret_packet));
            } else {
                if (suit_ret_packet[0] == 0xFF) {
                    // the instruction failed, what now?
                    // indicate failed packet somehow?
                    // turn off status LEDs for now...
                    Platform_Status_LED_control(0,0,0); 
                }
            }     

            // increment the unit counter
            Platform_Tracker_Unit++;
            if (Platform_Tracker_Unit > 63) {
                  Platform_Tracker_Unit = 0;
            }
            // reset until the next interrupt
            Platform_Tracker_Status = false;    
  }
  return(RetCode); 
}

//! Stop the platform 
Platform_RetCode_t Platform_Stop(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  
  #if (HL_DEBUG != 0) 
    dbg.printf("MKIII : Platform : MODE : Stop\r\n");
  #endif

  //! stop all motors
  for (int i=0; i<=HL_MOTOR_TOTAL ; i++) {
    Platform_Motor_Stop(i);
  }
  
  // trackers (disable?) in sleep mode
  Platform_Tracker_Flag = false;
  
  //turn off Suit LED's
  Platform_Status_LED_control(0,0,0);  
  Platform_LED_control(0,0,0);
 
  return(RetCode); 
}

//! Restart the platform based on a previous error code
Platform_RetCode_t Platform_Restart(Platform_RetCode_t ErrorCode) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : MODE : Restart : %i Caused\r\n", ErrorCode);
    #endif

  switch (ErrorCode) {
    //! @TODO : Add other error handling here
    case PLATFORM_ERR_IO :
      //! check for a USB Host Connection
      if (pcUSB_Disconnect) {
        // if disconnected:
        // TODO: set all contplay to 0 and disable tracking
        // ! Turn On RED LED for Error \n
        Platform_Status_LED_control(1,0,0);
        Platform_Tracker_Flag = false;
        
        // can't hang here - might be using BLTE now...
        /*
        while(pcUSB_Disconnect) { 
          // wait for a connection!
          wait_ms(500);
          Platform_Status_LED_control(0,0,0);
          wait_ms(500);
          Platform_Status_LED_control(1,0,0);
        } 
        */ 
      } else {
        Platform_Status_LED_control(0,0,0);
      }
      break; 
    default :
      // Restart the suit components???     
      break;
  }
    // change mode to run, test otherwise \n
    if (HL_TEST == 0) {
        Platform_Mode = PLATFORM_RUN;
        #if (HL_DEBUG != 0) 
          dbg.printf("MKIII : Platform : MODE : Run\r\n");
        #endif
    } else {
        Platform_Mode = PLATFORM_TEST;
    }
     
  // Return to "run" the next state                  
  return(RetCode); 
}

//! sleep the platform 
Platform_RetCode_t Platform_Sleep(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  
      #if (HL_DEBUG != 0) 
        dbg.printf("MKIII : Platform : MODE : Sleep\r\n");
      #endif
      //! @TODO Check if any state/data needs to be saved
      //! Check if the Platform is stopped already
      if (Platform_Mode == PLATFORM_SLEEP) {
          //! Sleep the processor
          sleep();
      } else {
          Platform_Stop();
          sleep();
      }
      
  RetCode = PLATFORM_SUCCESS;            
  return(RetCode);
}

//! wake the platform 
Platform_RetCode_t Platform_Wake(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  
  //! @TODO Add the platform wake code
      #if (HL_DEBUG != 0) 
        dbg.printf("MKIII : Platform : MODE : Wake\r\n");
      #endif
  //! @TODO Check if any state/data needs to be restored    
  //! change the mode to "run"
  Platform_Mode = PLATFORM_RUN;
  RetCode = PLATFORM_SUCCESS;
  //turn on Suit LED's
  Platform_Status_LED_control(1,1,1);  
  Platform_LED_control(1,1,1);

  return(RetCode); 
}

/**
  * @fn         void Platform_Configure(void)
  * @brief      called to initialize all the hardware system configuration tables
  * @param      None.
  * @retval     None.
                we will use two (bitmapped) arrays (platform_motor_table and platform_imu_table)
                to control the configuration and operation state. This will give us
                the flexibility to add "Optional" sensors and motors later.
                N.B. need a save/restore for this into EEPROM/FLASH eventually, not in SOW3
                     otherwise disabling a unit won't survive a suit reboot; although,
                     the host PC driver could track that and save on the hard disk, etc.
                     @TODO : add configuration table read/write commands to Instruction Class
  */ 
void Platform_Configure(void) {
  // First mark all units as "none" to init the tables
  memset(platform_motor_table, HL_UNIT_NONE, sizeof(platform_motor_table));
  platform_motor_unit  = HL_UNIT_NONE;
  memset(platform_sensor_table, HL_UNIT_NONE, sizeof(platform_sensor_table));
  platform_sensor_unit = HL_UNIT_NONE;
  
  // now add in the required motors (HL_MOTOR1 - HL_MOTOR16)
  for (int i=HL_MOTOR1; i<=HL_MOTOR16; i++) {
    platform_motor_table[i] |= HL_UNIT_REQUIRED;
  }
  // let's add an "optional" motor as the last one (63) for feature testing...
  platform_motor_table[(HL_MOTOR_TOTAL -1)] |= HL_UNIT_OPTIONAL;

  // now add in the required sensors
  platform_sensor_table[HL_IMU1] |= HL_UNIT_REQUIRED;
  platform_sensor_table[HL_IMU2] |= HL_UNIT_REQUIRED;
  platform_sensor_table[HL_IMU3] |= HL_UNIT_REQUIRED;
  
  // let's add an "optional" sensor as the last one for feature testing...
  //platform_sensor_table[(HL_IMU_TOTAL -1)] |= HL_UNIT_OPTIONAL;
}
    
// ================================================================
// Platform Motor Functions
// ================================================================

//! initialize one platform motors
Platform_RetCode_t Platform_Motor_Init(uint8_t addr) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  //! Only Try to Initialize "Required" and "Optional" motors
  char status = (HL_UNIT_REQUIRED | HL_UNIT_OPTIONAL);
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  if ((platform_motor_table[addr] & status) != HL_UNIT_NONE) {
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;
    //! Make sure any other channels are closed
    d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL); 
    d_tca.openChannel(splitter_addr, driver_addr);
 
    status = drv.readRegister8(DRV2605_REG_STATUS);
    if (drv.begin() == true) {   
      drv.selectLibrary(1);              // @TODO : document Library/Waveform constants better...
      drv.setMode(DRV2605_MODE_INTTRIG);  
      drv.setWaveform(0, 1);
      drv.setWaveform(1, 0);
      // should we actually go?
      drv.go();
      #if (HL_DEBUG > 3) 
        dbg.printf("MKIII : Platform : DRV  : Channel %d : %d : DRV2605 Initialized \r\n", splitter_addr, driver_addr);
      #endif   
      // mark the motor status as Installed, Ready, and Enabled
      platform_motor_table[addr] |=  (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED );
      RetCode = PLATFORM_SUCCESS;
    } else {  
      #if (HL_DEBUG != 0) 
        // only complain if the UNIT is Required
        if ((platform_motor_table[addr] & HL_UNIT_REQUIRED) != HL_UNIT_NONE) {
          dbg.printf("MKIII : Platform : DRV  : Channel %d : %d : DRV2605 Fail = 0x%x \r\n", splitter_addr, driver_addr, status);
        }
      #endif
      RetCode = PLATFORM_NO_RESOURCE;
      // mark the status as NOT-installed and NOT-ready
      platform_motor_table[addr] &=  ~(HL_UNIT_INSTALLED | HL_UNIT_READY );
    }
    //! Make sure any other channels are closed
    d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL);
    if (Platform_Mode == PLATFORM_START) wait(HL_MOTOR_STARTUP_DELAY);
  }    
  return(RetCode); 
}

//--------------------------------
//! initialize all platform motors 
Platform_RetCode_t Platform_Motor_Init_All(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;  
  
  #if (HL_DEBUG != 0) 
    dbg.printf("MKIII : Platform : DRV  : Initialize All \r\n");
  #endif
  d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL);
  for (int i=0; i<HL_MOTOR_TOTAL; i++) {
    ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
    if ((RetCode = Platform_Motor_Init(i)) != PLATFORM_SUCCESS) {
        RetCode = PLATFORM_NO_RESOURCE;  // motor resource not found, disconnected?
    }
  }  
  return(RetCode); 
}

//--------------------------------
//! Plays a single effect on one motor
void Platform_Motor_Play_Effect(uint8_t addr, uint8_t effect) {

  // The motor should be Installed, Ready, and Enabled... 
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
  if ((platform_motor_table[addr] & status )== status){
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    
    drv.setWaveform(0, effect);  // play effect 
    drv.setWaveform(1, 0);           // end waveform 
    // play the effect!
    drv.go();
    
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
  }
}

//--------------------------------
//! Play an effect continuously on one motor
void Platform_Motor_Play_Continuous(uint8_t addr, uint8_t effect) {
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED | HL_UNIT_MODE);
  // if not marked as continuous mode, mark it as continuous mode now...
  platform_motor_table[addr] |= HL_UNIT_MODE;
  if ((platform_motor_table[addr] & status) == status){    
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr); // select the channel
    drv.setWaveform(0, effect);                    // play effect 
    drv.setWaveform(1, 0);                         // end waveform 
    drv.go();                                      // play the effect!
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr); 
  }
}

//--------------------------------
//! Halt a continuous effect immediately on one motor
void Platform_Motor_Stop(uint8_t addr) {
  char status = (HL_UNIT_REQUIRED | HL_UNIT_OPTIONAL);
  if ((platform_motor_table[addr] & status) != HL_UNIT_NONE) {
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    //halt the motor
    drv.halt();
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
    // mark as non-continuous, clear the HL_UNIT_MODE bit
    platform_motor_table[addr] &= ~(HL_UNIT_MODE);
  }
}

//--------------------------------
//! Updates the continuous play routinely 
//! @TODO check for overload of I2C bus & throttle requests if needed
void Platform_Motor_Update_Continuous(void) {
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED | HL_UNIT_MODE);
  for (int addr=0; addr<HL_MOTOR_TOTAL; addr++) {
    if ((platform_motor_table[addr] & status) == status){    
        //The TCA address is which group of 8 the address in, rounding down
        uint8_t splitter_addr = addr/8;
        //The in-tca driver address is the amount left over
        uint8_t driver_addr = addr%8;
        d_tca.openChannel(splitter_addr, driver_addr);
        // check busy bit?
        drv.go();
        // close the channel
        d_tca.closeChannel(splitter_addr); 
    }
  }
}

//--------------------------------
//! Plays the audio input  on one motor
// @TODO No status tracking in audio mode, need a separate flag for that?
// should we make that drv "not ready" (but Installed and Initialized) to indicate audio mode?
//
void Platform_Motor_Play_Audio(uint8_t addr, uint8_t vibectrl, uint8_t minlevel, uint8_t maxlevel, uint8_t mindrv, uint8_t maxdrv) {
    //! make sure that the other channels are closed
    d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL);
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;
    //! Open the Channel
    d_tca.openChannel(splitter_addr, driver_addr);
    //! Reinitialize the driver
    drv.begin();
    //! set audio mode - drv.playAudio(0x00, 0x01, 0x40, 0x00, 0x7F);
    drv.playAudio(vibectrl, minlevel, maxlevel, mindrv, maxdrv);
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
}

//--------------------------------
//! Stops the audio input on one motor
void Platform_Motor_Stop_Audio(uint8_t addr) {
    // make sure that the other channels are closed
    d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL);
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;
    // Open the Channel
    d_tca.openChannel(splitter_addr, driver_addr);
    // set audio mode - drv.playAudio(0x00, 0x01, 0x40, 0x00, 0x7F);
    drv.stopAudio();
    // Reinitialize the driver
    drv.begin();
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
}

//--------------------------------
//! Set INT Triggering on one motor
void Platform_Motor_Set_INTTRIGMODE(uint8_t addr) {

  // The motor should be Installed, Ready, and Enabled... 
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
  if ((platform_motor_table[addr] & status )== status){
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_INTTRIG);
    
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
  }
}

//--------------------------------
//! Set RTP mode on one motor
void Platform_Motor_Set_RTPMODE(uint8_t addr) {

  // The motor should be Installed, Ready, and Enabled... 
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
  if ((platform_motor_table[addr] & status )== status){
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    
    drv.setMode(DRV2605_MODE_REALTIME); 
    
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
  }
}

//--------------------------------
//! Play RTP on one motor
void Platform_Motor_Play_RTP(uint8_t addr, uint8_t regVal) {

  // The motor should be Installed, Ready, and Enabled... 
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
  if ((platform_motor_table[addr] & status )== status){
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    
    drv.writeRegister8(DRV2605_REG_RTPIN, regVal);
    
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
  }
}

//--------------------------------
//! Set RTP mode on one motor
void Platform_Motor_Set_Go(uint8_t addr) {

  // The motor should be Installed, Ready, and Enabled... 
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
  if ((platform_motor_table[addr] & status )== status){
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    
    drv.go(); 
    
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
  }
}

//--------------------------------
//! Set Waveform on one motor
void Platform_Motor_Set_Waveform(uint8_t addr, uint8_t waveform) {

  // The motor should be Installed, Ready, and Enabled... 
  char status = (HL_UNIT_INSTALLED | HL_UNIT_READY | HL_UNIT_ENABLED);
  if ((platform_motor_table[addr] & status )== status){
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;

    d_tca.openChannel(splitter_addr, driver_addr);
    
    drv.setWaveform(0, waveform);  // play effect 
    drv.setWaveform(1, 0);         // end waveform
    
    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
  }
}

//--------------------------------
/** Return a UniqueID for Each motor
    Based on the STM32 UniqueID + Motor Address  
    Supress leading zeros, and then pad to 32 characters.
    The last two digits are for Motors, added to the suit UniqueID
    because the DRV/Motor doesn't have it's own unique/serial number 
    */
void Platform_Motor_Get_UniqueID(uint8_t addr, char *destStr){ 
    unsigned long *Unique = (unsigned long *)0x1FFFF7E8;
    sprintf(destStr, "%lu%02d\000", Unique[0], (addr+1));
}
 
void Platform_Motor_Get_Register(uint8_t addr, uint8_t regNum, char *reg) {
    //! Write a motor driver register
    // make sure that the other channels are closed
    d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL);
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;
    // Open the Channel
    d_tca.openChannel(splitter_addr, driver_addr);

    reg[0] = (uint8_t) drv.readRegister8(regNum); 

    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
}

 
void Platform_Motor_Set_Register(uint8_t addr, uint8_t regNum, char *reg) {
    //! Write a motor driver register
    // make sure that the other channels are closed
    d_tca.closeChannel(HL_MOTOR_TCA_CLOSE_ALL);
    //The TCA address is which group of 8 the address in, rounding down
    uint8_t splitter_addr = addr/8;
    //The in-tca driver address is the amount left over
    uint8_t driver_addr = addr%8;
    // Open the Channel
    d_tca.openChannel(splitter_addr, driver_addr);

    drv.writeRegister8(regNum, reg[0]); 

    //close channel in case the next instruction is to new TCA
    d_tca.closeChannel(splitter_addr);
}
 
// ================================================================
// Platform IMU Tracker Functions
// ================================================================

//! initialize one IMU tracker
Platform_RetCode_t Platform_IMU_Init(uint8_t addr) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  char regVal;
   //! Only Try to Initialize "Required" and "Optional" sensors
  char status = (HL_UNIT_REQUIRED | HL_UNIT_OPTIONAL);
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  if ((platform_sensor_table[addr] & status) != HL_UNIT_NONE) {
        //The TCA address is which group of 8 the address in, rounding down
        uint8_t splitter_addr = addr/8;
        //uint8_t splitter_addr = HL_IMU_TCA;  //tca for trackers is always 7
        //The in-tca driver address is the amount left over
        uint8_t driver_addr = addr%8;
        //uint8_t driver_addr = addr;
        //! Make sure any other channels are closed
        t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL); 
        t_tca.openChannel(splitter_addr, driver_addr);

        if (bno.begin() == true) {
            #if (HL_DEBUG > 3) 
                dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Initialized\r\n", splitter_addr, driver_addr);
            #endif    
            // mark the status as installed, ready, enabled
            platform_sensor_table[addr] |= (HL_UNIT_INSTALLED | HL_UNIT_ENABLED | HL_UNIT_READY );
        } else {  
            #if (HL_DEBUG != 0) 
                // only complain if the UNIT is Required
                if ((platform_sensor_table[addr] & HL_UNIT_REQUIRED) != HL_UNIT_NONE) {
                    dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Init Fail = 0x%X \r\n", splitter_addr, driver_addr, regVal);
                }
            #endif
            RetCode = PLATFORM_NO_RESOURCE;
            // mark the status as NOT-installed and NOT-ready
            platform_sensor_table[addr] &=  ~(HL_UNIT_INSTALLED | HL_UNIT_READY );
        }
  }

  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL);    
  return(RetCode); 
}

Platform_RetCode_t Platform_IMU_Init_All(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  
  #if (HL_DEBUG != 0) 
    dbg.printf("MKIII : Platform : IMU  : Initialize All \r\n");
  #endif 
  
  // scan all of 'em
  for(int i=0; i<HL_IMU_TOTAL; i++) {
      if ((RetCode = Platform_IMU_Init(i)) != PLATFORM_SUCCESS) {
          RetCode = PLATFORM_SUCCESS;
      }    
  }
  RetCode = PLATFORM_SUCCESS;
  return(RetCode);   
}

Platform_RetCode_t Platform_IMU_Get_UniqueID(uint8_t addr, char *destStr) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  //! Service the ioWatchdog Timer
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  //The TCA address is which group of 8 the address in, rounding down
  uint8_t splitter_addr = addr/8;
  //uint8_t splitter_addr = HL_IMU_TCA;  //tca for trackers is always 7
  //The in-tca driver address is the amount left over
  uint8_t driver_addr = addr%8;
  //uint8_t driver_addr = addr;
  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL); 
  t_tca.openChannel(splitter_addr, driver_addr);

  if (bno.get_UniqueID(destStr) == true) {
    #if (HL_DEBUG > 3) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Data\r\n", splitter_addr, driver_addr);
    #endif    
  } else {  
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Fail\r\n", splitter_addr, driver_addr);
    #endif
    RetCode = PLATFORM_NO_RESOURCE; // No IMU Tracker working at that address...
  }

  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL);    
  return(RetCode);   
}

Platform_RetCode_t Platform_IMU_Get_Data(uint8_t addr, char *destStr) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  //! Service the ioWatchdog Timer
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  //The TCA address is which group of 8 the address in, rounding down
  uint8_t splitter_addr = addr/8;
  //uint8_t splitter_addr = HL_IMU_TCA;  //tca for trackers is always 7
  //The in-tca driver address is the amount left over
  uint8_t driver_addr = addr%8;
  //uint8_t driver_addr = addr;
  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL); 
  t_tca.openChannel(splitter_addr, driver_addr);

  //coord_data_t quat_data;
  if (bno.get_quat(destStr) == true) {
    #if (HL_DEBUG > 3) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Data\r\n", splitter_addr, driver_addr);
    #endif    
  } else {  
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Fail\r\n", splitter_addr, driver_addr);
    #endif
    RetCode = PLATFORM_NO_RESOURCE; // No IMU Tracker working at that address...
  }
  
   //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL);    
  return(RetCode);   
}

Platform_RetCode_t Platform_IMU_Get_Compass(uint8_t addr, char *destStr) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  //! Service the ioWatchdog Timer
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  //The TCA address is which group of 8 the address in, rounding down
  uint8_t splitter_addr = addr/8;
  //uint8_t splitter_addr = HL_IMU_TCA;  //tca for trackers is always 7
  //The in-tca driver address is the amount left over
  uint8_t driver_addr = addr%8;
  //uint8_t driver_addr = addr;
  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL); 
  t_tca.openChannel(splitter_addr, driver_addr);

  if (bno.get_mag(destStr) == true) {
    #if (HL_DEBUG > 3) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Data\r\n", splitter_addr, driver_addr);
    #endif    
  } else {  
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Fail\r\n", splitter_addr, driver_addr);
    #endif
    RetCode = PLATFORM_NO_RESOURCE; // No IMU Tracker working at that address...
  }

  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL);    
  return(RetCode);   
}

Platform_RetCode_t Platform_IMU_Get_Gravity(uint8_t addr, char *destStr) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;
  //! Service the ioWatchdog Timer
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
  //The TCA address is which group of 8 the address in, rounding down
  uint8_t splitter_addr = addr/8;
  //uint8_t splitter_addr = HL_IMU_TCA;  //tca for trackers is always 7
  //The in-tca driver address is the amount left over
  uint8_t driver_addr = addr%8;
  //uint8_t driver_addr = addr;
  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL); 
  t_tca.openChannel(splitter_addr, driver_addr);

  if (bno.get_grav(destStr) == true) {
    #if (HL_DEBUG > 3) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Data\r\n", splitter_addr, driver_addr);
    #endif    
  } else {  
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : IMU  : Channel %d : %d : BNO055 Read Fail\r\n", splitter_addr, driver_addr);
    #endif
    RetCode = PLATFORM_NO_RESOURCE; // No IMU Tracker working at that address...
  }

  //! Make sure any other channels are closed
  t_tca.closeChannel(HL_IMU_TCA_CLOSE_ALL);    
  return(RetCode);   
}
     
// ================================================================
// Platform LED Functions
// ================================================================

//! Control the Chest LED with 6 bytes to control RGB LED
//! Nota Bene : It's really RGW - white LED and active low
void Platform_LED_control(float red, float blue, float green) {
    pwm_red.write(1-red);
    pwm_blue.write(1-blue);
    pwm_green.write(1-green);
}

//! Control Board Status LEDs with 6 bytes to control RGB
//! Green = Status Good, Red = Host Disconnected, Blue = Status Condition
//! Nota Bene : It's really RGB LED and active HIGH
void Platform_Status_LED_control(float red, float blue, float green) {

    if (red >= 0) {
        if (red == 0) {
          led_red = 0;
        } else {
          led_red = 1;
        }
    }    
    if (blue >= 0) {
        if (blue == 0) {
          led_blue = 0;
        } else {
          led_blue = 1;
        }
    }    
    if (green >= 0) {
        if (green == 0) {
          led_green = 0;
        } else {
          led_green = 1;
        }
    }    
}

// ================================================================
// Platform Serial/UART Functions
// ================================================================

uint8_t Platform_io_Get_Packet(uint8_t channel, char *chan_buf, uint8_t buflen) {
uint8_t charCnt =0; //default return

  // channel select the I/O stream
  switch(channel) {
    case HL_PLATFORM_IO_PC :
            // copy the current serial input buffer
            if (platform_pc_newline) {
                memcpy( chan_buf, &platform_pc_cmd_packet[0], PKT_SIZE );
                platform_pc_newline = false;
                platform_pc_charCnt = 0;
                charCnt = PKT_SIZE;
            }    
         break;
    case HL_PLATFORM_IO_BTLE :
            // copy the current serial input buffer
            if (platform_btle_newline) {
                memcpy( chan_buf, &platform_btle_cmd_packet[0], PKT_SIZE );
                platform_btle_newline = false;
                platform_btle_charCnt = 0;
                charCnt = PKT_SIZE;
            }    
        break;
    case HL_PLATFORM_IO_DBG :
            // Nota Bene : only available if NOT in HL_DEBUG mode, only available in production mode!
            // copy the current serial input buffer
            if (platform_dbg_newline) {
                memcpy( chan_buf, &platform_dbg_cmd_packet[0], PKT_SIZE );
                platform_dbg_newline = false;
                platform_dbg_charCnt = 0;
                charCnt = PKT_SIZE;
            }    
        break;
    case HL_PLATFORM_IO_TEST :
            // copy the current test buffer
            memcpy( chan_buf, &test_packets[Platform_TestCount][0], PKT_SIZE );
            charCnt = PKT_SIZE;
        break;
    case HL_PLATFORM_IO_NULL :
        break;
    default :
        break;
  }      
  return(charCnt);
}

uint8_t Platform_io_Set_Packet(uint8_t channel, char *chan_buf, uint8_t buflen) {
 uint8_t i = 0;
  // channel select the I/O stream
  switch(channel) {
    case HL_PLATFORM_IO_PC :
        for (i=0; i<buflen; i++) {
            // Nota Bene : Blocking until a write buffer is available
            // @TODO Add output mbed os5 queue to make non-blocking
            while (!pc.writeable());
            pc.putc( chan_buf[i] ); 
        }
        break;
    case HL_PLATFORM_IO_BTLE :
        for (i=0; i<buflen; i++) {
            // Nota Bene : Blocking until a write buffer is available
            // @TODO Add output mbed os5 queue to make non-blocking
            while (!btle.writeable());
            btle.putc( chan_buf[i] ); 
        }
        break;
    case HL_PLATFORM_IO_DBG :
        for (i=0; i<buflen; i++) {
            // Nota Bene : Blocking until a write buffer is available
            // @TODO Add output mbed os5 queue to make non-blocking
            while (!dbg.writeable());
            dbg.putc( chan_buf[i] ); 
        }
        break;
    case HL_PLATFORM_IO_TEST :
        // print test return packets
        //
        if (memcmp( chan_buf, &test_returns[Platform_TestCount][0], buflen ) != 0) {
        #if (HL_DEBUG != 0)
            dbg.printf("MKIII : Platform : CMD : RET : ");
            for (i=0; i< buflen; i++) {          
                dbg.printf("%x %x : ", chan_buf[i], test_returns[Platform_TestCount][i]);
            }
            dbg.printf("\r\n");
        #endif
        }   
        break;
    case HL_PLATFORM_IO_NULL :
        default :
        break;
  }      
  // TODO: add return success/fail checking
  return(buflen);
}

void Platform_pc_rxCallback(MODSERIAL_IRQ_INFO *q) {
    MODSERIAL *serial = q->serial;
  // Note : 
  // the original "packet discriminator" code allowed ill-formed packets after overflow (bad...)
  // so we'll enforce a valid packet structure or completely "drop" the packet
  //
  // Assumptions : since this is a callback we can assume at least one character is ready
  //
  // Platform_pc_rxCallback will only flag a new packet if it is complete,
  // otherwise it will drop the packet, reset the buffers, and try again
  //
  // IMPACT: If an event is REQUIRED to occur, then it's "SET" function MUST set a flag
  // that can be interogated using a "GET" command to insure that the packet wasn't dropped!
  //  
  //
  // have we seen a valid packet or not?
  if (!platform_pc_newline) {
    // no, keep processing the input stream
    // still assembling a packet, with no overflow, stuff the character(s) and increment the count
    if ( platform_pc_charCnt < PKT_SIZE ) {
            // change from getting one character to getting them all, if overflow...
            int buf_count = serial->rxBufferGetCount();
            int copy_count = 0;
            if (buf_count + platform_pc_charCnt <= PKT_SIZE) {
                copy_count = buf_count;
            } else {
                copy_count =  PKT_SIZE - platform_pc_charCnt; 
            }   
            serial->move( &platform_pc_cmd_packet[platform_pc_charCnt], copy_count);
            platform_pc_charCnt += copy_count;       
    }
    // test for a valid packet
    if ( platform_pc_charCnt >= PKT_SIZE ) {
    // check the packet validity
            if ( 
               (platform_pc_cmd_packet[0]  == 0x24) &&
               (platform_pc_cmd_packet[1]  == 0x02) &&
               (platform_pc_cmd_packet[13] == 0xFF) &&
               (platform_pc_cmd_packet[14] == 0x0D) &&
               (platform_pc_cmd_packet[15] == 0x0A) ) {
                // indicate a valid packet and return
                platform_pc_newline = true;
            } else {      
                // bad packet, reset the buffer
                platform_pc_newline = false;
                platform_pc_charCnt = 0; 
                //Platform_Status = PLATFORM_ERR_IO;                    
            }  
    }
  }    
}

void Platform_btle_rxCallback(MODSERIAL_IRQ_INFO *q) {
    MODSERIAL *serial = q->serial;
  // Note : 
  // the original "packet discriminator" code allowed ill-formed packets after overflow (bad...)
  // so we'll enforce a valid packet structure or completely "drop" the packet
  //
  // Assumptions : since this is a callback we can assume at least one character is ready
  //
  // Platform_btle_rxCallback will only flag a new packet if it is complete,
  // otherwise it will drop the packet, reset the buffers, and try again
  //
  // IMPACT: If an event is REQUIRED to occur, then it's "SET" function MUST set a flag
  // that can be interogated using a "GET" command to insure that the packet wasn't dropped!
  //  
  // have we seen a valid packet or not?
  if (!platform_btle_newline) {
    // no, keep processing the input stream
    // still assembling a packet, with no overflow, stuff the character(s) and increment the count
    if ( platform_btle_charCnt < PKT_SIZE ) {
            // change from getting one character to getting them all, if overflow...
            int buf_count = serial->rxBufferGetCount();
            int copy_count = 0;
            if (buf_count + platform_btle_charCnt <= PKT_SIZE) {
                copy_count = buf_count;
            } else {
                copy_count =  PKT_SIZE - platform_btle_charCnt; 
            }   
            serial->move( &platform_btle_cmd_packet[platform_btle_charCnt], copy_count);
            platform_btle_charCnt += copy_count;       
    }
    // test for a valid packet
    if ( platform_btle_charCnt >= PKT_SIZE ) {
    // check the packet validity
            if ( 
               (platform_btle_cmd_packet[0]  == 0x24) &&
               (platform_btle_cmd_packet[1]  == 0x02) &&
               (platform_btle_cmd_packet[13] == 0xFF) &&
               (platform_btle_cmd_packet[14] == 0x0D) &&
               (platform_btle_cmd_packet[15] == 0x0A) ) {
                // indicate a valid packet and return
                platform_btle_newline = true;
            } else {      
                // bad packet, reset the buffer
                platform_btle_newline = false;
                platform_btle_charCnt = 0; 
                //Platform_Status = PLATFORM_ERR_IO;                    
            }  
    }
  }    
}

void Platform_USB_Disconnect(void) {
    //! USB Disconnect - Turn on the red (error) LED 
    Platform_Status_LED_control(1,-1,-1);
    Platform_Restart(PLATFORM_ERR_IO);
} 

/** format the unique serial number as a string.  
    Supress leading zeros, and then pad to 32 characters.
    The last two digits are for Motors, added to the suit UniqueID
    because the DRV/Motor doesn't have it's own unique/serial number */
// TODO : Change to mBed UUID function for BTLE compatibility
void Platform_Get_UniqueID(char *destStr){ 
    unsigned long *Unique = (unsigned long *)0x1FFFF7E8;
    sprintf(destStr, "%lu00\000", Unique[0]);
}

// ================================================================
// Platform Timing Functions
// ================================================================

void Platform_ioWatchdogService(void) {
    if (Platform_Host_Port == HL_PLATFORM_IO_BTLE) {
        //BLTE connection timed out, switch back to PC
        Platform_Host_Port = HL_PLATFORM_IO_PC;
    } else {
        //! Other IO Timeout - Turn on the red (error) LED 
        Platform_Status_LED_control(1,-1,-1);
        Platform_Restart(PLATFORM_TIMEOUT);
    }
}

void Platform_Set_RTC(time_t t)
{
    RTC_DateTypeDef dateStruct;
    RTC_TimeTypeDef timeStruct;
 
    Platform_RTC.Instance = RTC;
 
    // Enable write access to Backup domain
    HAL_PWR_EnableBkUpAccess();
 
    // Convert the time into a tm
    struct tm *timeinfo = localtime(&t);
 
    // Fill RTC structures
    dateStruct.WeekDay        = timeinfo->tm_wday;
    dateStruct.Month          = timeinfo->tm_mon + 1;
    dateStruct.Date           = timeinfo->tm_mday;
    dateStruct.Year           = timeinfo->tm_year - 100;
    timeStruct.Hours          = timeinfo->tm_hour;
    timeStruct.Minutes        = timeinfo->tm_min;
    timeStruct.Seconds        = timeinfo->tm_sec;
    timeStruct.TimeFormat     = RTC_HOURFORMAT12_PM;
    timeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    timeStruct.StoreOperation = RTC_STOREOPERATION_RESET;
 
    // Change the RTC current date/time
    HAL_RTC_SetDate(&Platform_RTC, &dateStruct, FORMAT_BIN);
    HAL_RTC_SetTime(&Platform_RTC, &timeStruct, FORMAT_BIN);
}
 
void Platform_Get_RTC(time_t * t, float * secfrac)
{
    RTC_HandleTypeDef Platform_RTC;
    RTC_DateTypeDef dateStruct;
    RTC_TimeTypeDef timeStruct;
    struct tm timeinfo;
 
    Platform_RTC.Instance = RTC;
 
    HAL_RTC_GetTime(&Platform_RTC, &timeStruct, FORMAT_BIN);
    HAL_RTC_GetDate(&Platform_RTC, &dateStruct, FORMAT_BIN);
 
    // Setup a tm structure based on the RTC
    timeinfo.tm_wday = dateStruct.WeekDay;
    timeinfo.tm_mon  = dateStruct.Month - 1;
    timeinfo.tm_mday = dateStruct.Date;
    timeinfo.tm_year = dateStruct.Year + 100;
    timeinfo.tm_hour = timeStruct.Hours;
    timeinfo.tm_min  = timeStruct.Minutes;
    timeinfo.tm_sec  = timeStruct.Seconds;
 
    // Convert to timestamp
    *t = mktime(&timeinfo);
    // get fractions we can use as timestamps
    *secfrac = ((float)(timeStruct.SecondFraction-timeStruct.SubSeconds)) / ((float)(timeStruct.SecondFraction+1));
    return;
}

//! Platform_Start_RTC : Startup the CPU/mbed RTC
void Platform_Start_RTC(void)  
{ 
    // WARNING: mbed/STM32 time functions hang the cpu!
    // this used to work in previous mbed-os versions ?
    // set_time(time);       // set RTC clock  
    // A working RTC  time stamp function example at :
    // https://os.mbed.com/questions/61064/How-to-enable-Nucleos-internal-lowspeed-/
    // https://os.mbed.com/users/gregeric/notebook/sub-second-timing-taken-from-nucleo-stm32f4-rtc/
    
    RCC_OscInitTypeDef RCC_OscInitStruct;
    uint32_t rtc_freq = 0;
    Platform_RTC.Instance = RTC;
 
        // Enable Power clock
        __PWR_CLK_ENABLE();
 
        // Enable access to Backup domain
        HAL_PWR_EnableBkUpAccess();
 
        // Reset Backup domain
        __HAL_RCC_BACKUPRESET_FORCE();
        __HAL_RCC_BACKUPRESET_RELEASE();
 
            // Enable LSI clock
            RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
            //RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI ;
            RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_NONE; // Mandatory, otherwise the PLL is reconfigured!
            RCC_OscInitStruct.LSEState       = RCC_LSE_OFF;
            RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
            if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
                dbg.printf("RTC error: LSI clock initialization failed!\\");
            }
            // Connect LSI to RTC
            __HAL_RCC_RTC_CLKPRESCALER(RCC_RTCCLKSOURCE_LSI);
            __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
            // Nota Bene : This value is LSI typical value. 
            // TODO : Should be measured precisely using a timer input capture and calibrated
            rtc_freq = LSI_VALUE;
 
        // Enable RTC
        __HAL_RCC_RTC_ENABLE();
 
        Platform_RTC.Init.HourFormat     = RTC_HOURFORMAT_24;
        Platform_RTC.Init.AsynchPrediv   = 127;
        Platform_RTC.Init.SynchPrediv    = (rtc_freq / 128) - 1;
        Platform_RTC.Init.OutPut         = RTC_OUTPUT_DISABLE;
        Platform_RTC.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
        Platform_RTC.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
 
        if (HAL_RTC_Init(&Platform_RTC) != HAL_OK) {
            dbg.printf("RTC error: RTC initialization failed.");
        }
}

//! Platform_Set_Build_Time - use the firmware build date as the starting RTC time
time_t Platform_Set_Build_Time(char const *date, char const *time) {
    char s_month[5];
    int year;
    struct tm t;
    static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    sscanf(date, "%s %d %d", s_month, &t.tm_mday, &year);
    sscanf(time, "%2d %*c %2d %*c %2d", &t.tm_hour, &t.tm_min, &t.tm_sec);
    // Find where is s_month in month_names. Deduce month value.
    t.tm_mon = (strstr(month_names, s_month) - month_names) / 3;    
    t.tm_year = year - 1900;    
    return mktime(&t);
}


// ================================================================
// Platform Test Functions
// ================================================================
// Nota Bene: This section is subject to change frequently during debuging
//
//! Platform_Test : (self) test the hardware platform 
//
Platform_RetCode_t Platform_Test(void) {
  Platform_RetCode_t RetCode = PLATFORM_SUCCESS;

  //! Timeout counter for I/O Operations
  //! We use a "side-effect" of the Timeout function,
  //! as attaching the handler function resets Timeout timers.
  //! so we "attach" to keep the timer from timing out  
  //! (re)setup ioWatchdog to call ioWatchdogServiceService after HL_TIMEOUT seconds
  ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);

  #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : MODE : Self-Test Started\r\n");
  #endif

  #if (HL_TEST != 0)
  Platform_Host_Port = HL_PLATFORM_IO_TEST;
  for (int test = 0; test < NUM_TESTS; test++) {
    if ((RetCode = Platform_Run_Task()) != PLATFORM_SUCCESS) {
         #if (HL_DEBUG != 0) 
            dbg.printf("MKIII : Platform : Test : %d : Failed! \r\n", test);
         #endif
    } else {
        #if (HL_DEBUG != 0) 
            dbg.printf("MKIII : Platform : Test : %d : Passed  \r\n", test);
        #endif
    }
  }
  #endif
  
   // add BLTE loop testing here ...
   /*
   #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : Test : BTLE Echo Test\r\n");
      while (1) {
            // TODO: Add escape character test...
            while(dbg.readable())  btle.putc(dbg.getc());
            while(btle.readable()) dbg.putc(btle.getc());
            ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
      }
  #endif
  */

    /* 
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : DRV  : Audio Test Mode \r\n");
    #endif
    for (int i=HL_MOTOR_TCA_LEFT;i<=HL_MOTOR_TCA_RIGHT;i++) {
        //drv set to audio mode
        for (int j=0; j<=7; j++) {
            ioWatchdog.attach(&Platform_ioWatchdogService, HL_TIMEOUT);
            #if (HL_DEBUG != 0) 
              dbg.printf("MKIII : Platform : DRV  : Channel %d : %d : Audio Mode\r\n", i, j);
            #endif
             
            //! Turn on Green LED for Testing
            Platform_Status_LED_control(0,1,0);
           
            //! Open the TCA channel \n
            d_tca.openChannel(i, j);   
            drv.begin();
            drv.playAudio(0x00, 0x01, 0x40, 0x00, 0x7F);
            wait_ms(HL_MINWAIT);
            //! Turn off Green LED for Testing
            Platform_Status_LED_control(0,0,0);
            wait_ms(HL_MINWAIT);   
         //close channel in case the next Task is to new TCA
        d_tca.closeChannel(i);
       }
    }
    */
    
  #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : MODE : Self-Test Completed\r\n");
  #endif

    //! After the Platform Self-Tests are completed, change to "run" mode
    Platform_Mode = PLATFORM_RUN;
    Platform_Host_Port = HL_PLATFORM_IO_PC;
    #if (HL_DEBUG != 0) 
      dbg.printf("MKIII : Platform : MODE : Run\r\n");
    #endif
    
  //! Turn off all LEDs (now application controlled)
  Platform_Blink_Blue = 0;
  Platform_Blink_Red = 0;
  Platform_Status_LED_control(0,0,0);  
  Platform_LED_control(0,0,0);  
  return(RetCode); 
}

//! Platform_Tracker_Tick
void Platform_Tracker_Tick(void) { 

  if (Platform_Mode == PLATFORM_RUN) {
    Platform_Tracker_Count++;
    // only run when Tracking is enabled
    if (Platform_Tracker_Flag) {
      Platform_Tracker_Status = true;
    } else {
      Platform_Tracker_Status = false;
    } 
    // periodic interrupt manages the LED blinking now...
    if (Platform_Tracker_Count > HL_PLATFORM_TRK_UPDATE) {
      Platform_Tracker_Count = 0;
      if (Platform_Blink_Flag) {
          Platform_Blink_Flag = false;
          Platform_Status_LED_control(Platform_Blink_Red,Platform_Blink_Blue,Platform_Blink_Green);
      } else {
          Platform_Blink_Flag = true;
          Platform_Status_LED_control(0,0,0);
      }                        
    } 
  }
}

void Platform_JumpToBootloader(void) {
    void (*SysMemBootJump)(void);
    
    /**
     * Step: Set system memory address. 
     *       
     *       For STM32F401RE, system memory is on 0x1FFF76DE
     *       For other families, check AN2606 document table 110 with descriptions of memory addresses 
     */
    volatile uint32_t addr = 0x1FFF76DE; //0x1FFF0000  
    
    /**
     * Step: Disable RCC, set it to default (after reset) settings
     *       Internal clock, no PLL, etc.
     */
    #if defined(USE_HAL_DRIVER)
        HAL_RCC_DeInit();
    #endif /* defined(USE_HAL_DRIVER) */

    #if defined(USE_STDPERIPH_DRIVER)
        RCC_DeInit();
    #endif /* defined(USE_STDPERIPH_DRIVER) */
    
    /**
     * Step: Disable systick timer and reset it to default values
     */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
 
    /**
     * Step: Disable all interrupts
     */
    __disable_irq();
    
    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     *       For each family registers may be different. 
     *       Check reference manual for each family.
     *
     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
     *       For others, check family reference manual
     */
    //Remap by hand... {
    #if defined(STM32F4)
        SYSCFG->MEMRMP = 0x01;
    #endif

    #if defined(STM32F0)
        SYSCFG->CFGR1 = 0x01;
    #endif
    //} ...or if you use HAL drivers
    //__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();    //Call HAL macro to do this for you
    
    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));
    
    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(uint32_t *)addr);
    
    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();
    
    /**
     * Step: Connect USB<->UART converter to dedicated USART pins and test
     *       and test with bootloader works with STM32 Flash Loader Demonstrator software
     */
}
 
/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen HL_platform group.
  *    @}
*/
