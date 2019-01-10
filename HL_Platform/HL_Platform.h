/*
Copyright Lucian Copeland, 2016
Rewritten by Tom Moxon, 2017

MIT License
*/

/*-----------------------------------------------------------------------------*/
/* Define to prevent recursive inclusion                                       */
/*-----------------------------------------------------------------------------*/
#ifndef HL_PLATFORM_H_
#define HL_PLATFORM_H_

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
 
/** @defgroup API The HL_Platform API */
/**   
   @par
    The "platform" encapsulates the hardware platform into a number of simple APIs. \n
    The platform" is independent of the application code, \n
    as the same platform could be used for different applications.  \n
    The "application" code is contained in the "task" class, \n
    and separates the application from the platform that it is running on.\n
    
   @par
     \n
     \n   See https://github.com/NullSpaceVR/Firmware/tree/master/mark-III
     \n   for a description of the Suit Host Communcations Protocol
     \n  
   @section   HL_platform_todo_list ToDo List
   @par
     \n       
     \n    1) Return Packet simplification request -
     \n       A) Set/Get Instruction format
     \n          Set instructions are assumed to be "fire and forget". (no return code)
     \n          They can set a status code which can be read with a "Get" instruction.
     \n       B) Only "Get" instructions send return packets
     \n       C) Return packet code is always it's own instruction id if successful
     \n       D) Return packet code is it's own instruction (id OR 0x80) if failed 
     \n          (i.e. only 0x7F instructions possible - 0x80 is the failed return code for instruction 0x00))
     \n       E) Change "Length" to "Packet Number" since instructions are fixed length
     \n          (add a "length field to any variable length instructions...)
     \n       F) Return packet includes the "Packet Number" it's return
     \n
     \n     
 */
 
//! STM32F401RET6 Specific Definitions
#define HSE_VALUE ((uint32_t)8000000)

#include <stm32f4xx.h>
#include <stm32f401xe.h>
#include <stm32f4xx_hal_conf.h>
#include <stm32f4xx_hal_rcc_ex.h>  
#include "stm32f4xx_hal.h"
  
#include "mbed.h"
#include "rtos.h"

#include "MODSERIAL.h"
#include "HL_TCA9548a.h"
#include "HL_BNO055.h"
#include "HL_DRV2605.h"

/*-----------------------------------------------------------------------------*/
/* Exported Types                                                              */
/*-----------------------------------------------------------------------------*/
/** @ingroup API 
 *  @purpose Function Return codes used by the platform
 */
typedef enum
{
    PLATFORM_SUCCESS,                /*!< success = no errors             */
    PLATFORM_TIMEOUT,                /*!< timeout occurred                */
    PLATFORM_ILL_ARG,                /*!< illegal argument                */
    PLATFORM_ILL_CMD,                /*!< illegal command                 */
    PLATFORM_ILL_STATE,              /*!< illegal state                   */
    PLATFORM_ILL_ADDR,               /*!< illegal address                 */
    PLATFORM_ILL_IO,                 /*!< illegal I/O operation           */
    PLATFORM_NO_RESOURCE,            /*!< insufficient resource           */
    PLATFORM_ERR_PLATFORM,           /*!< platform or porting layer error */
    PLATFORM_ERR_IO ,                /*!< general I/O system error        */
    PLATFORM_FAIL                    /*!< general failure - cause unknown */
} Platform_RetCode_t;

/** @ingroup API 
 *  @purpose operating modes (states) used by the platform
 */
 typedef enum
{
    PLATFORM_START,                 /*!< Start/Begin/Init                */
    PLATFORM_RESTART,               /*!< Retry Startup                   */
    PLATFORM_TEST,                  /*!< Test                            */
    PLATFORM_RUN,                   /*!< Run                             */
    PLATFORM_STOP,                  /*!< Stop                            */
    PLATFORM_SLEEP,                 /*!< Sleep                           */
    PLATFORM_WAKE                   /*!< Wake                            */
} Platform_Mode_t;


/*-----------------------------------------------------------------------------*/
/* Exported Constants                                                          */
/*-----------------------------------------------------------------------------*/
#define HL_SUIT_VERSION                3   /**< Platform Version Control                */
#define HL_PLATFORM_MAJOR_VERSION      0   /**< Platform Version Control                */
#define HL_PLATFORM_MINOR_VERSION      0   /**< Platform Version Control                */
#define HL_PLATFORM_REVISION           1   /**< Platform Version Control                */
  
#define HL_DEBUG                       1   /**< Define to enable debugging messages     */
#define HL_TEST                        4   /**< Define to enable test/assert modes      */
#define HL_TIMEOUT                     8   /**< Define for I/O watchdog timer (in seconds) */
#define HL_MINWAIT                    50   /**< Define for wait timer (in MILLIseconds) */
#define HL_MAXWAIT                   500   /**< Define for wait timer (in MILLIseconds) */

/* Tracker Update Rate */
#define HL_PLATFORM_TRK_UPDATE        50   /**< Define for tracker update timer (in Hertz) */
#define HL_PLATFORM_TRK_PERIOD        0.02  /**< 1/HL_PLATFORM_TRK_UPDATE */

/** Serial Port (UART) Definitions                                                  */
/** Default is 9600,N,8,1 for initial testing and development                       */
/** When it's operating correctly at 9600 baud, then we'll crank it up to 115200    */
#define HL_PC_BAUD                  115200 //9600
#define HL_BTLE_BAUD                115200 //57600
#define HL_DBG_BAUD                 115200 //9600

/** Platform I/O Channel Defines */
#define HL_PLATFORM_IO_TOTAL           5 //! Total number of I/O channels (null, pc, btle/usb/ debug, test)
#define HL_PLATFORM_IO_NULL            0 //! returns 0x00, drops output (i.e. a sink...)
#define HL_PLATFORM_IO_PC              1 //! default HOST connection
#define HL_PLATFORM_IO_BTLE            2 //! optional Bluetooth Connection
#define HL_PLATFORM_IO_DBG             3 //! debug (terminal) connection
#define HL_PLATFORM_IO_TEST            4 //! returns test packets in sequence

/** Motor Defines                                                                   */
#define HL_MOTOR_TOTAL                64  // Defines to increase code readability
#define HL_MOTOR1                     16
#define HL_MOTOR2                     17
#define HL_MOTOR3                     18
#define HL_MOTOR4                     19
#define HL_MOTOR5                     20
#define HL_MOTOR6                     21
#define HL_MOTOR7                     22
#define HL_MOTOR8                     23
#define HL_MOTOR9                     24
#define HL_MOTOR10                    25
#define HL_MOTOR11                    26
#define HL_MOTOR12                    27
#define HL_MOTOR13                    28
#define HL_MOTOR14                    29
#define HL_MOTOR15                    30
#define HL_MOTOR16                    31
#define HL_MOTOR_TCA_LEFT              2
#define HL_MOTOR_TCA_RIGHT             3
#define HL_MOTOR_TCA_CLOSE_ALL         7
#define HL_MOTOR_STARTUP_DELAY         0.1 // in seconds

/** IMU Tracker Defines                                                       */
#define HL_IMU_TOTAL                  64
#define HL_IMU1                       60
#define HL_IMU2                       57
#define HL_IMU3                       58
#define HL_IMU_TCA                     7
#define HL_IMU_TCA_CLOSE_ALL           7

/** Unit Status Table Bitmap Defines                                                       */
#define HL_UNIT_NONE                0x00 //! HL_UNIT_NONE - Ignore this address
#define HL_UNIT_REQUIRED            0x01 //! HL_UNIT_REQUIRED - Complain if not found in a scan
#define HL_UNIT_OPTIONAL            0x02 //! HL_UNIT_OPTIONAL - Be silent if not found in a scan
#define HL_UNIT_INSTALLED           0x04 //! HL_UNIT_INSTALLED -  Was found in a scan once upon a time
#define HL_UNIT_READY               0x08 //! HL_UNIT_READY -  Successfully Initialize and Ready
#define HL_UNIT_ERROR               0x10 //! HL_UNIT_ERROR -  TImed out at least once upon a time
#define HL_UNIT_BUSY                0x20 //! HL_UNIT_BUSY -  Busy during the last scan
#define HL_UNIT_MODE                0x40 //! HL_UNIT_MODE - 0 = Normal, 1 = Continuous
#define HL_UNIT_ENABLED             0x80 //! HL_UNIT_ENABLED - add to scan or ignore it
         
/** STM32F401RETC6 Pin Assignments                                                  */  
#define HL_USB_CTS_PIN              PA_0   /**   FTDI_RTS (FTDI connected to UART2) */
#define HL_USB_RTS_PIN              PA_1   /**   FTDI_CTS (FTDI connected to UART2) */
#define HL_USB_TXD_PIN              PA_2   /**   TDI_RXD (FTDI connected to UART2) [aka: Arduino Pin D1] */
#define HL_USB_RXD_PIN              PA_3   /**   FTDI_TXD  (FTDI connected to UART2) [aka: Arduino Pin D0] */
#define HL_USB_CON_PIN              PC_8   /**   PWREN_FTDI (FTDI connected to UART2) Indicates USB is connected when low. */
#define HL_PWM_B_PIN                PA_5   /**   PWM_B  (Blue LED on Center Sensor Board) */
#define HL_PWM_G_PIN                PA_7   /**   PWM_G (Green LED on Center Sensor Board) (R&G Swapped on stuffing?) */
#define HL_PWM_R_PIN                PA_6   /**   PWM_R (Red LED on Center Sensor Board) */
#define HL_BTLE_TXD_PIN             PA_9   /**   BMD_RXD  (BTLE connected to UART1) (On expansion connector) */
#define HL_BTLE_RXD_PIN             PA_10  /**   BMD_TXD  (BTLE connected to UART1) (On expansion connector) */
#define HL_BTLE_CTS_PIN             PA_11  /**   BMD_RTS   (BTLE connected to UART1) (On expansion connector) */
#define HL_BTLE_RTS_PIN             PA_12  /**   BMD_CTS  (BTLE connected to UART1) (On expansion connector) */
#define HL_DBG_TXD_PIN              PC_6   /**   UART6 TXD (Connected to six pin "UART" connector on control board) */
#define HL_DBG_RXD_PIN              PC_7   /**   UART6 RXD (Connected to six pin "UART" connector on control board) */
#define HL_LED_R_PIN                PC_13  /**   LED_RED (on control board)   (I added color LED's to the control board for visual status...) */
#define HL_LED_G_PIN                PC_14  /**   LED_GREEN (on control board) */
#define HL_LED_B_PIN                PC_15  /**   LED_BLUE (on control board) */
#define HL_I2C2_SDA_PIN             PB_3   /**   I2C2_SDA   Motor Driver Hub (DRV)         [aka: Arduino Pin D3] */
#define HL_I2C1_SCL_PIN             PB_8   /**   I2C1_SCL   (PCA9548A Sensor I2C Hub)   [aka: Arduino Pin D15] */
#define HL_I2C1_SDA_PIN             PB_9   /**   I2C1_SDA  (PCA9548A Sensor I2C Hub)   [aka: Arduino Pin D14] */
#define HL_I2C2_SCL_PIN             PB_10  /**   I2C2_SCL   Motor Driver Hub (DRV)          [aka: Arduino Pin D6] */

/** Some basic/misc constants */
#define LOW                             (0u)
#define HIGH                            (1u)

#define PKT_SIZE                    16

/*-----------------------------------------------------------------------------*/
/* Exported Macros                                                             */
/*-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
/* Exported Variable Declarations                                              */
/*-----------------------------------------------------------------------------*/

/** Platform Globals */
extern Platform_RetCode_t Platform_RetVal;       // current status
extern Platform_Mode_t    Platform_Mode;         // current mode
extern char               Platform_UniqueID[33]; // ARM Cortex Serial Number Register String (32 + null)
extern char               Platform_TestLevel;    // self-test/diagnostic level
extern uint8_t            Platform_TestCount;    // self-test/diagnostic counter
extern time_t             Platform_Time;         // Real-Time (stamp holder)

extern bool               Platform_Tracker_Flag;    // True if Tracker is enabled/running
extern bool               Platform_Tracker_Status;  // True if Tracker ready
extern uint16_t           Platform_Tracker_Count;   // Tracker Running Count
extern uint16_t           Platform_Tracker_Unit;    // Tracker Running Unit
extern bool platform_pc_newline;
extern bool platform_btle_newline;
extern bool platform_dbg_newline;

/* scorepad(s) for populated motors and sensors, with status */
extern char platform_motor_table[HL_MOTOR_TOTAL];
extern char platform_motor_unit;
extern char platform_sensor_table[HL_IMU_TOTAL];
extern char platform_sensor_unit;

  
/** @ingroup API 
 *  @purpose Function Return codes used by the platform
 */
 
/*-----------------------------------------------------------------------------*/
/* Exported Class/Function Declarations                                        */
/*-----------------------------------------------------------------------------*/
/** 
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  @purpose  Start the MKIII Platform  
  */   
extern Platform_RetCode_t Platform_Start(void);

/** 
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  @purpose  Stop the MKIII Platform  
  */   
extern Platform_RetCode_t Platform_Stop(void);

/**   
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  @purpose  Sleep the MKIII Platform  
  */
extern Platform_RetCode_t Platform_Sleep(void);

/**   
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Wake(void);

/**   
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Test(void);

/**   
  @ingroup API 
  @param    Platform_RetCode_t Platform Geneneral Return Type
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Restart(Platform_RetCode_t);

/**   
  @ingroup API 
  @param    mode The Platform Run Mode
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Run(Platform_Mode_t mode);

/**   
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Run_Task(void); 

/**   
  @ingroup API 
  @param    addr Unit Address
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Motor_Init(uint8_t addr);

/**   
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_Motor_Init_All(void);

/**   
  @ingroup API 
  @param    addr Unit Address
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_IMU_Init(uint8_t addr);

/**   
  @ingroup API 
  @param    void (None)
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_IMU_Init_All(void);
/**   
  @ingroup API 
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_IMU_Get_UniqueID(uint8_t addr, char *destStr);
/**   
  @ingroup API 
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_IMU_Get_Data(uint8_t addr, char *destStr);
/**   
  @ingroup API 
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_IMU_Get_Compass(uint8_t addr, char *destStr);
/**   
  @ingroup API 
  @return   Platform_RetCode_t Platform Geneneral Return Type
  */
extern Platform_RetCode_t Platform_IMU_Get_Gravity(uint8_t addr, char *destStr);

/**   
  @ingroup API 
  */
extern uint8_t            Platform_io_Get_Packet(uint8_t channel, char *chan_buf, uint8_t buflen);
/**   
  @ingroup API 
  */
extern uint8_t            Platform_io_Set_Packet(uint8_t channel, char *chan_buf, uint8_t buflen);
/**   
  @ingroup API 
  */
extern time_t             Platform_Set_Build_Time(char const *date, char const *time);
/**   
  @ingroup API 
  */
extern void               Platform_Tracker_Tick(void);
/**   
  @ingroup API 
  */  
extern void               Platform_Configure(void);
/**   
  @ingroup API 
  */
extern void               Platform_Get_UniqueID(char *destStr);
/**   
  @ingroup API 
  */
extern void               Platform_USB_Disconnect(void);
/**   
  @ingroup API 
  */
extern void               Platform_ioWatchdogService(void);
/**   
  @ingroup API 
  */
extern void               Platform_pc_rxCallback(MODSERIAL_IRQ_INFO *q);
/**   
  @ingroup API 
  */
extern void               Platform_btle_rxCallback(MODSERIAL_IRQ_INFO *q);
/**   
  @ingroup API 
  */
extern void               Platform_Status_LED_control(float red, float blue, float green);
/**   
  @ingroup API 
  */
extern void               Platform_LED_control(float red, float blue, float green);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Play_Effect(uint8_t addr, uint8_t effect);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Play_Continuous(uint8_t addr, uint8_t effect);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Stop(uint8_t addr);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Update_Continuous(void);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Play_Audio(uint8_t addr, uint8_t vibectrl, uint8_t minlevel, uint8_t maxlevel, uint8_t mindrv, uint8_t maxdrv);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Stop_Audio(uint8_t addr);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Get_UniqueID(uint8_t addr, char *destStr);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Get_Register(uint8_t addr, uint8_t regNum, char *reg);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Set_Register(uint8_t addr, uint8_t regNum, char *reg);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Set_INTTRIGMODE(uint8_t addr);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Set_RTPMODE(uint8_t addr);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Play_RTP(uint8_t addr, uint8_t regVal);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Set_Go(uint8_t addr);
/**   
  @ingroup API 
  */
extern void               Platform_Motor_Set_Waveform(uint8_t addr, uint8_t waveform);
/**   
  @ingroup API 
  */
extern void               Platform_Start_RTC(void); 
/**   
  @ingroup API 
  */
extern void               Platform_Get_RTC(time_t * t, float * secfrac);
/**   
  @ingroup API 
  */
extern void               Platform_Set_RTC(time_t t);


/*-----------------------------------------------------------------------------*/
#endif /* HL_PLATFORM_H_ */