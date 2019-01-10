/**
 ******************************************************************************
    @file        main.cpp
    @author      Lucian Copeland <lucian@nullspacevr.com>, December 2016
    @author      Tom Moxon <tom@moxon.com>, September 2017
    @version     1.0.0
    MIT License
  
    @brief       This file provides the Main program entry point/function.
    @mainpage    Platform Firmware Documentation
    @section     main_intro Introduction
    @par
    \nThe "main" function runs the platform code.
      Using mbed-os version 5.5 or later, the "main" routine
      runs in it's own os-thread, transparently to the user.
      If needed, other threads may be spawned later.
      (The NVSR HL FIrmware is currently single-threaded)
       
      The main function performs the following actions:
    \n
     1. Starts the platform code: 
        which in turn, starts and initializes each of the 
        library components that are part of the design.
   
     2. The main loop runs the platform code:
        which in turn, calls each of the library components,
        if any foreground processing is needed by that component. 
        Most data movement, and any control flag processing 
        occurs in the interrupt context, in the background.
        An interrupt driven state machine is implemented for those
        components requiring additonal data and control functions.  
     \n
    @par
    @section    main_packages Firmware Packages 
    @par
    The HL Firmware utilizes several external software packages \n
    with various licenses. Check each package for redistribution terms. \n
    @par
    \n
    @par
     @TODO (audit any other third-party code/tools for licensing terms) \n
    @par

 ******************************************************************************
*/

/*-----------------------------------------------------------------------------*/
/* Include Files                                                               */
/*-----------------------------------------------------------------------------*/
#include "HL_Platform.h"

/*-----------------------------------------------------------------------------*/
/** @defgroup main main
  * @{
  */
 
/*-----------------------------------------------------------------------------*/
/**
  * @fn         int main(void)
  * @brief      Firmware Main Entry Point
  * @brief      The main entry point
  * @param      None.
  * @retval     int (Main never returns, a return of int is used to indicate failure)
  */
int main() {
    /** Set the MKIII Hardware Platform to "Start" mode to initialize the platform */
    Platform_Mode = PLATFORM_START; 
    
    while(1) {
        /** Run the Hardware Platform, a state machine with START/RUN/SLEEP/WAKE/STOP/etc. "states" */
        if ((Platform_RetVal = Platform_Run(Platform_Mode)) != PLATFORM_SUCCESS) {
          /** if unsuccessful, try a "Platform restart" */
          if ((Platform_RetVal = Platform_Restart(Platform_RetVal)) != PLATFORM_SUCCESS) {
          /** "Platform restart" failed, stop the platform now */
            Platform_RetVal = Platform_Stop();
            /** infinite loop on unrecoverable suit restart */
            while(1) {
              /** deepsleep() the processor */
              deepsleep();  
            }
          }
        }
    }
}

/*-----------------------------------------------------------------------------*/
/**
  * Close the Doxygen main group.
  *    @}
*/

/* [] END OF FILE */