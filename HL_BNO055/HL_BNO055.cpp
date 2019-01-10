/*
BNO055.cpp
Based off the BNO055 library by StressedDave
Copyright (c) Dave Turner, 2015
Licensed for free use under Apache 2

Rewritten by Lucian Copeland and Tom Moxon, 2017
*/
#include "HL_BNO055.h"

//constructor
BNO055::BNO055(PinName sda, PinName scl): i2c(sda,scl){
    _address = BNO055_ADDRESS;
    accel_scale = 0.001f;
    rate_scale = 1.0f/16.0f;
    angle_scale = 1.0f/16.0f;
    temp_scale = 1;    
}

//public functions
//----------------------------------------------------------------------
// reset : The older BNO tracker boards don't have a stable reset circuit,
// so we can't depend on their state on power-up, so we assume nothing.
// (soft) Reset the BNO to be sure of the state of the chip
bool BNO055::reset(void) {
    char regVal;
    // BNO should operate in fast mode, make sure
    i2c.frequency(400000);
    // Read the current "trigger" register and set the reset bit
    regVal = readRegister8(BNO055_SYS_TRIGGER_ADDR);
    regVal |= 0x20;
    writeRegister8(BNO055_SYS_TRIGGER_ADDR, regVal);
    wait_ms(BNO055_RESET_WAIT); // datasheet says it's 650ms
    return true;   
}


// begin : "soft" initialization, if in doubt, call reset() first...
//
bool BNO055::begin(void) {
    
    //BNO should operate in fast mode
    i2c.frequency(400000);
    
    // make sure we are in Page 0
    writeRegister8(BNO055_PAGE_ID_ADDR, 0);
    // make sure BNO mode is CONFIG
    set_mode(OPERATION_MODE_CONFIG);
    
    // HL BNO board MUST use internal crystal setting!!!
    // (no external crystal installed)     
    // Set to no external crystal (should be default after reset)
    writeRegister8(BNO055_SYS_TRIGGER_ADDR, 0x00);  
     
    /* Make sure we have the right device */
    uint8_t id = readRegister8(BNO055_CHIP_ID_ADDR);
    if(id != BNO055_ID)
    {
        wait_ms(BNO055_RESET_WAIT); // hold on for boot
        id = readRegister8(BNO055_CHIP_ID_ADDR);
        if(id != BNO055_ID) {
            return false;  // still not? ok bail
        }
    }

   // Remap axes - this cause problems - make it stop...
   //writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR, 0x06);

    // Set to normal power mode
    writeRegister8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);  
        
    // Change mode to NDOF
    writeRegister8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    
    return true;
}
/**--------------------------------------------------------------*/
/** get/read functions                                           */
/**--------------------------------------------------------------*/
// get_UniqueID : return an array of chip info
// returns eight(8)ID bytes : Chip, ACC, MAG, GYR, SWL, SWM, Bootloader, UUID 
//
bool BNO055::get_UniqueID(char *UniqueID_buffer){
    writeRegister8(BNO055_PAGE_ID_ADDR, 0x00);
    UniqueID_buffer[0] = readRegister8(BNO055_CHIP_ID_ADDR);
    if(UniqueID_buffer[0] != BNO055_ID)
    {
        wait_ms(BNO055_RESET_WAIT); // wait for boot/reset
        UniqueID_buffer[0] = readRegister8(BNO055_CHIP_ID_ADDR);
        if(UniqueID_buffer[0] != BNO055_ID) {
            return(false);  // still not good? then fail
        }
    }
    // get the serial number - arrg!!! it's always 0x55? wtf?
    if (readLen(BNO055_CHIP_ID_ADDR+1, &UniqueID_buffer[1], 6) != true) return(false);
    writeRegister8(BNO055_PAGE_ID_ADDR, 0x01);
    if (readLen(BNO055_UNIQUE_ID_ADDR, &UniqueID_buffer[7], 1) != true) return(false);
    writeRegister8(BNO055_PAGE_ID_ADDR, 0x00);
    return true;
} 
 
int BNO055::get_temp(void) {
    uint8_t regVal = readRegister8(BNO055_TEMP_ADDR);
    int temp = regVal / temp_scale;
    return(temp);
}

bool BNO055::get_accel(char *buffer) {
    bool retval = true;
    // get the  data
    if (readLen(BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6) != true) retval = false;
    return(retval);
}

bool BNO055::get_gyro(char *buffer) {
    bool retval = true;
    // get the  data
    if (readLen(BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6) != true) retval = false;
    return(retval);
}

bool BNO055::get_mag(char *buffer) {
    bool retval = true;
    // get the  data
    if (readLen(BNO055_MAG_DATA_X_LSB_ADDR, buffer, 6) != true) retval = false;
    return(retval);
}

bool BNO055::get_lia(char *buffer) {
    bool retval = true; 
    // get the  data
    if (readLen(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6) != true) retval = false;
    return(retval);
}

bool BNO055::get_grav(char *buffer) {
    bool retval = true;

    // get the  data
    if (readLen(BNO055_GRAVITY_DATA_X_LSB_ADDR, buffer, 6) != true) retval = false;
    return(retval);
}

bool BNO055::get_quat(char *buffer) {
    bool retval = true;
    // get the  data
    if (readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8) != true) retval = false;
    return(retval);
}

bool BNO055::get_angles(char *buffer) {
    bool retval = true;

    // get the  data
    if (readLen(BNO055_EULER_H_LSB_ADDR, buffer, 6) != true) retval = false;
    return(retval);
}

uint8_t BNO055::get_calib_status(void){
     uint8_t regVal = readRegister8(BNO055_CALIB_STAT_ADDR);
     return(regVal);
}

bool BNO055::get_calib_data(char *calib_data) {
    bool retval = true;
    set_mode(OPERATION_MODE_CONFIG);
    // get the calibration data
    if (readLen(ACCEL_OFFSET_X_LSB_ADDR, &calib_data[0], 22) != true) retval = false;
    restore_mode();
    return(retval);
} 
 
/**--------------------------------------------------------------*/
/** set/write functions                                          */
/**--------------------------------------------------------------*/
bool BNO055::set_calib_data(char *calib_data) {
    bool retval = true;
    set_mode(OPERATION_MODE_CONFIG);

    // set the calibration data
    if (writeLen(ACCEL_OFFSET_X_LSB_ADDR, &calib_data[0], 22) != true) retval = false;
    restore_mode();
    return(retval);
} 
 
void BNO055::set_accel_units(char units){
    uint8_t usel = readRegister8(BNO055_UNIT_SEL_ADDR);
    if(units == MPERSPERS){
        usel &= 0xFE;
        accel_scale = 0.01f;
        }
    else {
        usel |= units;
        accel_scale = 0.001f;
        }
    writeRegister8(BNO055_UNIT_SEL_ADDR, usel);
}

void BNO055::set_anglerate_units(char units){
    uint8_t usel = readRegister8(BNO055_UNIT_SEL_ADDR);
    if (units == DEG_PER_SEC){
        usel &=  0xFD;
        rate_scale = 1.0f/16.0f;
        }
    else {
        usel |= units;
        rate_scale = 1.0f/900.0f;
        }
    writeRegister8(BNO055_UNIT_SEL_ADDR, usel);
}    
 
void BNO055::set_angle_units(char units){
    uint8_t usel = readRegister8(BNO055_UNIT_SEL_ADDR);
    if (units == DEGREES){
        usel &=  0xFB;
        angle_scale = 1.0f/16.0f;
        }
    else {
        usel |= units;
        rate_scale = 1.0f/900.0f;
        }
    writeRegister8(BNO055_UNIT_SEL_ADDR, usel);
}

void BNO055::set_temp_units(char units){
    uint8_t usel = readRegister8(BNO055_UNIT_SEL_ADDR);
    if (units == CENTIGRADE){
        usel &= 0xEF;
        temp_scale = 1;
        }
    else {
        usel |= units;
        temp_scale = 2;
        }
    writeRegister8(BNO055_UNIT_SEL_ADDR, usel);
}    

void BNO055::set_orientation(char units){
    uint8_t usel = readRegister8(BNO055_UNIT_SEL_ADDR);
    if (units == WINDOWS) {
        usel &= 0x7F;
    } else {
        usel |= units;
    }
    writeRegister8(BNO055_UNIT_SEL_ADDR, usel);
}        
       
void BNO055::set_mapping(char orientation){
    writeRegister8(BNO055_PAGE_ID_ADDR, 0);
    switch (orientation){
        case 0: 
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x04);
            break;
        case 1:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
            break;
        case 2:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
            break;
        case 3:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x02);
            break;
        case 4:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x03);
            break;
        case 5:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x01);
            break;
        case 6:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x21);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x07);
            break;
        case 7:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x05);
            break;
        default:
            writeRegister8(BNO055_AXIS_MAP_CONFIG_ADDR,0x24);
            writeRegister8(BNO055_AXIS_MAP_SIGN_ADDR,0x00);
        }
}
        
bool BNO055::set_mode(uint8_t mode) {
  _saved_mode = _mode;
  _mode = mode;
  writeRegister8(BNO055_OPR_MODE_ADDR, _mode);
  wait_ms(30);
  return(true);
}

bool BNO055::restore_mode(void) {
  _mode = _saved_mode;
  writeRegister8(BNO055_OPR_MODE_ADDR, _mode);
  wait_ms(30);
  return(true);
}

bool BNO055::set_extCrystalUse(bool usextal) {
  uint8_t modeback = _mode;
  
  /* Switch to config mode (just in case since this is the default) */
  set_mode(OPERATION_MODE_CONFIG);
  writeRegister8(BNO055_PAGE_ID_ADDR, 0);
  uint8_t usel = readRegister8(BNO055_SYS_TRIGGER_ADDR);
  if (usextal) {
      usel |= 0x80;
  } else {
      usel &= 0x7F;
  }
  writeRegister8(BNO055_SYS_TRIGGER_ADDR, usel);
  /* restore the mode */
  set_mode(modeback);
  return(true);
}

bool BNO055::writeRegister8(char reg, char val) {
    char buff[2] = {reg, val};
    if (i2c.write(_address, buff, 2, false) !=0) {
      return (false);
    } else {
      return true;
    }
}

bool BNO055::writeLen(char reg, char *buffer, uint8_t len) {
    i2c.write(_address, &reg, 1, true);
    if (i2c.read(_address, buffer, len, false) !=0) {
      return (false);
    } else {
      return true;
    }
}

bool BNO055::read_byte(char reg, char val) {
    if (i2c.write(_address, &reg, 1, true) !=0) {
      return (false);
    } else {
      if (i2c.read(_address, &val, 1, false) !=0) {
        return (false);
      } else {
        return true;
      }
    }
}

char BNO055::readRegister8(char reg) {
    char result;
    i2c.write(_address, &reg, 1, true); 
    i2c.read(_address, &result, 1, false); 
    return result;
}

bool BNO055::readLen(char reg, char *buffer, uint8_t len) {
    i2c.write(_address, &reg, 1, true);
    if (i2c.read(_address, buffer, len, false) !=0) {
      return (false);
    } else {
      return true;
    }
}

/** End of HL_BNO055.cpp */