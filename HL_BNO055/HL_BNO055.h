/*
BNO055.h
Based off the BNO055 library by StressedDave
Copyright (c) Dave Turner, 2015
Licensed for free use under Apache 2

Rewritten by Lucian Copeland and Tom Moxon, 2017
*/

#include "mbed.h"
 
#define BNO055_ADDRESS_LOW      0x28
#define BNO055_ADDRESS_HIGH     0x29
#define BNO055_ADDRESS          (BNO055_ADDRESS_LOW<<1)  //! BNO055_ADDRESS = (0x28<<1) = 7 bit Addressing
#define BNO055_ID               0xA0
#define BNO055_RESET_WAIT        675

/* Page id register definition */
#define BNO055_PAGE_ID_ADDR                                       0X07 

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                                       0x00 
#define BNO055_ACCEL_REV_ID_ADDR                                  0x01 
#define BNO055_MAG_REV_ID_ADDR                                    0x02 
#define BNO055_GYRO_REV_ID_ADDR                                   0x03 
#define BNO055_SW_REV_ID_LSB_ADDR                                 0x04 
#define BNO055_SW_REV_ID_MSB_ADDR                                 0x05 
#define BNO055_BL_REV_ID_ADDR                                     0X06 

/* Accel data register */
#define BNO055_ACCEL_DATA_X_LSB_ADDR                              0X08 
#define BNO055_ACCEL_DATA_X_MSB_ADDR                              0X09 
#define BNO055_ACCEL_DATA_Y_LSB_ADDR                              0X0A 
#define BNO055_ACCEL_DATA_Y_MSB_ADDR                              0X0B 
#define BNO055_ACCEL_DATA_Z_LSB_ADDR                              0X0C 
#define BNO055_ACCEL_DATA_Z_MSB_ADDR                              0X0D 

/* Mag data register */
#define BNO055_MAG_DATA_X_LSB_ADDR                                0X0E 
#define BNO055_MAG_DATA_X_MSB_ADDR                                0X0F 
#define BNO055_MAG_DATA_Y_LSB_ADDR                                0X10 
#define BNO055_MAG_DATA_Y_MSB_ADDR                                0X11 
#define BNO055_MAG_DATA_Z_LSB_ADDR                                0X12 
#define BNO055_MAG_DATA_Z_MSB_ADDR                                0X13 

/* Gyro data registers */
#define BNO055_GYRO_DATA_X_LSB_ADDR                               0X14 
#define BNO055_GYRO_DATA_X_MSB_ADDR                               0X15 
#define BNO055_GYRO_DATA_Y_LSB_ADDR                               0X16 
#define BNO055_GYRO_DATA_Y_MSB_ADDR                               0X17 
#define BNO055_GYRO_DATA_Z_LSB_ADDR                               0X18 
#define BNO055_GYRO_DATA_Z_MSB_ADDR                               0X19 

/* Euler data registers */
#define BNO055_EULER_H_LSB_ADDR                                   0X1A 
#define BNO055_EULER_H_MSB_ADDR                                   0X1B 
#define BNO055_EULER_R_LSB_ADDR                                   0X1C 
#define BNO055_EULER_R_MSB_ADDR                                   0X1D 
#define BNO055_EULER_P_LSB_ADDR                                   0X1E 
#define BNO055_EULER_P_MSB_ADDR                                   0X1F 

/* Quaternion data registers */
#define BNO055_QUATERNION_DATA_W_LSB_ADDR                         0X20 
#define BNO055_QUATERNION_DATA_W_MSB_ADDR                         0X21 
#define BNO055_QUATERNION_DATA_X_LSB_ADDR                         0X22 
#define BNO055_QUATERNION_DATA_X_MSB_ADDR                         0X23 
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR                         0X24 
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR                         0X25 
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR                         0X26 
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR                         0X27 

/* Linear acceleration data registers */
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                       0X28 
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                       0X29 
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                       0X2A 
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                       0X2B 
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                       0X2C 
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                       0X2D 

/* Gravity data registers */
#define BNO055_GRAVITY_DATA_X_LSB_ADDR                            0X2E 
#define BNO055_GRAVITY_DATA_X_MSB_ADDR                            0X2F 
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR                            0X30 
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR                            0X31 
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR                            0X32 
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR                            0X33 

/* Temperature data register */
#define BNO055_TEMP_ADDR                                          0X34 

/* Status registers */
#define BNO055_CALIB_STAT_ADDR                                    0X35 
#define BNO055_SELFTEST_RESULT_ADDR                               0X36 
#define BNO055_INTR_STAT_ADDR                                     0X37 

#define BNO055_SYS_CLK_STAT_ADDR                                  0X38 
#define BNO055_SYS_STAT_ADDR                                      0X39 
#define BNO055_SYS_ERR_ADDR                                       0X3A 

/* Unit selection register */
#define BNO055_UNIT_SEL_ADDR                                      0X3B 
#define BNO055_DATA_SELECT_ADDR                                   0X3C 

/* Mode registers */
#define BNO055_OPR_MODE_ADDR                                      0X3D 
#define BNO055_PWR_MODE_ADDR                                      0X3E 

#define BNO055_SYS_TRIGGER_ADDR                                   0X3F 
#define BNO055_TEMP_SOURCE_ADDR                                   0X40 

/* Axis remap registers */
#define BNO055_AXIS_MAP_CONFIG_ADDR                               0X41 
#define BNO055_AXIS_MAP_SIGN_ADDR                                 0X42 

/* SIC registers */
#define BNO055_SIC_MATRIX_0_LSB_ADDR                              0X43 
#define BNO055_SIC_MATRIX_0_MSB_ADDR                              0X44 
#define BNO055_SIC_MATRIX_1_LSB_ADDR                              0X45 
#define BNO055_SIC_MATRIX_1_MSB_ADDR                              0X46 
#define BNO055_SIC_MATRIX_2_LSB_ADDR                              0X47 
#define BNO055_SIC_MATRIX_2_MSB_ADDR                              0X48 
#define BNO055_SIC_MATRIX_3_LSB_ADDR                              0X49 
#define BNO055_SIC_MATRIX_3_MSB_ADDR                              0X4A 
#define BNO055_SIC_MATRIX_4_LSB_ADDR                              0X4B 
#define BNO055_SIC_MATRIX_4_MSB_ADDR                              0X4C 
#define BNO055_SIC_MATRIX_5_LSB_ADDR                              0X4D 
#define BNO055_SIC_MATRIX_5_MSB_ADDR                              0X4E 
#define BNO055_SIC_MATRIX_6_LSB_ADDR                              0X4F 
#define BNO055_SIC_MATRIX_6_MSB_ADDR                              0X50 
#define BNO055_SIC_MATRIX_7_LSB_ADDR                              0X51 
#define BNO055_SIC_MATRIX_7_MSB_ADDR                              0X52 
#define BNO055_SIC_MATRIX_8_LSB_ADDR                              0X53 
#define BNO055_SIC_MATRIX_8_MSB_ADDR                              0X54 

/* Accelerometer Offset registers */
#define ACCEL_OFFSET_X_LSB_ADDR                                   0X55 
#define ACCEL_OFFSET_X_MSB_ADDR                                   0X56 
#define ACCEL_OFFSET_Y_LSB_ADDR                                   0X57 
#define ACCEL_OFFSET_Y_MSB_ADDR                                   0X58 
#define ACCEL_OFFSET_Z_LSB_ADDR                                   0X59 
#define ACCEL_OFFSET_Z_MSB_ADDR                                   0X5A 

/* Magnetometer Offset registers */
#define MAG_OFFSET_X_LSB_ADDR                                     0X5B 
#define MAG_OFFSET_X_MSB_ADDR                                     0X5C 
#define MAG_OFFSET_Y_LSB_ADDR                                     0X5D 
#define MAG_OFFSET_Y_MSB_ADDR                                     0X5E 
#define MAG_OFFSET_Z_LSB_ADDR                                     0X5F 
#define MAG_OFFSET_Z_MSB_ADDR                                     0X60 

/* Gyroscope Offset register s*/
#define GYRO_OFFSET_X_LSB_ADDR                                    0X61 
#define GYRO_OFFSET_X_MSB_ADDR                                    0X62 
#define GYRO_OFFSET_Y_LSB_ADDR                                    0X63 
#define GYRO_OFFSET_Y_MSB_ADDR                                    0X64 
#define GYRO_OFFSET_Z_LSB_ADDR                                    0X65 
#define GYRO_OFFSET_Z_MSB_ADDR                                    0X66 

/* Radius registers */
#define ACCEL_RADIUS_LSB_ADDR                                     0X67 
#define ACCEL_RADIUS_MSB_ADDR                                     0X68 
#define MAG_RADIUS_LSB_ADDR                                       0X69 
#define MAG_RADIUS_MSB_ADDR                                       0X6A

//power modes
#define POWER_MODE_NORMAL                                         0X00 
#define POWER_MODE_LOWPOWER                                       0X01 
#define POWER_MODE_SUSPEND                                        0X02

/* Operation mode settings*/
#define OPERATION_MODE_CONFIG                                     0X00 
#define OPERATION_MODE_ACCONLY                                    0X01 
#define OPERATION_MODE_MAGONLY                                    0X02 
#define OPERATION_MODE_GYRONLY                                    0X03 
#define OPERATION_MODE_ACCMAG                                     0X04 
#define OPERATION_MODE_ACCGYRO                                    0X05 
#define OPERATION_MODE_MAGGYRO                                    0X06 
#define OPERATION_MODE_AMG                                        0X07 
#define OPERATION_MODE_IMUPLUS                                    0X08 
#define OPERATION_MODE_COMPASS                                    0X09 
#define OPERATION_MODE_M4G                                        0X0A 
#define OPERATION_MODE_NDOF_FMC_OFF                               0X0B 
#define OPERATION_MODE_NDOF                                       0X0C

#define REMAP_CONFIG_P0                                           0x21 
#define REMAP_CONFIG_P1                                           0x24  // default
#define REMAP_CONFIG_P2                                           0x24 
#define REMAP_CONFIG_P3                                           0x21 
#define REMAP_CONFIG_P4                                           0x24 
#define REMAP_CONFIG_P5                                           0x21 
#define REMAP_CONFIG_P6                                           0x21 
#define REMAP_CONFIG_P7                                           0x24

#define REMAP_SIGN_P0                                             0x04 
#define REMAP_SIGN_P1                                             0x00  // default
#define REMAP_SIGN_P2                                             0x06 
#define REMAP_SIGN_P3                                             0x02 
#define REMAP_SIGN_P4                                             0x03 
#define REMAP_SIGN_P5                                             0x01 
#define REMAP_SIGN_P6                                             0x07 
#define REMAP_SIGN_P7                                             0x05
/* Page 1 registers */
#define BNO055_UNIQUE_ID_ADDR                                     0x50

// Unit modes
#define MPERSPERS   0x00
#define MILLIG      0x01
#define DEG_PER_SEC 0x00
#define RAD_PER_SEC 0x02
#define DEGREES     0x00
#define RADIANS     0x04
#define CENTIGRADE  0x00
#define FAHRENHEIT  0x10
#define WINDOWS     0x00
#define ANDROID     0x80

typedef struct coord_data {
    char    regdata[8];           // raw
    int16_t regw,regx,regy,regz;  // formatted
    float   w,x,y,z;              // scaled, float
    } coord_data_t;
    
/** BNO055 Class
 *  Library for controlling the BNO055 IMU motion tracking sensor
 */
class BNO055 {
    public:
        BNO055(PinName sda, PinName scl);
        bool reset();
        bool begin();
        bool get_UniqueID(char *UniqueID_buffer);
        int  get_temp(void);
        bool get_accel(char *buffer);
        bool get_gyro(char *buffer);   
        bool get_mag(char *buffer);   
        bool get_lia(char *buffer);   
        bool get_grav(char *buffer);   
        bool get_quat(char *buffer);   
        bool get_angles(char *buffer);   
             
        uint8_t get_calib_status(void);
        bool get_calib_data(char *calib_data);
        bool set_calib_data(char *calib_data);        
        void set_mapping(char orientation);
        void set_accel_units(char units);
        void set_anglerate_units(char units);
        void set_angle_units(char units);
        void set_temp_units(char units);
        void set_orientation(char units);
        bool set_mode(uint8_t mode);
        bool set_extCrystalUse(bool usextal);
        bool restore_mode();        
       
        uint8_t _address;
        uint8_t _mode;
        uint8_t _saved_mode; 
        
    protected:
        float accel_scale;
        float rate_scale;
        float angle_scale;
        int temp_scale; 

    private:
        I2C i2c;
        char readRegister8(char reg);     
        bool read_byte(char reg, char val);
        bool readLen (char reg, char *buffer, uint8_t len);
        bool writeRegister8(char reg, char val);
        bool writeLen(char reg, char *buffer, uint8_t len);
               
};