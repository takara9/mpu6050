#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# MPU6050 Driver
#
# 2016/12/29 
# Maho Takara
#
# Reference documents
#   https://github.com/kriswiner/MPU-6050/blob/master/MPU6050BasicExample.ino
#   https://store.invensense.com/Datasheets/invensense/RM-MPU-6000A.pdf
#

from smbus2 import SMBus # https://pypi.python.org/pypi/smbus2/0.1.4
import math              # mathmatics
from time import sleep   # time module

# I2C
# slave address
DEV_ADDR   = 0x68       # device address

# MPU-6050 Registor Map
# https://store.invensense.com/Datasheets/invensense/RM-MPU-6000A.pdf
# register address
XGOFFS_TC        = 0x00 # Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
YGOFFS_TC        = 0x01                                                                          
ZGOFFS_TC        = 0x02
X_FINE_GAIN      = 0x03 # [7:0] fine gain
Y_FINE_GAIN      = 0x04
Z_FINE_GAIN      = 0x05
XA_OFFSET_H      = 0x06 # User-defined trim values for accelerometer
XA_OFFSET_L_TC   = 0x07
YA_OFFSET_H      = 0x08
YA_OFFSET_L_TC   = 0x09
ZA_OFFSET_H      = 0x0A
ZA_OFFSET_L_TC   = 0x0B
SELF_TEST_X      = 0x0D
SELF_TEST_Y      = 0x0E    
SELF_TEST_Z      = 0x0F
SELF_TEST_A      = 0x10
XG_OFFS_USRH     = 0x13  # User-defined trim values for gyroscope; supported in MPU-6050?
XG_OFFS_USRL     = 0x14
YG_OFFS_USRH     = 0x15
YG_OFFS_USRL     = 0x16
ZG_OFFS_USRH     = 0x17
ZG_OFFS_USRL     = 0x18
SMPLRT_DIV       = 0x19
CONFIG           = 0x1A
GYRO_CONFIG      = 0x1B
ACCEL_CONFIG     = 0x1C
FF_THR           = 0x1D  # Free-fall
FF_DUR           = 0x1E  # Free-fall
MOT_THR          = 0x1F  # Motion detection threshold bits [7:0]
MOT_DUR          = 0x20  # Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
ZMOT_THR         = 0x21  # Zero-motion detection threshold bits [7:0]
ZRMOT_DUR        = 0x22  # Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
FIFO_EN          = 0x23
I2C_MST_CTRL     = 0x24   
I2C_SLV0_ADDR    = 0x25
I2C_SLV0_REG     = 0x26
I2C_SLV0_CTRL    = 0x27
I2C_SLV1_ADDR    = 0x28
I2C_SLV1_REG     = 0x29
I2C_SLV1_CTRL    = 0x2A
I2C_SLV2_ADDR    = 0x2B
I2C_SLV2_REG     = 0x2C
I2C_SLV2_CTRL    = 0x2D
I2C_SLV3_ADDR    = 0x2E
I2C_SLV3_REG     = 0x2F
I2C_SLV3_CTRL    = 0x30
I2C_SLV4_ADDR    = 0x31
I2C_SLV4_REG     = 0x32
I2C_SLV4_DO      = 0x33
I2C_SLV4_CTRL    = 0x34
I2C_SLV4_DI      = 0x35
I2C_MST_STATUS   = 0x36
INT_PIN_CFG      = 0x37
INT_ENABLE       = 0x38
DMP_INT_STATUS   = 0x39  # Check DMP interrupt
INT_STATUS       = 0x3A
ACCEL_XOUT_H     = 0x3B
ACCEL_XOUT_L     = 0x3C
ACCEL_YOUT_H     = 0x3D
ACCEL_YOUT_L     = 0x3E
ACCEL_ZOUT_H     = 0x3F
ACCEL_ZOUT_L     = 0x40
TEMP_OUT_H       = 0x41
TEMP_OUT_L       = 0x42
GYRO_XOUT_H      = 0x43
GYRO_XOUT_L      = 0x44
GYRO_YOUT_H      = 0x45
GYRO_YOUT_L      = 0x46
GYRO_ZOUT_H      = 0x47
GYRO_ZOUT_L      = 0x48
EXT_SENS_DATA_00 = 0x49
EXT_SENS_DATA_01 = 0x4A
EXT_SENS_DATA_02 = 0x4B
EXT_SENS_DATA_03 = 0x4C
EXT_SENS_DATA_04 = 0x4D
EXT_SENS_DATA_05 = 0x4E
EXT_SENS_DATA_06 = 0x4F
EXT_SENS_DATA_07 = 0x50
EXT_SENS_DATA_08 = 0x51
EXT_SENS_DATA_09 = 0x52
EXT_SENS_DATA_10 = 0x53
EXT_SENS_DATA_11 = 0x54
EXT_SENS_DATA_12 = 0x55
EXT_SENS_DATA_13 = 0x56
EXT_SENS_DATA_14 = 0x57
EXT_SENS_DATA_15 = 0x58
EXT_SENS_DATA_16 = 0x59
EXT_SENS_DATA_17 = 0x5A
EXT_SENS_DATA_18 = 0x5B
EXT_SENS_DATA_19 = 0x5C
EXT_SENS_DATA_20 = 0x5D
EXT_SENS_DATA_21 = 0x5E
EXT_SENS_DATA_22 = 0x5F
EXT_SENS_DATA_23 = 0x60
MOT_DETECT_STATUS = 0x61
I2C_SLV0_DO      = 0x63
I2C_SLV1_DO      = 0x64
I2C_SLV2_DO      = 0x65
I2C_SLV3_DO      = 0x66
I2C_MST_DELAY_CTRL = 0x67
SIGNAL_PATH_RESET  = 0x68
MOT_DETECT_CTRL   = 0x69
USER_CTRL        = 0x6A  # Bit 7 enable DMP, bit 3 reset DMP
PWR_MGMT_1       = 0x6B # Device defaults to the SLEEP mode
PWR_MGMT_2       = 0x6C
DMP_BANK         = 0x6D  # Activates a specific bank in the DMP
DMP_RW_PNT       = 0x6E  # Set read/write pointer to a specific start address in specified DMP bank
DMP_REG          = 0x6F  # Register in DMP from which to read or to which to write
DMP_REG_1        = 0x70
DMP_REG_2        = 0x71
FIFO_COUNTH      = 0x72
FIFO_COUNTL      = 0x73
FIFO_R_W         = 0x74
WHO_AM_I         = 0x75 # Should return = 0x68



class mpu6050:
    def __init__(self):

        # Global variable
	self.bus = SMBus(1)
	self.gyroBias = 3*[0.0]
	self.accelBias = 3*[0.0]

        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)
        c = self.bus.read_byte_data(DEV_ADDR,WHO_AM_I)
        if c == 0x68:
            print "MPU-6050 is online..."

        # Start by performing self test and reporting values
        SelfTest = self.MPU6050selfTest()

        print "x-axis self test: acceleration trim within : %2.4f pst of factory value" % SelfTest[0]
        print "y-axis self test: acceleration trim within : %2.4f pst of factory value" % SelfTest[1]
        print "z-axis self test: acceleration trim within : %2.4f pst of factory value" % SelfTest[2]

        print "x-axis self test: gyration trim within : %2.4f pst of factory value" % SelfTest[3]
        print "y-axis self test: gyration trim within : %2.4f pst of factory value" % SelfTest[4]
        print "z-axis self test: gyration trim within : %2.4f pst of factory value" % SelfTest[5]

        self_test_pass = True
        for i in range(0,6):
            if SelfTest[i] > 1.0:
                self_test_pass = False
        
        print "Selt-Test Pass: ",self_test_pass

        # Calibrate gyro and accelerometers, load biases in bias registers  
        (gyroBias,accelBias) = self.calibrateMPU6050()
        print "Bias X-gyro rate: %2.1f degrees/sec " % gyroBias[0]
        print "Bias Y-gyro rate: %2.1f degrees/sec " % gyroBias[1]
        print "Bisa Z-gyro rate: %2.1f degrees/sec " % gyroBias[2]
        print "X-acceleration Bias: %4.0f mg" % (1000 * float(accelBias[0]))
        print "Y-acceleration Bias: %4.0f mg" % (1000 * float(accelBias[1]))
        print "Z-acceleration Bias: %4.0f mg" % (1000 * float(accelBias[2]))


        # Initialize device for active mode read of acclerometer, gyroscope, and temperature
        self.initMPU6050()
        print "MPU6050 initialized for active data mode...."


    # Accelerometer and gyroscope self test; check calibration wrt factory settings
    # Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    def MPU6050selfTest(self):
        # Configure the accelerometer for self-test
        # Enable self test on all three axes and set accelerometer range to +/- 8 g
	self.bus.write_byte_data(DEV_ADDR, ACCEL_CONFIG, 0xF0)
        # Enable self test on all three axes and set gyro range to +/- 250 degrees/s
        self.bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, 0xF0)
        # Delay a while to let the device execute the self-test
        sleep(0.25)

        rawData = 4 * [0]
        selfTest = 6 * [0]
        factoryTrim = 6 * [0.0]
        destination = 6 * [0.0]

        # X-axis self-test results
        rawData[0] = self.bus.read_byte_data(DEV_ADDR, SELF_TEST_X)
        # Y-axis self-test results
        rawData[1] = self.bus.read_byte_data(DEV_ADDR, SELF_TEST_Y)
        # Z-axis self-test results
        rawData[2] = self.bus.read_byte_data(DEV_ADDR, SELF_TEST_Z)
        # Mixed-axis self-test results
        rawData[3] = self.bus.read_byte_data(DEV_ADDR, SELF_TEST_A)

        # Extract the acceleration test results first
        # XA_TEST result is a five-bit unsigned integer
        selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4
        # YA_TEST result is a five-bit unsigned integer
        selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2
        # ZA_TEST result is a five-bit unsigned integer
        selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0

        # Extract the gyration test results first
        # XG_TEST result is a five-bit unsigned integer
        selfTest[3] = rawData[0] & 0x1F
        # YG_TEST result is a five-bit unsigned integer
        selfTest[4] = rawData[1] & 0x1F
        # ZG_TEST result is a five-bit unsigned integer   
        selfTest[5] = rawData[2] & 0x1F

        # Process results to allow final comparison with factory set values
        # FT[Xa] factory trim calculation
        tmp = float(selfTest[0])
        factoryTrim[0] = (4096.0*0.34)*( pow( (0.92/0.34),((tmp - 1.0)/30.0)) )
        # FT[Ya] factory trim calculation
        tmp = float(selfTest[1])
        factoryTrim[1] = (4096.0*0.34)*( pow( (0.92/0.34),((tmp - 1.0)/30.0)) )
        # FT[Za] factory trim calculation
        tmp = float(selfTest[2])
        factoryTrim[2] = (4096.0*0.34)*( pow( (0.92/0.34),((tmp - 1.0)/30.0)) )
        # FT[Xg] factory trim calculation
        tmp = float(selfTest[3])
        factoryTrim[3] = (25.0*131.0)*( pow( 1.046 , (tmp - 1.0)) )
        # FT[Yg] factory trim calculation
        tmp = float(selfTest[4])
        factoryTrim[4] = (-25.0*131.0)*( pow( 1.046 , (tmp - 1.0)) )
        # FT[Zg] factory trim calculation
        tmp = float(selfTest[5])
        factoryTrim[5] = (25.0*131.0)*( pow( 1.046 , (tmp - 1.0)) )

        # Output self-test results and factory trim calculation
        #print selfTest[0], selfTest[1], selfTest[2]
        #print selfTest[3], selfTest[4], selfTest[5]
        #print factoryTrim[0], factoryTrim[1], factoryTrim[2]
        #print factoryTrim[3], factoryTrim[4], factoryTrim[5]

        # Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
        # To get to percent, must multiply by 100 and subtract result from 100
        for i in range(0,6):
            # Report percent differences
            tmp = float(selfTest[i])
            destination[i] = 100.0 + 100.0*(tmp - factoryTrim[i])/factoryTrim[i]

        return destination

    # Calibrate gyro and accelerometers, load biases in bias registers  
    #
    # Function which accumulates gyro and accelerometer data after device initialization. 
    # It calculates the average of the at-rest readings and then loads the resulting 
    # offsets into accelerometer and gyro bias registers.
    def calibrateMPU6050(self):
        data = 12 * [0]
        gyro_bias  = 3 * [0]
        accel_bias = 3 * [0]

        # reset device, reset all registers, clear gyro and accelerometer bias registers
        # Write a one to bit 7 reset bit; toggle reset device delay(100)
        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0x80)
        sleep(0.8)
    
        # get stable time source
        # Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0x01)
        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_2, 0x00)
        sleep(0.2)


        # Configure device for bias calculation
        # Disable all interrupts
        self.bus.write_byte_data(DEV_ADDR, INT_ENABLE, 0x00)
        # Disable FIFO
        self.bus.write_byte_data(DEV_ADDR, FIFO_EN, 0x00)
        # Turn on internal clock source
        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0x00)
        # Disable I2C master
        self.bus.write_byte_data(DEV_ADDR, I2C_MST_CTRL, 0x00)
        # Disable FIFO and I2C master modes
        self.bus.write_byte_data(DEV_ADDR, USER_CTRL, 0x00)
        # Reset FIFO and DMP
        self.bus.write_byte_data(DEV_ADDR, USER_CTRL, 0x0C)
        sleep(0.15)


        # Configure MPU6050 gyro and accelerometer for bias calculation
        # Set low-pass filter to 188 Hz
        self.bus.write_byte_data(DEV_ADDR, CONFIG, 0x01)
        # Set sample rate to 1 kHz
        self.bus.write_byte_data(DEV_ADDR, SMPLRT_DIV, 0x00)
        # Set gyro full-scale to 250 degrees per second, maximum sensitivity
        self.bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, 0x00)
        # Set accelerometer full-scale to 2 g, maximum sensitivity
        self.bus.write_byte_data(DEV_ADDR, ACCEL_CONFIG, 0x00)
        # = 131 LSB/degrees/sec
        gyrosensitivity  = 131
        # = 16384 LSB/g
        accelsensitivity = 16384


        # Configure FIFO to capture accelerometer and gyro data for bias calculation
        # Enable FIFO  
        self.bus.write_byte_data(DEV_ADDR, USER_CTRL, 0x40)
        # Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
        self.bus.write_byte_data(DEV_ADDR, FIFO_EN, 0x78)
        # accumulate 80 samples in 80 milliseconds = 960 bytes
        sleep(0.080)

        # At end of sample accumulation, turn off FIFO sensor read
        # Disable gyro and accelerometer sensors for FIFO
        self.bus.write_byte_data(DEV_ADDR, FIFO_EN, 0x00)
        # read FIFO sample count
        data = self.bus.read_i2c_block_data(DEV_ADDR, FIFO_COUNTH, 2)
        fifo_count = (data[0] << 8) | data[1]
        # How many sets of full gyro and accelerometer data for averaging
        packet_count = fifo_count/12;

        for ii in range(0, packet_count):
            accel_temp = [0, 0, 0]
            gyro_temp  = [0, 0, 0]

            # read data for averaging
            data = self.bus.read_i2c_block_data(DEV_ADDR, FIFO_R_W, 12)

            # Form signed 16-bit integer for each sample in FIFO
            accel_temp[0] = self.signed_int(data[0] << 8 | data[1])
            accel_temp[1] = self.signed_int(data[2] << 8 | data[3])
            accel_temp[2] = self.signed_int(data[4] << 8 | data[5])
            gyro_temp[0]  = self.signed_int(data[6] << 8 | data[7])
            gyro_temp[1]  = self.signed_int(data[8] << 8 | data[9])
            gyro_temp[2]  = self.signed_int(data[10] << 8 | data[11])
    
            # Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            for i in range(0,3):
                accel_bias[i] = accel_bias[i] + accel_temp[i]
                gyro_bias[i]  = gyro_bias[i] + gyro_temp[i]

            # End of for ii

        # Normalize sums to get average count biases
        for i in range(0,3):
            accel_bias[i] = accel_bias[i] / packet_count
            gyro_bias[i] = gyro_bias[i] / packet_count

        # Remove gravity from the z-axis accelerometer bias calculation
        # 基板の向きが地面と平行になっていること
        if accel_bias[2] > 0:
            accel_bias[2] = accel_bias[2] - accelsensitivity
        else:
            accel_bias[2] = accel_bias[2] + accelsensitivity

        # construct gyro bias in deg/s for later manual subtraction
        self.gyroBias[0] = float(gyro_bias[0]) / float(gyrosensitivity)
        self.gyroBias[1] = float(gyro_bias[1]) / float(gyrosensitivity)
        self.gyroBias[2] = float(gyro_bias[2]) / float(gyrosensitivity)

        # Construct the accelerometer biases for push to the hardware accelerometer bias registers. 
        # These registers contain factory trim values which must be added to the calculated 
        # accelerometer biases; on boot up these registers will hold non-zero values. 
        # In addition, bit 0 of the lower byte must be preserved since it is used for temperature
        # compensation calculations. Accelerometer bias registers expect bias input 
        # as 2048 LSB per g, so that the accelerometer biases calculated above must be divided by 8.

        # A place to hold the factory accelerometer trim biases
        accel_bias_reg = [0, 0, 0]

        # Read factory accelerometer trim values
        data_byte = self.bus.read_i2c_block_data(DEV_ADDR, XA_OFFSET_H, 2)
        accel_bias_reg[0] = self.signed_int((data_byte[0] << 8)| data_byte[1])
        data_byte = self.bus.read_i2c_block_data(DEV_ADDR, YA_OFFSET_H, 2)
        accel_bias_reg[1] = self.signed_int((data_byte[0] << 8) | data_byte[1])
        data_byte = self.bus.read_i2c_block_data(DEV_ADDR, ZA_OFFSET_H, 2)
        accel_bias_reg[2] = self.signed_int((data_byte[0] << 8) | data_byte[1])
  
        # Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
        mask = 1

        # Define array to hold mask bit for each accelerometer bias axis
        mask_bit = [0, 0, 0]
  
        # If temperature compensation bit is set, record that fact in mask_bit
        for ii in range(0,3):
            if accel_bias_reg[ii] & mask:
                mask_bit[ii] = 0x01


        # Construct total accelerometer bias, 
        # including calculated average accelerometer bias from above
        # Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
        accel_bias_reg[0] = accel_bias_reg[0] - accel_bias[0]/8
        accel_bias_reg[1] = accel_bias_reg[1] - accel_bias[1]/8
        accel_bias_reg[2] = accel_bias_reg[2] - accel_bias[2]/8

        # Output scaled accelerometer biases for manual subtraction in the main program
        self.accelBias[0] = float(accel_bias[0]) / float(accelsensitivity)
        self.accelBias[1] = float(accel_bias[1]) / float(accelsensitivity)
        self.accelBias[2] = float(accel_bias[2]) / float(accelsensitivity)
    
        return(self.gyroBias,self.accelBias)


    # Initialize MPU6050 device
    def initMPU6050(self):
        # wake up device-don't need this here if using calibration function below
        # Clear sleep mode bit (6), enable all sensors 
        #writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
        # Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  
        #delay(100); 

        # get stable time source
        # Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
        self.bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0x01)

        # Configure Gyro and Accelerometer
        # Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
        # DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
        self.bus.write_byte_data(DEV_ADDR, CONFIG, 0x03)
 
        # Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        # Use a 200 Hz sample rate 
        self.bus.write_byte_data(DEV_ADDR, SMPLRT_DIV, 0x04)


        # Set gyroscope full scale range
        # Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
        c = self.bus.read_byte_data(DEV_ADDR, GYRO_CONFIG)
        Gscale = 0 #GFS_250DPS

        # Clear self-test bits [7:5] 
        self.bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, c & ~0xE0)
        # Clear AFS bits [4:3]
        self.bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, c & ~0x18)
        # Set full scale range for the gyro
        self.bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, c | Gscale << 3)


        # Set accelerometer configuration
        c = self.bus.read_byte_data(DEV_ADDR, ACCEL_CONFIG)
        Ascale = 0 #AFS_2G;

        # Clear self-test bits [7:5] 
        self.bus.write_byte_data(DEV_ADDR, ACCEL_CONFIG, c & ~0xE0)
        # Clear AFS bits [4:3]
        self.bus.write_byte_data(DEV_ADDR, ACCEL_CONFIG, c & ~0x18)
        # Set full scale range for the accelerometer 
        self.bus.write_byte_data(DEV_ADDR, ACCEL_CONFIG, c | Ascale << 3)

        # Configure Interrupts and Bypass Enable
        # Set interrupt pin active high, push-pull, 
        # and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
        # can join the I2C bus and all can be controlled by the Arduino as master
        self.bus.write_byte_data(DEV_ADDR, INT_PIN_CFG, 0x02)

        # Enable data ready (bit 0) interrupt
        self.bus.write_byte_data(DEV_ADDR, INT_ENABLE, 0x01)


        
    def readAccelData(self):
        # x/y/z accel register data stored here
        accelCount = 3 * [0]

        # Read the six raw data registers into data array    
        rawData = self.bus.read_i2c_block_data(DEV_ADDR, ACCEL_XOUT_H, 6)

        # Turn the MSB and LSB into a signed 16-bit value
        accelCount[0] = self.signed_int((rawData[0] << 8) | rawData[1])
        accelCount[1] = self.signed_int((rawData[2] << 8) | rawData[3])
        accelCount[2] = self.signed_int((rawData[4] << 8) | rawData[5])

        # Now we'll calculate the accleration value into actual mg's
        # get actual mg value, this depends on scale being set
        aRes = 2.0/32768.0
        ax = (float(accelCount[0] * aRes) - self.accelBias[0]) * 1000
        ay = (float(accelCount[1] * aRes) - self.accelBias[1]) * 1000
        az = (float(accelCount[2] * aRes) - self.accelBias[2]) * 1000
        return(ax,ay,az)


    def readGyroData(self):
        # x/y/z gyro register data stored here
        gyroCount = 3 * [0]

        # Read the six raw data registers sequentially into data array
        rawData = self.bus.read_i2c_block_data(DEV_ADDR, GYRO_XOUT_H, 6)

        # Turn the MSB and LSB into a signed 16-bit value
        gyroCount[0] = self.signed_int((rawData[0] << 8) | rawData[1])
        gyroCount[1] = self.signed_int((rawData[2] << 8) | rawData[3])
        gyroCount[2] = self.signed_int((rawData[4] << 8) | rawData[5])

        # Calculate the gyro value into actual degrees per second
        # get actual gyro value, this depends on scale being set
        gRes = 250.0/32768.0;
        gx = float(gyroCount[0] * gRes) - self.gyroBias[0]
        gy = float(gyroCount[1] * gRes) - self.gyroBias[1]
        gz = float(gyroCount[2] * gRes) - self.gyroBias[2]
        return(gx,gy,gz)


    def readTempData(self):
        # Read the two raw data registers sequentially into data array 
        rawData = self.bus.read_i2c_block_data(DEV_ADDR, TEMP_OUT_H, 2)
        # Turn the MSB and LSB into a 16-bit value
        rawData = rawData[0] << 8 | rawData[1]
        # Tuen the unsigned int to the signed int
        # Temperature in degrees Centigrade
        return float(self.signed_int(rawData) / 340.0) + 36.53

    def signed_int(self,unsigned_int):
        if (unsigned_int >= 0x8000): # negative value
            return -((0xffff - unsigned_int) + 1)
        else:                        # positive value
            return unsigned_int

    def is_online(self):
        # If data ready bit set, all data registers have new data
        # check if data ready interrupt
        return (self.bus.read_byte_data(DEV_ADDR, INT_STATUS) & 0x01)


if __name__ == "__main__":
    pass


