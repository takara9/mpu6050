#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import mpu6050
from time import sleep   

if __name__ == "__main__":
    m = mpu6050.mpu6050()

    # Main loop
    while (m.is_online() == 1):

        # Read the x/y/z adc values
        (ax,ay,az) = m.readAccelData()
        # Print acceleration values in milligs!
        print "X-acceleration: %4.0f mg" % ax
        print "Y-acceleration: %4.0f mg" % ay
        print "Z-acceleration: %4.0f mg" % az


        # Read the x/y/z adc values
        (gx,gy,gz) = m.readGyroData()
        # Print gyro values in degree/sec
        print "X-gyro rate: %2.1f degrees/sec " % gx
        print "Y-gyro rate: %2.1f degrees/sec " % gy
        print "Z-gyro rate: %2.1f degrees/sec " % gz


        # Read the x/y/z adc values
        temperature = m.readTempData()
        # Print temperature in degrees Centigrade      
        # Print T values to tenths of s degree C
        print "Temperature is %2.2f degrees C"  % temperature
        print
        sleep(1)


    print "MPU6050 is offline"
