import numpy as np
import serial
import time

class READIMU(object):
    def __init__(self, ComPort) -> None:
        #IMU variables
        self.ComPort = ComPort
        self.AngleX_Left = 0
        self.AngleX_Right = 0
        self.AngleVelX_Left = 0
        self.AngleVelX_Right = 0
        self.Header1 = 0xc4
        self.Header2 = 0xc4
        self.XIMU=0
        self.L_XIMU_int16=0
        self.R_XIMU_int16=0
        self.L_XVIMU_int16=0
        self.R_XVIMU_int16=0
        self.XIMUL=0
        self.XIMUR=0
        self.XVIMUL=0
        self.XVIMUR=0
        #Serial Variables --------
        self.buffer = 0x00
        self.buffer_len = 0x00
        #Serial Begin -------------
        #self.Serial_IMU = serial.Serial(ComPort, 230400, timeout=0.007, parity=serial.PARITY_NONE)
        #self.Serial_IMU = serial.Serial(ComPort, 115200, timeout=0.007, parity=serial.PARITY_NONE)
        self.Serial_IMU = serial.Serial(ComPort, 115200, timeout=0.007, parity=serial.PARITY_NONE)

        print('Serial Open Success')
        #Serial END---------------

    def read(self):
        self.buffer = self.Serial_IMU.read(11)
        self.buffer_len = len(self.buffer)
        #print(self.buffer)

    def send(self,b1,b2,b3,b4):
        self.Serial_IMU.write(bytearray([0x40, 0x41, 0x42, 
                               b1, b2, b3, b4, 
                               0x43]))
    
    def printHEX(self, Vprint):
        print([hex(x) for x in Vprint])

    def ToUint(self, x, x_min, x_max, nbits):
        span = x_max - x_min

        if (x < x_min):
            x = x_min

        if (x > x_max):
            x = x_max
        toUint=((x - x_min) * ((float)((1 << nbits) - 1) / span))
        return toUint
        

    def ToFloat(self,x_int, x_min, x_max, nbits):
        span = x_max - x_min
        offset_value = x_min
        toFloat= x_int * span / float((((1 << nbits) - 1))) + offset_value
        return toFloat


    def decode(self):
        #if len(self.buffer)==7 and self.buffer[0]==0x3a and self.buffer[1]==0xc4 :
        if len(self.buffer)==11 and self.buffer[0]==0x31 and self.buffer[1]==0x32  and self.buffer[10]==0x33:
            self.L_XIMU_int16=(self.buffer[2] << 8) | (self.buffer[3])
            self.R_XIMU_int16=(self.buffer[4] << 8) | (self.buffer[5])
            self.L_XVIMU_int16=(self.buffer[6] << 8) | (self.buffer[7])
            self.R_XVIMU_int16=(self.buffer[8] << 8) | (self.buffer[9])


            self.XIMUL=self.ToFloat(self.L_XIMU_int16,-180,180,16)
            self.XIMUR=self.ToFloat(self.R_XIMU_int16,-180,180,16)
            self.XVIMUL=self.ToFloat(self.L_XVIMU_int16,-800,800,16)
            self.XVIMUR=self.ToFloat(self.R_XVIMU_int16,-800,800,16)

            '''    self.XIMU=((self.buffer[6] << 24) | (self.buffer[5]) << 16| 
            (self.buffer[4] << 8) | (self.buffer[3]))'''
            

            #print(hex(self.buffer))




