#!/usr/bin/python-sirius
# -*- coding: utf-8 -*-
"""


"""

import mmap
import os


# ----- ADDRESS AND OFFSETS - AM572x
PRU1_ADDR = 0x4b200000
PRU2_ADDR = 0x4b280000
SHRAM_OFFSET = 0x00010000

# ----- Available PRUs
PRUS = {
        0:{"remoteproc":4, "subsystem":1, "device":0, "posShramOffset":8  , "clk_t": 17, "data_t":  5}, 
        1:{"remoteproc":5, "subsystem":1, "device":1, "posShramOffset":40 , "clk_t": 18, "data_t":  9}, 
        2:{"remoteproc":6, "subsystem":2, "device":0, "posShramOffset":8  , "clk_t":  4, "data_t":  3}, 
        3:{"remoteproc":7, "subsystem":2, "device":1, "posShramOffset":40 , "clk_t": 10, "data_t": 11}
        }

# ----- Open memory mapping
fd = os.open("/dev/mem", os.O_RDWR)
pru_data = {
            1:mmap.mmap(fd, length=0x1000, access=mmap.ACCESS_WRITE, offset=(PRU1_ADDR + SHRAM_OFFSET)),
            2:mmap.mmap(fd, length=0x1000, access=mmap.ACCESS_WRITE, offset=(PRU2_ADDR + SHRAM_OFFSET))
            }


class encoderHeidenhain:
    def __init__(self, pru_num):
        if pru_num in PRUS.keys():
            self.pru = pru_num
            pru_data[PRUS[self.pru]["subsystem"]][0] = (PRUS[self.pru]["subsystem"] << 1) + PRUS[self.pru]["device"]
            pru_data[PRUS[self.pru]["subsystem"]][1] = PRUS[self.pru]["clk_t"]
            pru_data[PRUS[self.pru]["subsystem"]][2] = PRUS[self.pru]["data_t"]

            #os.system("cp Teste-PRU.out /lib/firmware/encoder.out")
            os.system("echo 'stop' > /sys/class/remoteproc/remoteproc{}/state".format(PRUS[self.pru]["remoteproc"]))
            os.system("echo 'encoder.out' > /sys/class/remoteproc/remoteproc{}/firmware".format(PRUS[self.pru]["remoteproc"]))
            os.system("echo 'start' > /sys/class/remoteproc/remoteproc{}/state".format(PRUS[self.pru]["remoteproc"]))
            #self.position = self.getPosition()
        else:
            raise ValueError('PRU number is not available !')
    
    def getPosition(self):
        _byte0 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posShramOffset"]+0]
        _byte1 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posShramOffset"]+1]
        _byte2 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posShramOffset"]+2]
        _byte3 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posShramOffset"]+3]
        _byte4 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posShramOffset"]+4]

        _reverseBits00to31 = '{:32b}'.format(_byte0 + (_byte1 << 8) + (_byte2 << 16) + (_byte3 << 24)).replace(" ","0")
        _reverseBits32to34 = '{:4b}'.format(_byte4).replace(" ","0")
        _binaryData = (_reverseBits00to31 + _reverseBits32to34)[::-1]

        return int(_binaryData, 2)

    def getMemory(self, offset):
        if(pru_data[PRUS[self.pru]["subsystem"]][20] == 0):
            return 'busy'
        else:
            return pru_data[PRUS[self.pru]["subsystem"]][offset]

if __name__ == "__main__":
    exit()
