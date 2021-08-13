#!/usr/bin/python-sirius
# -*- coding: utf-8 -*-
"""


[00..03] Command
[04..07] Position LSB (L..M)
[08..11] Position MSB (L..M)
[12..15] CRC
[16..19] Mutex
"""

import mmap



# ----- ADDRESS AND OFFSETS - AM572x
PRU1_ADDR = 0x4b200000
PRU2_ADDR = 0x4b280000
SHRAM_OFFSET = 0x00010000

# ----- Available PRUs
PRUS = {
        0:{"subsystem":1, "posShramOffset":4}, 
        1:{"subsystem":1, "posShramOffset":24}, 
        2:{"subsystem":2, "posShramOffset":4}, 
        3:{"subsystem":2, "posShramOffset":24}
        }

# ----- Open memory mapping
fd = open("/dev/mem", "r")
pru_data = {
            1:mmap.mmap(fd.fileno(), length=0x1000, access=mmap.ACCESS_READ, offset=(PRU1_ADDR + SHRAM_OFFSET)),
            2:mmap.mmap(fd.fileno(), length=0x1000, access=mmap.ACCESS_READ, offset=(PRU2_ADDR + SHRAM_OFFSET))
            }


class encoderHeidenhain:
    def __init__(self, pru_num):
        if pru_num in PRUS.keys():
            self.pru = pru_num
            self.position = self.getPosition()
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
        if(pru_data[PRUS[self.pru]["subsystem"]][16] == 0):
            return 2
        else:
            return pru_data[PRUS[self.pru]["subsystem"]][offset]



if __name__ == "__main__":
    exit()
