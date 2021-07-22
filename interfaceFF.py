#!/usr/bin/python-sirius
# -*- coding: utf-8 -*-

"""
"""


import mmap



# ----- ADDRESS AND OFFSETS - AM572x
PRU1_ADDR = 0x4b200000
PRU2_ADDR = 0x4b280000
SHRAM_OFFSET = 0x00010000

# ----- Available PRUs
PRUS = {
        0:{"subsystem":1, "posOffset":4}, 
        1:{"subsystem":1, "posOffset":8}, 
        2:{"subsystem":2, "posOffset":4}, 
        3:{"subsystem":2, "posOffset":8}
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
        _byte0 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posOffset"]+0]
        _byte1 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posOffset"]+1]
        _byte2 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posOffset"]+2]
        _byte3 = pru_data[PRUS[self.pru]["subsystem"]][PRUS[self.pru]["posOffset"]+3]

        self.position =  _byte0 + (_byte1 << 8) + (_byte2 << 16) + (_byte3 << 24)
        return self.position

