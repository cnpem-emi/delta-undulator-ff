/*
 * Heidenhain Encoder Sniffer
 *
 * For Delta Undulator Feed-forward corrections
 * 1-channel sniffer
 *
 * To be run on a BeagleBone-AI PRU.
 *
 *
 * Under development
 * Patricia Nallin - CNPEM
 * patricia.nallin@cnpem.br
 *
 * */

#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include <stdint.h>
#include <stdio.h>
#include "resource_table_empty.h"

extern uint64_t readBitsAsmCh0(uint32_t);
extern uint64_t readBitsAsmCh1(uint32_t);
extern uint64_t readBitsAsmCh2(uint32_t);
extern uint64_t readBitsAsmCh3(uint32_t);

volatile register uint32_t __R31;
volatile register uint32_t __R30;

#define PRU_SHARED_MEM_ADDR 0x00010000

#define RETURN_LINE_TIMEOUT -1
#define RETURN_OK 0

#define FALLING_EDGE 0
#define RISING_EDGE 1

volatile uint8_t continue_allowed;
volatile uint8_t tbit_c, tbit_d;

void dummyCycles(uint8_t nCycles, uint16_t samplingEdge);
uint32_t readBits(uint8_t nBits, uint16_t samplingEdge);

void main() {
  // General variables
  uint32_t loop = 0;
  uint64_t dataEncoder;
  uint8_t offsetCommand, offsetPosM, offsetPosL, offsetCRC, offsetMutex;
  uint8_t subsystemPRU, devPRU, unit;

  // Clear SYSCFG[STANDBY_INIT] to enable OCP master port->Shared memory
  CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

  // Pointer to shram
  volatile int* shram = (volatile int*)PRU_SHARED_MEM_ADDR;

  /* GPI Mode 0, GPO Mode 0 */
  CT_CFG.GPCFG0 = 0;

  devPRU = shram[0] & 0x01;               // Should be 0 or 1
  subsystemPRU = (shram[0] >> 1) & 0xFF;  // Should be 1 or 2

  tbit_c = (shram[0] >> 8) & 0xFF;
  tbit_d = (shram[0] >> 16) & 0xFF;

  // INVALID INITIALIZATION - MSByte gets 0xFF
  if ((subsystemPRU > 2) | (subsystemPRU == 0)) {
    shram[0] = shram[0] | 0xFF000000;
  } else {
    shram[0] = 0;
  }

  offsetCommand = (devPRU * 8) + 1;
  offsetPosL = (devPRU * 8) + 2;
  offsetPosM = (devPRU * 8) + 3;
  offsetCRC = (devPRU * 8) + 4;
  offsetMutex = (devPRU * 8) + 5;

  unit = ((subsystemPRU - 1) * 2) + devPRU;
  shram[20] = tbit_c + (tbit_d << 8) + (unit << 16);

  switch (unit) {
    case 0:
      tbit_c = 17;
      tbit_d = 5;
      break;
    case 1:
      tbit_c = 18;
      tbit_d = 9;
      break;
    case 2:
      tbit_c = 4;
      tbit_d = 3;
      break;
    case 3:
      tbit_c = 10;
      tbit_d = 11;
      break;
  }

  while (1) {
    // Start on a beginning of communication. Wait clock HIGH for, at least, some usec
    for (loop = 1; loop != 200; loop++) {
      if (((__R31 >> tbit_c) & 0x01) == 0) {
        // Reset loop counting
        loop = 1;
      }
    }
    continue_allowed = 1;

    // ----------------------------------------
    // Wait START OF TRANSMISSION - CLK |_
    while (((__R31 >> tbit_c) & 0x01) == 1) {
      // shram[offsetPosL] = 0x77777777;
      // shram[offsetPosM] = 0x7;
    }

    // ----------------------------------------
    // Wait two clock cycles
    dummyCycles(2, RISING_EDGE);

    // ----------------------------------------
    // Get Mode Command
    if (continue_allowed) {
      shram[offsetCommand] = 0;
      dummyCycles(8, RISING_EDGE);
    }

    // ----------------------------------------
    // Wait START OF REPLY - DATA _|
    for (loop = 1; loop != 0; loop++) {
      if (((__R31 >> tbit_d) & 0x01) == 1) {
        break;
      }
    }
    if (loop == 0) {
      continue_allowed = 0;
    }

    // Skip two clock cycles - Error Flags
    dummyCycles(1, RISING_EDGE);

    // ----------------------------------------
    // Get Data (position value) + CRC
    if (continue_allowed) {
      // Set mutex
      // asm(" SET        r30.t1\n");
      shram[offsetMutex] = 0;
      // Acquire POSITION
      switch (unit) {
        case 0:
          dataEncoder = readBitsAsmCh0(36);
          break;
        case 1:
          dataEncoder = readBitsAsmCh1(36);
          break;
        case 2:
          dataEncoder = readBitsAsmCh2(36);
          break;
        case 3:
          dataEncoder = readBitsAsmCh3(36);
          break;
      }

      // dataEncoder = readBitsAsmCh2(36);//, FALLING_EDGE);
      //  IGNORE CRC ------ Acquire CRC
      // shram[offsetCRC] = readBits(5, FALLING_EDGE);

      // Save position value into shram
      shram[offsetPosL] = dataEncoder & 0xFFFFFFFF;
      shram[offsetPosM] = dataEncoder >> 32;
      // Clear mutex
      shram[offsetMutex] = 1;
      // asm(" CLR        r30.t1\n");
    }
    /*else
     {
         shram[offsetMutex] = 0;
         shram[offsetPosL] = 0x77777777;
         shram[offsetPosM] = 0x7;
         shram[offsetMutex] = 1;
     }*/
    //__delay_cycles(100000000); // half-second delay
  }
}

void dummyCycles(uint8_t nCycles, uint16_t samplingEdge) {
  uint32_t wait;
  for (; nCycles != 0; nCycles--) {
    // Wait EDGE
    while (((__R31 >> tbit_c) & 0x01) != samplingEdge) {
      /*if(wait == 0)
      {
          continue_allowed = 0;
          return;
      }*/
    }
    // Wait EDGE
    while (((__R31 >> tbit_c) & 0x01) == samplingEdge) { /*
                                                            wait++;
                                                            if(wait == 0)
                                                            {
                                                                continue_allowed = 0;
                                                                return;
                                                            }*/
    }
  }
  return;
}

uint32_t readBits(uint8_t nBits, uint16_t samplingEdge) {
  uint32_t wait;
  uint32_t data = 0;
  for (; nBits != 0; nBits--) {
    // Wait SAMPLING EDGE
    data = (data << 1);

    // while(((__R31 >> tbit_c) & 0x01) != samplingEdge)
    for (wait = 1; ((__R31 >> tbit_c) & 0x01) != samplingEdge; wait++) {
      if (wait == 0) {
        continue_allowed = 0;
        return 0;
      }
    }

    // GET DATA INFO
    data += ((__R31 >> tbit_d) & 0x01);

    // Wait complementary

    // while(((__R31 >> tbit_c) & 0x01) == samplingEdge)
    for (wait = 1; (((__R31 >> tbit_c) & 0x01) == samplingEdge) && (wait != 0); wait++) {
      if (wait == 0) {
        continue_allowed = 0;
        return 0;
      }
    }
  }
  return data;
}
