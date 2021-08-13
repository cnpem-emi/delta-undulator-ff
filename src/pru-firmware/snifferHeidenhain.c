/*
 * Heidenhain Encoder Sniffer
 *
 * For Delta Undulator Feed-forward corrections
 * 1-channel sniffer
 *
 * To be run on a BeagleBone-AI PRU.
 * THIS FIRST VERSION IS TO BE RUN ON PRU 2_0 (THIRD PRU = REMOTEPROC6)
 *
 *
 * Under development
 * Patricia Nallin - CNPEM
 * patricia.nallin@cnpem.br
 *
 * */

#include <stdint.h>
#include <stdio.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include <pru_intc.h>
#include "resource_table_empty.h"

volatile register uint32_t __R31;
volatile register uint32_t __R30;

#define PRU_SHARED_MEM_ADDR     0x00010000
#define CLK_ENDAT               ((__R31 >> 4) & 0x01)
#define DATA_ENDAT              ((__R31 >> 3) & 0x01)

#define RETURN_LINE_TIMEOUT     -1
#define RETURN_OK               0

#define FALLING_EDGE            0
#define RISING_EDGE             1


volatile uint8_t continue_allowed;


void dummyCycles(uint8_t nCycles, uint16_t samplingEdge);
uint32_t readBits(uint8_t nBits, uint16_t samplingEdge);



void main(){

   // Clear SYSCFG[STANDBY_INIT] to enable OCP master port->Shared memory
   CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

   // Pointer to shram
   volatile int* shram = (volatile int *) PRU_SHARED_MEM_ADDR;
   shram[0] = 0xED;
   shram[1] = 0xDEADBEEF;
   shram[2] = 0;
   shram[3] = 1;

   // General variables
   uint8_t  loop=0;

  /* GPI Mode 0, GPO Mode 0 */
   CT_CFG.GPCFG0 = 0;

   while(1){

       // Start on a beginning of communication. Wait clock HIGH for, at least, 30 us
       for(loop=1; loop!=100; loop++)
       {
           if(CLK_ENDAT == 1)
           {
               // Reset loop counting
               loop = 1;
           }
       }
       continue_allowed = 1;

       // ----------------------------------------
       // Wait START OF TRANSMISSION - CLK |_
       while(CLK_ENDAT == 1)
       {
       }

       // ----------------------------------------
       // Wait two clock cycles
       dummyCycles(2, RISING_EDGE);

       // ----------------------------------------
       // Get Mode Command
       if(continue_allowed)
       {
           shram[0] = readBits(8, RISING_EDGE);
       }

       // ----------------------------------------
       // Wait START OF REPLY - DATA _|
       for(loop=1; loop!=100; loop++)
       {
           if(DATA_ENDAT == 1)
           {
               break;
           }
           continue_allowed = 0;
       }

       // Wait clock LOW
       for(loop=1; loop!=100; loop++)
       {
           if(CLK_ENDAT == 0)
           {
               break;
           }
           continue_allowed = 0;
       }
       continue_allowed = 1;

       // Skip two clock cycles - Error Flags
       dummyCycles(2, RISING_EDGE);


       // ----------------------------------------
       // Get Data (position value) + CRC
       if(continue_allowed)
       {
           // Set mutex
           shram[4] = 0;
           // Acquire POSITION
           shram[1] = readBits(32, FALLING_EDGE);
           shram[2] = readBits(3, FALLING_EDGE);
           // Acquire CRC
           shram[3] = readBits(5, FALLING_EDGE);
           // Clear mutex
           shram[4] = 1;
       }
       //__delay_cycles(100000000); // half-second delay
   }
}


void dummyCycles(uint8_t nCycles, uint16_t samplingEdge)
{
    uint16_t wait;
    for(; nCycles != 0; nCycles--)
           {
               // Wait RISING EDGE
               wait = 0;
               while(CLK_ENDAT != samplingEdge)
               {
                   wait++;
                   if(wait == 0)
                   {
                       continue_allowed = 0;
                       return;
                   }
               }
               // Wait FALLING EDGE
               wait = 0;
               while(CLK_ENDAT == 1)
               {
                   wait++;
                   if(wait == 0)
                   {
                       continue_allowed = 0;
                       return;
                   }
               }
           }
    return;
}

uint32_t readBits(uint8_t nBits, uint16_t samplingEdge)
{
    uint16_t wait;
    uint32_t data = 0;
    for(; nBits != 0; nBits--)
           {
               // Wait SAMPLING EDGE
               wait = 0;
               while(CLK_ENDAT != samplingEdge)
               {
                   wait++;
                   if(wait == 0)
                   {
                       continue_allowed = 0;
                       return 0;
                   }
               }

               // GET DATA INFO

               data = (data << 1) + DATA_ENDAT;

               // Wait complementary
               wait = 0;
               while(CLK_ENDAT == samplingEdge)
               {
                   wait++;
                   if(wait == 0)
                   {
                       continue_allowed = 0;
                       return 0;
                   }
               }
           }
    return data;
}

