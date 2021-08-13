/*
* Heidenhain Absolute Encoder Reading
* Bench tests for Delta Undulator Sniffing Position
*
*
* Patricia Nallin - IOT Team
* 12.aug.2021
*
* Target platform: LPC1768
*/

#include "mbed.h"

DigitalOut clk(p21);
DigitalOut dados(p23);
DigitalOut habilita(p25);
DigitalOut led(LED1);
DigitalIn dadosIn(p27);
DigitalIn running(p22);
//InterruptIn button(p22);



Serial pc(USBTX,USBRX,115200);

 
volatile uint64_t delay_us;   


uint64_t readHeidenhain(){
    int i, crc=0;
    uint64_t leitura = 0;
    uint64_t numeroUm = 1;
    
    // Command 000 111 
    habilita = 1;
    
    clk = 0;
    wait_us(2);
    
    for(i=0;i<2;i++)
    {
        clk = 0;
        clk = 1;
    }
    
    dados = 0;
    
    for(i=0;i<3;i++)
    {
        clk = 0;
        clk = 1;
    }
    
    dados = 1;
    
    for(i=0;i<3;i++)
    {
        clk = 0;
        clk = 1;
    }
    
    habilita = 0;
    
    for(i=0;i<2;i++)
    {
        clk = 0;
        clk = 1;
    }
    
    clk = 0;

    // Wait for start BIT
    while(dadosIn == 0)
    {
        clk = 1;
        clk = 0;
    }
    

    // Read FLAGS
    for(i=0;i<2;i++)
    {
        clk = 1;
        clk = 0;
    }
    
    // Read DATA
    for(i=0;i<35;i++)
    {
        clk = 1;
        clk = 0;
        if(dadosIn)
        {
            leitura += (numeroUm << i);
        }
    }
    
    // Read CRC
    for(i=4;i>=0;i--)
    {
        clk = 1;
        clk = 0;
        crc += (dadosIn << i);
    }
    
    clk = 1;
    
    printf("CRC = %d\n\r", crc);
    return leitura;
}
 
int main() {
    uint64_t valor;
    clk = 1;
    dados = 1;
    habilita = 0;
    
    
    // spin in a main loop. flipper will interrupt it to call flip
    while(1) {
        if(running)
        {
            led = !led;
            valor = readHeidenhain();
            printf("Position = %llu (0x%09llX)\n\n\n\r", valor, valor);
        }
        wait_us(1000);
        
    }
}
