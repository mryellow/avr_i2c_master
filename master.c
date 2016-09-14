#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
//#include <stdlib.h>
#include "uart.h"
#include "twi.h"

// settings for I2C
uint8_t I2C_buffer[sizeof(int)];
#define I2C_SLAVE_ADDRESS 0x10
void handle_I2C_error(volatile uint8_t TWI_match_addr, uint8_t status);

/*
uint32_t litendtoint(uint8_t *lit_int) {
    return (uint16_t)lit_int[0] <<  0
         | (uint16_t)lit_int[1] <<  8;
         //| (uint32_t)lit_int[2] << 16
         //| (uint32_t)lit_int[3] << 24;
}
*/

int main(void) {
  //uint8_t cnt = 1;

  // Initialize I2C
  // http://www.nerdkits.com/forum/thread/1554/
  TWI_init( F_CPU,                      // clock frequency
            100000L,                    // desired TWI/IC2 bitrate
            I2C_buffer,                 // pointer to comm buffer
            sizeof(I2C_buffer),         // size of comm buffer
            0                           // optional pointer to callback function
            );

  uart0_init(UART_BAUD_SELECT(9600, F_CPU));

  sei();

  while(1) {
    // TODO: I2C: Write sensor reading address
    //I2C_buffer = 0x01;

    // transmit
    TWI_master_start_write_then_read( I2C_SLAVE_ADDRESS,  // slave device address
                                      sizeof(I2C_buffer), // number of bytes to write
                                      sizeof(I2C_buffer)  // number of bytes to read
                                      );

    // wait for completion
    while(TWI_busy);

    // if error, notify and quit
    if(TWI_error){
      break;
    }

    // Send value to UART
    char buf [5];
    snprintf(buf, sizeof(buf), "%d\r\n", I2C_buffer[0]);
    uart0_puts(buf);

    //cnt++;
  }

  return(0);
}
