#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdio.h>
//#include <stdlib.h>
#include "uart.h"
#include "twi.h"

#define SENSOR_NUM 8

// settings for I2C
//uint8_t I2C_buffer[sizeof(int)];
uint8_t I2C_buffer[SENSOR_NUM];
#define I2C_SLAVE_ADDRESS 0x10
void handle_I2C_error(volatile uint8_t TWI_match_addr, uint8_t status);

int main(void) {
  // Initialise I2C
  // http://www.nerdkits.com/forum/thread/1554/
  TWI_init( F_CPU,                      // clock frequency
            100000L,                    // desired TWI/IC2 bitrate
            I2C_buffer,                 // pointer to comm buffer
            sizeof(I2C_buffer),         // size of comm buffer
            0                           // optional pointer to callback function
            );

  // Initialise UART
  uart0_init(UART_BAUD_SELECT(9600, F_CPU));

  sei();

  while(1) {
    /*
    //I2C_buffer[0] = 1;
    TWI_master_start_write_then_read( I2C_SLAVE_ADDRESS,  // slave device address
                                      sizeof(I2C_buffer), // number of bytes to write
                                      sizeof(I2C_buffer)  // number of bytes to read
                                      );
                                      */
    TWI_master_start_read(  I2C_SLAVE_ADDRESS,  // slave device address
                            sizeof(I2C_buffer)  // number of bytes to read
                            );

    // wait for completion
    while(TWI_busy);

    // if error, notify and quit
    if(TWI_error){
      // TODO: Reinitialise.
      break;
    }

    // Send value to UART
    // FIXME: Use proper size.
    char buf [25];
    snprintf(buf, sizeof(buf), "%d %d %d %d %d %d %d %d\r\n", 
      I2C_buffer[0],
      I2C_buffer[1],
      I2C_buffer[2],
      I2C_buffer[3],
      I2C_buffer[4],
      I2C_buffer[5],
      I2C_buffer[6],
      I2C_buffer[7]
    );
    uart0_puts(buf);
  }

  return(0);
}
