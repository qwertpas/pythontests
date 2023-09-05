#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define CMD_SET_VOLTAGE (0b10000000)
#define CMD_SET_SPEED (0b10010000)
#define CMD_SET_POSITION (0b10100000)
#define CMD_SET_CURRENT (0b10110000)
#define CMD_GET_POSITION (0b11000000)
#define CMD_GET_SPEED (0b11010000)
#define CMD_GET_CURRENT (0b11100000)
#define CMD_GET_TEMPERATURE (0b11110000)
#define CMD_MASK (0xF0)

#define UARTSIZE 3

typedef struct {
    uint8_t uart_TX[UARTSIZE];  // send to Teensy
    uint8_t uart_RX[UARTSIZE];  // receive from Teensy (may be out of order)
    uint16_t uart_cmd[UARTSIZE]; // first byte is command, rest is data packed into 16 bits
} PeripherialStruct;
PeripherialStruct p;

int main() {

    p.uart_RX[0] = (CMD_SET_VOLTAGE | 4);
    p.uart_RX[1] = 64;
    p.uart_RX[2] = 112;

    p.uart_cmd[0] = p.uart_RX[0] & CMD_MASK;
    p.uart_cmd[1] = (p.uart_RX[1] << 7) | (p.uart_RX[2]);


    uint8_t reverse = 0;
    uint16_t mag = 0;


    // If negative, take the absolute value assuming two's complement
    reverse = (p.uart_cmd[1] >> 13) & 1;
    mag = reverse ? (~p.uart_cmd[1]) + 1 : p.uart_cmd[1];

    // if (p.uart_cmd[0] == CMD_SET_VOLTAGE) {
    //     reverse = (p.uart_cmd[1] & 0x20) >> 5; // check MSB if positive or negative int
    //     mag = abs(p.uart_cmd[1]);
    // }


    
    

    printf("uart_cmd[0]: %d, uart_cmd[1]: %d \n", p.uart_cmd[0], p.uart_cmd[1]);
    printf("reverse: %d, mag: %d \n", reverse, mag);

    return 0;
}
