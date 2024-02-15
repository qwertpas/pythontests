#include <math.h>
#include <stdio.h>
#include <stdlib.h>

int16_t abs16(int16_t val) {
    if (val < 0)
        return -val;
    else
        return val;
}

uint16_t to2comp7(int16_t val) {
    if (val < 0) {
        return (uint8_t)(-((~val) + 1) & 0b01111111);
    } else {
        return (uint8_t)(val & 0b01111111);
    }
}

int8_t pad7(uint8_t num7) {
    if (num7 & 0x40)
        return num7 | 0x80;
    else
        return num7;
}

int16_t pad14(uint8_t num7_0, uint8_t num7_1) {
    int16_t res = (num7_0 << 7) | (num7_1);
    if (res & 0x2000)
        return res | 0xC000;
    else
        return res;
}

int main(int argc, char const *argv[]) {

    // int16_t I_u = 22;
    // int16_t I_v = -9;
    // int16_t I_w = -22;

    // int16_t I_phase = abs16(I_u);
    // if (abs16(I_v) > I_phase)
    //     I_phase = abs16(I_v);
    // if (abs16(I_w) > I_phase)
    //     I_phase = abs16(I_w);

    // printf("%d \n", pad7(to2comp7(I_u)));
    // printf("%d \n", pad7(to2comp7(I_v)));
    // printf("%d \n", pad7(to2comp7(I_w)));
    // printf("%d \n", (uint8_t)((I_phase)) & 0b01111111);
    // return 0;

    int32_t cont_angle = -1;
    uint8_t uart_TX_2 = (uint8_t)(cont_angle >> 7) & 0b01111111;
    uint8_t uart_TX_3 = (uint8_t)(cont_angle) & 0b01111111;
    printf("0x%X, 0x%X \n", uart_TX_2, uart_TX_3);

    int16_t decoded = pad14(uart_TX_2, uart_TX_3);
    printf("decoded: %d \n", decoded);
}
