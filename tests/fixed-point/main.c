#include <stdio.h>
#include <stdint.h>

#define SHIFT_AMOUNT 8 

typedef uint16_t fp_int;

fp_int fp_mul(uint8_t a, fp_int b) {
    fp_int a_fp = a << SHIFT_AMOUNT;
    fp_int temp = (fp_int) a * b;

    uint8_t res = temp >> SHIFT_AMOUNT;

    return res;
}

fp_int fp_div(uint8_t a, fp_int b) {
    fp_int temp = (uint16_t) a << SHIFT_AMOUNT;
    temp += b >> 1;
    printf("a: %d, b: %d, temp: %d\n", a, b, temp);
    uint16_t res = (temp / b);
    printf("res: %d\n", res);

    return res;
}

int main() {
    printf("\n");
    uint8_t a = 120;
    fp_int b = 0x0080; // 0.5 in fixed point
    uint16_t c;
    
    c = fp_mul(a, b);
    printf("%d * 0.5 = %d (=%d)\n", a, c, c>>SHIFT_AMOUNT);

    printf("\n");
    b = 0x200; // 2 in fixed point
    c = fp_div(a, b);
    printf("%d / 2 = %d (=%d)\n", a, c,c>>SHIFT_AMOUNT);
    printf("\n");

    printf("\n");
    b = 0x300; // 2 in fixed point
    c = fp_div(a, b);
    printf("%d / 3 = %d (=%d)\n", a, c, c>>SHIFT_AMOUNT);
    printf("\n");
}

