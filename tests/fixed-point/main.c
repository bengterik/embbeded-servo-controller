#include <stdio.h>

void modulo() {
    int a = 10;

    for (int b = -10; b < a; b++) {     
        int result = a % b; 
        printf("%d mod %d = %d\n", b, a, b % a);
    }
}

int main() {
    modulo();
    return 0;
}
