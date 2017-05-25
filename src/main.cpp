#include <stdio.h>
#include "commands.h"

int main(int argc, char* argv[]) {
    printf("compiles and runs\n");
    char test[MAX_CMD_BUF_SIZE];
    next_cmd(test, stdin, MAX_CMD_BUF_SIZE);
    printf("test: %s\n", test);
    
    float current[] = {1, 2, 3, 4};
    float target[XYZE_DIM];

    parse_xyze(test, current, target);
    printf("%f, %f, %f, %f\n", target[0], target[1], target[2], target[3]);
}
