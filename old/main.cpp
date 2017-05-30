#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "harness.h"

#define MAX_CMD_BUF_SIZE 96
#define XYZE_DIM 4
#define XYZE_X 0
#define XYZE_Y 1
#define XYZE_Z 2
#define XYZE_E 3

void next_cmd(char* cmd, FILE* fp, int max_buf_size=MAX_CMD_BUF_SIZE) {
    memset(cmd, '\0', max_buf_size);
    int in_cmt = 0;
    int chars_read = 0;
    int c;
    while ((c = getc(fp)) != EOF && (chars_read + 1) < max_buf_size) {
        // New line / return carriages.
        if (c == '\n' || c == '\r') {
            in_cmt = 0;  // EOL implies EOC.
            if (chars_read == 0) {
                continue;  // Skip empty lines.
            } else {
                cmd[chars_read] = '\0';  // Reached end of command.
                return;
            }
        // Handle the escapes.
        } else if (c == '\\') {
            c = getc(stdin);  // Skip the escape.
            if (in_cmt == 0) cmd[chars_read++] = c;  // Escs only for comments?
        // Comment detection.
        } else if (c == ';') {
            in_cmt = 1;
        // Read normally.
        } else {
            cmd[chars_read++] = c;
        }
    }
    cmd[chars_read] = '\0';
}

void parse_g0_g1_xyze(char* cmd, float cur[XYZE_DIM], float tgt[XYZE_DIM]) {
    for (int i = 0; i < XYZE_DIM; i++) { tgt[i] = cur[i]; }
    int len = strlen(cmd);
    for (int i = 0; i < len; i++) {
        if (cmd[i] == 'X' || cmd[i] == 'x') {
            tgt[XYZE_X] = atof(cmd + (i * sizeof(char)) + 1);
        } else if (cmd[i] == 'Y' || cmd[i] == 'y') {
            tgt[XYZE_Y] = atof(cmd + (i * sizeof(char)) + 1);
        } else if (cmd[i] == 'Z' || cmd[i] == 'z') {
            tgt[XYZE_Z] = atof(cmd + (i * sizeof(char)) + 1);
        } else if (cmd[i] == 'E' || cmd[i] == 'e') {
            tgt[XYZE_E] = atof(cmd + (i * sizeof(char)) + 1);
        } else {
            // :)
        }
    }
}


int main(int argc, char* argv[]) {
    // printf("compiles and runs\n");
    // char test[MAX_CMD_BUF_SIZE];
    // next_cmd(test, stdin, MAX_CMD_BUF_SIZE);
    // printf("test: %s\n", test);
    
    float cur[] = {150, 51, 52, 53};
    // float tgt[XYZE_DIM];
    float tgt[] = {155, 56, 57, 58};

    // parse_g0_g1_xyze(test, cur, tgt);
    // printf("%f, %f, %f, %f\n", tgt[0], tgt[1], tgt[2], tgt[3]);

    printf("cur: [%f, %f, %f, %f]\n", cur[0], cur[1], cur[2], cur[3]);
    printf("tgt: [%f, %f, %f, %f]\n", tgt[0], tgt[1], tgt[2], tgt[3]);
    calc_move(cur, tgt);
}
