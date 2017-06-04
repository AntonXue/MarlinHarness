#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "harness.h"

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
    if ((argc - 1) % 4 != 0) {
        fprintf(stderr, "Need to input multiples of 4\n");
        exit(1);
    }

    int num_moves = (argc - 1) / 4;
    char moves[num_moves][MAX_CMD_BUF_SIZE];
    
    for (int i = 0; i < num_moves; i++) {
        sprintf(moves[i], "G1 X%f Y%f Z%f E%f",
          (float)strtod(argv[(i * 4) + 1], NULL),
          (float)strtod(argv[(i * 4) + 2], NULL),
          (float)strtod(argv[(i * 4) + 3], NULL),
          (float)strtod(argv[(i * 4) + 4], NULL));
    }

    calc_moves(0, moves, num_moves);

    printf("%f\n", acc_mms[XYZE_X]);
    printf("%f\n", acc_mms[XYZE_Y]);
    printf("%f\n", acc_mms[XYZE_Z]);
    printf("%f\n", acc_mms[XYZE_E]);
}
