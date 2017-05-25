#include "commands.h"

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

void parse_xyze(char* cmd, float current[XYZE_DIM], float target[XYZE_DIM]) {
    for (int i = 0; i < XYZE_DIM; i++) target[i] = current[i];
    int len = strlen(cmd);
    for (int i = 0; i < len; i++) {
        if (cmd[i] == 'X' || cmd[i] == 'x') {
            target[0] = atof(cmd + (i * sizeof(char)) + 1);
        } else if (cmd[i] == 'Y' || cmd[i] == 'y') {
            target[1] = atof(cmd + (i * sizeof(char)) + 1);
        } else if (cmd[i] == 'Z' || cmd[i] == 'z') {
            target[2] = atof(cmd + (i * sizeof(char)) + 1);
        } else if (cmd[i] == 'E' || cmd[i] == 'e') {
            target[3] = atof(cmd + (i * sizeof(char)) + 1);
        } else {
            // :)
        }
    }
}

