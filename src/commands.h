#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CMD_BUF_SIZE 96
#define XYZE_DIM 4

void next_cmd(char* cmd, FILE* fp, int max_buf_size);

void parse_xyze(char* cmd, float current[], float target[]);

#endif
