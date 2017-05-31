#ifndef HARNESS_H
#define HARNESS_H

#define MAX_CMD_BUF_SIZE 96
#define XYZE_DIM 4
#define XYZE_X 0
#define XYZE_Y 1
#define XYZE_Z 2
#define XYZE_E 3

// void calc_move(float cur[], float tgt[MAX_CMD_BUF_SIZE][XYZE_DIM]);
void calc_moves(const char* cmds[], int num_cmds=MAX_CMD_BUF_SIZE);

#endif
