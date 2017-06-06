#ifndef HARNESS_H
#define HARNESS_H

#define MAX_CMD_BUF_SIZE 96
#define XYZE_DIM 4
#define XYZE_X 0
#define XYZE_Y 1
#define XYZE_Z 2
#define XYZE_E 3

extern long acc_steps[XYZE_DIM];
extern float acc_mms[XYZE_DIM];
extern float delta[3];

void calc_moves(int debug, char cmds[][MAX_CMD_BUF_SIZE], int num_cmds=MAX_CMD_BUF_SIZE);

#endif
