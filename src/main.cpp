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
    /*
    const char* moves[] = { "G1 X80.495 Y77.039 E3.56411",
                            "G1 X81.1 Y76.538 E3.58091",
                            "G1 X83.527 Y74.775 E3.64502",
                            "G1 X84.19 Y74.354 E3.66181",
                            "G1 X86.818 Y72.909 E3.72591",
                            "G1 X87.529 Y72.575 E3.74271",
                            "G1 X89.425 Y71.824 E3.78631",
                            "G1 X90.317 Y71.471 E3.80682",
                            "G1 X91.064 Y71.228 E3.82361",
                            "G1 X93.969 Y70.482 E3.88772",
                            "G1 X94.741 Y70.335 E3.90451",
                            "G1 X97.716 Y69.959 E3.96862",
                            "G1 X98.501 Y69.91 E3.98542",
                            "G1 X101.499 Y69.91 E4.04952",
                            "G1 X102.284 Y69.959 E4.06632",
                            "G1 X105.259 Y70.335 E4.13043",
                            "G1 X106.031 Y70.482 E4.14722",
                            "G1 X108.936 Y71.228 E4.21133",
                            "G1 X109.683 Y71.471 E4.22813",
                            "G1 X112.471 Y72.575 E4.29223",
                            "G1 X113.182 Y72.909 E4.30903",
                            "G1 X114.969 Y73.892 E4.35262",
                            "G1 X115.811 Y74.354 E4.37315",
                            "G1 X116.473 Y74.775 E4.38993",
                            "G1 X118.9 Y76.538 E4.45404",
                            "G1 X119.505 Y77.039 E4.47084",
                            "G1 X121.692 Y79.092 E4.53495",
                            "G1 X122.229 Y79.664 E4.55174",
                            "G1 X124.141 Y81.975 E4.61584",
                            "G1 X124.603 Y82.611 E4.63264",
                            "G1 X126.21 Y85.143 E4.69675",
                            "G1 X126.588 Y85.831 E4.71355",
                            "G1 X127.865 Y88.545 E4.77765",
                            "G1 X128.154 Y89.275 E4.79445",
                            "G1 X129.081 Y92.128 E4.85855",
                            "G1 X129.276 Y92.889 E4.87535",
                            "G1 X129.838 Y95.835 E4.93946",
                            "G1 X129.937 Y96.614 E4.95625",
                            "G1 X130.125 Y99.607 E5.02036",
                            "G1 X130.125 Y100.393 E5.03716",
                            "G1 X129.937 Y103.386 E5.10126",
                            "G1 X129.838 Y104.165 E5.11806",
                            "G1 X129.485 Y106.021 E5.15843",
                            "G1 X129.276 Y107.112 E5.18217",
                            "G1 X129.081 Y107.872 E5.19896",
                            "G1 X128.154 Y110.725 E5.26307",
                            "G1 X127.865 Y111.455 E5.27986",
                            "G1 X126.588 Y114.169 E5.34397",
                            "G1 X126.21 Y114.857 E5.36077" };
    int num_moves = 121 - 73 + 1;
    */
    const char* moves[] { "G1 X200.0 Y150.0 Z100.0 E10.0"
                        , "G1 X0.0 Y0.0 Z0.0 E0.0"
                        , "G1 X200.0 Y150.0 Z100.0 E10.0" };
    int num_moves = 3;
    calc_moves(moves, num_moves);
}
