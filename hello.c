#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <mex.h>

#define BUF_SIZE 8192

double eps = 0.000001;
double location[4] = { 0 };

double target[4] = { 0 };

double sq(double a) { return a * a; }

double mag_err(double err) {
    if (fabs(err) < eps) {
        return 0;
    } else {
        return err / eps;
    }
}

void do_the_call(int n, double cmds[]) {
    char buf[BUF_SIZE];
    memset(buf, '\0', BUF_SIZE);
    sprintf(buf, "/home/anton/MarlinHarness/harness");
    int i;
    for (i = 0; i < n; i++) {
        sprintf(buf, "%s %f", buf, cmds[i]);
    }

    FILE* fp = popen(buf, "r");
    char stuff[1024];
    if (fp == NULL) {
        fprintf(stderr, "Failed to run command\n");
        exit(1);
    }

    int d = 0;
    while (fgets(stuff, sizeof(stuff) - 1, fp) != NULL && d < 4) {
        location[d++] = strtod(stuff, NULL);
    }
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
    int amt = mxGetN(prhs[0]);
    double* test = mxGetPr(prhs[0]);

    int j;
    for (j = 0; j < 20; j++) {
        printf("test[%d]: %f\n", j, test[j]);
    }

    if (amt % 4 != 0) {
        fprintf(stderr, "%d is not a multiple of 4!\n", amt);
        exit(1);
    }

    target[0] = test[amt - 4];
    target[1] = test[amt - 3];
    target[2] = test[amt - 2];
    target[3] = test[amt - 1];

    do_the_call(amt, test);
    
    double value = - sqrt(sq(target[0] - location[0]) +
                          sq(target[1] - location[1]) +
                          sq(target[2] - location[2]));

    /*
    printf("Actual: [%f, %f, %f, %f]\n",
          location[0], location[1], location[2], location[3]);

    printf("Target: [%f, %f, %f, %f]\n",
          target[0], target[1], target[2], target[3]);
    */

    plhs[0] = mxCreateDoubleScalar(mag_err(value));
}

int main(int argc, char* argv[]) {
    printf("Meant to be called from inside Matlab!\n");
}

