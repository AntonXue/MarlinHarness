#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mex.h>

#define BUF_SIZE 8192

double* diffs;

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
        diffs[d++] = strtod(stuff, NULL);
    }
}

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]) {
    double amt = mxGetScalar(prhs[0]);
    double* test = mxGetPr(prhs[1]);

    plhs[0] = mxCreateDoubleMatrix(1, 4, mxREAL);
    diffs = mxGetPr(plhs[0]);

    do_the_call(amt, test);
}

int main(int argc, char* argv[]) {
    /*
      double test[] = {1, 2, 3, 4, 5, 6, 7, 8 };
      do_the_call(8, test);

      int i;
      for (i = 0; i < 4; i++) {
          printf("hue: %f\n", diffs[i]);
      }
    */
    printf("Meant to be called from inside Matlab!\n");
}

