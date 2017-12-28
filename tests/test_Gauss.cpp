#include <cstdio>
#include <ctime>
#include <cstdlib>
#include "math_module.h"

/* Test for Gauss right back SLAE solver. */
int main () {
  int N = 3;
  double *a;
  double *x;
  double diff;
  int i, j;

  srand(time(NULL));

  a = new double[N * (N + 1)];
  x = new double[N];

  printf ("a = \n");
  for (i = 0; i < N; ++i)
    {
      for (j = 0; j < N + 1; ++j)
        {
          a[i * (N+1) + j] = rand () % 100;
          printf ("%f ", a[i * (N+1) + j]);
        }
      printf ("\n");
    }

  /* Gauss right back solver. */
  gauss_right_back (x, a, N);

  /* Solution. */
  printf ("x = \n");
  for (i = 0; i < N; ++i)
    printf ("%f\n", x[i]);

  /* Precision estimation. */
  for (i = 0; i < N; ++i)
    printf ("%f\n", x[i] - a[i * (N + 1) + N]);


  printf ("discrepancy:\n");
  for (i = 0; i < N; ++i)
    {
      for (j = 0; j < N; ++j)
        {
          diff += a[i * (N + 1) + j] * x[j];
        }
      diff -= a[i * (N + 1) + N];
      printf ("%f\n", diff);
      diff = 0;
    }

  delete [] a;
  delete [] x;

  return 0;
}
