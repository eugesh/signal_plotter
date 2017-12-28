#include <cstdio>
#include "math_module.h"


/*
 * Linear Least-Square method test.
 */
int main () {
  int i, j;
  /* Degree of Polynomial approximation. */
  int d = 2;
  /* Number of measurements. */
  int n = 10;
  /* Vector of x values. */
  double x[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0};
  /* Vector of y values. */
  double y[10] = {2, 5, 0, 7, 3, 0, 1, 0, 9, 0};
  /* Matrix A, (d + 1) * n. */
  double *A;
  /* X, n */
  double *X;
  /* Vector of polynomial coefficients, B. */
  double *B;
  /* A transposed. */
  double *AT;
  /* Matrix ATA for LSM, (d + 1) * (d + 1). */
  double *ATA;
  /* Matrix ATA for LSM, with additional column for rights parts used in Gauss method, (d + 2) * (d + 1). */
  double *ATA_gauss;
  /* Vector ATY for LSM, (d + 1). */
  double *ATY;

  print_matrix("x: ", x, n, 1);
  print_matrix("y: ", y, 1, n);

  A = new double[(d + 1) * n];
  X = new double[n];
  B = new double[n];
  AT = new double[n * (d + 1)];
  ATA = new double[(d + 1) * (d + 1)];
  ATA_gauss = new double[(d + 2) * (d + 1)];
  ATY = new double[(d + 1)];

  /* Fill A matrix with powers of xi, (d + 1) * n. */
  for (i = 0; i < n; ++i)
    {
      for (j = 0; j < d + 1; ++j)
        {
          if ((i * (d+1) + j) % (d+1) == 0)
            A[i * (d+1) + j] = x[i];
          else
            A[i * (d+1) + j] = A[i * (d+1) + j - 1] * x[i];
        }
    }

  print_matrix("A: ", A, d+1, n);

  /* Fill in vector B. */
  for (i = 0; i < n; ++i)
    {
      B[i] = y[i];
    }

  /* Transpose matrix A. */
  transpose (AT, A, d+1, n);

  print_matrix("AT: ", AT, n, d+1);

  /* Calculate AT * A, (d + 1) * (d + 1). */
  matrix_mult(ATA, AT, n, d+1, A, d+1);

  print_matrix("ATA: ", ATA, d+1, d+1);

  /* Calculate ATY, . */
  matrix_mult(ATY, AT, n, d+1, y, 1);

  print_matrix("ATY: ", ATY, 1, d+1);

  /* Embed ATY into ATA. */
  for (i = 0; i < d + 1; ++i)
    {
      for (j = 0; j < d + 1; ++j)
        {
          ATA_gauss[i * (d + 2) + j] = ATA[i * (d + 1) + j];
        }
      ATA_gauss[i * (d + 2) + d + 1] = ATY[i];
    }

  print_matrix("ATA_gauss: ", ATA_gauss, d+2, d+1);

  /* Solve linear equation. */
  gauss_right_back(X, ATA_gauss, d+1);

  print_matrix("Coefficients: ", X, 1, d+1);

  /* Verify solution. ATA * X - B. */
  matrix_mult(B, ATA, d+1, d+1, X, 1);
  print_matrix("Right estimated: ", B, d+1, 1);
  printf ("Discrepancy: \n");
  for(i = 0; i < d + 1; ++i)
    {
      printf ("%f \n", ATY[i] - B[i]);
    }

  /* Plot polynomial curve. */


  /* Free memory. */
  delete [] A;
  delete [] (X);
  delete [] (B);
  delete [] (AT);
  delete [] (ATA);
  delete [] (ATA_gauss);
  delete [] (ATY);

  return 0;
}



/*
 * Matrix multiplication test.
 */
int main_mat_mul_test () {
  int i, j;
  int a_w = 2;
  int a_h = 3;
  int b_w = 3;
  /*  double a[9] = { 3, 4, 5, 6, 7, 3, -1, 4, 5};
  double b[9] = { 3, 4, 5, 6, 7, 3, -1, 4, 6};
  double c[9] = {0};*/
  double a[6] = { 3, 4, 5, 6, 1, 2};
  double b[9] = { 3, 4, 5, 6, 0, 2, 1, 4, 1};
  double c[6] = { 0 };

  matrix_mult(c, a, a_w, a_h, b, b_w);

  for (i = 0; i < a_h; ++i)
    {
      for (j = 0; j < b_w; ++j)
        {
          printf ("%f ", c[i * b_w + j]);
        }
      printf ("\n");
    }

  return 0;
}
