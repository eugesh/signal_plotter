#ifndef __MATH_MODULE_H__
#define __MATH_MODULE_H__

#include <iostream>
#include <cmath>
#include <vector>

/* Константа для исключения деления на ноль. */
#define EPS 0.0001

void print_matrix (const char* message, double *A, int w , int h)
{
  int i, j;

  printf ("%s:\n", message);
  for (i = 0; i < h; ++i)
    {
      for (j = 0; j < w; ++j) {
        printf ("%f ", A[i * w + j]);
      }
      printf ("\n");
    }
}

/*
 *  Метод Гаусса. Прямой и обратный проходы.
 *  Gauss method. Direct and back pass.
 *  Дополнено проверками деления на ноль, что все равно не исключает возможность расхождения решения.
 */
int gauss_right_back (double *x, double *a, int n) {
  double buf;
  int i, j, k;
  int a_width = n + 1;

  /* Direct steps. */
  for (i = 0; i < n - 1; i++)
    for (j = i + 1; j < n; j++)
      {
        /* Проверка деления на ноль. */
        if (fabs(a[j * a_width + i]) > EPS)
          buf = a[i * a_width + i] / a[j * a_width + i];
        else
          buf = a[i * a_width + i] / EPS;

        for (k = 0; k <= n; k++)
          a[j * a_width + k] = a[j * a_width + k] * buf - a[i * a_width + k];
      }

  /* Back steps. */
  /* Проверка деления на ноль. */
  if (fabs(a[(n-1) * a_width + n-1]) > EPS)
    x[n-1] = a[(n-1) * a_width + n] / a[(n-1) * a_width + n-1];
  else
    x[n-1] = a[(n-1) * a_width + n] / EPS;

  for (i = n - 2; i >= 0; i--)
    {
      buf = 0;
      for (j = i + 1; j < n; j++)
        buf += a[i * a_width + j] * x[j];
      /* Проверка деления на ноль. */
      if(fabs(a[i * a_width + i]) > EPS)
        x[i] = (a[i * a_width + n] - buf) / a[i * a_width + i];
      else
        x[i] = (a[i * a_width + n] - buf) / EPS;
    }

  return 0;
}

/**
 * Matrix multiplication.
 *
 * \param[out] c_out output matrix, size is w_a * w_a
 * \param[in] a_in input left side matrix, size is w_a * h_a
 * \param[in] w_a
 * \param[in] h_a
 * \param[in] b_in
 * \param[in] h_b
 *
 */
void matrix_mult (double *c_out, double *a_in, int w_a, int h_a, double *b_in, int w_b) {
  int i, j, k;
  double sum = 0;

  for (i = 0; i < h_a; ++i)
    {
      for (j = 0; j < w_b; ++j)
        {
          for (k = 0; k < w_a; ++k)
            {
              sum += a_in[i * w_a + k] * b_in[k * w_b + j];
            }
          c_out[i * w_b + j] = sum;
          sum = 0;
        }
    }
}

void transpose(double *dst, double *src, int M, int N) {
    // #pragma omp parallel for
    int n;
    for(n = 0; n < N * M; n++) {
        int i = n / N;
        int j = n % N;
        dst[n] = src[M * j + i];
    }
}

// The model is {[xi 1]} * [a; b] = {yi}
void linear_approximation(double *a, double *b, std::vector<double> x, std::vector<double> y) {
    unsigned int N = x.size();
    double *A, *AT; // N * 2
    double *ATA, *ATA_Gauss; // 2 * 2
    double *B; // N
    double *X, *ATB; // 2
    A = new double[N * 2];
    AT = new double[2 * N];
    ATA = new double[2 * 2];
    ATA_Gauss = new double[2 * (2 + 1)];
    B = new double[N];
    X = new double[2];
    ATB = new double[2];

    for(unsigned int i = 0; i < N; ++i) {
        A[2 * i] = x[i];
        A[2 * i + 1] = 1;
        B[i] = y[i];
    }

    print_matrix("A: ", A, 2, N);
    print_matrix("B: ", B, 1, N);

    /* Transpose matrix A. */
    transpose (AT, A, 2, N);

    print_matrix("AT: ", AT, N, 2);

    /* Calculate AT * A, (d + 1) * (d + 1). */
    matrix_mult(ATA, AT, N, 2, A, 2);

    print_matrix("ATA: ", ATA, 2, 2);

    /* Calculate ATB, . */
    matrix_mult(ATB, AT, N, 2, B, 1);

    print_matrix("ATB: ", ATB, 1, 2);

    /* Embed ATB into ATA. */
    for (int i = 0; i < 2; ++i)
      {
        for (int j = 0; j < 2; ++j)
          {
            ATA_Gauss[i * (2 + 1) + j] = ATA[i * 2 + j];
          }
        ATA_Gauss[i * (2 + 1) + 2] = ATB[i];
      }

    print_matrix("ATA_Gauss: ", ATA_Gauss, 2 + 1, 2);

    /* Solve linear equation. */
    gauss_right_back(X, ATA_Gauss, 2);

    print_matrix("Coefficients: ", X, 1, 2);

    // Result.
    *a = X[0]; *b = X[1];

    delete[] A;
    delete[] B;
    delete[] X;
    delete[] AT;
    delete[] ATA;
    delete[] ATA_Gauss;
    delete[] ATB;
}

#endif
