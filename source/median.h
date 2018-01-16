#ifndef ___MEDIAN_H__
#define ___MEDIAN_H__

#include <stdio.h>
#include <stdlib.h>

int compare (const void * elem1, const void * elem2);
void median1d(float *out, float const *in, int msize);

/**
 * Comparison function.
 *
 */
/*int
compare (const void * elem1, const void * elem2)
{
    int f = *((int*)elem1);
    int s = *((int*)elem2);
    if (f > s) return  1;
    if (f < s) return -1;
    return 0;
}*/

int
compare (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}

/**
 *	Standard median 1d filter.
 *	Doesn't perform inplace filtering, so
 *	arrays should be pointers to physically different memory places.
 *
 * \param[out] - output filtered array(time series);
 * \param[in] - input  filtered array(time series);
 * \param[in] - length of the array;
 * \param[in] - size of sliding mask, odd number = {3, 5, 7, ..., l / 2 - 1 }.
 *
 */
void
median1d(float *out, float const *in, unsigned int l, unsigned int msize) {
  // Run in array over with sliding mask.
  int shift = msize / 2;
  for(unsigned int i = shift; i < l - shift; ++i) {
    // Get current mask.
    float* mask = new float[msize];
    for(int j = -shift; j < shift; ++j) {
        // printf("shift + j = %d\n", shift + j);
        mask[shift + j] = in[i + j];
    }

    // Sort mask.
    qsort(mask, msize, sizeof(float), compare);

    /*for (unsigned int i = 0; i < msize; ++i)
        printf("%f ", mask[i]);
    printf("\n");*/

    // Get pivoting element.
    float pivot = mask[shift];

    // Set it to output array.
    out[i] = pivot;

    delete[] mask;
  }

}

#endif
