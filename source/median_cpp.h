#ifndef __CV_FILTER_MEDIAN_CPP_H__
#define __CV_FILTER_MEDIAN_CPP_H__

#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <stdlib.h>

template <typename T>
void median1d(std::vector<T> & out, std::vector<T> const& in, int msize);

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

/*int
compare (const void * a, const void * b) {
   return ( *(int*)a - *(int*)b );
}*/

/**
 *  Standard median 1d filter.
 *  Doesn't perform in-place filtering, so
 *  arrays should be pointers to physically different memory places.
 *
 * \param[out] - output filtered array(time series), memory must be preallocated, length must match with length of 'in';
 * \param[in] - input array(time series);
 * \param[in] - size of sliding mask, odd number = {3, 5, 7, ..., l / 2 - 1 }.
 *
 */
template <typename T>
void median1d(std::vector<T> & out, std::vector<T> const& in, int msize) {
  unsigned int l = in.size();
  // Run in array over with sliding mask.
  int shift = msize / 2;
  for(unsigned int i = shift; i < l - shift; ++i) {
    // Get current mask.
    std::vector<T> mask(msize);
    for(int j = -shift; j < shift; ++j) {
      mask[shift + j] = in[i + j];
    }

    // Sort mask in descending or ascending order.
    std::sort(mask.begin(), mask.end());

    // Get pivoting element.
    float pivot = mask[shift];

    // Set it to output array.
    out[i] = pivot;
  }
}

#endif
