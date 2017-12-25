#include "stdio.h"
#include "stdlib.h"
#include "time.h"
#include "math.h"
#include "median.h"

static const unsigned int N = 1000000;
static const float noise_percent = 0.1;

int main() {
    srand(time(NULL));
    float* test_ar = new float[N];
    float* test_ar_filtered = new float[N];

    unsigned int noise_step = (unsigned int) (1 / noise_percent);

    // Create testing array.
    for(unsigned int i = 0; i < N; ++i) {
        if(rand() % noise_step == 0)
            test_ar[i] = -1 + rand() % 3;
    }

    // Print out testing array.
    float sum = 0;
    for(unsigned int i=0; i < N; ++i) {
        // printf("%f ", test_ar[i]);
        sum += abs(test_ar[i]);
    }
    printf("sum = %f\n", sum);

    // Filter array.
    median1d(test_ar_filtered, test_ar, N, 19);

    sum = 0;
    // Print out the result.
    for(unsigned int i=0; i < N; ++i) {
        // printf("%f ", test_ar_filtered[i]);
        sum += abs(test_ar_filtered[i]);
    }

    printf("sum = %f\n", sum);

    delete[] test_ar;
    delete[] test_ar_filtered;

    return 0;
}
