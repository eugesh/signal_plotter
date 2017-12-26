#include <cmath>
#include <cstdlib>
#include <cfloat>
#include <climits>
#include "signals_eval.h"

static const double eps = 1e-21;

/*
 * This function should work with raw, not filtered signal.
 */
unsigned int
find_radio_signal_termination(Samples const& data) {
    unsigned int end_time_stamp = 0;

    // Find max and min abs value.
    double max_val = 0, min_val = DBL_MAX;
    for(unsigned int i = 0; i < data.size(); ++i) {
        if(max_val < fabs(data[i]))
            max_val = fabs(data[i]);
        if(min_val > fabs(data[i]))
            min_val = fabs(data[i]);
    }

    printf("max_val = %.21f, min_val = %.21f\n", max_val, min_val);

    /*
     * Go from the end to the beginning of signal and
     * find the first amplitude value higher than e.g. 0.3 of the max abs value.
     */
    unsigned int interm_index = 0;
    for(unsigned int i = data.size() - 1; i > 0; --i) {
        if(fabs(data[i]) > 0.1 * max_val) {
            interm_index = i;
            break;
        }
    }

    printf("interm_index = %d\n", interm_index);

    /*
     * Go from time stamp found in previous step toward the end of not filtered signal.
     * Find time stamp of intersection with zero. This is the end of radio impulse.
     */
    for(unsigned int i = interm_index; i < data.size(); ++i) {
        if ((fabs(data[i]) - min_val) < eps) {
            printf("(fabs(data[i]) - min_val) = %.21f\n", (fabs(data[i]) - min_val));
            end_time_stamp = i;
            break;
        }
    }

    return end_time_stamp;
}

/**
 * Intersections with zero finder.
 */
Intervals
find_all_zeros_indices(Samples const& data) {
    Intervals intervals;

    return intervals;
}

/**
 * Peaks counter. Returns vector of all peaks, including outliers.
 */
Peaks
find_all_peaks (Samples const& data, Intervals const& zero_intervals) {
    Peaks all_peaks;

    return all_peaks;
}

/*
 * Find relevant peaks.
 */
Peaks
find_real_peaks (Samples const& data, Peaks const& all_peaks) {
    Peaks peaks;
    // Find max_area among peaks. Area is approximated as rectangle for simplicity.

    // Find all peaks with area larger than 0.05 * max_area.

    return peaks;
}

