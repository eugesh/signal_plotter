#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <cstdlib>
#include <cfloat>
#include <climits>
#include "signals_eval.h"
#include "math_module.h"

static const double eps = 1e-12;

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
        if(fabs(data[i]) > 0.4 * max_val) {
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
    printf ("find_all_zeros_indices - start\n"); // debug
    Intervals intervals;

    // Find abs min element.
    double min = DBL_MAX;
    for (unsigned int i = 0; i < data.size(); ++i)
        if(min > fabs(data[i]))
            min = fabs(data[i]);

    printf("min = %.21f\n", min); // debug

    bool is_interval = false;
    for (unsigned int i = 0; i < data.size() - 1; ++i) {
        if (fabs(data[i]) < min + eps && fabs(data[i + 1]) < min + eps && !is_interval) {
            is_interval = true;
            intervals.push_back(std::make_pair(i, i + 1));
            printf ("start = %d", i); // debug
        }
        else if (fabs(data[i]) < eps && fabs(data[i + 1]) > min + eps &&  is_interval) {
            is_interval = false;
            intervals.back().second = i;
            printf (", end = %d\n", i); // debug
        }
    }

    printf ("find_all_zeros_indices - end\n"); // debug
    return intervals;
}

/**
 * Peaks counter. Returns vector of all peaks, including outliers.
 */
Peaks
find_all_peaks (Samples const& data, Intervals const& zero_intervals) {
    printf ("find_all_peaks - start\n"); // debug
    Peaks all_peaks;

    // Iterate over intervals.
    for (unsigned int i = 0; i < zero_intervals.size() - 1; ++i) {
        Peak peak;
        peak.start_index = zero_intervals[i].second;
        peak.end_index = zero_intervals[i + 1].first;

        // Iterate through interval.
        double max_val = 0;
        unsigned int extremum_index = 0;
        for (unsigned int j = zero_intervals[i].second; j < zero_intervals[i + 1].first; ++j) {
            if (max_val < fabs(data[j])) {
                max_val = fabs(data[j]);
                extremum_index = j;
            }
        }
        // peak.extremum_val = data[extremum_index];
        peak.extremum_index = extremum_index;
        printf ("peak.extremum_val = %f\n", data[extremum_index]); // debug
        all_peaks.push_back(peak);
    }

    printf ("find_all_peaks - finish.\n"); // debug

    return all_peaks;
}

/*
 * Find relevant peaks.
 * Get rid of outliers by area under curve thresholding.
 *
 * \param threshold_ratio (0,1) - part of an area of peak with the maximal area.
 * \return vector of real peaks.
 *
 */
Peaks
find_real_peaks (Samples const& data, Peaks const& all_peaks, double threshold_ratio) {
    Peaks peaks;

    // Calculate set of area and find max_area among peaks.
    // Area is approximated as rectangle for simplicity.
    std::vector<double> area_vec;
    double max_area = 0;
    for(unsigned int i=0; i < all_peaks.size(); ++i) {
        // Area of rectangle.
        double area = 0.5 * (all_peaks[i].end_index - all_peaks[i].start_index) * fabs(data[all_peaks[i].extremum_index]);
        area_vec.push_back(area);
        if(max_area < area)
            max_area = area;
    }

    // Find all peaks with area larger than 0.05 * max_area.
    for(unsigned int i=0; i < all_peaks.size(); ++i) {
        if (area_vec[i] > threshold_ratio * max_area)
            peaks.push_back(all_peaks[i]);
    }

    return peaks;
}

/**
 * Estimate period.
 */
unsigned int
estimate_period(Peaks const& peaks) {
    unsigned int half_period_cumsum = 0;

    for (unsigned int i = 0; i < peaks.size(); ++i) {
        unsigned int half_period = peaks[i].end_index - peaks[i].start_index;
        if(half_period > 0)
            half_period_cumsum += half_period;
        else
            printf("Error: %dth period < 0", i);
    }

    return 2 * half_period_cumsum / peaks.size();
}

/**
 * Estimate frequency.
 */
double
estimate_frequency(Peaks const& peaks, double first, double step) {
    unsigned int first_index = peaks.front().start_index;
    unsigned int last_index = peaks.back().end_index;

    unsigned int period = estimate_period(peaks);

    return (( (last_index * step + first) * step - first_index * step + first) * step) / (period * step);
}

/**
 * Estimate quality of oscillation (Q factor = w0 / (2 * attenuation rate) ).
 *
 */
double
estimate_quality(Samples const& data, Peaks const& peaks) {
    double q_factor = .0;

    double A0 = fabs(data[peaks.front().extremum_index]);
    double An = fabs(data[peaks.back().extremum_index]);
    // double n_of_periods = (double) (peaks.size()) / 2.0;
    double n_of_periods = (peaks.size()) / 2;
    q_factor = M_PI * n_of_periods / (log(A0 / An));

    printf("A0 = %f\n", A0);
    printf("An = %f\n", An);
    printf("log(A0 / An) = %f\n", log(A0 / An));
    printf("n_of_periods = %f\n", n_of_periods);
    printf("q_factor = %f\n", q_factor);

    return q_factor;
}

/**
 * Estimate q-factor with least squares.
 * model: sum(ln(yi) - a * xi + b)^2 -> min
 */
double
estimate_quality_ls(double *a, double *b, Samples const& data, Peaks const& peaks, double first, double step) {
    // The problem is {[xi 1]} * [a; b] = {log(yi)}
    double q_factor = .0;

    std::vector<double> x, y;
    // double a, b;

    for (unsigned int i = 0; i < peaks.size(); ++i) {
        y.push_back(log(fabs(data[peaks[i].extremum_index])));
        x.push_back(peaks[i].extremum_index);// * step + first);
    }

    // linear_approximation(&a, &b, x, y);
    linear_approximation(a, b, x, y);

    printf("a = %f, b = %f\n", *a, *b);

    return q_factor;
}

/**
 * Interface function for all previous functions.
 */
void signal_analyzer(double *a, double *b, Samples const& data, double *q_factor, double *freq, double first, double step) {
    printf ("signal_analyzer - start\n");

    Intervals zero_intervals = find_all_zeros_indices(data);

    Peaks all_peaks = find_all_peaks(data, zero_intervals);

    Peaks real_peaks = find_real_peaks(data, all_peaks, 0.05);

    *freq = estimate_frequency(real_peaks, first, step);

    *q_factor = estimate_quality(data, real_peaks);
    *q_factor = estimate_quality_ls(a, b, data, real_peaks, first, step);
}
