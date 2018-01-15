#ifndef __SIGNALS_EVAL_H__
#define __SIGNALS_EVAL_H__

#include <iostream>
#include <vector>
#include <utility> // pair

/**
 * Peak of sinusoidal function.
 */
struct Peak {
    // Start index of peak, last index of zero interval.
    unsigned int start_index;
    // Last index of peak, first index of zero interval.
    unsigned int end_index;
    // Signed value: max or min.
    // double extremum_val;
    unsigned int extremum_index;
};

typedef double Real;
typedef std::vector<Real> Samples;
typedef std::vector<std::pair<unsigned int, unsigned int> > Intervals;
typedef std::vector<Peak> Peaks;

/**
 * Peaks finder.
 */

/**
 * Intersections with zero finder.
 */
Intervals find_all_zeros_indices(Samples const& data, unsigned int start, unsigned int end);

/**
 * Peaks counter. Returns vector of all peaks, including outliers.
 *
 * \param zero_intervals - start and end point of zero interval.
 */
Peaks find_all_peaks(Samples const& data, Intervals const& zero_intervals);

/*
 * Find relevant peaks.
 */
Peaks find_real_peaks(Samples const& data, Peaks const& all_peaks, double threshold_ratio);

/**
 * Estimate period.
 */
double estimate_period(Peaks const& peaks);

/**
 * Estimate frequency.
 */
double estimate_frequency(Samples const& data, Peaks const& peaks, double first, double step, unsigned int start, unsigned int end);

/**
 * Estimate frequency by Fourier spectrum analysis.
 */
double estimate_frequency_fft(Samples const& data, double first, double step, unsigned int start, unsigned int end);

/**
 * Estimate quality of oscillation (Q factor = w0 / (2 * attenuation rate) ).
 *
 */
double estimate_quality(Samples const& data, Peaks const& peaks, unsigned int start, unsigned int end);

double estimate_quality_ls(double *a, double *b, Samples const& data, Peaks const& peaks, double first, double step, unsigned int r_end_i, unsigned int start, unsigned int end);


/**
 * Interface function for all previous functions.
 */
// void signal_analyzer(double *a, double *b, Samples const& data, double *q_factor, double *freq, double first, double step, unsigned int r_end_i);

/**
 * The first intersection with zero.
 *
 * \return time stamp of the last zero value.
 */
float find_first_zero(Samples const& data);

/**
 * The last intersection with zero.
 *
 * \return time stamp of the last zero value.
 */
float find_last_zero(Samples const& data);

/**
 * Radio signal termination seeker.
 *
 * \return time stamp of the radio signal termination.
 */
unsigned int find_radio_signal_termination(Samples const& data);

/**
 * Curve fitting.
 *
 */

/**
 * Antenna parameters estimator.
 *
 */

#endif
