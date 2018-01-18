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
/*
 * Centers of intervals.
 */
std::vector<unsigned int> intervals2points(Intervals const& intervals);

/**
 * Sign changes searching.
 *
 * \param data input signal;
 * \param start the first index of analysing interval;
 * \param end the last index of analysing interval;
 *
 * \return vector of indices of sign changes.
 */
std::vector<unsigned int> sign_changes(Samples const& data, unsigned int start, unsigned int end);

/**
 * Peaks extractor. Returns vector of all peaks, including outliers.
 *
 * \param data - time series to analyze;
 * \param zero_intervals - start and end point of zero interval;
 * \param start - start index in data to analyze;
 * \param end - finish index in data to analyze
 * \return
 *
 */
Peaks find_all_peaks(Samples const& data, Intervals const& zero_intervals);

/**
 * Peaks extractor. Returns vector of all peaks, including outliers.
 *
 * \param data - time series to analyze;
 * \param zero_points - sign change points;
 * \param start - start index in data to analyze;
 * \param end - finish index in data to analyze
 * \return
 *
 */
Peaks find_all_peaks(Samples const& data, std::vector<unsigned int> const& zero_points);

/*
 * Find relevant peaks.
 */
Peaks find_real_peaks(std::vector<unsigned int> &zero_points, Samples const& data, Peaks const& all_peaks, double threshold_ratio);

/**
 * Estimate period.
 */
double estimate_period(Peaks const& peaks);

/**
 * Estimate frequency.
 */
double estimate_frequency(Peaks const& peaks, double first, double step);

/**
 * Estimate frequency by Fourier spectrum analysis.
 */
double estimate_frequency_fft(Samples const& data, double first, double step);

/**
 * Estimate quality of oscillation (Q factor = w0 / (2 * attenuation rate) ).
 *
 */
double estimate_quality(Samples const& data, Peaks const& peaks);

double estimate_quality_ls(double *a, double *b, Samples const& data, Peaks const& peaks, double first, double step, unsigned int r_end_i);


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
 *
 */
void half_periods_verificator(Intervals const& half_periods, float *max_dev, float *mean_dev);

void half_periods_verificator(std::vector<unsigned int> const& zero_points, float *max_dev, float *mean_dev);
/**
 * Curve fitting.
 *
 */

/**
 * Antenna parameters estimator.
 *
 */

#endif
