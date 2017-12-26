#ifndef __SIGNALS_EVAL_H__
#define __SIGNALS_EVAL_H__

#include <iostream>
#include <vector>
#include <utility> // pair

struct Peak {
    unsigned int start_index;
    unsigned int end_index;
    double extremum_val;
};

typedef double Real;
// typedef std::vector<std::vector<Real> > Samples;
typedef std::vector<Real> Samples;
typedef std::vector<std::pair<unsigned int, unsigned int> > Intervals;
typedef std::vector<Peak> Peaks;

/**
 * Peaks finder.
 */

/**
 * Intersections with zero finder.
 */
Intervals find_all_zeros_indices(Samples const& data);

/**
 * Peaks counter. Returns vector of all peaks, including outliers.
 *
 * \param zero_intervals - start and end point of zero interval.
 */
Peaks find_all_peaks (Samples const& data, Intervals const& zero_intervals);

/*
 * Find relevant peaks.
 */
Peaks find_real_peaks (Samples const& data, Peaks const& all_peaks);

/**
 * The first intersection with zero.
 *
 * \return time stamp of the last zero value.
 */
float find_first_zero (Samples const& data);

/**
 * The last intersection with zero.
 *
 * \return time stamp of the last zero value.
 */
float find_last_zero (Samples const& data);

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
