#ifndef __SIGNALS_EVAL_H__
#define __SIGNALS_EVAL_H__

typedef double Real;
// typedef std::vector<std::vector<Real> > Samples;
typedef std::vector<Real> Samples;

/**
 * Peaks finder.
 */

/**
 * Intersections with zero finder.
 */

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
float radio_signal_termination(Samples const& data);

/**
 * Curve fitting.
 *
 */

/**
 * Antenna parameters estimator.
 *
 */

#endif
