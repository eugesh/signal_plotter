#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <cstdlib>
#include <cfloat>
#include <climits>
#include <complex>
#include "signals_eval.h"
#include "math_module.h"
#include <unsupported/Eigen/FFT>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const double eps = 1e-12;

/*Intervals
peaks2intervals(Peaks const& peaks) {
	Intervals intervals;
	for (int i = 0; i < peaks.size() - 1; ++i) {
		intervals.push_back(std::make_pair());
	}

	return intervals;
}*/

/**
 * The simplest way: center.
 */
std::vector<unsigned int> intervals2points(Intervals const& intervals) {
	std::vector<unsigned int>points;
	for(unsigned int i = 0; i < intervals.size(); ++i) {
		points.push_back(intervals[i].first + (intervals[i].second - intervals[i].first) / 2);
	}

	return points;
}

template <typename T>
T
spectrum_analysis(std::vector<std::complex<T> > freqvec, T Fs) {
  unsigned int i_max = 0;

  // Find index of frequency with max rate.
  T max_val = std::abs(freqvec[0]);
  for (unsigned int i = 0; i < freqvec.size(); ++i) {
      T ampl = std::abs(freqvec[i]);
      if(max_val < ampl) {
          max_val = ampl;
          i_max = i;
      }
  }

  return T(i_max - 1) * Fs / (freqvec.size());
}

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
     * find the first amplitude value higher than e.g. 0.4 of the max abs value.
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
find_all_zeros_indices(Samples const& data, unsigned int start, unsigned int end) {
    printf ("find_all_zeros_indices - start\n"); // debug
    Intervals intervals;

    // Find abs min element.
    double min = DBL_MAX;
    for (unsigned int i = 0; i < data.size(); ++i)
        if(min > fabs(data[i]))
            min = fabs(data[i]);

    printf("min = %.21f\n", min); // debug

    bool is_interval = false;
    for (unsigned int i = start; i < end - 1; ++i) {
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

std::vector<unsigned int>
sign_changes(Samples const& data, unsigned int start, unsigned int end) {
	std::vector<unsigned int> zero_indices;

	for(unsigned int i = start; i < end-1; ++i) {
		if(copysign(1.0, (double)data[i]) != copysign(1.0, (double)data[i+1]))
			zero_indices.push_back(i);
	}

	return zero_indices;
}

/**
 * Peaks counter. Returns vector of all peaks, including outliers.
 */
Peaks
find_all_peaks (Samples const& data, Intervals const& zero_intervals) {
    printf("find_all_peaks - start\n"); // debug
    Peaks all_peaks;

    // Iterate over zero intervals.
    for(unsigned int i = 0; i < zero_intervals.size() - 1; ++i) {
        Peak peak;
        peak.start_index = zero_intervals[i].second;
        peak.end_index = zero_intervals[i + 1].first;

        // Iterate through non-zero interval.
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
        printf("max_val = %f\n", max_val);
        printf("peak.extremum_val = %f\n", data[extremum_index]); // debug
        all_peaks.push_back(peak);
    }

    printf("find_all_peaks - finish.\n"); // debug

    return all_peaks;
}

Peaks
find_all_peaks (Samples const& data, std::vector<unsigned int> const& zero_points) {
    Peaks all_peaks;

    // Iterate over zero intervals.
    for(unsigned int i = 0; i < zero_points.size() - 1; ++i) {
        Peak peak;
        peak.start_index = zero_points[i];
        peak.end_index = zero_points[i + 1];

        // Iterate through non-zero interval.
        double max_val = 0;
        unsigned int extremum_index = 0;
        for (unsigned int j = zero_points[i]; j < zero_points[i + 1]; ++j) {
            if (max_val < fabs(data[j])) {
                max_val = fabs(data[j]);
                extremum_index = j;
            }
        }
        // peak.extremum_val = data[extremum_index];
        peak.extremum_index = extremum_index;
        all_peaks.push_back(peak);
    }

    return all_peaks;
}

/*
 * Find relevant peaks.
 * Get rid of outliers by area under curve thresholding.
 *
 * \param zero_points[inout] - input vector of zero points, output cut vector;
 * \param threshold_ratio[in] = (0, 1) - minimal area as part of an area of peak with the maximal area;
 * \param
 *
 * \return vector of real peaks.
 */
Peaks
find_real_peaks(std::vector<unsigned int> &zero_points, Samples const& data, Peaks const& all_peaks, double threshold_ratio) {
    Peaks peaks;

    // Calculate set of area and find max_area among peaks.
    // Area is approximated as rectangle for simplicity.
    std::vector<double> area_vec;
    double max_area = 0.0;
    for(unsigned int i = 0; i < all_peaks.size(); ++i) {
        // Area of rectangle.
        double area = 0.5 * (all_peaks[i].end_index - all_peaks[i].start_index) * fabs(data[all_peaks[i].extremum_index]);
        area_vec.push_back(area);
        if(max_area < area)
            max_area = area;
    }

    // Find all peaks with area larger than 0.05 * max_area.
    for(unsigned int i = 0; i < all_peaks.size(); ++i) {
        if (area_vec[i] > threshold_ratio * max_area) {
            peaks.push_back(all_peaks[i]);
        }
    }

    // Eliminate vector of zero pints. Remove all points to the right side of the last peak.
    printf("peaks.back().end_index = %d\n", peaks.back().end_index);
    for (int i = all_peaks.size(); i > 0 && peaks.back().end_index < zero_points.back(); i--) {
    	zero_points.pop_back();
    	printf("zero_points.back() = %d\n", zero_points.back());
    }

    return peaks;
}

/**
 * Estimate period.
 */
double
estimate_period(Peaks const& peaks) {
    unsigned int half_period_cumsum = 0;

    for (unsigned int i = 0; i < peaks.size(); ++i) {
        unsigned int half_period = peaks[i].end_index - peaks[i].start_index;
        if(half_period > 0)
            half_period_cumsum += half_period;
        else
            printf("Error: %dth period < 0\n", i);
    }

    // return 2.0 * double(half_period_cumsum) / double(peaks.size());
    unsigned int first_index = peaks.front().start_index;
    unsigned int last_index = peaks.back().end_index;
    return 2.0 * double(last_index - first_index) / double(peaks.size());
}

/**
 * Estimate frequency.
 */
/*double
estimate_frequency(Peaks const& peaks, double first, double step) {
    unsigned int first_index = peaks.front().start_index;
    unsigned int last_index = peaks.back().end_index;

    printf("((last_index - first_index) = %d, step = %.21f\n", (last_index - first_index), step);

    return double (peaks.size()) / (double(last_index - first_index) * step * 2.0);
}*/
/*double
estimate_frequency(Peaks const& peaks, double first, double step) {
    unsigned int first_index = peaks.front().start_index;
    unsigned int last_index = peaks.back().end_index;

    unsigned int period = estimate_period(peaks);

    printf("last_index - first_index = %d, ", last_index - first_index);
    printf("period = %d, ", period);

    return (last_index - first_index) * step / period;
}*/

double
estimate_frequency(Peaks const& peaks, double first, double step) {
    // unsigned int first_index = peaks.front().start_index;
    // Indices of the first and the last peaks.
    unsigned int first_index = peaks.front().start_index;
    // unsigned int first_index = peaks[2].start_index;
    unsigned int last_index = peaks.back().end_index;

    printf("last_index - first_index = %d, ", last_index - first_index);

    unsigned int even_peaks_num = peaks.size() - peaks.size() % 2;

    double even_peaks_range = floor(double(last_index - first_index));

    printf("even_peaks_num = %d\n", even_peaks_num);

    return even_peaks_num / (2.0 * even_peaks_range * step);
}

/**
 * Realization with Eigen3's fft.
 */
double
estimate_frequency_fft(Samples const& data, double first, double step) {
  double Fs = 1.0 / step;
  printf("Fs = %f\n", Fs);
  Eigen::FFT<Real> fft;
  std::vector<std::complex<Real> > freqvec;

  fft.fwd(freqvec, data);

  return spectrum_analysis(freqvec, Fs);
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

    double n_of_periods = double(peaks.size() - 1) / 2.0;
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
 * The problem is {[xi 1]} * [a; b] = {log(yi)}
 */
double
estimate_quality_ls(double *a, double *b, Samples const& data, Peaks const& peaks, double first, double step, unsigned int r_end_i) {
    std::vector<double> x, y;

    for (unsigned int i = 0; i < peaks.size(); ++i) {
        if(peaks[i].extremum_index > r_end_i) {
            y.push_back(log(fabs(data[peaks[i].extremum_index])));
            x.push_back(peaks[i].extremum_index);// * step + first);
        }
    }

    // linear_approximation(&a, &b, x, y);
    linear_approximation(a, b, x, y);

    printf("a = %f, b = %f\n", *a, *b);

    return *a;
}

void
half_periods_verificator(Intervals const& zero_intervals, float *max_dev, float *mean_dev) {
	float sum = 0.0;
	*mean_dev = 0, *max_dev = 0;
	// Find half period mean.
	for(unsigned int i = 0; i < zero_intervals.size() - 1; ++i) {
		unsigned int zero_interval_left = zero_intervals[i].second - zero_intervals[i].first;
		unsigned int zero_interval_right = zero_intervals[i + 1].second - zero_intervals[i + 1].first;
		unsigned int half_period = zero_intervals[i + 1].first - zero_intervals[i].second;
		unsigned int full_half_period = half_period + (zero_interval_left + zero_interval_right) / 2;
		printf("zero_interval_left = %d, zero_interval_right = %d, full_half_period = %d\n", zero_interval_left, zero_interval_right, full_half_period);
		sum += full_half_period;
	}
	float mean_full_half_period = sum / (zero_intervals.size() - 1);

	// Find max and mean deviation.
	sum = 0.0;
	for(unsigned int i = 0; i < zero_intervals.size() - 1; ++i) {
		unsigned int zero_interval_left = zero_intervals[i].second - zero_intervals[i].first;
		unsigned int zero_interval_right = zero_intervals[i + 1].second - zero_intervals[i + 1].first;
		unsigned int half_period = zero_intervals[i + 1].first - zero_intervals[i].second;
		float full_half_period = half_period + (zero_interval_left + zero_interval_right) / 2;
		float cur_dev = full_half_period / mean_full_half_period;
		sum += cur_dev;
		if(*max_dev < cur_dev)
			*max_dev = cur_dev;
	}

	*mean_dev = sum / (zero_intervals.size() - 1);
}

void
half_periods_verificator(std::vector<unsigned int> const& zero_points, float *max_dev, float *mean_dev) {
	float sum = 0.0;
	*mean_dev = 0, *max_dev = 0;
	// Find half period mean.
	for(unsigned int i = 0; i < zero_points.size() - 1; ++i) {
		unsigned int half_period = zero_points[i + 1] - zero_points[i];
		printf("half_period = %d\n", half_period);
		sum += half_period;
	}
	float mean_half_period = sum / (zero_points.size() - 1);

	// Find max and mean deviation.
	sum = 0.0;
	for(unsigned int i = 0; i < zero_points.size() - 1; ++i) {
		unsigned int half_period = zero_points[i + 1] - zero_points[i];
		float cur_dev = fabs(mean_half_period - float(half_period)) / mean_half_period;
		sum += cur_dev;
		if(*max_dev < cur_dev)
			*max_dev = cur_dev;
	}

	*mean_dev = sum / (zero_points.size() - 1);
}

/**
 * Interface function for all previous functions.
 */
/*void signal_analyzer(double *a, double *b, Samples const& data, double *q_factor, double *freq, double first, double step, unsigned int r_end_i) {
    printf ("signal_analyzer - start\n");

    Intervals zero_intervals = find_all_zeros_indices(data);

    Peaks all_peaks = find_all_peaks(data, zero_intervals);

    Peaks real_peaks = find_real_peaks(data, all_peaks, 0.05);

    *freq = estimate_frequency(real_peaks, first, step);

    *q_factor = estimate_quality(data, real_peaks);
    estimate_quality_ls(a, b, data, real_peaks, first, step, r_end_i);
}*/
