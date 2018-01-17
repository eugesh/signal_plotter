#ifndef __LOWPASS_H__
#define __LOWPASS_H__

#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <cstdlib>
#include <cfloat>
#include <climits>
#include <complex>
#include <unsupported/Eigen/FFT>

typedef double real;

typedef std::vector<real> vec_f;
typedef std::vector<std::complex<real> > vec_cf;
typedef std::vector<int> vec_i;

/**
 * Low Pass in frequency domain. Works with frequency vector.
 */
vec_cf lp_freq(vec_cf const& vfreq, real Fs, real cut_f) {
	unsigned int L = vfreq.size();

	// The first and the last frequencies.
	real first_freq = 0 * Fs / L;
	real last_freq = (L - 1) * Fs / L;

	// Index to cut frequency vector.
	unsigned int cut_index = floor((last_freq - first_freq) / cut_f);

	if(cut_index > vfreq.size())
		cut_index = vfreq.size() - 1;

	return vec_cf(vfreq.begin(), vfreq.begin() + cut_index);
}

template<typename T> // T - vector
T
rediscretization(T const& vec, unsigned int out_size) {
	T out(out_size);
	double scale = double(vec.size()) / out_size;

	for(unsigned int i = 0; i < out_size; ++i) {
		out[i] = vec[int(i * scale)];
	}

	return out;
}

template<typename T> // T - vector
void
amplitude_rescale(T &in_out_vec, T const& vec) {
	// Find average scale
	double scale_sum = 0;
	for(unsigned int i = 0; i < vec.size(); ++i)
		scale_sum += fabs(double(vec[i]) / in_out_vec[i]);

	double scale = scale_sum / vec.size();

	// Rescale to original ampl.
	for(unsigned int i = 0; i < vec.size(); ++i) {
		in_out_vec[i] = in_out_vec[i] * scale;
	}
}

/*template<typename T> // T - vector
void
amplitude_rescale(T &in_out_vec, T const& vec) {
	// Find max ampl.
	double max_ampl_out = 0;
	double max_ampl_in = 0;
	for(unsigned int i = 0; i < vec.size(); ++i) {
		if(max_ampl_in < fabs(vec[i])) {
			max_ampl_in = fabs(vec[i]);
		}
		if(max_ampl_out < fabs(in_out_vec[i])) {
			max_ampl_out = fabs(in_out_vec[i]);
		}
	}

	double scale = max_ampl_in / max_ampl_out;

	// Rescale to original ampl.
	for(unsigned int i = 0; i < vec.size(); ++i) {
		in_out_vec[i] = in_out_vec[i] * scale;
	}
}*/

/**
 * Low Pass in frequency domain. Works with amplitude.
 */
vec_f lp_ampl(vec_f v_ampl, real step, real cut_f) {
	vec_f filtered;

	// Sampling frequency.
  double Fs = 1.0 / step;
  printf("Fs = %f\n", Fs);
  printf("v_ampl.size() = %u\n", v_ampl.size());

  Eigen::FFT<Real> fft;
  std::vector<std::complex<real> > freqvec;

	// Forward Fourier transform.
  fft.fwd(freqvec, v_ampl);

  // Apply lowpass filter.
  freqvec = lp_freq(freqvec, Fs, cut_f);

  // Backward Fourier transform.
  fft.inv(filtered, freqvec);

  printf("filtered.size() = %u\n", filtered.size());

  // Retrieve discretization.
  vec_f filtered_out = rediscretization(filtered, v_ampl.size());

  // Rescale amplitude.
  amplitude_rescale(filtered_out, v_ampl);

	return filtered_out;
	// Stored "plans" get destroyed with fft's destructor.
}

#endif
