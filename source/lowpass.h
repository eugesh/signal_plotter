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
 * Cuts off high frequencies.
 */
vec_cf lp_freq_cutoff(vec_cf const& vfreq, real Fs, real cut_f) {
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

/**
 * Low Pass in frequency domain. Works with frequency vector.
 * Applies the simplest transfer function.
 */
vec_cf lp_freq(vec_cf const& vfreq, real Fs, real cut_f) {
	unsigned int L = vfreq.size();
	vec_cf out_vec(vfreq.size());

	// The first and the last frequencies.
	real first_freq = 0 * Fs / L;
	real last_freq = (L - 1) * Fs / L;

	// Index to cut frequency vector.
	unsigned int cut_index = vfreq.size() - 1;
	if ((last_freq - first_freq) > 0)
		cut_index = floor(vfreq.size() * cut_f / (last_freq - first_freq));

	if(cut_index > vfreq.size())
		cut_index = vfreq.size() - 1;

	for(unsigned int i = 0; i < vfreq.size(); ++i) {
		if(i < cut_index) {
			out_vec[i] = vfreq[i];
		}
		else {
			out_vec[i] = 0.000001 * vfreq[i];
		}
	}

	return out_vec;
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

/*
 * Central function of the file.
 */
/**
 * Low Pass in frequency domain. Works with amplitude.
 *
 * \param v_ampl input signal;
 * \param step discretization interval size, Fs = 1 / step;
 * \param cut_f threshold frequency, higher which ones are suppressed;
 *
 * \return filtered vector.
 */
vec_f lp_ampl(vec_f v_ampl, real step, real cut_f) {
	vec_f filtered;

	// Sampling frequency.
  double Fs = 1.0 / step;

  Eigen::FFT<Real> fft;
  std::vector<std::complex<real> > freqvec;

	// Forward Fourier transform.
  fft.fwd(freqvec, v_ampl);

  // Apply lowpass filter.
  freqvec = lp_freq(freqvec, Fs, cut_f);

  // Backward Fourier transform.
  fft.inv(filtered, freqvec);

	return filtered;
	// Stored "plans" get destroyed with fft's destructor.
}

#endif
