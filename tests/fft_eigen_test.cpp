#include <cmath>
#include <complex>
#include <vector>
#include <unsupported/Eigen/FFT>

typedef float real;

/**
 * Generates sinusoidal time series.
 * y = y0 + A * sin(w * t + theta)
 *
 * \param Fs - sampling frequency;
 * \param freq - omega / 2 * pi.
 */
std::vector<real> GenSinus(unsigned int N, double Fs, double y0, double A, double omega, double theta) {
  std::vector<real> vec;

  for (unsigned int i = 0; i < N; ++i) {
    vec.push_back(y0 + A * sin(omega * double(i) / Fs + theta));
  }

  return vec;
}

real spectrum_analysis(std::vector<std::complex<real> > freqvec, double Fs) {
  real freq = 0.0;
  unsigned int i_max = 0;

  // Find max.
  real max_val = std::abs(freqvec[0]);
  for (unsigned int i = 0; i < freqvec.size(); ++i) {
      real ampl = std::abs(freqvec[i]);
      if(max_val < ampl) {
          max_val = ampl;
          i_max = i;
      }
  }

  freq = real(i_max - 1) * Fs / (freqvec.size());

  return freq;
}

int main() {
  Eigen::FFT<real> fft;

  double Fs = 10000;
  double omega = 1000;

  std::vector<real> timevec = GenSinus(1e5, Fs, 0, 10, omega, 0.5);
  std::vector<std::complex<real> > freqvec;

  fft.fwd(freqvec, timevec);
  // Manipulate freqvec
  real freq = spectrum_analysis(freqvec, Fs);

  printf("freq = %f\n", freq);
  printf("omega - omega_res = %f\n", omega - freq * 2 * M_PI);
  printf("error, % = %f\n", (omega - freq * 2 * M_PI) * 100 / omega);

  fft.inv(timevec, freqvec);

  // stored "plans" get destroyed with fft destructor

  return 0;
}
