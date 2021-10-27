import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import hilbert, chirp, find_peaks

path2file = 'D:/workspace/screenco/signal_plotter/data/sweep/sweep_RLC_16.csv'
path2file = 'D:/workspace/screenco/signal_plotter/data/sweep/sweep_RLC_8_diff500Hz.csv'

duration = .10
fs = 100000.0
samples = int(fs*duration)
samples = 2500
#t = np.arange(samples) / fs
#We create a chirp of which the frequency increases from 20 Hz to 100 Hz and apply an amplitude modulation.

#signal = chirp(t, 20.0, t[-1], 100.0)
#signal *= (1.0 + 0.5 * np.sin(2.0*np.pi*3.0*t) )
#The amplitude envelope is given by magnitude of the analytic signal.
#The instantaneous frequency can be obtained by differentiating the instantaneous phase in respect to time.
#The instantaneous phase corresponds to the phase angle of the analytic signal.

with open(path2file, newline='') as csvfile:
     csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
     count = 0
     signal = []
     t = []
     for row in csvreader:
         print(', '.join(row))
         if count > 2:
            t.append(row[0])
            signal.append(row[1])
         count += 1

t = np.array(t,dtype='double')
signal = np.array(signal,dtype='double')


analytic_signal = hilbert(signal)
amplitude_envelope = np.abs(analytic_signal)
instantaneous_phase = np.unwrap(np.angle(analytic_signal))
instantaneous_frequency = (np.diff(instantaneous_phase) /
                           (2.0*np.pi) * fs)
fig, (ax0, ax1, ax2, ax3) = plt.subplots(nrows=4)
ax0.plot(t, signal, label='signal')
ax0.plot(t, amplitude_envelope, label='envelope')
ax0.plot(t, analytic_signal, label='analytic_signal')
ax0.plot(t[1:t.size], np.diff(analytic_signal), label='diff')
#ax0.plot(t[1:t.size], analytic_signal[0:-1] - np.diff(analytic_signal), label='diff-sig')
ax0.set_xlabel("Frequency, Hz")
ax0.legend()

ax1.plot(t[1:], instantaneous_frequency, label='instantaneous_frequency')
ax1.set_xlabel("Frequency, Hz")
ax1.legend()

ax2.plot(t, instantaneous_phase, label='instantaneous_phase')
ax2.set_xlabel("Frequency, Hz")
ax2.legend()

ipeaks_plus = find_peaks(analytic_signal)
ipeaks_minus = find_peaks(-analytic_signal)

ax3.plot(t[ipeaks_plus[0]], analytic_signal[ipeaks_plus[0]], label='peaks+')
ax3.plot(t[ipeaks_minus[0]], analytic_signal[ipeaks_minus[0]], label='peaks-')
ax3.set_xlabel("Frequency, Hz")
ax3.legend()

fig.tight_layout()
fig.show()
