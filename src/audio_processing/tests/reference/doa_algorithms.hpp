#pragma once

#include <complex>
#include <cstddef>
#include <utility>
#include <vector>

namespace esp32_doa {

using Complex = std::complex<double>;

struct Config {
    int sample_rate = 48000;
    int channels = 8;
    int nfft = 512;
    int hop = 256;
    double sound_speed_m_s = 343.0;
    double radius_m = 0.056;
    double min_frequency_hz = 500.0;
    double max_frequency_hz = 4000.0;
    int grid_points = 360;
    std::vector<double> microphone_azimuths_deg = {
        157.5, 202.5, 247.5, 292.5, 337.5, 22.5, 67.5, 112.5};
};

struct Stft {
    int sample_count = 0;
    int snapshots = 0;
    int bins = 0;
    int channels = 0;
    // Layout is [snapshot][frequency bin][channel].
    std::vector<Complex> spectra;
    Complex &at(int snapshot, int frequency, int channel);
    const Complex &at(int snapshot, int frequency, int channel) const;
};

struct DoaResult {
    double angle_degrees = 0.0;
    double peak_sharpness = 0.0;
    std::vector<double> spectrum;
};

struct DasResult {
    std::vector<double> audio;
};

Stft analyze_stft(const std::vector<double> &audio, int sample_count,
                 const Config &config);
DoaResult srp_phat(const Stft &stft, const Config &config);
DasResult das(const Stft &stft, const Config &config, double angle_degrees);

// Convenience functions for callers that already have samples in row-major
// samples x channels form.
DoaResult estimate(const std::vector<double> &audio, int sample_count,
                   const Config &config);
DasResult beamform(const std::vector<double> &audio, int sample_count,
                    const Config &config, double angle_degrees);

}  // namespace esp32_doa
