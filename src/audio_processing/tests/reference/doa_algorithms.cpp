#include "doa_algorithms.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <utility>

namespace esp32_doa {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTolerance = 1e-14;

std::size_t index(const Stft &s, int snapshot, int frequency, int channel) {
    return (static_cast<std::size_t>(snapshot) * s.bins + frequency) *
           s.channels + channel;
}

void fft(std::vector<Complex> &x, bool inverse) {
    const std::size_t n = x.size();
    if (n == 0 || (n & (n - 1)) != 0) {
        throw std::invalid_argument("nfft must be a power of two");
    }
    for (std::size_t i = 1, j = 0; i < n; ++i) {
        std::size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }
    for (std::size_t len = 2; len <= n; len <<= 1) {
        const double sign = inverse ? 1.0 : -1.0;
        const Complex root = std::polar(1.0, sign * 2.0 * kPi / len);
        for (std::size_t start = 0; start < n; start += len) {
            Complex w(1.0, 0.0);
            const std::size_t half = len >> 1;
            for (std::size_t j = 0; j < half; ++j) {
                const Complex u = x[start + j];
                const Complex v = x[start + j + half] * w;
                x[start + j] = u + v;
                x[start + j + half] = u - v;
                w *= root;
            }
        }
    }
    if (inverse) {
        for (Complex &v : x) v /= static_cast<double>(n);
    }
}

double hann(int n, int length) {
    if (length <= 1) return 1.0;
    return 0.5 - 0.5 * std::cos(2.0 * kPi * n / (length - 1));
}

std::vector<int> frequency_bins(const Config &c) {
    const int first = static_cast<int>(std::floor(
        c.min_frequency_hz * c.nfft / c.sample_rate + 0.5));
    const int last = static_cast<int>(std::floor(
        c.max_frequency_hz * c.nfft / c.sample_rate + 0.5));
    std::vector<int> bins;
    for (int k = first; k < last && k <= c.nfft / 2; ++k)
        if (k >= 0) bins.push_back(k);
    return bins;
}

void validate(const Config &c) {
    if (c.sample_rate <= 0 || c.channels <= 0 || c.nfft <= 0 ||
        (c.nfft & (c.nfft - 1)) != 0 || c.hop <= 0 || c.hop > c.nfft)
        throw std::invalid_argument("invalid STFT configuration");
    if (static_cast<int>(c.microphone_azimuths_deg.size()) != c.channels)
        throw std::invalid_argument("one microphone azimuth is required per channel");
    if (!(c.min_frequency_hz >= 0.0 && c.min_frequency_hz < c.max_frequency_hz &&
          c.max_frequency_hz < c.sample_rate / 2.0))
        throw std::invalid_argument("invalid frequency range");
}

std::vector<std::pair<double, double>> microphone_positions(const Config &c) {
    std::vector<std::pair<double, double>> p(c.channels);
    for (int m = 0; m < c.channels; ++m) {
        const double a = c.microphone_azimuths_deg[m] * kPi / 180.0;
        p[m] = {c.radius_m * std::cos(a), c.radius_m * std::sin(a)};
    }
    return p;
}

double wrap_degrees(double angle) {
    angle = std::fmod(angle, 360.0);
    return angle < 0.0 ? angle + 360.0 : angle;
}

std::vector<Complex> mode_vector(const Config &c, int frequency,
                                 double angle_degrees) {
    const auto positions = microphone_positions(c);
    const double a = angle_degrees * kPi / 180.0;
    const double dx = std::cos(a), dy = std::sin(a);
    const double omega = 2.0 * kPi * c.sample_rate * frequency / c.nfft;
    std::vector<Complex> mode(c.channels);
    for (int m = 0; m < c.channels; ++m) {
        const double tau = (dx * positions[m].first + dy * positions[m].second) /
                           c.sound_speed_m_s;
        mode[m] = std::polar(1.0, omega * tau);
    }
    return mode;
}

double spectrum_sharpness(const std::vector<double> &spectrum) {
    if (spectrum.empty()) return 0.0;
    std::vector<double> sorted = spectrum;
    std::sort(sorted.begin(), sorted.end());
    const double peak = sorted.back();
    const double floor = sorted[sorted.size() / 2];
    const double spread = peak - sorted.front();
    return std::clamp((peak - floor) / (spread + 1e-12), 0.0, 1.0);
}

DoaResult finish_spectrum(std::vector<double> spectrum) {
    auto peak = std::max_element(spectrum.begin(), spectrum.end());
    const int index = static_cast<int>(peak - spectrum.begin());
    DoaResult result;
    result.angle_degrees = wrap_degrees(360.0 * index / spectrum.size());
    result.peak_sharpness = spectrum_sharpness(spectrum);
    result.spectrum = std::move(spectrum);
    return result;
}

}  // namespace

Complex &Stft::at(int snapshot, int frequency, int channel) {
    return spectra[index(*this, snapshot, frequency, channel)];
}
const Complex &Stft::at(int snapshot, int frequency, int channel) const {
    return spectra[index(*this, snapshot, frequency, channel)];
}

Stft analyze_stft(const std::vector<double> &audio, int sample_count,
                 const Config &config) {
    validate(config);
    if (sample_count < config.nfft ||
        static_cast<std::size_t>(sample_count) * config.channels > audio.size())
        throw std::invalid_argument("audio does not contain enough samples");
    Stft result;
    result.sample_count = sample_count;
    result.snapshots = (sample_count - config.nfft) / config.hop + 1;
    result.bins = config.nfft / 2 + 1;
    result.channels = config.channels;
    result.spectra.assign(static_cast<std::size_t>(result.snapshots) * result.bins *
                              result.channels,
                          Complex(0.0));
    std::vector<double> means(config.channels, 0.0);
    for (int n = 0; n < sample_count; ++n)
        for (int m = 0; m < config.channels; ++m)
            means[m] += audio[static_cast<std::size_t>(n) * config.channels + m] /
                        sample_count;
    for (int s = 0; s < result.snapshots; ++s) {
        const int start = s * config.hop;
        for (int m = 0; m < config.channels; ++m) {
            std::vector<Complex> frame(config.nfft);
            for (int n = 0; n < config.nfft; ++n)
                frame[n] = (audio[static_cast<std::size_t>(start + n) * config.channels + m] -
                            means[m]) * hann(n, config.nfft);
            fft(frame, false);
            for (int k = 0; k < result.bins; ++k) result.at(s, k, m) = frame[k];
        }
    }
    return result;
}

DoaResult srp_phat(const Stft &stft, const Config &config) {
    validate(config);
    const std::vector<int> bins = frequency_bins(config);
    const int pairs = config.channels * (config.channels - 1) / 2;
    std::vector<double> response(config.grid_points, 0.0);
    const auto positions = microphone_positions(config);
    for (int g = 0; g < config.grid_points; ++g) {
        const double angle = 360.0 * g / config.grid_points * kPi / 180.0;
        const double dx = std::cos(angle), dy = std::sin(angle);
        double score = 0.0;
        for (int k : bins) {
            std::vector<std::vector<Complex>> cc(config.channels,
                                                  std::vector<Complex>(config.channels));
            for (int i = 0; i < config.channels; ++i)
                for (int j = 0; j < config.channels; ++j)
                    for (int s = 0; s < stft.snapshots; ++s) {
                        Complex xi = stft.at(s, k, i), xj = stft.at(s, k, j);
                        const double ai = std::max(std::abs(xi), kTolerance);
                        const double aj = std::max(std::abs(xj), kTolerance);
                        cc[i][j] += (xi / ai) * std::conj(xj / aj);
                    }
            const double omega = 2.0 * kPi * config.sample_rate * k / config.nfft;
            for (int i = 0; i < config.channels; ++i) {
                const double ti = (dx * positions[i].first + dy * positions[i].second) /
                                  config.sound_speed_m_s;
                for (int j = i + 1; j < config.channels; ++j) {
                    const double tj = (dx * positions[j].first + dy * positions[j].second) /
                                      config.sound_speed_m_s;
                    const Complex mode = std::polar(1.0, omega * (tj - ti));
                    score += 2.0 * std::real(cc[i][j] * mode);
                }
            }
        }
        const double dc = static_cast<double>(stft.snapshots * config.channels * bins.size());
        response[g] = (score + dc) /
                      (stft.snapshots * bins.size() * static_cast<double>(pairs));
    }
    return finish_spectrum(std::move(response));
}

DasResult das(const Stft &stft, const Config &config, double angle_degrees) {
    validate(config);
    const int n = config.channels;
    std::vector<Complex> weights(static_cast<std::size_t>(stft.bins) * n);
    for (int k = 0; k < stft.bins; ++k) {
        const auto steering = mode_vector(config, k, angle_degrees);
        for (int m = 0; m < n; ++m) weights[k * n + m] = steering[m] / static_cast<double>(n);
    }
    std::vector<double> output(stft.sample_count, 0.0), normalization(stft.sample_count, 0.0);
    for (int s = 0; s < stft.snapshots; ++s) {
        std::vector<Complex> spectrum(config.nfft, Complex(0.0));
        for (int k = 0; k < stft.bins; ++k) {
            Complex value(0.0);
            for (int m = 0; m < n; ++m) value += std::conj(weights[k * n + m]) * stft.at(s, k, m);
            spectrum[k] = value;
            if (k > 0 && k < config.nfft / 2) spectrum[config.nfft - k] = std::conj(value);
        }
        fft(spectrum, true);
        const int start = s * config.hop;
        for (int i = 0; i < config.nfft; ++i) {
            const double w = hann(i, config.nfft);
            output[start + i] += spectrum[i].real() * w;
            normalization[start + i] += w * w;
        }
    }
    for (int i = 0; i < stft.sample_count; ++i)
        output[i] = normalization[i] > 1e-6 ? output[i] / normalization[i] : 0.0;
    return {std::move(output)};
}

DoaResult estimate(const std::vector<double> &audio, int sample_count,
                   const Config &config) {
    return srp_phat(analyze_stft(audio, sample_count, config), config);
}

DasResult beamform(const std::vector<double> &audio, int sample_count,
                   const Config &config, double angle_degrees) {
    return das(analyze_stft(audio, sample_count, config), config, angle_degrees);
}

} // namespace esp32_doa
