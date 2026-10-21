#include "array_dsp.hpp"
#include <algorithm>
#include <cmath>

namespace array_dsp {
namespace {
constexpr float pi = 3.14159265358979323846f;
constexpr float azimuths[channels] = {157.5f,202.5f,247.5f,292.5f,337.5f,22.5f,67.5f,112.5f};
using Vector = std::array<C, channels>;
void positions(float radius, std::array<float, channels> &x,
               std::array<float, channels> &y) {
    for (int m = 0; m < channels; ++m) {
        x[m] = radius * std::cos(azimuths[m] * pi / 180);
        y[m] = radius * std::sin(azimuths[m] * pi / 180);
    }
}
Vector mode(const std::array<float, channels> &x, const std::array<float, channels> &y,
            int k, float angle) {
    const float dx = std::cos(angle * pi / 180), dy = std::sin(angle * pi / 180);
    const float omega = 2 * pi * sample_rate * k / nfft;
    Vector result;
    for (int m = 0; m < channels; ++m)
        result[m] = std::polar(1.0f, omega * (dx * x[m] + dy * y[m]) / 343.0f);
    return result;
}
} // namespace

Beamformer::Beamformer(float radius_m, ChannelGains gains) : input_gains_(gains) {
    positions(radius_m, x_, y_);
    for (int i = 0; i < nfft; ++i) window_[i] = 0.5f - 0.5f * std::cos(2 * pi * i / (nfft - 1));
    for (int i = 0; i < nfft / 2; ++i) roots_[i] = std::polar(1.0f, -2 * pi * i / nfft);
}
void Beamformer::fft(std::array<C, nfft> &x, bool inverse) const {
    for (int i = 1, j = 0; i < nfft; ++i) {
        int bit = nfft >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(x[i], x[j]);
    }
    for (int len = 2; len <= nfft; len <<= 1) {
        for (int start = 0; start < nfft; start += len)
            for (int j = 0; j < len / 2; ++j) {
                C root = roots_[j * nfft / len];
                if (inverse) root = std::conj(root);
                C u = x[start + j], v = x[start + j + len / 2] * root;
                x[start + j] = u + v;
                x[start + j + len / 2] = u - v;
            }
    }
    if (inverse) for (C &value : x) value /= static_cast<float>(nfft);
}
void Beamformer::process(const int16_t *input, float angle, float *output) {
    std::array<C, nfft> frame;
    float power = 0;
    for (int m = 0; m < channels; ++m) {
        // Apply the common/individual gains in float, before DC removal and
        // STFT. Keep calibrated history too; never clip back to 16-bit PCM.
        const float input_scale = input_gains_[m] / 32768.0f;
        float mean = 0;
        for (int i = 0; i < hop; ++i) mean += previous_[i * channels + m] + input[i * channels + m] * input_scale;
        mean /= nfft;
        for (int i = 0; i < hop; ++i) {
            float sample = input[i * channels + m] * input_scale;
            frame[i] = (previous_[i * channels + m] - mean) * window_[i];
            frame[i + hop] = (sample - mean) * window_[i + hop];
            previous_[i * channels + m] = sample;
            power += sample * sample / (hop * channels);
        }
        fft(frame, false);
        for (int k = 0; k < bins; ++k) spectra_[k][m] = frame[k];
    }
    // ~0.17-second PHAT memory; startup uses an unbiased running mean.
    float alpha = frames_ < 31 ? 1.0f / (frames_ + 1) : 1.0f / 32;
    stats_.power += alpha * (power - stats_.power);
    for (int f = 0; f < band_bins; ++f) {
        const auto &x = spectra_[f + first_bin];
        Vector normalized;
        for (int i = 0; i < channels; ++i) normalized[i] = x[i] / std::max(std::abs(x[i]), 1e-12f);
        for (int i = 0; i < channels; ++i)
            for (int j = i; j < channels; ++j) {
                int index = i * channels + j, mirror = j * channels + i;
                C &phat = stats_.phat[f][index];
                phat += alpha * (normalized[i] * std::conj(normalized[j]) - phat);
                stats_.phat[f][mirror] = std::conj(phat);
            }
    }
    if (frames_ < 31) ++frames_;
    // Pyroomacoustics rake_delay_and_sum_weights with one far-field source,
    // no distance attenuation: w_m(f) = a_m(f) / M. Use array-center phase;
    // the existing one-hop STFT/OLA latency provides the causal audio delay.
    // Refresh only when DOA changes, rather than solving matrices every hop.
    if (!std::isfinite(angle)) angle = 0;
    if (!weights_valid_ || angle != steering_angle_) {
        // Smooth tracking changes the angle every hop. Generate bins by
        // complex recurrence, requiring only one sin/cos pair per microphone.
        const float dx = std::cos(angle * pi / 180), dy = std::sin(angle * pi / 180);
        const float omega = 2 * pi * sample_rate / nfft;
        for (int m = 0; m < channels; ++m) {
            const C step = std::polar(1.0f, omega * (dx * x_[m] + dy * y_[m]) / 343.0f);
            C weight(1.0f / channels, 0);
            for (int k = 0; k < bins; ++k) {
                weights_[k][m] = weight;
                weight *= step;
            }
        }
        steering_angle_ = angle;
        weights_valid_ = true;
    }
    frame.fill(0);
    for (int k = 0; k < bins; ++k) {
        for (int m = 0; m < channels; ++m) frame[k] += std::conj(weights_[k][m]) * spectra_[k][m];
        if (k == 0 || k == nfft / 2) frame[k] = C(frame[k].real(), 0);
        else frame[nfft - k] = std::conj(frame[k]);
    }
    fft(frame, true);
    for (int i = 0; i < hop; ++i) {
        float norm = window_[i] * window_[i] + window_[i + hop] * window_[i + hop];
        output[i] = (overlap_[i] + frame[i].real() * window_[i]) / norm;
        overlap_[i] = frame[i + hop].real() * window_[i + hop];
    }
}

Locator::Locator(float radius_m)
    : steering_(band_bins * directions * channels) {
    std::array<float, channels> x, y;
    positions(radius_m, x, y);
    for (int f = 0; f < band_bins; ++f)
        for (int g = 0; g < directions; ++g) {
            Vector a = mode(x, y, f + first_bin, float(g));
            std::copy(a.begin(), a.end(), steering_.begin() + (f * directions + g) * channels);
        }
}
Bearing Locator::estimate(const Statistics &stats) {
    // Do not turn a flat/silent spectrum into a fictitious zero-degree source.
    if (stats.power < 1e-8f) return {};
    std::array<float, directions> spectrum{};
    for (int f = 0; f < band_bins; ++f) {
        for (int g = 0; g < directions; ++g) {
            const C *a = &steering_[(f * directions + g) * channels];
            for (int i = 0; i < channels; ++i)
                for (int j = i + 1; j < channels; ++j)
                    spectrum[g] += 2 * (stats.phat[f][i * channels + j] * std::conj(a[i]) * a[j]).real();
        }
    }
    int peak = int(std::max_element(spectrum.begin(), spectrum.end()) - spectrum.begin());
    float maximum = spectrum[peak], minimum = *std::min_element(spectrum.begin(), spectrum.end());
    if (maximum - minimum < 1e-6f) return {};
    std::nth_element(spectrum.begin(), spectrum.begin() + directions / 2, spectrum.end());
    float sharpness = (maximum - spectrum[directions / 2]) / (maximum - minimum);
    return {float(peak), sharpness, true};
}

Resampler::Resampler() {
    for (int phase = 0; phase < 2; ++phase) {
        float sum = 0;
        for (int i = 0; i < resampler_taps; ++i) {
            float t = i - 23.5f - phase * 0.5f;
            float sinc = std::abs(t) < 1e-6f ? 0.6f : std::sin(pi * 0.6f * t) / (pi * t);
            float window = 0.54f - 0.46f * std::cos(2 * pi * i / (resampler_taps - 1));
            taps_[phase][i] = sinc * window;
            sum += taps_[phase][i];
        }
        for (float &tap : taps_[phase]) tap /= sum;
    }
}
bool Resampler::push(float input, float &output) {
    history_[pos_] = input;
    pos_ = (pos_ + 1) % history_.size();
    phase_ += 2;
    if (phase_ < 3) return false;
    phase_ -= 3;
    output = 0;
    for (unsigned i = 0; i < history_.size(); ++i)
        output += history_[(pos_ + history_.size() - 1 - i) % history_.size()] * taps_[phase_][i];
    return true;
}
} // namespace array_dsp
