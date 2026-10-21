#pragma once
#include <array>
#include <complex>
#include <cstdint>
#include <vector>

namespace array_dsp {
using C = std::complex<float>;
constexpr int channels = 8, nfft = 256, hop = nfft / 2;
constexpr int sample_rate = 24000, bins = nfft / 2 + 1;
constexpr int resampler_taps = 48;
// Same rounded, exclusive upper frequency-bin convention as PC reference.
constexpr int first_bin = 5, last_bin = 43, band_bins = last_bin - first_bin;
constexpr int directions = 360;
using Matrix = std::array<C, channels * channels>;
using ChannelGains = std::array<float, channels>;
inline constexpr ChannelGains unity_channel_gains{1,1,1,1,1,1,1,1};
struct Statistics {
    std::array<Matrix, band_bins> phat{};
    float power = 0;
};
struct Bearing { float degrees = 0; float sharpness = 0; bool valid = false; };

// Streaming frequency-domain DAS: w = steering / channels, fixed scratch,
// SRP-PHAT statistics and continuous normalized OLA across hop boundaries.
class Beamformer {
public:
    explicit Beamformer(float radius_m, ChannelGains gains = unity_channel_gains);
    void process(const int16_t *interleaved, float angle, float *output);
    const Statistics &statistics() const { return stats_; }
private:
    void fft(std::array<C, nfft> &x, bool inverse) const;
    std::array<float, channels> x_{}, y_{};
    ChannelGains input_gains_;
    std::array<float, nfft> window_{};
    std::array<C, nfft / 2> roots_{};
    std::array<float, hop * channels> previous_{};
    std::array<float, hop> overlap_{};
    std::array<std::array<C, channels>, bins> spectra_{}, weights_{};
    Statistics stats_{};
    unsigned frames_ = 0;
    float steering_angle_ = 0;
    bool weights_valid_ = false;
};
class Locator {
public:
    explicit Locator(float radius_m);
    Bearing estimate(const Statistics &stats);
private:
    std::vector<C> steering_;
};

// Stateful rational 2/3 resampler: 24 kHz -> 16 kHz. 48-tap windowed sinc,
// unity DC gain and 7.2-kHz cutoff.
class Resampler {
public:
    Resampler();
    // One input sample gives 0 or 1 output; phase persists across all blocks.
    bool push(float input, float &output);
private:
    std::array<float, resampler_taps> history_{};
    std::array<std::array<float, resampler_taps>, 2> taps_{};
    unsigned pos_ = 0, phase_ = 0;
};
} // namespace array_dsp
