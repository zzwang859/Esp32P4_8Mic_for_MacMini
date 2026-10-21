#include "array_dsp.hpp"
#include "input_gain_calibration.hpp"
#include "doa_algorithms.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>
using namespace array_dsp;
constexpr double pi = 3.14159265358979323846;
void require(bool condition, const char *message) {
    if (!condition) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}
double distance(double a, double b) { double d = std::abs(a - b); return std::min(d, 360 - d); }
void plane_wave(int16_t *input, int start, double angle, bool broadband) {
    constexpr double positions[] = {157.5,202.5,247.5,292.5,337.5,22.5,67.5,112.5};
    constexpr double frequencies[] = {730,990,1270,1630,1970,2390,2830,3310,3790};
    for (int n = 0; n < hop; ++n)
        for (int m = 0; m < channels; ++m) {
            double tau = 0.056 * std::cos((positions[m] - angle) * pi / 180) / 343;
            double t = double(start + n) / sample_rate + tau, value = 0;
            if (broadband) {
                for (int f = 0; f < 9; ++f) value += 0.025 * std::sin(2 * pi * frequencies[f] * t + f * 0.71);
            } else value = 0.2 * std::sin(2 * pi * 1500 * t);
            input[n * channels + m] = int16_t(std::lround(value * 32767));
        }
}
void verify_das_against_pc(ChannelGains gains, float angle) {
    std::vector<int16_t> input(hop * channels);
    std::vector<float> output(hop);
    auto beamformer = std::make_unique<Beamformer>(0.056f, gains);
    std::vector<double> pc_input(100 * hop * channels), actual(100 * hop);
    for (int block = 0; block < 100; ++block) {
        plane_wave(input.data(), block * hop, angle, false);
        for (int i = 0; i < hop * channels; ++i)
            pc_input[block * hop * channels + i] = input[i] / 32768.0 * gains[i % channels];
        beamformer->process(input.data(), angle, output.data());
        for (int n = 0; n < hop; ++n) actual[block * hop + n] = output[n];
    }
    esp32_doa::Config pc_config;
    pc_config.sample_rate = sample_rate; pc_config.nfft = nfft; pc_config.hop = hop;
    auto expected_audio = esp32_doa::beamform(pc_input, 100 * hop, pc_config, angle).audio;
    double error = 0, signal = 0;
    for (int block = 50; block < 100; ++block) {
        for (int n = 0; n < hop; ++n) {
            double expected = expected_audio[block * hop + n - hop];
            error += std::pow(actual[block * hop + n] - expected, 2);
            signal += expected * expected;
        }
    }
    std::cout << "DAS continuous OLA vs PC (angle=" << angle << ", gain[0]=" << gains[0]
              << ") relative RMS error=" << std::sqrt(error / signal) << '\n';
    require(std::sqrt(error / signal) < 0.005, "DAS port gain/OLA boundary mismatch vs PC");
}

void verify_das_normalization_and_steering() {
    std::vector<int16_t> input(hop * channels);
    std::vector<float> output(hop);
    // Co-located channels have no phase shift: their sum must be divided by 8.
    for (int active : {1, channels}) {
        auto das = std::make_unique<Beamformer>(0.0f);
        for (int block = 0; block < 12; ++block) {
            for (int i = 0; i < hop; ++i) {
                int16_t value = int16_t(std::lround(12000 * std::sin(2 * pi * 1500 * (block * hop + i) / sample_rate)));
                for (int m = 0; m < channels; ++m) input[i * channels + m] = m < active ? value : 0;
            }
            das->process(input.data(), 73, output.data());
            if (block < 2) continue;
            for (int i = 0; i < hop; ++i) {
                int16_t value = int16_t(std::lround(12000 * std::sin(2 * pi * 1500 * ((block - 1) * hop + i) / sample_rate)));
                require(std::abs(output[i] - value / 32768.0f * active / channels) < 2e-6f,
                        "DAS 1/M normalization or one-hop OLA delay mismatch");
            }
        }
    }
    // A changed bearing must refresh cached weights on the very next hop.
    auto switching = std::make_unique<Beamformer>(0.056f);
    auto aimed = std::make_unique<Beamformer>(0.056f);
    auto away = std::make_unique<Beamformer>(0.056f);
    std::vector<float> expected(hop), rejected(hop);
    double aligned_power = 0, rejected_power = 0;
    for (int block = 0; block < 40; ++block) {
        plane_wave(input.data(), block * hop, 73, false);
        switching->process(input.data(), block < 10 ? 253 : 73, output.data());
        aimed->process(input.data(), 73, expected.data());
        away->process(input.data(), 253, rejected.data());
        if (block < 12) continue;
        for (int i = 0; i < hop; ++i) {
            require(std::abs(output[i] - expected[i]) < 1e-6f, "DAS stale steering after direction change");
            aligned_power += expected[i] * expected[i];
            rejected_power += rejected[i] * rejected[i];
        }
    }
    require(aligned_power > 4 * rejected_power, "DAS steering sign or off-axis rejection mismatch");
    std::cout << "DAS normalization, delay, steering update and directional response passed\n";
}

int main() {
    auto locator = std::make_unique<Locator>(0.056f);
    std::vector<int16_t> input(hop * channels);
    std::vector<float> output(hop);
    for (double angle : {0.0,22.0,73.0,157.0,202.0,292.0,359.0}) {
        auto beamformer = std::make_unique<Beamformer>(0.056f);
        for (int block = 0; block < 100; ++block) {
            plane_wave(input.data(), block * hop, angle, true);
            beamformer->process(input.data(), float(angle), output.data());
            for (float sample : output) require(std::isfinite(sample), "DAS must stay finite");
        }
        Bearing found = locator->estimate(beamformer->statistics());
        std::cout << "SRP-PHAT expected=" << angle << " found=" << found.degrees << '\n';
        require(found.valid && distance(found.degrees, angle) <= 2, "DOA bearing/sign/wrap mismatch");
    }
    for (float angle : {0.0f, 73.0f, 202.0f, 359.0f}) {
        verify_das_against_pc(unity_channel_gains, angle);
        verify_das_against_pc(calibrated_channel_gains, angle);
    }
    verify_das_normalization_and_steering();

    // Check gain index/order, calibrated previous-hop history and values >1.0
    // and ensure positive gain calibration preserves PHAT phase at 1500 Hz.
    auto raw = std::make_unique<Beamformer>(0.056f);
    auto calibrated = std::make_unique<Beamformer>(0.056f, calibrated_channel_gains);
    for (int block = 0; block < 100; ++block) {
        plane_wave(input.data(), block * hop, 73, false);
        raw->process(input.data(), 73, output.data());
        calibrated->process(input.data(), 73, output.data());
    }
    constexpr int frequency = 16 - first_bin;
    for (int i = 0; i < channels; ++i) {
        for (int j = 0; j < channels; ++j) {
            C expected = raw->statistics().phat[frequency][i * channels + j];
            C actual = calibrated->statistics().phat[frequency][i * channels + j];
            require(std::abs(actual - expected) < 2e-5f,
                    "positive calibration must preserve PHAT phase without clipping");
        }
    }
    // Model microphone imbalance inverse to the measured calibration.
    calibrated = std::make_unique<Beamformer>(0.056f, calibrated_channel_gains);
    for (int block = 0; block < 100; ++block) {
        plane_wave(input.data(), block * hop, 73, true);
        for (int i = 0; i < hop * channels; ++i)
            input[i] = int16_t(std::lround(input[i] / calibrated_channel_gains[i % channels]));
        calibrated->process(input.data(), 73, output.data());
    }
    Bearing found = locator->estimate(calibrated->statistics());
    require(found.valid && distance(found.degrees, 73) <= 2,
            "calibrated imbalanced microphones must recover DOA");
    std::cout << "Pre-STFT JSON gain calibration checks passed\n";
    Statistics silence{};
    require(!locator->estimate(silence).valid, "silence must be invalid");
    Resampler resampler;
    int emitted = 0;
    float sample = 0;
    for (int n = 0; n < 24000; ++n) {
        if (resampler.push(0.2f, sample)) {
            ++emitted;
            if (n > 200) require(std::abs(sample - 0.2f) < 1e-5, "resampler DC gain");
        }
    }
    require(emitted == 16000, "resampler must preserve exact 2/3 sample count");
    Resampler stopband;
    double sum = 0; int count = 0;
    for (int n = 0; n < 24000; ++n) {
        if (stopband.push(float(std::sin(2 * pi * 11000 * n / sample_rate)), sample) && n > 200) {
            sum += sample * sample; ++count;
        }
    }
    std::cout << "resampler 11-kHz alias RMS=" << std::sqrt(sum / count) << '\n';
    require(std::sqrt(sum / count) < 0.01, "resampler anti-alias filtering");
    std::cout << "All array DSP checks passed\n";
}
