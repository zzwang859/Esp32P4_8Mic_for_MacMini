#include "nscore/nscore.h"
#include "nscore/ns_fft.h"
#include "post_ns_stream.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

void check(bool ok, const char *message) {
    if (!ok) { std::cerr << "FAIL: " << message << '\n'; std::exit(1); }
}

void fft_checks() {
    nscore_fft_instance plan;
    check(nscore_fft_init(&plan, 256) == 0, "FFT plan");
    check(nscore_fft_init(&plan, 128) != 0, "reject unsupported FFT size");
    for (int kind = 0; kind < 5; ++kind) {
        std::array<float, 256> input{}, spectrum{}, roundtrip{};
        for (int i = 0; i < 256; ++i) {
            input[i] = kind == 0 ? float(i == 17) : kind == 1 ? 1.0f
                : kind == 2 ? (i % 2 ? -1.0f : 1.0f)
                : kind == 3 ? float(std::sin(6.283185307179586 * 37 * i / 256))
                : float(std::sin(i * 0.17) + std::cos(i * 1.97));
        }
        nscore_fft(&plan, input.data(), spectrum.data(), 0);
        for (int k = 0; k <= 128; ++k) {
            double real = 0, imag = 0;
            for (int i = 0; i < 256; ++i) {
                const double phase = -6.283185307179586 * k * i / 256;
                real += input[i] * std::cos(phase);
                imag += input[i] * std::sin(phase);
            }
            const float actual = k == 0 ? spectrum[0] : k == 128 ? spectrum[1] : spectrum[2*k];
            check(std::abs(real - actual) < 0.0002, "real FFT packed bins vs independent DFT");
            if (k != 0 && k != 128)
                check(std::abs(imag - spectrum[2*k+1]) < 0.0002, "imaginary FFT packed bins");
        }
        nscore_fft(&plan, spectrum.data(), roundtrip.data(), 1);
        for (int i = 0; i < 256; ++i)
            check(std::abs(input[i] - roundtrip[i]) < 0.000002f, "IFFT scale and roundtrip");
    }
}

double noise_rms(unsigned gain) {
    array_nscore_init(gain);
    std::array<int16_t, 160> input{}, output{};
    uint32_t random = 12345;
    double energy = 0;
    for (int block = 0; block < 800; ++block) {
        for (auto &value : input) {
            random = random * 1664525u + 1013904223u;
            value = int16_t(int(random >> 20) - 2048);
        }
        array_nscore_process(input.data(), output.data());
        if (gain == 100) check(input == output, "100% bypass preserves exact signed PCM");
        if (block >= 600) for (auto value : output) energy += double(value) * value;
    }
    return std::sqrt(energy / (200 * 160));
}

void core_checks() {
    std::array<int16_t, 160> input{}, output{};
    array_nscore_init(25);
    for (int frame = 0; frame < 10; ++frame) {
        array_nscore_process(input.data(), output.data());
        for (auto value : output) check(value == 0, "silence remains silence");
    }
    const double bypass = noise_rms(100), light = noise_rms(50), normal = noise_rms(25), strong = noise_rms(1);
    std::cout << "Stationary noise RMS bypass/50%/25%/1%: " << bypass << "/" << light << "/" << normal << "/" << strong << '\n';
    check(light < bypass * 0.7 && normal < light * 0.8 && strong < normal * 0.8,
          "gain floor controls stationary noise attenuation");
    // Reset must restore the original deterministic state, not old noise/synthesis history.
    for (size_t i = 0; i < input.size(); ++i) input[i] = int16_t(i * 173 - 13000);
    array_nscore_init(25);
    array_nscore_process(input.data(), output.data());
    auto first = output;
    for (int i = 0; i < 20; ++i) array_nscore_process(input.data(), output.data());
    array_nscore_init(25);
    array_nscore_process(input.data(), output.data());
    check(output == first, "reset clears adaptive/OLA state");
    input.fill(0); input[0] = 20000;
    array_nscore_init(25);
    array_nscore_process(input.data(), output.data());
    const auto peak = std::max_element(output.begin(), output.end(),
        [](int16_t a, int16_t b) { return std::abs(int(a)) < std::abs(int(b)); });
    check(peak - output.begin() == 96, "impulse confirms 96-sample nominal OLA delay");
    array_nscore_init(100);
    input.fill(INT16_MIN); array_nscore_process(input.data(), output.data()); check(input == output, "negative full-scale bypass");
    input.fill(INT16_MAX); array_nscore_process(input.data(), output.data()); check(input == output, "positive full-scale bypass");
    // Near-unity filtering must not inherit the original non-COLA window's
    // periodic gain ripple. Compare against a tone delayed by exactly 96 samples.
    array_nscore_init(99);
    double error = 0, reference_energy = 0;
    for (int block = 0; block < 100; ++block) {
        for (int i = 0; i < 160; ++i)
            input[i] = int16_t(std::lround(12000 * std::sin(6.283185307179586 * 731 * (block * 160 + i) / 16000)));
        array_nscore_process(input.data(), output.data());
        if (block < 10) continue;
        for (int i = 0; i < 160; ++i) {
            const double wanted = 12000 * std::sin(6.283185307179586 * 731 * (block * 160 + i - 96) / 16000);
            error += std::pow(output[i] - wanted, 2);
            reference_energy += wanted * wanted;
        }
    }
    check(std::sqrt(error / reference_energy) < 0.02, "OLA unity gain and continuous tone across frames");
}

void framing_checks(bool bypass) {
    using namespace array_dsp;
    PostNsFramer framer(bypass);
    std::vector<int16_t> input(2560), output;
    std::vector<PostNsMetadata> expected, actual;
    for (size_t i = 0; i < input.size(); ++i) input[i] = int16_t(i - 1280);
    auto copy = [](const int16_t *in, int16_t *out) { std::copy_n(in, 160, out); };
    auto emit = [&](const int16_t *out, const PostNsMetadata *meta, size_t count) {
        output.insert(output.end(), out, out + count); actual.insert(actual.end(), meta, meta + count);
    };
    // 512-sample AFE blocks cross both 160-sample and metadata boundaries.
    for (int block = 0; block < 5; ++block) {
        PostNsMetadata meta{int16_t(block * 1000 - 18000), int8_t(block % 2)};
        expected.insert(expected.end(), 512, meta);
        framer.feed(input.data() + block * 512, 512, meta, copy, emit);
    }
    check(output == input, "512-to-160 framing preserves every input sample");
    for (size_t i = 0; i < actual.size(); ++i) {
        const auto wanted = !bypass && i < 96 ? PostNsMetadata{} : expected[i - (bypass ? 0 : 96)];
        check(actual[i].vad == wanted.vad && actual[i].doa == wanted.doa, "metadata follows exact OLA delay");
    }
    framer.feed(input.data(), 159, {9000, 1}, copy, emit);
    framer.reset(); output.clear(); actual.clear();
    framer.feed(input.data() + 160, 160, {0, 0}, copy, emit);
    check(output.size() == 160 && output[0] == input[160], "gap discards partial frame");
    check(actual[0].vad == (bypass ? 0 : -1), "gap clears old metadata history");
}

int main() {
    fft_checks(); core_checks(); framing_checks(false); framing_checks(true);
    std::cout << "PASS: NsCore FFT, suppression, reset, framing and aligned metadata\n";
}
