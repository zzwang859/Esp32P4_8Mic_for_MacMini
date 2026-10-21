#include "doa_tracking.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

void expect(float actual, float wanted) {
    if (!std::isfinite(actual) || std::abs(std::remainder(actual - wanted, 360.0f)) > 0.003f) {
        std::cerr << "DOA tracking: expected " << wanted << ", got " << actual << '\n';
        std::exit(1);
    }
}

int main() {
    using array_dsp::DoaTracking;
    DoaTracking forward(30, 0.01f);
    expect(forward.update(90, false, 1000000), 0); // Unknown/silence: frozen.
    expect(forward.update(90, true, 2000000), 0); // No startup jump.
    for (int i = 1; i <= 100; ++i)
        expect(forward.update(90, true, 2000000 + i * 10000), i * 0.3f);
    expect(forward.update(90, false, 4000000), 30);
    expect(forward.update(180, false, 9000000), 30);
    expect(forward.update(90, true, 10000000), 30); // No silence catch-up.
    expect(forward.update(90, true, 10010000), 30.3f);
    expect(forward.update(90, true, 11010000), 30.6f); // Stall: at most one hop.
    expect(forward.update(90, true, 11017000), 30.81f); // Short interval: 30 deg/s.
    expect(forward.update(std::numeric_limits<float>::quiet_NaN(), true, 11027000), 30.81f);
    expect(forward.update(90, true, 11026000), 30.81f); // Bad clock: no motion.

    DoaTracking wrap_up(30, 0.01f, 350), wrap_down(30, 0.01f, 10);
    wrap_up.update(10, true, 0);
    wrap_down.update(350, true, 0);
    for (int i = 1; i <= 100; ++i) {
        expect(wrap_up.update(10, true, i * 10000), std::min(350 + i * 0.3f, 370.0f));
        expect(wrap_down.update(350, true, i * 10000), std::max(10 - i * 0.3f, -10.0f));
    }
    // New targets can reverse the shortest-path direction without a jump.
    expect(wrap_up.update(350, true, 1010000), 9.7f);
    expect(wrap_down.update(10, true, 1010000), 350.3f);

    DoaTracking faster(60, 0.01f), opposite(30, 0.01f);
    faster.update(90, true, 0);
    expect(faster.update(90, true, 10000), 0.6f);
    opposite.update(180, true, 0);
    expect(opposite.update(180, true, 10000), 0.3f); // Equal paths: increasing angle.
    std::cout << "PASS: VAD freeze/resume, rate, stalls, shortest paths, wrap and retarget\n";
}
