# NsCore ESP32-P4 port

Source: the supplied `STM32F429_NS_Example/User/NsCore` implementation by
深圳创成微电子 / 王自振. Original copyright notices are retained in the copied
core and window files. Firmware builds use these local port files and do not
depend on the ignored STM32 example directory.

The original 32-kHz example accepts separate low/high bands. This port uses
its 16-kHz low-band noise estimation, speech probability and Wiener filter
with the original mixed Hann/flat-top window: 256-point FFT, 160-sample step.
There is no upsampling or QMF high-band path in this mono pipeline.

Platform changes:

- Removed STM32 headers, hardware FFT branches, timer/profiling writes and
  unused high-band processing. State and scratch use ordinary allocated RAM.
- Replaced CMSIS FFT with a portable float32 radix-2 implementation preserving
  its packed DC/Nyquist/complex format and 1/N inverse normalization.
- Retained the original approximate log/exp functions, estimator constants,
  overdrive=1 and gain-floor control. Used float32 literals and `fabsf` to avoid
  software double arithmetic on ESP32-P4. Reset clears all adaptive/OLA state.
- Normalized the analysis/synthesis overlap gain: the supplied sine window
  does not sum to a constant with the original 160-sample step. This avoids
  periodic volume modulation while preserving the spectral estimator/filter.
- Added exact PCM bypass at 100% minimum gain. No per-frame heap allocation.

`Microphone array processing` →
`Post-AFE NsCore minimum gain (%, lower = stronger; 100 = bypass)` maps the
integer percentage to `denoiseBound = value / 100.0f`. Default 25 reproduces
the original 0.25 floor. Smaller floors can remove more noise and more speech;
these are spectral gain floors, not a guaranteed measured noise attenuation.

`afe_fetch` copies 16-kHz PCM and VAD/DOA metadata to an eight-block PSRAM queue
without waiting. The core1 `post_afe_ns` task (priority 4) reframes arbitrary
AFE fetch lengths into 160 samples, runs NS, and feeds the denoised PCM to KWS
and normal Capture2. DAS/OLA/resampling keeps priority 6, DOA keeps priority 3;
AFE and KWS remain on core0. USB diagnostic left sources remain independent;
KWS always receives the actual microphone stream after NS.

Active NS has a nominal 256−160=96-sample (6-ms) OLA delay and needs up to
10 ms to collect a processing frame, plus actual queue/scheduling waits.
VAD and DOA metadata are delayed by the same 96 samples before Capture2
publication; the initial metadata is unknown VAD / initial 0° DOA. Bypass
has no OLA delay. AFE VAD itself and its DOA motion gate remain before NS.

Queue overflow drops whole incoming blocks and counts lost samples. Sequence
gaps and AFE errors reset NS history, incomplete frames and the KWS stream
in order after older queued audio. The background statistics log reports NS
task CPU, frame average/worst time, dropped samples and resets. Validate CPU
headroom and recognition/voice quality on the board before tuning the floor.

Host validation lives in `../tests/nscore_test.cpp` (independent DFT, noise,
reset, framing and metadata) and `../tests/test_post_ns_pipeline.py` (actual
queue/task code linked to the C algorithm, gain/bypass and USB diagnostic
modes, drops, errors and downstream KWS/Capture2 routing).
