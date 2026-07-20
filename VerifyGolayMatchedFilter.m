% VerifyGolayMatchedFilter.m
% Independent Octave verification of the Vanguard X operational Golay-64
% construction, matched filters, complementary sum and Doppler sensitivity.
% No Octave packages are required.

clear;
close all;
clc;

golay_length = 64;
sample_rate_hz = 40.0e6;
chip_rate_hz = 20.0e6;
samples_per_chip = sample_rate_hz / chip_rate_hz;
physical_pri_sec = 250.0e-6;
rf_frequency_hz = 9.4e9;
radial_velocity_mps = -9.0;
speed_of_light_mps = 299792458.0;

if samples_per_chip != round(samples_per_chip)
    error('Chip rate must give an integer number of samples per chip');
endif

% This is the same recursive construction used by WaveformLibrary.py:
% A(n+1) = [A(n), B(n)]
% B(n+1) = [A(n), -B(n)]
golay_a = 1;
golay_b = 1;
while length(golay_a) < golay_length
    previous_a = golay_a;
    previous_b = golay_b;
    golay_a = [previous_a, previous_b];
    golay_b = [previous_a, -previous_b];
endwhile

if length(golay_a) != golay_length
    error('Golay length must be a power of two');
endif

% Aperiodic chip autocorrelations. conv(x, conj(fliplr(x))) is the exact
% matched-filter operation and avoids requiring Octave's signal package.
chip_ac_a = conv(golay_a, conj(fliplr(golay_a)));
chip_ac_b = conv(golay_b, conj(fliplr(golay_b)));
chip_complementary = chip_ac_a + chip_ac_b;
chip_lags = -(golay_length - 1):(golay_length - 1);
chip_peak_index = golay_length;
chip_sidelobe_mask = chip_lags != 0;

% Expand each chip to the exact 40 MS/s samples transmitted by Vanguard X.
sampled_a = kron(golay_a, ones(1, samples_per_chip));
sampled_b = kron(golay_b, ones(1, samples_per_chip));
sampled_length = length(sampled_a);

sample_ac_a = conv(sampled_a, conj(fliplr(sampled_a)));
sample_ac_b = conv(sampled_b, conj(fliplr(sampled_b)));
sample_complementary = sample_ac_a + sample_ac_b;
sample_lags = -(sampled_length - 1):(sampled_length - 1);
sample_peak_index = sampled_length;

% With two rectangular samples per chip, +/-1 sample is the triangular
% mainlobe, not a sidelobe. True sampled sidelobes begin outside that region.
sample_mainlobe_mask = abs(sample_lags) <= (samples_per_chip - 1);
sample_sidelobe_mask = !sample_mainlobe_mask;
stationary_reference = abs(sample_complementary(sample_peak_index));
stationary_db = 20.0 * log10(max(
    abs(sample_complementary) / stationary_reference,
    1.0e-15
));

% Reproduce the -9 m/s return shown on the Vanguard X display. The B pulse
% begins one physical PRI after A, so it has an A-to-B Doppler phase rotation.
wavelength_m = speed_of_light_mps / rf_frequency_hz;
doppler_hz = 2.0 * radial_velocity_mps / wavelength_m;
sample_number = 0:(sampled_length - 1);
received_a = sampled_a .* exp(
    1i * 2.0 * pi * doppler_hz * sample_number / sample_rate_hz
);
received_b = sampled_b .* exp(
    1i * 2.0 * pi * doppler_hz * (
        physical_pri_sec + sample_number / sample_rate_hz
    )
);

matched_a_moving = conv(received_a, conj(fliplr(sampled_a)));
matched_b_moving = conv(received_b, conj(fliplr(sampled_b)));
moving_uncompensated = matched_a_moving + matched_b_moving;
moving_uncompensated_db = 20.0 * log10(max(
    abs(moving_uncompensated) / stationary_reference,
    1.0e-15
));

% For this known single Doppler hypothesis only, undo the B pulse-start phase.
% This demonstrates the expected result; an operational radar needs a bank of
% Doppler hypotheses rather than using one known target velocity.
b_phase_correction = exp(-1i * 2.0 * pi * doppler_hz * physical_pri_sec);
moving_compensated = (
    matched_a_moving + b_phase_correction * matched_b_moving
);
moving_compensated_db = 20.0 * log10(max(
    abs(moving_compensated) / stationary_reference,
    1.0e-15
));

chip_peak = abs(chip_complementary(chip_peak_index));
chip_max_sidelobe = max(abs(chip_complementary(chip_sidelobe_mask)));
sample_peak = abs(sample_complementary(sample_peak_index));
sample_max_sidelobe = max(abs(sample_complementary(sample_sidelobe_mask)));
moving_peak_db = moving_uncompensated_db(sample_peak_index);
moving_psl_db = max(moving_uncompensated_db(sample_sidelobe_mask));
compensated_psl_db = max(moving_compensated_db(sample_sidelobe_mask));
a_to_b_phase_deg = 360.0 * doppler_hz * physical_pri_sec;

fprintf('Vanguard X Golay-64 independent verification\n');
fprintf('  chip correlation peak:                 %.1f\n', chip_peak);
fprintf('  chip maximum off-peak sidelobe:        %.3g\n', chip_max_sidelobe);
fprintf('  sampled correlation peak:              %.1f\n', sample_peak);
fprintf('  sampled maximum sidelobe outside ML:   %.3g\n', sample_max_sidelobe);
fprintf('  sampled adjacent mainlobe sample:      %.2f dB\n', ...
    stationary_db(sample_peak_index + 1));
fprintf('  velocity / Doppler:                    %.2f m/s / %.3f Hz\n', ...
    radial_velocity_mps, doppler_hz);
fprintf('  A-to-B Doppler phase:                  %.3f deg\n', ...
    a_to_b_phase_deg);
fprintf('  uncompensated moving peak:             %.3f dB\n', ...
    moving_peak_db);
fprintf('  uncompensated moving peak sidelobe:    %.3f dB\n', ...
    moving_psl_db);
fprintf('  known-Doppler compensated sidelobe:    %.3f dB\n', ...
    compensated_psl_db);

range_lag_m = (
    sample_lags / sample_rate_hz * speed_of_light_mps / 2.0
);

figure(1);
clf;

subplot(2, 2, 1);
stairs(0:(golay_length - 1), golay_a, 'b', 'linewidth', 1.2);
hold on;
stairs(0:(golay_length - 1), golay_b, 'r', 'linewidth', 1.2);
grid on;
ylim([-1.4, 1.4]);
xlabel('Chip');
ylabel('Value');
title('Golay-64 A and B');
legend('A', 'B');

subplot(2, 2, 2);
plot(chip_lags, 20.0 * log10(max(
    abs(chip_complementary) / chip_peak,
    1.0e-15
)), 'k', 'linewidth', 1.2);
grid on;
ylim([-100, 5]);
xlabel('Chip lag');
ylabel('Magnitude (dB)');
title('Stationary chip complementary sum');

subplot(2, 2, 3);
plot(range_lag_m, stationary_db, 'g', 'linewidth', 1.2);
grid on;
ylim([-100, 5]);
xlim([-250, 250]);
xlabel('Range lag (m)');
ylabel('Magnitude (dB)');
title('Stationary sampled matched filters');

subplot(2, 2, 4);
plot(range_lag_m, moving_uncompensated_db, 'r', 'linewidth', 1.2);
hold on;
plot(range_lag_m, moving_compensated_db, 'b--', 'linewidth', 1.2);
grid on;
ylim([-60, 5]);
xlim([-500, 500]);
xlabel('Range lag (m)');
ylabel('Magnitude (dB)');
title('-9 m/s Doppler sensitivity');
legend('Uncompensated', 'Known-Doppler compensated');

print('VerifyGolayMatchedFilter.png', '-dpng', '-r150');
fprintf('  plot written to VerifyGolayMatchedFilter.png\n');
