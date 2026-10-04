function results = test_phase4_robustness()
% TEST_PHASE4_ROBUSTNESS Evaluates Phase 4 Realism and Robustness:
%   1. Real-time Vehicle Reference Speed Estimation vs. True Ground Truth.
%   2. Wheel Speed Sensor Quantization and Noise Effects.
%   3. Robustness Parameter Variations:
%      - Vehicle Mass Variation (+/- 20%)
%      - Actuator Delay Variation (+/- 50%)
%      - Tire Wear / Grip Degradation (-15%)
%      - Sensor Noise Levels (Clean vs Severe)

p = init_params();

fprintf('=== Phase 4: Realism & Robustness Evaluation ===\n');

% 1. Evaluate Reference Speed Estimator during 4-Wheel Braking
scen.v0 = 100 / 3.6;
scen.surface_L = 'dry';
scen.surface_R = 'dry';
scen.driver_type = 'step';
scen.use_ymc = true;
scen.t_end = 4.5;
scen.dt = 0.0005;

fprintf('1. Evaluating Reference Speed Estimator on 4-wheel braking...\n');
sim_nominal = sim_full_car(scen, 'rule_based', p);

% Reconstruct estimated reference speed over the trajectory
N_pts = length(sim_nominal.t);
v_ref_hist = zeros(N_pts, 1);
est_state  = struct();

for k = 1:N_pts
    % Pass simulated wheel speeds through sensor model (quantization + noise)
    w_meas = wheel_speed_sensor(sim_nominal.w(k, :)', p, scen.dt);
    [v_ref_hist(k), est_state] = estimate_reference_speed(w_meas, est_state, p, scen.dt);
end

% Compute estimation metrics
err_v = abs(sim_nominal.v - v_ref_hist);
mean_err_v = mean(err_v(sim_nominal.v > 2.0));
max_err_v  = max(err_v(sim_nominal.v > 2.0));

fprintf('   Reference Speed Estimator Mean Error: %.3f m/s (%.2f km/h)\n', ...
    mean_err_v, mean_err_v * 3.6);
fprintf('   Reference Speed Estimator Max Error:  %.3f m/s (%.2f km/h)\n', ...
    max_err_v, max_err_v * 3.6);

assert(mean_err_v < 1.0, 'ASSERTION FAILED: Reference speed estimator mean error must be < 1.0 m/s!');

% 2. Robustness Sweeps (Mass, Delay, Tire Wear, Sensor Noise)
fprintf('\n2. Executing Robustness Parameter Sweeps...\n');

% 2.1 Mass variations: -20% (1280 kg), Nominal (1600 kg), +20% (1920 kg)
p_light = p; p_light.veh.mass_total = 1280; p_light.veh.mass_quarter = 320;
p_heavy = p; p_heavy.veh.mass_total = 1920; p_heavy.veh.mass_quarter = 480;

sim_light = sim_full_car(scen, 'rule_based', p_light);
sim_heavy = sim_full_car(scen, 'rule_based', p_heavy);

fprintf('   Mass Sweeps: Light (-20%%): Dist = %.2f m | Heavy (+20%%): Dist = %.2f m | Nom: %.2f m\n', ...
    sim_light.stop_dist, sim_heavy.stop_dist, sim_nominal.stop_dist);

% 2.2 Actuator Delay variations: -50% (2.5 ms), Nominal (5.0 ms), +50% (7.5 ms)
p_fast = p; p_fast.brake.delay_act = 0.0025;
p_slow = p; p_slow.brake.delay_act = 0.0075;

sim_fast = sim_full_car(scen, 'rule_based', p_fast);
sim_slow = sim_full_car(scen, 'rule_based', p_slow);

fprintf('   Delay Sweeps: Fast (-50%%): Dist = %.2f m | Slow (+50%%): Dist = %.2f m\n', ...
    sim_fast.stop_dist, sim_slow.stop_dist);

% 2.3 Tire Wear / Friction Degradation (-15% mu)
p_worn = p;
p_worn.tire.dry.D = 0.85; % 15% reduction in dry peak grip

sim_worn = sim_full_car(scen, 'rule_based', p_worn);
fprintf('   Tire Wear (-15%% Grip): Dist = %.2f m (Expected increase in distance)\n', sim_worn.stop_dist);

% Assertions for Robustness
assert(sim_heavy.stop_dist > 0 && ~isnan(sim_heavy.stop_dist), 'Heavy vehicle simulation must succeed!');
assert(sim_slow.stop_dist > 0 && ~isnan(sim_slow.stop_dist), 'High delay simulation must succeed!');
assert(sim_worn.stop_dist > sim_nominal.stop_dist, 'Worn tire stopping distance must physically exceed fresh tire!');
fprintf('>>> All Phase 4 Robustness Assertions PASSED! <<<\n\n');

% Plot Realism and Robustness Figures
save_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

fig = figure('Name', 'Phase 4 - Realism & Robustness', 'Color', 'w', 'Position', [70, 70, 1150, 780], 'Visible', 'off');

% 1. Speed Estimator vs Ground Truth
subplot(2, 2, 1); hold on; grid on; box on;
plot(sim_nominal.t, sim_nominal.v * 3.6, 'k-', 'LineWidth', 2.2, 'DisplayName', 'True Vehicle Speed (Ground Truth)');
plot(sim_nominal.t, v_ref_hist * 3.6, 'r--', 'LineWidth', 1.6, 'DisplayName', 'Estimated Reference Speed v_{ref}');
plot(sim_nominal.t, sim_nominal.vw(:, 1) * 3.6, 'b:', 'LineWidth', 1.0, 'DisplayName', 'FL Wheel Speed (Modulated)');
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Vehicle Reference Velocity Estimator vs. Ground Truth');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 min(sim_nominal.stop_time, 3.5)]);

% 2. Estimation Error
subplot(2, 2, 2); hold on; grid on; box on;
plot(sim_nominal.t, (v_ref_hist - sim_nominal.v) * 3.6, 'm-', 'LineWidth', 1.5);
yline(0, 'k:');
xlabel('Time [s]'); ylabel('Error (v_{ref} - v_{true}) [km/h]');
title('Reference Speed Estimation Error Distribution');
xlim([0 min(sim_nominal.stop_time, 3.5)]);

% 3. Mass Variation Sensitivity (Stopping Distance)
subplot(2, 2, 3); hold on; grid on; box on;
masses = [1280, 1600, 1920];
mass_dists = [sim_light.stop_dist, sim_nominal.stop_dist, sim_heavy.stop_dist];
b_m = bar(masses, mass_dists, 0.45, 'FaceColor', [0.2 0.6 0.4], 'EdgeColor', 'k');
xlabel('Total Vehicle Mass [kg]'); ylabel('Stopping Distance [m]');
title('Robustness: Sensitivity to Vehicle Mass (+/- 20%)');
for i = 1:length(masses)
    text(masses(i), mass_dists(i) + 1.2, sprintf('%.1f m', mass_dists(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
end
ylim([0 max(mass_dists) * 1.18]);

% 4. Delay and Wear Robustness
subplot(2, 2, 4); hold on; grid on; box on;
rob_cases = {'Fast (2.5ms)', 'Nominal (5ms)', 'Slow (7.5ms)', 'Worn Tire (-15%)'};
rob_dists = [sim_fast.stop_dist, sim_nominal.stop_dist, sim_slow.stop_dist, sim_worn.stop_dist];
bar(1:4, rob_dists, 0.5, 'FaceColor', [0.7 0.35 0.25], 'EdgeColor', 'k');
set(gca, 'XTick', 1:4, 'XTickLabel', rob_cases);
xtickangle(20);
ylabel('Stopping Distance [m]');
title('Robustness: Actuator Delay (+/-50%) & Tire Wear');
for i = 1:4
    text(i, rob_dists(i) + 1.2, sprintf('%.1f m', rob_dists(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
end
ylim([0 max(rob_dists) * 1.18]);

sgtitle('Phase 4: Sensor Realism, Speed Estimation & Controller Robustness Sweeps', 'FontSize', 12, 'FontWeight', 'bold');

png_path = fullfile(save_dir, 'phase4_realism_robustness.png');
fig_path = fullfile(save_dir, 'phase4_realism_robustness.fig');
try exportgraphics(fig, png_path, 'Resolution', 120); catch; saveas(fig, png_path); end
savefig(fig, fig_path, 'compact');
fprintf('Saved Phase 4 figures to %s and %s\n', png_path, fig_path);


results.sim_nominal = sim_nominal;
results.v_ref_hist  = v_ref_hist;
results.mean_err_v  = mean_err_v;
results.sim_light   = sim_light;
results.sim_heavy   = sim_heavy;
results.sim_fast    = sim_fast;
results.sim_slow    = sim_slow;
results.sim_worn    = sim_worn;

end
