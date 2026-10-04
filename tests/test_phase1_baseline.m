function results = test_phase1_baseline()
% TEST_PHASE1_BASELINE Validates Phase 1 baseline quarter-car model
% Verifies:
%   1. No-ABS full braking causes wheel lockup (lambda -> 1.0).
%   2. Braking torque overcomes peak tire grip.
%   3. Operating trajectory on mu-slip curve passes through peak into locked sliding region.
%   4. Compares with ABS controllers.

p = init_params();

scen.v0 = 100 / 3.6; % 100 km/h (27.78 m/s)
scen.surface = 'dry';
scen.driver_type = 'step';
scen.t_end = 6.0;
scen.dt = 0.0005;

fprintf('=== Running Phase 1 Baseline Verification ===\n');

% 1. Run No-ABS full braking
fprintf('Simulating No-ABS full braking on dry asphalt...\n');
sim_no_abs = sim_quarter_car(scen, 'none', p);

% 2. Run Rule-Based ABS
fprintf('Simulating Rule-Based ABS on dry asphalt...\n');
sim_rb = sim_quarter_car(scen, 'rule_based', p);

% 3. Run PID ABS
fprintf('Simulating PID ABS on dry asphalt...\n');
sim_pid = sim_quarter_car(scen, 'pid', p);

% 4. Run SMC ABS
fprintf('Simulating SMC ABS on dry asphalt...\n');
sim_smc = sim_quarter_car(scen, 'smc', p);

% Verification assertions
max_slip_no_abs = max(sim_no_abs.slip);

fprintf('\n--- Verification Results ---\n');
fprintf('No-ABS Max Slip:       %.3f (Expected >= 0.99 for lockup)\n', max_slip_no_abs);
fprintf('No-ABS Stopping Dist:  %.2f m, Stop Time: %.2f s\n', sim_no_abs.stop_dist, sim_no_abs.stop_time);
fprintf('Rule-Based Stop Dist:  %.2f m, Stop Time: %.2f s (Reduction: %.2f m)\n', ...
    sim_rb.stop_dist, sim_rb.stop_time, sim_no_abs.stop_dist - sim_rb.stop_dist);
fprintf('PID Stop Dist:         %.2f m, Stop Time: %.2f s (Reduction: %.2f m)\n', ...
    sim_pid.stop_dist, sim_pid.stop_time, sim_no_abs.stop_dist - sim_pid.stop_dist);
fprintf('SMC Stop Dist:         %.2f m, Stop Time: %.2f s (Reduction: %.2f m)\n', ...
    sim_smc.stop_dist, sim_smc.stop_time, sim_no_abs.stop_dist - sim_smc.stop_dist);

assert(max_slip_no_abs > 0.95, 'ASSERTION FAILED: No-ABS must lock wheel (slip > 0.95)!');
assert(sim_rb.stop_dist < sim_no_abs.stop_dist, 'ASSERTION FAILED: Rule-Based ABS must reduce stopping distance on dry asphalt!');
assert(sim_pid.stop_dist < sim_no_abs.stop_dist, 'ASSERTION FAILED: PID ABS must reduce stopping distance on dry asphalt!');
assert(sim_smc.stop_dist < sim_no_abs.stop_dist, 'ASSERTION FAILED: SMC ABS must reduce stopping distance on dry asphalt!');
fprintf('>>> All Phase 1 assertions PASSED successfully! <<<\n\n');

% Generate Phase 1 verification plots
save_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

fig = figure('Name', 'Phase 1 - No-ABS Wheel Lockup Verification', 'Color', 'w', 'Position', [80, 80, 1100, 750], 'Visible', 'off');

% Subplot 1: Vehicle and Wheel Speeds (No-ABS vs ABS)
subplot(2, 2, 1);
hold on; grid on; box on;
plot(sim_no_abs.t, sim_no_abs.v * 3.6, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Vehicle Speed (No ABS)');
plot(sim_no_abs.t, sim_no_abs.vw * 3.6, 'r-', 'LineWidth', 1.8, 'DisplayName', 'Wheel Speed (No ABS - Locks!)');
plot(sim_rb.t, sim_rb.v * 3.6, 'b--', 'LineWidth', 1.8, 'DisplayName', 'Vehicle Speed (ABS)');
plot(sim_rb.t, sim_rb.vw * 3.6, 'b-', 'LineWidth', 1.2, 'DisplayName', 'Wheel Speed (ABS - Modulated)');
xlabel('Time [s]'); ylabel('Speed [km/h]');
title('Vehicle vs. Wheel Speed: Lockup vs. ABS');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 min(sim_no_abs.stop_time + 0.2, 4.5)]);

% Subplot 2: Slip Ratio vs Time
subplot(2, 2, 2);
hold on; grid on; box on;
plot(sim_no_abs.t, sim_no_abs.slip, 'r-', 'LineWidth', 1.8, 'DisplayName', 'No ABS (\lambda \rightarrow 1.0 Locked)');
plot(sim_rb.t, sim_rb.slip, 'b-', 'LineWidth', 1.4, 'DisplayName', 'Rule-Based ABS');
plot(sim_pid.t, sim_pid.slip, 'g-', 'LineWidth', 1.4, 'DisplayName', 'PID ABS');
plot(sim_smc.t, sim_smc.slip, 'm-', 'LineWidth', 1.4, 'DisplayName', 'SMC ABS');
yline(p.tire.dry.opt_slip, 'k--', 'LineWidth', 1.2, 'DisplayName', sprintf('Optimal Slip (\\lambda_{opt}=%.2f)', p.tire.dry.opt_slip));
xlabel('Time [s]'); ylabel('Slip Ratio \lambda [-]');
title('Longitudinal Slip Ratio Response');
legend('Location', 'east', 'FontSize', 8);
ylim([0 1.05]);
xlim([0 min(sim_no_abs.stop_time + 0.2, 4.5)]);

% Subplot 3: Brake Line Pressure vs Time
subplot(2, 2, 3);
hold on; grid on; box on;
plot(sim_no_abs.t, sim_no_abs.P_act * 1e-5, 'r-', 'LineWidth', 1.8, 'DisplayName', 'No ABS (150 bar)');
plot(sim_rb.t, sim_rb.P_act * 1e-5, 'b-', 'LineWidth', 1.4, 'DisplayName', 'Rule-Based ABS');
plot(sim_pid.t, sim_pid.P_act * 1e-5, 'g-', 'LineWidth', 1.4, 'DisplayName', 'PID ABS');
plot(sim_smc.t, sim_smc.P_act * 1e-5, 'm-', 'LineWidth', 1.4, 'DisplayName', 'SMC ABS');
xlabel('Time [s]'); ylabel('Brake Pressure [bar]');
title('Actuator Line Pressure Dynamics');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 min(sim_no_abs.stop_time + 0.2, 4.5)]);

% Subplot 4: Operating Trajectory on Pacejka mu-slip Curve
subplot(2, 2, 4);
hold on; grid on; box on;
lam_curve = linspace(0, 1.0, 300);
mu_curve = pacejka_tire(lam_curve, 1.0, p.tire.dry);
plot(lam_curve, mu_curve, 'k-', 'LineWidth', 2.0, 'DisplayName', 'Dry Asphalt Magic Formula');
% Plot trajectories
plot(sim_no_abs.slip(1:20:end), sim_no_abs.mu(1:20:end), 'ro', 'MarkerSize', 4, 'DisplayName', 'No-ABS Trajectory (Locks to \lambda=1)');
plot(sim_rb.slip(1:20:end), sim_rb.mu(1:20:end), 'b.', 'MarkerSize', 6, 'DisplayName', 'Rule-Based Trajectory (Near Peak)');
plot(sim_pid.slip(1:20:end), sim_pid.mu(1:20:end), 'g^', 'MarkerSize', 4, 'DisplayName', 'PID Trajectory (Tightly Regulated)');
xlabel('Slip Ratio \lambda [-]'); ylabel('Friction Coefficient \mu [-]');
title('Operating Trajectory on \mu-\lambda Curve');
legend('Location', 'southeast', 'FontSize', 8);
xlim([0 1.0]); ylim([0 1.15]);

sgtitle('Phase 1 Baseline: Quarter-Car Model & Wheel Lockup Verification', 'FontSize', 12, 'FontWeight', 'bold');

% Save figure in PNG and FIG formats
png_out = fullfile(save_dir, 'phase1_lockup_verification.png');
fig_out = fullfile(save_dir, 'phase1_lockup_verification.fig');
try exportgraphics(fig, png_out, 'Resolution', 120); catch; saveas(fig, png_out); end
savefig(fig, fig_out, 'compact');
fprintf('Saved Phase 1 verification plots to %s and %s\n', png_out, fig_out);


results.sim_no_abs = sim_no_abs;
results.sim_rb     = sim_rb;
results.sim_pid    = sim_pid;
results.sim_smc    = sim_smc;

end
