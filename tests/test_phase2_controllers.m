function results = test_phase2_controllers()
% TEST_PHASE2_CONTROLLERS Validates all Phase 2 ABS controllers
% Evaluates:
%   1. Rule-based threshold ABS (Bosch 3-phase)
%   2. PID slip controller with anti-windup
%   3. Sliding Mode Controller (SMC) with boundary layer
%   4. Adaptive target-slip controller with friction estimator
%
% Generates comparison plots, verifies slip tracking, and logs metrics.

p = init_params();

controllers = {'rule_based', 'pid', 'smc', 'adaptive'};
ctrl_names  = {'Rule-Based (Bosch)', 'PID Slip', 'Sliding Mode (SMC)', 'Adaptive / Estimator'};
colors      = {[0.15 0.50 0.85], [0.15 0.75 0.25], [0.85 0.35 0.15], [0.65 0.15 0.85]};

scen.v0 = 100 / 3.6; % 100 km/h
scen.surface = 'dry';
scen.driver_type = 'step';
scen.t_end = 5.0;
scen.dt = 0.0005;

fprintf('=== Phase 2: ABS Controller Comparative Evaluation ===\n');

% Baseline No-ABS
s_no_abs = sim_quarter_car(scen, 'none', p);
fprintf('%-22s: Dist = %5.2f m | Time = %4.2f s | Max Slip = %.3f\n', ...
    'No-ABS (Locked)', s_no_abs.stop_dist, s_no_abs.stop_time, max(s_no_abs.slip));

sim_results = cell(length(controllers), 1);
metrics     = struct();

for i = 1:length(controllers)
    c_type = controllers{i};
    s_out  = sim_quarter_car(scen, c_type, p);
    sim_results{i} = s_out;
    
    % Active braking window (exclude final stop below 2 m/s and pedal apply transient)
    mask = (s_out.t > 0.20) & (s_out.v > 2.0);
    slip_active = s_out.slip(mask);
    t_active    = s_out.t(mask);
    P_active    = s_out.P_act(mask);
    
    mean_slip = mean(slip_active);
    max_slip  = max(slip_active);
    slip_rmse = sqrt(mean((slip_active - p.ctrl.pid.target_slip).^2));
    
    % Pressure chatter index: mean absolute derivative of pressure
    dP_dt = diff(P_active) ./ diff(t_active);
    chatter_idx = mean(abs(dP_dt)) * 1e-6; % [MPa/s]
    
    metrics.(c_type).stop_dist   = s_out.stop_dist;
    metrics.(c_type).stop_time   = s_out.stop_time;
    metrics.(c_type).mean_slip   = mean_slip;
    metrics.(c_type).max_slip    = max_slip;
    metrics.(c_type).slip_rmse   = slip_rmse;
    metrics.(c_type).chatter_idx = chatter_idx;
    
    fprintf('%-22s: Dist = %5.2f m | Time = %4.2f s | Mean Slip = %.3f | RMSE = %.3f | Chatter = %5.1f MPa/s\n', ...
        ctrl_names{i}, s_out.stop_dist, s_out.stop_time, mean_slip, slip_rmse, chatter_idx);
    
    % Assertions
    assert(s_out.stop_dist < s_no_abs.stop_dist, ...
        sprintf('%s must reduce stopping distance relative to No-ABS!', ctrl_names{i}));
    assert(max_slip < 0.80, ...
        sprintf('%s must prevent wheel lockup during active braking!', ctrl_names{i}));
end

fprintf('\n>>> All Phase 2 Controller Assertions PASSED! <<<\n\n');

% Plot Comparative Controller Figures
save_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

fig = figure('Name', 'Phase 2 - Controller Comparison', 'Color', 'w', 'Position', [60, 60, 1150, 780], 'Visible', 'off');

% 1. Speed trajectories
subplot(2, 2, 1); hold on; grid on; box on;
plot(s_no_abs.t, s_no_abs.v * 3.6, 'k--', 'LineWidth', 1.8, 'DisplayName', 'No ABS (Locked)');
for i = 1:length(controllers)
    s = sim_results{i};
    plot(s.t, s.v * 3.6, 'Color', colors{i}, 'LineWidth', 1.8, 'DisplayName', ctrl_names{i});
end
xlabel('Time [s]'); ylabel('Vehicle Speed [km/h]');
title('Vehicle Deceleration Trajectories (100 km/h on Dry Asphalt)');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 4.0]);

% 2. Slip Ratio Regulation
subplot(2, 2, 2); hold on; grid on; box on;
plot(s_no_abs.t, s_no_abs.slip, 'k--', 'LineWidth', 1.4, 'DisplayName', 'No ABS');
for i = 1:length(controllers)
    s = sim_results{i};
    plot(s.t, s.slip, 'Color', colors{i}, 'LineWidth', 1.4, 'DisplayName', ctrl_names{i});
end
yline(p.tire.dry.opt_slip, 'r--', 'LineWidth', 1.2, 'DisplayName', '\lambda_{optimal} (0.18)');
xlabel('Time [s]'); ylabel('Slip Ratio \lambda [-]');
title('Longitudinal Slip Ratio Regulation');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 3.5]); ylim([0 0.5]);

% 3. Hydraulic Brake Line Pressure
subplot(2, 2, 3); hold on; grid on; box on;
plot(s_no_abs.t, s_no_abs.P_act * 1e-5, 'k--', 'LineWidth', 1.4, 'DisplayName', 'No ABS (150 bar)');
for i = 1:length(controllers)
    s = sim_results{i};
    plot(s.t, s.P_act * 1e-5, 'Color', colors{i}, 'LineWidth', 1.4, 'DisplayName', ctrl_names{i});
end
xlabel('Time [s]'); ylabel('Line Pressure [bar]');
title('Actuator Line Pressure Modulation');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 3.5]); ylim([0 160]);

% 4. Stopping Distance Comparison Bar Chart
subplot(2, 2, 4); hold on; grid on; box on;
bar_labels = [{'No ABS'}, ctrl_names];
bar_dists  = [s_no_abs.stop_dist];
for i = 1:length(controllers)
    bar_dists = [bar_dists, sim_results{i}.stop_dist];
end
b = bar(bar_dists, 'FaceColor', [0.2 0.45 0.75], 'EdgeColor', 'k');
set(gca, 'XTickLabel', bar_labels, 'XTick', 1:length(bar_labels));
xtickangle(25);
ylabel('Stopping Distance [m]');
title('Stopping Distance Benchmark (Dry Asphalt, 100 \rightarrow 0 km/h)');
for idx = 1:length(bar_dists)
    text(idx, bar_dists(idx) + 1.2, sprintf('%.1f m', bar_dists(idx)), ...
        'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');
end
ylim([0 max(bar_dists) * 1.18]);

sgtitle('Phase 2: Comprehensive ABS Controller Benchmark & Slip Regulation', 'FontSize', 12, 'FontWeight', 'bold');

png_path = fullfile(save_dir, 'phase2_controller_comparison.png');
fig_path = fullfile(save_dir, 'phase2_controller_comparison.fig');
try exportgraphics(fig, png_path, 'Resolution', 120); catch; saveas(fig, png_path); end
savefig(fig, fig_path, 'compact');
fprintf('Saved Phase 2 comparison figures to %s and %s\n', png_path, fig_path);


results.metrics = metrics;
results.sim_results = sim_results;
results.s_no_abs = s_no_abs;

end
