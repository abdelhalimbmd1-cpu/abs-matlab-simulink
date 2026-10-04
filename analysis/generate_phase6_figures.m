function generate_phase6_figures(record_list, results_dir, p)
% GENERATE_PHASE6_FIGURES Generates publication-ready figures for Phase 6
%
% Generates:
%   1. stopping_distance_by_surface (Bar chart across surfaces at 100 km/h)
%   2. speed_sensitivity (Stopping distance vs initial speed: 60, 100, 130 km/h)
%   3. surface_transitions (Mid-braking transitions: Dry->Ice, Wet->Dry)
%   4. chatter_vs_effort (Actuator chatter index vs hydraulic control effort)
%   5. split_mu_yaw_summary (Split-mu yaw dynamics and heading control)

if nargin < 2 || isempty(results_dir)
    results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
end
if nargin < 3 || isempty(p)
    p = init_params();
end

fprintf('Generating publication-quality Phase 6 analysis figures...\n');

controllers = {'none', 'rule_based', 'pid', 'smc', 'adaptive'};
ctrl_names  = {'No-ABS', 'Rule-Based', 'PID', 'SMC', 'Adaptive'};
colors      = {[0.4 0.4 0.4], [0.15 0.50 0.85], [0.15 0.75 0.25], [0.85 0.35 0.15], [0.65 0.15 0.85]};

%% Figure 1: Stopping Distance by Surface (100 km/h)
fig1 = figure('Name', 'Stopping Distance by Surface', 'Color', 'w', 'Position', [60, 60, 1000, 600], 'Visible', 'off');
surfaces = {'dry', 'wet', 'snow', 'ice'};
surf_labels = {'Dry Asphalt (\mu=1.0)', 'Wet Asphalt (\mu=0.75)', 'Packed Snow (\mu=0.30)', 'Smooth Ice (\mu=0.15)'};

dist_matrix = zeros(length(surfaces), length(controllers));
for s = 1:length(surfaces)
    sc_id = sprintf('%s_100kmh_step', surfaces{s});
    for c = 1:length(controllers)
        idx = find(strcmp({record_list.scenario_id}, sc_id) & strcmp({record_list.controller}, controllers{c}));
        if ~isempty(idx)
            dist_matrix(s, c) = record_list(idx).stop_dist;
        end
    end
end

b = bar(dist_matrix, 'grouped');
for c = 1:length(controllers)
    b(c).FaceColor = colors{c};
end
set(gca, 'XTick', 1:length(surfaces), 'XTickLabel', surf_labels, 'FontSize', 10);
ylabel('Stopping Distance [m]', 'FontSize', 11, 'FontWeight', 'bold');
title('100 \rightarrow 0 km/h Stopping Distance across Road Surfaces', 'FontSize', 12, 'FontWeight', 'bold');
legend(ctrl_names, 'Location', 'northwest', 'FontSize', 9);
grid on; box on;

% Value labels on dry & wet bars
for s = 1:2
    for c = 1:length(controllers)
        x_pos = b(c).XEndPoints(s);
        y_pos = b(c).YEndPoints(s);
        text(x_pos, y_pos + 1.5, sprintf('%.1f', y_pos), 'HorizontalAlignment', 'center', ...
            'FontSize', 7.5, 'Rotation', 90);
    end
end

try exportgraphics(fig1, fullfile(results_dir, 'stopping_distance_by_surface.png'), 'Resolution', 120); catch; saveas(fig1, fullfile(results_dir, 'stopping_distance_by_surface.png')); end
savefig(fig1, fullfile(results_dir, 'stopping_distance_by_surface.fig'), 'compact');
close(fig1);

%% Figure 2: Speed Sensitivity (60, 100, 130 km/h)
fig2 = figure('Name', 'Speed Sensitivity', 'Color', 'w', 'Position', [70, 70, 1050, 550], 'Visible', 'off');
speeds = [60, 100, 130];

subplot(1, 2, 1); hold on; grid on; box on;
for c = 1:length(controllers)
    dists = zeros(size(speeds));
    for v = 1:length(speeds)
        sc_id = sprintf('dry_%dkmh_step', speeds(v));
        idx = find(strcmp({record_list.scenario_id}, sc_id) & strcmp({record_list.controller}, controllers{c}));
        if ~isempty(idx), dists(v) = record_list(idx).stop_dist; end
    end
    plot(speeds, dists, 'o-', 'Color', colors{c}, 'LineWidth', 2.0, 'MarkerSize', 6, 'DisplayName', ctrl_names{c});
end
xlabel('Initial Speed [km/h]'); ylabel('Stopping Distance [m]');
title('Dry Asphalt: Stopping Distance vs. Initial Speed');
legend('Location', 'northwest', 'FontSize', 8);

subplot(1, 2, 2); hold on; grid on; box on;
for c = 1:length(controllers)
    dists = zeros(size(speeds));
    for v = 1:length(speeds)
        sc_id = sprintf('wet_%dkmh_step', speeds(v));
        idx = find(strcmp({record_list.scenario_id}, sc_id) & strcmp({record_list.controller}, controllers{c}));
        if ~isempty(idx), dists(v) = record_list(idx).stop_dist; end
    end
    plot(speeds, dists, 's--', 'Color', colors{c}, 'LineWidth', 2.0, 'MarkerSize', 6, 'DisplayName', ctrl_names{c});
end
xlabel('Initial Speed [km/h]'); ylabel('Stopping Distance [m]');
title('Wet Asphalt: Stopping Distance vs. Initial Speed');
legend('Location', 'northwest', 'FontSize', 8);

sgtitle('Speed Sensitivity Analysis (60, 100, 130 km/h)', 'FontSize', 12, 'FontWeight', 'bold');
try exportgraphics(fig2, fullfile(results_dir, 'speed_sensitivity.png'), 'Resolution', 120); catch; saveas(fig2, fullfile(results_dir, 'speed_sensitivity.png')); end
savefig(fig2, fullfile(results_dir, 'speed_sensitivity.fig'), 'compact');
close(fig2);


%% Figure 3: Surface Transitions (Dry -> Ice)
fig3 = figure('Name', 'Surface Transitions', 'Color', 'w', 'Position', [80, 80, 1100, 750], 'Visible', 'off');
sc_trans.v0 = 100 / 3.6;
sc_trans.surface = 'transition';
sc_trans.driver_type = 'step';
sc_trans.t_end = 8.0;
sc_trans.dt = 0.0005;

sim_no_trans   = sim_quarter_car(sc_trans, 'none', p);
sim_rb_trans   = sim_quarter_car(sc_trans, 'rule_based', p);
sim_pid_trans  = sim_quarter_car(sc_trans, 'pid', p);
sim_smc_trans  = sim_quarter_car(sc_trans, 'smc', p);
sim_ad_trans   = sim_quarter_car(sc_trans, 'adaptive', p);

subplot(3, 1, 1); hold on; grid on; box on;
plot(sim_no_trans.t, sim_no_trans.v * 3.6, 'k--', 'LineWidth', 1.8, 'DisplayName', 'No ABS');
plot(sim_rb_trans.t, sim_rb_trans.v * 3.6, 'Color', colors{2}, 'LineWidth', 1.8, 'DisplayName', 'Rule-Based');
plot(sim_pid_trans.t, sim_pid_trans.v * 3.6, 'Color', colors{3}, 'LineWidth', 1.8, 'DisplayName', 'PID');
plot(sim_smc_trans.t, sim_smc_trans.v * 3.6, 'Color', colors{4}, 'LineWidth', 1.8, 'DisplayName', 'SMC');
plot(sim_ad_trans.t, sim_ad_trans.v * 3.6, 'Color', colors{5}, 'LineWidth', 1.8, 'DisplayName', 'Adaptive');
xline(1.0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Transition to Ice (t=1.0s)');
ylabel('Speed [km/h]'); title('Mid-Braking Transition: Dry Asphalt \rightarrow Smooth Ice at t = 1.0s');
legend('Location', 'northeast', 'FontSize', 8); xlim([0 6.0]);

subplot(3, 1, 2); hold on; grid on; box on;
plot(sim_no_trans.t, sim_no_trans.slip, 'k--', 'LineWidth', 1.2, 'DisplayName', 'No ABS');
plot(sim_rb_trans.t, sim_rb_trans.slip, 'Color', colors{2}, 'LineWidth', 1.2, 'DisplayName', 'Rule-Based');
plot(sim_pid_trans.t, sim_pid_trans.slip, 'Color', colors{3}, 'LineWidth', 1.2, 'DisplayName', 'PID');
plot(sim_smc_trans.t, sim_smc_trans.slip, 'Color', colors{4}, 'LineWidth', 1.2, 'DisplayName', 'SMC');
plot(sim_ad_trans.t, sim_ad_trans.slip, 'Color', colors{5}, 'LineWidth', 1.2, 'DisplayName', 'Adaptive');
xline(1.0, 'r--', 'LineWidth', 1.5);
ylabel('Slip Ratio \lambda [-]'); title('Slip Regulation across Surface Discontinuity');
ylim([0 1.05]); xlim([0 6.0]);

subplot(3, 1, 3); hold on; grid on; box on;
plot(sim_no_trans.t, sim_no_trans.P_act * 1e-5, 'k--', 'LineWidth', 1.2, 'DisplayName', 'No ABS');
plot(sim_rb_trans.t, sim_rb_trans.P_act * 1e-5, 'Color', colors{2}, 'LineWidth', 1.2, 'DisplayName', 'Rule-Based');
plot(sim_pid_trans.t, sim_pid_trans.P_act * 1e-5, 'Color', colors{3}, 'LineWidth', 1.2, 'DisplayName', 'PID');
plot(sim_smc_trans.t, sim_smc_trans.P_act * 1e-5, 'Color', colors{4}, 'LineWidth', 1.2, 'DisplayName', 'SMC');
plot(sim_ad_trans.t, sim_ad_trans.P_act * 1e-5, 'Color', colors{5}, 'LineWidth', 1.2, 'DisplayName', 'Adaptive');
xline(1.0, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Line Pressure [bar]'); title('Hydraulic Pressure Adaptation to Low Grip');
xlim([0 6.0]);

try exportgraphics(fig3, fullfile(results_dir, 'surface_transitions.png'), 'Resolution', 120); catch; saveas(fig3, fullfile(results_dir, 'surface_transitions.png')); end
savefig(fig3, fullfile(results_dir, 'surface_transitions.fig'), 'compact');
close(fig3);

%% Figure 4: Chatter Index vs Control Effort
fig4 = figure('Name', 'Chatter vs Effort', 'Color', 'w', 'Position', [90, 90, 850, 550], 'Visible', 'off');
hold on; grid on; box on;

sc_dry_idx = find(strcmp({record_list.scenario_id}, 'dry_100kmh_step'));
for i = 1:length(sc_dry_idx)
    r = record_list(sc_dry_idx(i));
    c_idx = find(strcmp(controllers, r.controller));
    scatter(r.control_effort, r.chatter_index, 140, colors{c_idx}, 'filled', 'MarkerEdgeColor', 'k', ...
        'DisplayName', r.ctrl_name);
    text(r.control_effort + 0.3, r.chatter_index + 1.5, r.ctrl_name, 'FontSize', 9, 'FontWeight', 'bold');
end
xlabel('Control Effort \int P_{act} dt [MPa\cdot s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Chatter Index (Mean |dP/dt|) [MPa/s]', 'FontSize', 11, 'FontWeight', 'bold');
title('Actuator Wear vs. Control Effort Trade-off (Dry 100 km/h)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);

try exportgraphics(fig4, fullfile(results_dir, 'chatter_vs_effort.png'), 'Resolution', 120); catch; saveas(fig4, fullfile(results_dir, 'chatter_vs_effort.png')); end
savefig(fig4, fullfile(results_dir, 'chatter_vs_effort.fig'), 'compact');
close(fig4);

%% Figure 5: Split-mu Yaw Summary
fig5 = figure('Name', 'Split-mu Yaw Summary', 'Color', 'w', 'Position', [100, 100, 1050, 600], 'Visible', 'off');
sc_sp.v0 = 100 / 3.6;
sc_sp.surface_L = 'dry';
sc_sp.surface_R = 'ice';
sc_sp.driver_type = 'step';
sc_sp.t_end = 6.0;
sc_sp.dt = 0.001;

sc_sp.use_ymc = false;
s_unmit = sim_full_car(sc_sp, 'rule_based', p);
sc_sp.use_ymc = true;
s_mit   = sim_full_car(sc_sp, 'rule_based', p);

subplot(1, 2, 1); hold on; grid on; box on;
plot(s_unmit.t, s_unmit.yaw_angle * (180/pi), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Unmitigated (Spinout)');
plot(s_mit.t, s_mit.yaw_angle * (180/pi), 'g-', 'LineWidth', 2.0, 'DisplayName', 'Select-Low + YML (Stable)');
xlabel('Time [s]'); ylabel('Yaw Angle \psi [deg]');
title('Vehicle Heading Deviation Comparison');
legend('Location', 'northwest', 'FontSize', 9);
xlim([0 3.5]);

subplot(1, 2, 2); hold on; grid on; box on;
plot(s_unmit.t, s_unmit.yaw_rate * (180/pi), 'r--', 'LineWidth', 1.8, 'DisplayName', 'Unmitigated (Spinout)');
plot(s_mit.t, s_mit.yaw_rate * (180/pi), 'g-', 'LineWidth', 1.8, 'DisplayName', 'Select-Low + YML (Stable)');
xlabel('Time [s]'); ylabel('Yaw Rate \omega_z [deg/s]');
title('Yaw Rate Stability Response');
legend('Location', 'northeast', 'FontSize', 9);
xlim([0 3.5]);

sgtitle('Split-\mu Directional Stability Assessment', 'FontSize', 12, 'FontWeight', 'bold');
try exportgraphics(fig5, fullfile(results_dir, 'split_mu_yaw_summary.png'), 'Resolution', 120); catch; saveas(fig5, fullfile(results_dir, 'split_mu_yaw_summary.png')); end
savefig(fig5, fullfile(results_dir, 'split_mu_yaw_summary.fig'), 'compact');
close(fig5);

fprintf('All Phase 6 analysis figures exported successfully to %s!\n', results_dir);

end

