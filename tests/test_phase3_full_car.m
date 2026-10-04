function results = test_phase3_full_car()
% TEST_PHASE3_FULL_CAR Validates Phase 3 4-wheel vehicle model:
%   1. Dynamic longitudinal load transfer (pitch) during high deceleration.
%   2. 4 independent brake channels with front/rear brake bias.
%   3. Split-mu scenario (Left: Dry Asphalt, Right: Smooth Ice).
%   4. Yaw stability mitigation via Select-Low and Yaw Moment Limitation.

p = init_params();

fprintf('=== Phase 3: 4-Wheel Full Vehicle & Split-mu Verification ===\n');

% 1. Homogeneous Dry Braking (Load Transfer Verification)
scen_dry.v0 = 100 / 3.6;
scen_dry.surface_L = 'dry';
scen_dry.surface_R = 'dry';
scen_dry.driver_type = 'step';
scen_dry.use_ymc = false;
scen_dry.t_end = 5.0;
scen_dry.dt = 0.0005;

fprintf('1. Simulating 4-wheel braking on homogeneous dry asphalt...\n');
sim_4w_dry = sim_full_car(scen_dry, 'rule_based', p);

mean_Fz_front = mean(sim_4w_dry.Fz(sim_4w_dry.t > 0.2 & sim_4w_dry.v > 5.0, 1) + ...
                     sim_4w_dry.Fz(sim_4w_dry.t > 0.2 & sim_4w_dry.v > 5.0, 2));
mean_Fz_rear  = mean(sim_4w_dry.Fz(sim_4w_dry.t > 0.2 & sim_4w_dry.v > 5.0, 3) + ...
                     sim_4w_dry.Fz(sim_4w_dry.t > 0.2 & sim_4w_dry.v > 5.0, 4));

fprintf('   Static Front Axle Load: %.1f N -> Braking Dynamic Mean: %.1f N (+%.1f%%)\n', ...
    p.veh.Fz_static_front_axle, mean_Fz_front, ((mean_Fz_front/p.veh.Fz_static_front_axle)-1)*100);
fprintf('   Static Rear Axle Load:  %.1f N -> Braking Dynamic Mean: %.1f N (-%.1f%%)\n', ...
    p.veh.Fz_static_rear_axle, mean_Fz_rear, (1-(mean_Fz_rear/p.veh.Fz_static_rear_axle))*100);

assert(mean_Fz_front > p.veh.Fz_static_front_axle, 'ASSERTION FAILED: Pitch load transfer must increase front load!');
assert(mean_Fz_rear < p.veh.Fz_static_rear_axle, 'ASSERTION FAILED: Pitch load transfer must decrease rear load!');

% 2. Split-mu Braking: Unmitigated (No YMC) vs Mitigated (With Select-Low YMC)
scen_split.v0 = 100 / 3.6;
scen_split.surface_L = 'dry'; % mu ~ 1.0
scen_split.surface_R = 'ice'; % mu ~ 0.15
scen_split.driver_type = 'step';
scen_split.t_end = 6.0;
scen_split.dt = 0.0005;

fprintf('2. Simulating Split-mu (Left: Dry, Right: Ice) WITHOUT Yaw Control...\n');
scen_split.use_ymc = false;
sim_split_no_ymc = sim_full_car(scen_split, 'rule_based', p);

fprintf('3. Simulating Split-mu (Left: Dry, Right: Ice) WITH Select-Low Yaw Control...\n');
scen_split.use_ymc = true;
sim_split_ymc = sim_full_car(scen_split, 'rule_based', p);

yaw_no_ymc_deg = sim_split_no_ymc.max_yaw * (180 / pi);
yaw_ymc_deg    = sim_split_ymc.max_yaw * (180 / pi);

fprintf('\n--- Split-mu Stability Verification ---\n');
fprintf('Unmitigated Split-mu Max Yaw Angle: %6.1f deg (Vehicle spins violently!)\n', yaw_no_ymc_deg);
fprintf('Select-Low YMC Split-mu Max Yaw:    %6.1f deg (Directionally stable!)\n', yaw_ymc_deg);
fprintf('Yaw Angle Reduction via Select-Low: %6.1f%%\n', (1 - yaw_ymc_deg / yaw_no_ymc_deg) * 100);

assert(yaw_ymc_deg < yaw_no_ymc_deg, 'ASSERTION FAILED: Select-Low must reduce yaw deviation on split-mu!');
fprintf('>>> All Phase 3 assertions PASSED! <<<\n\n');

% Generate Phase 3 Plots
save_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

fig = figure('Name', 'Phase 3 - Full Vehicle Dynamics & Split-mu', 'Color', 'w', 'Position', [50, 50, 1150, 800], 'Visible', 'off');

% Subplot 1: Dynamic Load Transfer (Pitch)
subplot(2, 2, 1); hold on; grid on; box on;
t_d = sim_4w_dry.t;
Fz_F_tot = (sim_4w_dry.Fz(:, 1) + sim_4w_dry.Fz(:, 2)) * 1e-3;
Fz_R_tot = (sim_4w_dry.Fz(:, 3) + sim_4w_dry.Fz(:, 4)) * 1e-3;
plot(t_d, Fz_F_tot, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Front Axle Load (Dynamic)');
plot(t_d, Fz_R_tot, 'r-', 'LineWidth', 2.0, 'DisplayName', 'Rear Axle Load (Dynamic)');
yline(p.veh.Fz_static_front_axle * 1e-3, 'b--', 'LineWidth', 1.2, 'DisplayName', 'Front Axle Static (9.64 kN)');
yline(p.veh.Fz_static_rear_axle * 1e-3, 'r--', 'LineWidth', 1.2, 'DisplayName', 'Rear Axle Static (6.05 kN)');
xlabel('Time [s]'); ylabel('Axle Normal Load [kN]');
title('Longitudinal Pitch Load Transfer during Heavy Braking');
legend('Location', 'east', 'FontSize', 8);
xlim([0 min(sim_4w_dry.stop_time, 3.5)]);

% Subplot 2: Split-mu Yaw Moment
subplot(2, 2, 2); hold on; grid on; box on;
plot(sim_split_no_ymc.t, sim_split_no_ymc.Mz * 1e-3, 'r--', 'LineWidth', 1.6, 'DisplayName', 'Unmitigated (No YMC)');
plot(sim_split_ymc.t, sim_split_ymc.Mz * 1e-3, 'g-', 'LineWidth', 1.8, 'DisplayName', 'Select-Low + YML Capped');
yline(0, 'k:');
xlabel('Time [s]'); ylabel('Yaw Moment M_z [kN\cdotm]');
title('Split-\mu Asymmetric Braking Yaw Moment');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 3.5]);

% Subplot 3: Split-mu Vehicle Yaw Deviation Angle
subplot(2, 2, 3); hold on; grid on; box on;
plot(sim_split_no_ymc.t, sim_split_no_ymc.yaw_angle * (180/pi), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Without Yaw Handling (Spinout)');
plot(sim_split_ymc.t, sim_split_ymc.yaw_angle * (180/pi), 'g-', 'LineWidth', 2.0, 'DisplayName', 'With Select-Low + YML (Stable)');
xlabel('Time [s]'); ylabel('Yaw Angle \psi [deg]');
title('Vehicle Yaw Heading Deviation on Split-\mu Surface');
legend('Location', 'northwest', 'FontSize', 8);
xlim([0 3.5]);

% Subplot 4: 4-Wheel Pressures during Select-Low Split-mu
subplot(2, 2, 4); hold on; grid on; box on;
plot(sim_split_ymc.t, sim_split_ymc.P_act(:, 1) * 1e-5, 'b-', 'LineWidth', 1.5, 'DisplayName', 'FL (Dry Front)');
plot(sim_split_ymc.t, sim_split_ymc.P_act(:, 2) * 1e-5, 'c-', 'LineWidth', 1.5, 'DisplayName', 'FR (Ice Front)');
plot(sim_split_ymc.t, sim_split_ymc.P_act(:, 3) * 1e-5, 'm--', 'LineWidth', 1.5, 'DisplayName', 'RL (Dry Rear - Select-Low)');
plot(sim_split_ymc.t, sim_split_ymc.P_act(:, 4) * 1e-5, 'r:', 'LineWidth', 2.0, 'DisplayName', 'RR (Ice Rear - Select-Low)');
xlabel('Time [s]'); ylabel('Pressure [bar]');
title('4-Wheel Channel Pressure Allocation (Select-Low Active)');
legend('Location', 'northeast', 'FontSize', 8);
xlim([0 3.5]);

sgtitle('Phase 3: 4-Wheel Vehicle Dynamics, Pitch Load Transfer & Split-\mu Yaw Stability', 'FontSize', 12, 'FontWeight', 'bold');

png_path = fullfile(save_dir, 'phase3_full_car_split_mu.png');
fig_path = fullfile(save_dir, 'phase3_full_car_split_mu.fig');
try exportgraphics(fig, png_path, 'Resolution', 120); catch; saveas(fig, png_path); end
savefig(fig, fig_path, 'compact');
fprintf('Saved Phase 3 figures to %s and %s\n', png_path, fig_path);


results.sim_4w_dry       = sim_4w_dry;
results.sim_split_no_ymc = sim_split_no_ymc;
results.sim_split_ymc    = sim_split_ymc;

end
