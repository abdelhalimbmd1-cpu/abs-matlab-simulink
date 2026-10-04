function run_simulink(open_gui)
% RUN_SIMULINK Executes both ABS Simulink models and displays telemetry.
%
% Usage:
%   run_simulink       - Runs simulation of both models and displays results
%   run_simulink(true) - Runs simulation and opens both block diagrams in Simulink

if nargin < 1
    open_gui = false;
end

project_root = fileparts(mfilename('fullpath'));
fprintf('=================================================================\n');
fprintf('  SIMULINK ABS MODEL SIMULATION RUNNER\n');
fprintf('=================================================================\n\n');

% Set safe rendering
set(groot, 'defaultFigureRenderer', 'painters');

% Add paths
addpath(fullfile(project_root, 'params'));
addpath(fullfile(project_root, 'models'));
addpath(fullfile(project_root, 'controllers'));

% Initialize parameters in base workspace
fprintf('1. Initializing parameter database...\n');
params = init_params(); %#ok<NASGU>
assignin('base', 'params', params);

models_dir = fullfile(project_root, 'models');
results_dir = fullfile(project_root, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% 1. Simulate Quarter-Car Simulink Model
fprintf('2. Simulating Quarter-Car Model (abs_quarter_car.slx)...\n');
qc_model = 'abs_quarter_car';
qc_path = fullfile(models_dir, [qc_model, '.slx']);

if ~exist(qc_path, 'file')
    fprintf('   Building quarter-car model...\n');
    build_quarter_car_model();
end

load_system(qc_path);
t_start = tic;
simOut_q = sim(qc_model, 'StopTime', '5.0');
t_sim_q = toc(t_start);

% Extract signals
t_q    = simOut_q.tout;
v_q    = simOut_q.v_kmh;
vw_q   = simOut_q.vw_kmh;
slip_q = simOut_q.slip;
P_q    = simOut_q.P_bar;
dist_q = simOut_q.dist_m;

final_dist_q = dist_q(end);
stop_idx_q = find(v_q <= 0.5, 1);
if isempty(stop_idx_q)
    stop_time_q = t_q(end);
else
    stop_time_q = t_q(stop_idx_q);
end

fprintf('   -> Quarter-Car Simulation completed in %.2f s (Sim time: %.2f s)\n', t_sim_q, t_q(end));
fprintf('      Initial Speed:     %.1f km/h\n', v_q(1));
fprintf('      Stopping Distance: %.2f m\n', final_dist_q);
fprintf('      Stopping Time:     %.2f s\n', stop_time_q);
fprintf('      Peak Slip:         %.3f\n', max(slip_q));
fprintf('      Max Brake Pressure:%.1f bar\n\n', max(P_q));

%% 2. Simulate 4-Wheel Full-Car Simulink Model
fprintf('3. Simulating 4-Wheel Full-Car Model (abs_full_car.slx)...\n');
fc_model = 'abs_full_car';
fc_path = fullfile(models_dir, [fc_model, '.slx']);

if ~exist(fc_path, 'file')
    fprintf('   Building full-car model...\n');
    build_full_car_model();
end

load_system(fc_path);
t_start = tic;
simOut_f = sim(fc_model, 'StopTime', '5.0');
t_sim_f = toc(t_start);

% Extract signals
t_f      = simOut_f.tout;
v_f      = simOut_f.v_kmh;
dist_f   = simOut_f.dist_m;
yaw_f    = simOut_f.yaw_deg;
try
    yaw_rt_f = simOut_f.yaw_rate;
catch
    try
        yaw_rt_f = simOut_f.yaw_rate_dps;
    catch
        yaw_rt_f = zeros(size(t_f));
    end
end

final_dist_f = dist_f(end);
stop_idx_f = find(v_f <= 0.5, 1);
if isempty(stop_idx_f)
    stop_time_f = t_f(end);
else
    stop_time_f = t_f(stop_idx_f);
end

fprintf('   -> Full-Car Simulation completed in %.2f s (Sim time: %.2f s)\n', t_sim_f, t_f(end));
fprintf('      Initial Speed:     %.1f km/h\n', v_f(1));
fprintf('      Stopping Distance: %.2f m\n', final_dist_f);
fprintf('      Stopping Time:     %.2f s\n', stop_time_f);
fprintf('      Max Yaw Angle:     %.2f deg\n', max(abs(yaw_f)));
fprintf('      Max Yaw Rate:      %.2f deg/s\n\n', max(abs(yaw_rt_f)));

%% 3. Generate Telemetry Figure (High-Contrast Dark Theme)
try
    fig = figure('Name', 'Simulink ABS Results', 'Position', [100, 100, 1100, 750], 'Visible', 'off');
    set(fig, 'Color', [0.06, 0.06, 0.06], 'InvertHardcopy', 'off');
    
    % Subplot 1
    ax1 = subplot(2, 2, 1);
    plot(t_q, v_q, 'Color', [0.0, 0.8, 1.0], 'LineWidth', 2.0); hold on;
    plot(t_q, vw_q, 'Color', [1.0, 0.3, 0.3], 'LineStyle', '--', 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Speed [km/h]');
    legend('Vehicle Speed', 'Wheel Speed', 'Location', 'northeast', 'TextColor', 'w', 'Color', [0.15 0.15 0.15]);
    title('Quarter-Car: Longitudinal Speeds');
    
    % Subplot 2
    ax2 = subplot(2, 2, 2);
    yyaxis left;
    plot(t_q, slip_q, 'Color', [1.0, 0.4, 0.8], 'LineWidth', 1.8);
    ylabel('Slip Ratio \lambda'); ylim([0, 1]);
    ax2.YColor = [1.0, 0.4, 0.8];
    yyaxis right;
    plot(t_q, P_q, 'Color', [1.0, 0.85, 0.2], 'LineWidth', 1.8);
    ylabel('Brake Pressure [bar]'); ylim([0, 160]);
    ax2.YColor = [1.0, 0.85, 0.2];
    grid on; xlabel('Time [s]');
    title('Quarter-Car: Slip Ratio & Line Pressure');
    
    % Subplot 3
    ax3 = subplot(2, 2, 3);
    plot(t_f, v_f, 'Color', [0.0, 0.8, 1.0], 'LineWidth', 2.0); hold on;
    plot(t_f, dist_f, 'Color', [0.2, 0.9, 0.4], 'LineWidth', 1.8);
    grid on; xlabel('Time [s]'); ylabel('Speed [km/h] / Distance [m]');
    legend('Speed [km/h]', 'Distance [m]', 'Location', 'northwest', 'TextColor', 'w', 'Color', [0.15 0.15 0.15]);
    title('4-Wheel Vehicle: Velocity & Distance');
    
    % Subplot 4
    ax4 = subplot(2, 2, 4);
    yyaxis left;
    plot(t_f, yaw_f, 'Color', [1.0, 0.3, 0.3], 'LineWidth', 1.8);
    ylabel('Yaw Angle [deg]');
    ax4.YColor = [1.0, 0.3, 0.3];
    yyaxis right;
    plot(t_f, yaw_rt_f, 'Color', [0.2, 0.8, 1.0], 'LineWidth', 1.5);
    ylabel('Yaw Rate [deg/s]');
    ax4.YColor = [0.2, 0.8, 1.0];
    grid on; xlabel('Time [s]');
    title('4-Wheel Vehicle: Split-\mu Yaw Dynamics');
    
    % Apply dark theme styling to all axes
    all_ax = [ax1, ax2, ax3, ax4];
    for a = all_ax
        set(a, 'Color', [0.11, 0.11, 0.11], 'XColor', [0.8, 0.8, 0.8], ...
            'GridColor', [0.3, 0.3, 0.3], 'GridAlpha', 0.5);
        a.Title.Color = [1, 1, 1];
        a.Title.FontWeight = 'bold';
        a.XLabel.Color = [1, 1, 1];
    end
    ax1.YColor = [0.8, 0.8, 0.8];
    ax3.YColor = [0.8, 0.8, 0.8];
    
    fig_path = fullfile(results_dir, 'simulink_simulation_results.png');
    exportgraphics(fig, fig_path, 'Resolution', 150, 'BackgroundColor', 'current');
    close(fig);
    fprintf('4. Telemetry plot saved to: %s\n\n', fig_path);
catch me_fig
    fprintf('   (Plot generation skipped: %s)\n\n', me_fig.message);
end

%% 4. Open GUI if requested
if open_gui
    fprintf('5. Opening block diagrams in Simulink Editor...\n');
    open_system(qc_model);
    open_system(fc_model);
    fprintf('   -> Simulink editor windows launched!\n');
else
    close_system(qc_model, 0);
    close_system(fc_model, 0);
end

fprintf('=================================================================\n');
fprintf('  SIMULINK SIMULATION COMPLETED SUCCESSFULLY!\n');
fprintf('=================================================================\n');

end
