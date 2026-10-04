function sim_out = sim_full_car(scenario, controller_type, p)
% SIM_FULL_CAR 4-wheel full vehicle ABS simulation with dynamic pitch load transfer,
% 4 independent hydraulic brake channels, split-mu friction, and select-low yaw control.
%
% Inputs:
%   scenario        - Struct:
%                     .v0          - Initial speed [m/s]
%                     .surface_L   - Left road surface ('dry', 'wet', 'snow', 'ice')
%                     .surface_R   - Right road surface ('dry', 'wet', 'snow', 'ice')
%                     .driver_type - 'step', 'ramp', 'panic'
%                     .use_ymc     - Enable select-low yaw moment control (default true)
%                     .t_end       - Max duration [s]
%                     .dt          - Integration step [s] (default 0.0005)
%   controller_type - 'none', 'rule_based', 'pid', 'smc', 'adaptive'
%   p               - Parameter struct from init_params()
%
% Outputs:
%   sim_out         - Complete 4-wheel time histories, yaw response, stopping metrics.

if nargin < 3 || isempty(p)
    p = init_params();
end
if nargin < 2 || isempty(controller_type)
    controller_type = 'rule_based';
end

v0          = scenario.v0;
driver_type = 'step';
if isfield(scenario, 'driver_type'), driver_type = scenario.driver_type; end
t_end = 6.0;
if isfield(scenario, 't_end'), t_end = scenario.t_end; end
dt = 0.0005;
if isfield(scenario, 'dt'), dt = scenario.dt; end

use_ymc = true;
if isfield(scenario, 'use_ymc'), use_ymc = scenario.use_ymc; end

% Surface mapping
surf_L = scenario.surface_L;
surf_R = scenario.surface_R;

tire_L = p.tire.(surf_L);
tire_R = p.tire.(surf_R);

% Time vector
t_vec = 0:dt:t_end;
N_steps = length(t_vec);

% Preallocate output arrays
% Wheel indices: 1: FL, 2: FR, 3: RL, 4: RR
t_hist      = zeros(N_steps, 1);
v_hist      = zeros(N_steps, 1);
dist_hist   = zeros(N_steps, 1);
ax_hist     = zeros(N_steps, 1);
yaw_rate    = zeros(N_steps, 1); % [rad/s]
yaw_angle   = zeros(N_steps, 1); % [rad]
Mz_hist     = zeros(N_steps, 1); % Yaw moment [N*m]

w_hist      = zeros(N_steps, 4);
vw_hist     = zeros(N_steps, 4);
slip_hist   = zeros(N_steps, 4);
Fx_hist     = zeros(N_steps, 4);
Fz_hist     = zeros(N_steps, 4);
P_cmd_hist  = zeros(N_steps, 4);
P_act_hist  = zeros(N_steps, 4);

% Initial states
v       = v0;
x       = 0.0;
ax      = 0.0;
w       = (v0 / p.wheel.R) * ones(4, 1);
P_act   = zeros(4, 1);
psi_dot = 0.0; % Yaw rate
psi     = 0.0; % Yaw angle

% Actuator delay buffers for 4 channels
n_delay_steps = max(1, round(p.brake.delay_act / dt));
delay_buf     = zeros(n_delay_steps, 4);
buf_idx       = 1;

% 4 Independent Controller instances
ctrl_states = cell(4, 1);
for i = 1:4
    ctrl_states{i} = struct();
    ctrl_states{i}.phase = 0;
    ctrl_states{i}.active = false;
    ctrl_states{i}.t_phase = 0;
    ctrl_states{i}.P_cmd = 0;
    ctrl_states{i}.P_lock = 7.0e6;
    ctrl_states{i}.P_hold = 0;
    ctrl_states{i}.w_prev = w(i);
end

stopped  = false;
stop_idx = N_steps;

alpha_f = dt / (dt + 1.0 / p.sensor.filter_omega);
w_dot   = zeros(4, 1);

for k = 1:N_steps
    t = t_vec(k);
    
    % Driver demand
    t_press = 0.10;
    if t >= t_press
        switch driver_type
            case 'step'
                P_drv = p.brake.P_max;
            case 'ramp'
                P_drv = min(p.brake.P_max, p.brake.P_max * (t - t_press) / 0.5);
            case 'panic'
                P_drv = min(p.brake.P_max, p.brake.P_max * (t - t_press) / 0.08);
            otherwise
                P_drv = p.brake.P_max;
        end
    else
        P_drv = 0.0;
    end
    
    % Front/Rear brake bias
    % Front calipers receive up to P_drv, rear receive bias proportion
    P_drv_f = P_drv;
    P_drv_r = P_drv * (1 - p.brake.front_bias) / p.brake.front_bias; % Balanced hydraulic bias
    P_drv_wheels = [P_drv_f; P_drv_f; P_drv_r; P_drv_r];
    
    % Dynamic Normal Loads with Longitudinal Pitch Load Transfer
    % Under braking (ax < 0), load transfers to front axle
    L  = p.veh.wheelbase;
    m  = p.veh.mass_total;
    g  = p.veh.g;
    h  = p.veh.h_cog;
    a  = p.veh.a;
    b  = p.veh.b;
    
    % Pitch load transfer: delta_Fz = (m * |ax| * h) / L
    % When ax is negative (braking), front load increases, rear load decreases
    Fz_front_total = max(500, (m * g * b - m * ax * h) / L);
    Fz_rear_total  = max(500, (m * g * a + m * ax * h) / L);
    
    Fz_wheels = [Fz_front_total / 2; ... % FL
                 Fz_front_total / 2; ... % FR
                 Fz_rear_total  / 2; ... % RL
                 Fz_rear_total  / 2];    % RR
             
    % Wheel speeds, slip, and wheel accelerations
    vw = w * p.wheel.R;
    slip = (v - vw) ./ max(v, p.wheel.eps_v);
    slip = max(0.0, min(1.0, slip));
    
    for i = 1:4
        w_raw_dot = (w(i) - ctrl_states{i}.w_prev) / dt;
        w_dot(i)  = (1 - alpha_f) * w_dot(i) + alpha_f * w_raw_dot;
        ctrl_states{i}.w_prev = w(i);
    end
    aw = w_dot * p.wheel.R;
    
    % Controller execution per wheel
    P_cmd_raw = zeros(4, 1);
    if stopped || t < t_press
        P_cmd_raw = P_drv_wheels;
    else
        switch lower(controller_type)
            case {'none', 'no_abs'}
                P_cmd_raw = P_drv_wheels;
                
            case 'rule_based'
                for i = 1:4
                    [P_cmd_raw(i), ctrl_states{i}] = controller_rule_based(...
                        v, w(i), aw(i), slip(i), P_drv_wheels(i), P_act(i), ctrl_states{i}, p, dt);
                end
                
            case 'pid'
                for i = 1:4
                    [P_cmd_raw(i), ctrl_states{i}] = controller_pid(...
                        slip(i), P_drv_wheels(i), ctrl_states{i}, p, dt);
                end
                
            case 'smc'
                for i = 1:4
                    [P_cmd_raw(i), ctrl_states{i}] = controller_smc(...
                        v, w(i), slip(i), P_drv_wheels(i), ctrl_states{i}, p, dt);
                end
                
            case 'adaptive'
                for i = 1:4
                    [P_cmd_raw(i), ctrl_states{i}] = controller_adaptive(...
                        v, w(i), aw(i), slip(i), P_drv_wheels(i), ctrl_states{i}, p, dt);
                end
                
            otherwise
                error('Unknown controller: %s', controller_type);
        end
    end
    
    % Select-Low and Yaw Moment Control (YMC) for Split-mu Stability
    P_cmd = P_cmd_raw;
    if use_ymc && strcmp(controller_type, 'rule_based') || (use_ymc && ~strcmp(controller_type, 'none'))
        % 1. Rear Axle Select-Low: lock rear pressures to the lowest wheel command
        % to prevent rear lateral breakaway and maintain directional stability
        P_rear_select_low = min(P_cmd(3), P_cmd(4));
        P_cmd(3) = P_rear_select_low;
        P_cmd(4) = P_rear_select_low;
        
        % 2. Front Axle Yaw Moment Limitation (YML):
        % Restrict the maximum left-to-right front brake pressure difference
        delta_P_front = P_cmd(1) - P_cmd(2);
        max_delta_P   = p.ctrl.split_mu.max_delta_P;
        if abs(delta_P_front) > max_delta_P
            if delta_P_front > 0
                % Left is higher: cap left front pressure
                P_cmd(1) = P_cmd(2) + max_delta_P;
            else
                % Right is higher: cap right front pressure
                P_cmd(2) = P_cmd(1) + max_delta_P;
            end
        end
    end
    
    % Actuator Dynamics for 4 Channels
    delay_buf(buf_idx, :) = P_cmd';
    buf_idx = mod(buf_idx, n_delay_steps) + 1;
    P_delayed = delay_buf(buf_idx, :)';
    
    for i = 1:4
        dP_des = (P_delayed(i) - P_act(i)) / p.brake.tau_act;
        if dP_des > 0
            dP = min(dP_des, p.brake.rate_build);
        else
            dP = max(dP_des, -p.brake.rate_dump);
        end
        P_act(i) = max(p.brake.P_min, min(p.brake.P_max, P_act(i) + dP * dt));
    end
    
    Tb = p.brake.Kb * P_act;
    
    % Tire forces calculation
    % Left side (FL, RL) on surf_L; Right side (FR, RR) on surf_R
    tire_wheels = {tire_L, tire_R, tire_L, tire_R};
    mu = zeros(4, 1);
    Fx = zeros(4, 1);
    for i = 1:4
        [mu(i), Fx(i)] = pacejka_tire(slip(i), Fz_wheels(i), tire_wheels{i});
    end
    
    % Vehicle Equations of Motion:
    % 1. Total longitudinal force: Fx_total = sum(Fx)
    Fx_total = sum(Fx);
    
    % 2. Yaw Moment due to asymmetric braking forces:
    % Track width d = 1.55 m
    % Mz = ((Fx_FL + Fx_RL) - (Fx_FR + Fx_RR)) * (d / 2)
    % Positive Mz yaws vehicle to the left
    d_track = p.veh.track_width;
    Mz = ((Fx(1) + Fx(3)) - (Fx(2) + Fx(4))) * (d_track / 2);
    
    if ~stopped
        ax = -Fx_total / m;
        v  = v + ax * dt;
        x  = x + v * dt;
        
        % Yaw integration
        psi_ddot = Mz / p.veh.Iz;
        psi_dot  = psi_dot + psi_ddot * dt;
        psi      = psi + psi_dot * dt;
        
        % Wheel spin integration
        for i = 1:4
            dw_i = (-Tb(i) + Fx(i) * p.wheel.R) / p.wheel.Jw;
            w(i) = max(0.0, w(i) + dw_i * dt);
        end
        
        if v <= p.wheel.v_stop
            stopped = true;
            v = 0;
            w(:) = 0;
            ax = 0;
            psi_dot = 0;
            stop_idx = k;
        end
    else
        ax = 0;
        v = 0;
        w(:) = 0;
        Fx(:) = 0;
        Mz = 0;
        Tb(:) = 0;
    end
    
    % Log histories
    t_hist(k)       = t;
    v_hist(k)       = v;
    dist_hist(k)    = x;
    ax_hist(k)      = ax;
    yaw_rate(k)     = psi_dot;
    yaw_angle(k)    = psi;
    Mz_hist(k)      = Mz;
    w_hist(k, :)    = w';
    vw_hist(k, :)   = vw';
    slip_hist(k, :) = slip';
    Fx_hist(k, :)   = Fx';
    Fz_hist(k, :)   = Fz_wheels';
    P_cmd_hist(k,:) = P_cmd';
    P_act_hist(k,:) = P_act';
    
    if stopped && (k - stop_idx > round(0.10 / dt))
        break;
    end
end

idx_range = 1:min(k, N_steps);

sim_out.t         = t_hist(idx_range);
sim_out.v         = v_hist(idx_range);
sim_out.dist      = dist_hist(idx_range);
sim_out.ax        = ax_hist(idx_range);
sim_out.yaw_rate  = yaw_rate(idx_range);
sim_out.yaw_angle = yaw_angle(idx_range);
sim_out.Mz        = Mz_hist(idx_range);
sim_out.w         = w_hist(idx_range, :);
sim_out.vw        = vw_hist(idx_range, :);
sim_out.slip      = slip_hist(idx_range, :);
sim_out.Fx        = Fx_hist(idx_range, :);
sim_out.Fz        = Fz_hist(idx_range, :);
sim_out.P_cmd     = P_cmd_hist(idx_range, :);
sim_out.P_act     = P_act_hist(idx_range, :);
sim_out.stop_time = t_hist(stop_idx);
sim_out.stop_dist = dist_hist(stop_idx);
sim_out.v0        = v0;
sim_out.surface   = surf_L;
sim_out.max_yaw   = max(abs(yaw_angle(idx_range)));
sim_out.max_yaw_rate = max(abs(yaw_rate(idx_range)));
sim_out.controller = controller_type;

end

