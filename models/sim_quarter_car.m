function sim_out = sim_quarter_car(scenario, controller_type, p)
% SIM_QUARTER_CAR High-fidelity quarter-car ABS simulation
%
% Inputs:
%   scenario        - Struct containing initial conditions and road surface:
%                     .v0          - Initial speed [m/s] (e.g. 100 km/h = 27.78 m/s)
%                     .surface     - Road surface name: 'dry', 'wet', 'snow', 'ice', or 'transition'
%                     .driver_type - 'step', 'ramp', or 'panic'
%                     .t_end       - Max simulation duration [s] (default 10s)
%                     .dt          - Integration step [s] (default 0.0005s)
%   controller_type - 'none', 'rule_based', 'pid', 'smc', 'adaptive'
%   p               - Parameter struct from init_params()
%
% Output:
%   sim_out         - Struct with complete time histories and performance metrics

if nargin < 3 || isempty(p)
    p = init_params();
end
if nargin < 2 || isempty(controller_type)
    controller_type = 'none';
end

% Extract scenario settings
v0          = scenario.v0;
surface     = scenario.surface;
driver_type = 'step';
if isfield(scenario, 'driver_type')
    driver_type = scenario.driver_type;
end
t_end = 10.0;
if isfield(scenario, 't_end')
    t_end = scenario.t_end;
end
dt = 0.0005; % 0.5 ms high-resolution integration step
if isfield(scenario, 'dt')
    dt = scenario.dt;
end

% Time vector
t_vec = 0:dt:t_end;
N_steps = length(t_vec);

% Preallocate output arrays
t_hist      = zeros(N_steps, 1);
v_hist      = zeros(N_steps, 1);
w_hist      = zeros(N_steps, 1);
vw_hist     = zeros(N_steps, 1);
a_veh_hist  = zeros(N_steps, 1);
a_whl_hist  = zeros(N_steps, 1);
slip_hist   = zeros(N_steps, 1);
mu_hist     = zeros(N_steps, 1);
Fx_hist     = zeros(N_steps, 1);
dist_hist   = zeros(N_steps, 1);
P_cmd_hist  = zeros(N_steps, 1);
P_act_hist  = zeros(N_steps, 1);
Tb_hist     = zeros(N_steps, 1);
phase_hist  = zeros(N_steps, 1); % Controller internal phase (for rule-based)

% Initial states
v   = v0;
w   = v0 / p.wheel.R; % Pure rolling initially
x   = 0.0;
P   = 0.0;            % Initial brake pressure
w_filt = w;
w_dot  = 0.0;

% Actuator delay buffer (circular FIFO buffer)
n_delay_steps = max(1, round(p.brake.delay_act / dt));
delay_buffer  = zeros(n_delay_steps, 1);
buf_idx       = 1;

% Controller state structures
ctrl_state = struct();
ctrl_state.phase = 0; % 0: Build, 1: Hold, 2: Dump
ctrl_state.pid_int = 0;
ctrl_state.pid_prev_err = 0;
ctrl_state.filt_w_dot = 0;
ctrl_state.w_prev = w;
ctrl_state.mu_est = 0.8;
ctrl_state.t_last_action = 0;

% Stopping flag
stopped = false;
stop_idx = N_steps;

% Pre-filter constant for numerical differentiation of wheel deceleration
alpha_f = dt / (dt + 1.0 / p.sensor.filter_omega);

for k = 1:N_steps
    t = t_vec(k);
    
    % Determine road surface at current time (support mid-braking transitions)
    if ischar(surface)
        if strcmp(surface, 'transition')
            % Transition dry -> ice at t = 1.0s
            if t < 1.0
                tire_p = p.tire.dry;
            else
                tire_p = p.tire.ice;
            end
        elseif strcmp(surface, 'transition_wet_dry')
            % Transition wet -> dry at t = 1.0s
            if t < 1.0
                tire_p = p.tire.wet;
            else
                tire_p = p.tire.dry;
            end
        else
            tire_p = p.tire.(surface);
        end
    elseif isstruct(surface)
        tire_p = surface;
    end
    
    % Driver brake demand P_driver (commanded brake pressure)
    t_press = 0.10; % Brake pedal press time [s]
    switch driver_type
        case 'step'
            if t >= t_press
                P_driver = p.brake.P_max;
            else
                P_driver = 0.0;
            end
        case 'ramp'
            if t >= t_press
                P_driver = min(p.brake.P_max, p.brake.P_max * (t - t_press) / 0.5);
            else
                P_driver = 0.0;
            end
        case 'panic'
            % Fast rise to max pressure
            if t >= t_press
                P_driver = min(p.brake.P_max, p.brake.P_max * (t - t_press) / 0.08);
            else
                P_driver = 0.0;
            end
        otherwise
            P_driver = p.brake.P_max;
    end
    
    % Wheel and vehicle kinematic calculations
    v_wheel = w * p.wheel.R;
    slip = (v - v_wheel) / max(v, p.wheel.eps_v);
    slip = max(0, min(1.0, slip)); % Bounded [0, 1]
    
    % Wheel deceleration computation (filtered)
    w_raw_dot = (w - ctrl_state.w_prev) / dt;
    w_dot = (1 - alpha_f) * w_dot + alpha_f * w_raw_dot;
    ctrl_state.w_prev = w;
    a_wheel = w_dot * p.wheel.R; % Linear deceleration equivalent [m/s^2]
    
    % Execute controller to determine commanded brake pressure P_cmd
    if stopped || t < t_press
        P_cmd = P_driver;
        c_phase = 0;
    else
        switch lower(controller_type)
            case {'none', 'no_abs'}
                P_cmd = P_driver;
                c_phase = 0;
                
            case 'rule_based'
                [P_cmd, ctrl_state] = controller_rule_based(v, w, a_wheel, slip, P_driver, P, ctrl_state, p, dt);
                c_phase = ctrl_state.phase;
                
            case 'pid'
                [P_cmd, ctrl_state] = controller_pid(slip, P_driver, ctrl_state, p, dt);
                c_phase = 1;
                
            case 'smc'
                [P_cmd, ctrl_state] = controller_smc(v, w, slip, P_driver, ctrl_state, p, dt);
                c_phase = 1;
                
            case 'adaptive'
                [P_cmd, ctrl_state] = controller_adaptive(v, w, a_wheel, slip, P_driver, ctrl_state, p, dt);
                c_phase = 1;
                
            otherwise
                error('Unknown controller type: %s', controller_type);
        end
    end
    
    % Actuator Dynamics:
    % 1. Pure transport delay via FIFO buffer
    delay_buffer(buf_idx) = P_cmd;
    buf_idx = mod(buf_idx, n_delay_steps) + 1;
    P_delayed = delay_buffer(buf_idx);
    
    % 2. First-order lag with asymmetric rate limits (dump rate vs build rate)
    dP_des = (P_delayed - P) / p.brake.tau_act;
    if dP_des > 0
        dP_actual = min(dP_des, p.brake.rate_build);
    else
        dP_actual = max(dP_des, -p.brake.rate_dump);
    end
    P = P + dP_actual * dt;
    
    % 3. Actuator saturation
    P = max(p.brake.P_min, min(p.brake.P_max, P));
    
    % Brake Torque
    Tb = p.brake.Kb * P;
    
    % Tire Force (Pacejka Magic Formula)
    Fz = p.veh.Fz_quarter_static;
    [mu, Fx] = pacejka_tire(slip, Fz, tire_p);
    
    % Equations of Motion:
    % Quarter car vehicle longitudinal: m * dv/dt = -Fx
    % Quarter car wheel rotation: J * dw/dt = -Tb + Fx * R
    if ~stopped
        dv_dt = -Fx / p.veh.mass_quarter;
        dw_dt = (-Tb + Fx * p.wheel.R) / p.wheel.Jw;
        
        % Integration (Forward Euler at high resolution 0.5 ms)
        v = v + dv_dt * dt;
        w = w + dw_dt * dt;
        x = x + v * dt;
        
        % Prevent wheel from spinning backward under braking
        if w < 0
            w = 0;
        end
        
        % Check stopping criterion
        if v <= p.wheel.v_stop
            stopped = true;
            v = 0;
            w = 0;
            dv_dt = 0;
            dw_dt = 0;
            stop_idx = k;
        end
    else
        dv_dt = 0;
        dw_dt = 0;
        v = 0;
        w = 0;
        Fx = 0;
        mu = 0;
        Tb = 0;
    end
    
    % Log states and signals
    t_hist(k)     = t;
    v_hist(k)     = v;
    w_hist(k)     = w;
    vw_hist(k)    = v_wheel;
    a_veh_hist(k) = dv_dt;
    a_whl_hist(k) = a_wheel;
    slip_hist(k)  = slip;
    mu_hist(k)    = mu;
    Fx_hist(k)    = Fx;
    dist_hist(k)  = x;
    P_cmd_hist(k) = P_cmd;
    P_act_hist(k) = P;
    Tb_hist(k)    = Tb;
    phase_hist(k) = c_phase;
    
    % If stopped, break early after a short post-stop dwell (0.1s)
    if stopped && (k - stop_idx > round(0.10 / dt))
        break;
    end
end

% Truncate history to actual simulation duration
actual_len = min(k, N_steps);
idx_range = 1:actual_len;

sim_out.t         = t_hist(idx_range);
sim_out.v         = v_hist(idx_range);
sim_out.w         = w_hist(idx_range);
sim_out.vw        = vw_hist(idx_range);
sim_out.a_veh     = a_veh_hist(idx_range);
sim_out.a_whl     = a_whl_hist(idx_range);
sim_out.slip      = slip_hist(idx_range);
sim_out.mu        = mu_hist(idx_range);
sim_out.Fx        = Fx_hist(idx_range);
sim_out.dist      = dist_hist(idx_range);
sim_out.P_cmd     = P_cmd_hist(idx_range);
sim_out.P_act     = P_act_hist(idx_range);
sim_out.Tb        = Tb_hist(idx_range);
sim_out.phase     = phase_hist(idx_range);
sim_out.stop_time = t_hist(stop_idx);
sim_out.stop_dist = dist_hist(stop_idx);
sim_out.v0        = v0;
sim_out.surface   = surface;
sim_out.controller = controller_type;

end
