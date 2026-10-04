function p = init_params()
% INIT_PARAMS Initializes all vehicle, tire, brake actuator, road, and controller parameters
% for the Anti-lock Braking System (ABS) simulation.
%
% All units are strictly SI:
%   - Mass: kg
%   - Length / Radius / Height: m
%   - Inertia: kg*m^2
%   - Velocity: m/s
%   - Force: N
%   - Pressure: Pa (1 bar = 1e5 Pa)
%   - Torque: N*m
%   - Time: s

%% 1. Vehicle Parameters (C-segment Passenger Sedan)
p.veh.mass_total   = 1600;          % Total vehicle curb mass [kg]
p.veh.mass_quarter = p.veh.mass_total / 4; % Quarter-car equivalent mass [kg] (400 kg)
p.veh.wheelbase    = 2.60;          % Wheelbase L = a + b [m]
p.veh.a            = 1.15;          % Distance from CoG to front axle [m]
p.veh.b            = 1.45;          % Distance from CoG to rear axle [m]
p.veh.track_width  = 1.55;          % Track width d (lateral distance between left & right wheels) [m]
p.veh.h_cog        = 0.50;          % Height of Center of Gravity [m]
p.veh.Iz           = 2400;          % Vehicle yaw moment of inertia [kg*m^2]
p.veh.g            = 9.81;          % Gravitational acceleration [m/s^2]

% Static axle loads
p.veh.Fz_static_front_axle = p.veh.mass_total * p.veh.g * (p.veh.b / p.veh.wheelbase); % [N] (~9643 N total, ~4821 N/wheel)
p.veh.Fz_static_rear_axle  = p.veh.mass_total * p.veh.g * (p.veh.a / p.veh.wheelbase); % [N] (~6053 N total, ~3026 N/wheel)
p.veh.Fz_quarter_static    = p.veh.mass_quarter * p.veh.g;                             % [N] (3924 N)

%% 2. Wheel & Tire Physical Parameters
p.wheel.R      = 0.32;              % Effective rolling radius [m] (205/55 R16)
p.wheel.Jw     = 1.20;              % Wheel rotational inertia (tire + rim + rotor) [kg*m^2]
p.wheel.eps_v  = 0.10;              % Small velocity threshold to prevent division by zero in slip [m/s]
p.wheel.v_stop = 0.50;              % Vehicle stop velocity threshold [m/s]

%% 3. Tire Friction Models (Pacejka Magic Formula - Calibrated to SAE Standards)
% Formula: mu(lambda) = D * sin(C * atan(B*lambda - E*(B*lambda - atan(B*lambda))))

% 3.1 Dry Asphalt (Peak mu ~ 1.00 at lambda=0.18, Locked mu ~ 0.76)
p.tire.dry.B = 10.0;
p.tire.dry.C = 1.60;
p.tire.dry.D = 1.00;
p.tire.dry.E = 0.40;
p.tire.dry.opt_slip = 0.18;

% 3.2 Wet Asphalt (Peak mu ~ 0.75 at lambda=0.15, Locked mu ~ 0.58)
p.tire.wet.B = 12.0;
p.tire.wet.C = 1.55;
p.tire.wet.D = 0.75;
p.tire.wet.E = 0.30;
p.tire.wet.opt_slip = 0.15;

% 3.3 Packed Snow (Peak mu ~ 0.30 at lambda=0.13, Locked mu ~ 0.24)
p.tire.snow.B = 15.0;
p.tire.snow.C = 1.50;
p.tire.snow.D = 0.30;
p.tire.snow.E = 0.20;
p.tire.snow.opt_slip = 0.13;

% 3.4 Smooth Ice (Peak mu ~ 0.15 at lambda=0.10, Locked mu ~ 0.12)
p.tire.ice.B = 20.0;
p.tire.ice.C = 1.45;
p.tire.ice.D = 0.15;
p.tire.ice.E = 0.10;
p.tire.ice.opt_slip = 0.10;

%% 4. Brake Actuator Parameters (Electro-Hydraulic Brake Unit)
p.brake.P_max       = 15.0e6;       % Maximum hydraulic line pressure [Pa] (150 bar)
p.brake.P_min       = 0.0;          % Minimum hydraulic line pressure [Pa]
p.brake.Kb          = 1.8e-4;       % Brake torque constant [N*m / Pa] (18 Nm / bar)
p.brake.tau_act     = 0.025;        % Hydraulic first-order lag time constant [s] (25 ms)
p.brake.delay_act   = 0.005;        % Pure transport delay [s] (5 ms)
p.brake.rate_build  = 8.0e7;        % Maximum pressure build rate [Pa/s] (800 bar/s)
p.brake.rate_dump   = 1.2e8;        % Maximum pressure dump rate [Pa/s] (1200 bar/s)
p.brake.front_bias  = 0.65;         % Front/Rear brake torque bias ratio (65% front, 35% rear)

%% 5. Sensor & Estimator Parameters
p.sensor.encoder_teeth = 48;        % Wheel speed tone ring teeth count
p.sensor.sample_time   = 0.005;     % ABS ECU sample period [s] (200 Hz / 5 ms)
p.sensor.noise_sigma_w = 0.25;      % Wheel speed measurement noise std dev [rad/s]
p.sensor.filter_omega  = 150.0;     % Wheel deceleration derivative filter cutoff [rad/s]
p.sensor.ref_decel_max = 11.0;      % Maximum plausible vehicle deceleration for estimator [m/s^2]

%% 6. Controller Parameters
% 6.1 Rule-Based Threshold Controller (Bosch 3-phase logic)
p.ctrl.rb.decel_release_thresh = -20.0; % Wheel decel threshold to dump pressure [m/s^2]
p.ctrl.rb.slip_release_thresh  = 0.19;  % Longitudinal slip threshold to dump pressure
p.ctrl.rb.accel_hold_thresh    = 4.0;   % Wheel re-accel threshold to hold pressure [m/s^2]
p.ctrl.rb.accel_build_thresh   = 1.5;   % Wheel accel drop threshold to resume pressure build [m/s^2]
p.ctrl.rb.slip_build_thresh    = 0.14;  % Longitudinal slip threshold below which build is allowed

% 6.2 PID Slip Controller
p.ctrl.pid.target_slip = 0.16;      % Universal target slip
p.ctrl.pid.Kp          = 4.5e7;     % Proportional gain [Pa]
p.ctrl.pid.Ki          = 1.2e8;     % Integral gain [Pa/s]
p.ctrl.pid.Kd          = 6.0e5;     % Derivative gain [Pa*s]
p.ctrl.pid.N_filter    = 60.0;      % Derivative filter coefficient

% 6.3 Sliding Mode Slip Controller (SMC)
p.ctrl.smc.target_slip = 0.16;
p.ctrl.smc.K1          = 6.0e7;     % Switching gain [Pa]
p.ctrl.smc.K2          = 1.2e8;     % Integral gain [Pa/s]
p.ctrl.smc.boundary_layer = 0.035;  % Boundary layer thickness phi

% 6.4 Adaptive / Estimator Target Slip
p.ctrl.adapt.default_target = 0.16;
p.ctrl.adapt.mu_filter_tc   = 0.05; % Filter time constant for friction estimation [s]

% 6.5 Split-mu Select-Low Yaw Moment Control
p.ctrl.split_mu.enabled        = true;
p.ctrl.split_mu.max_delta_P    = 3.5e6; % Max allowable front L/R pressure difference [Pa] (35 bar)
p.ctrl.split_mu.build_rate_lim = 3.0e7; % Restricted build rate on high-mu side during split-mu [Pa/s]

end
