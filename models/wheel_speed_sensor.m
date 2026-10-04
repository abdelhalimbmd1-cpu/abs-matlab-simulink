function w_meas = wheel_speed_sensor(w_true, p, dt)
% WHEEL_SPEED_SENSOR Models a production Hall-effect / Variable Reluctance
% wheel speed tone ring sensor with pulse quantization, noise, and sampling.
%
% Inputs:
%   w_true - True wheel angular speed [rad/s] (vector or scalar)
%   p      - Parameter struct
%   dt     - Simulation time step [s]
%
% Output:
%   w_meas - Measured / digitized wheel angular speed [rad/s]

teeth = p.sensor.encoder_teeth; % e.g. 48 teeth
sigma = p.sensor.noise_sigma_w; % e.g. 0.25 rad/s (~0.28 km/h noise)

% Angular resolution per tooth
delta_theta = (2 * pi) / teeth;

% In a high-precision timer ECU (measuring tooth transit period t_period = delta_theta / w),
% quantization is equivalent to timer clock discretization (typically 1 us clock)
% For simulation, we discretize at the effective speed resolution (~0.05 rad/s)
w_quant_step = 0.05; % [rad/s] (~0.058 km/h)
w_quant = round(w_true / w_quant_step) * w_quant_step;

% Add Gaussian sensor noise
w_noise = sigma * randn(size(w_true));

w_meas = max(0.0, w_quant + w_noise);

end
