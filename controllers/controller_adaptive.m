function [P_cmd, ctrl_state] = controller_adaptive(v, w, a_wheel, slip, P_driver, ctrl_state, p, dt)
% CONTROLLER_ADAPTIVE Adaptive target-slip ABS controller with road friction estimation
%
% Continuously estimates instantaneous road surface friction mu_est based on vehicle
% deceleration and wheel dynamics. Dynamically adapts the target slip ratio lambda_target
% and feedforward equilibrium pressure to match the surface's true peak grip location:
%   - Ice  (mu ~ 0.1): lambda_target ~ 0.08
%   - Snow (mu ~ 0.3): lambda_target ~ 0.11
%   - Wet  (mu ~ 0.7): lambda_target ~ 0.14
%   - Dry  (mu ~ 1.0): lambda_target ~ 0.17

% 1. Friction Estimation via filtered deceleration
if ~isfield(ctrl_state, 'filt_ax')
    ctrl_state.filt_ax = 5.0; % Initial guess
    ctrl_state.adapt_int = 0.0;
end

% Estimate deceleration from wheel and vehicle speed difference rate
if ~isfield(ctrl_state, 'v_prev')
    ctrl_state.v_prev = v;
end
ax_raw = -(v - ctrl_state.v_prev) / dt;
ctrl_state.v_prev = v;

% Filter deceleration to remove high-frequency noise
alpha_mu = dt / (dt + p.ctrl.adapt.mu_filter_tc);
ctrl_state.filt_ax = (1 - alpha_mu) * ctrl_state.filt_ax + alpha_mu * max(0.2, ax_raw);

mu_est = ctrl_state.filt_ax / p.veh.g;
mu_est = max(0.10, min(1.10, mu_est));
ctrl_state.mu_est = mu_est;

% 2. Adaptive Target Slip Mapping
% Linear interpolation between ice (0.08) and dry (0.17)
lambda_target = 0.08 + 0.09 * ((mu_est - 0.10) / 0.90);
lambda_target = max(0.07, min(0.18, lambda_target));
ctrl_state.target_slip_current = lambda_target;

% 3. Adaptive Control Law
error = lambda_target - slip;

% Adapt gains according to estimated friction (higher gains on dry, gentler on ice)
gain_scale = 0.5 + 0.5 * (mu_est / 1.0);
Kp = p.ctrl.pid.Kp * gain_scale;
Ki = p.ctrl.pid.Ki * gain_scale;

% Adapted feedforward equilibrium pressure
Fz_q = p.veh.Fz_quarter_static;
P_eq = (mu_est * Fz_q * p.wheel.R) / p.brake.Kb;

% Integrator with anti-windup
ctrl_state.adapt_int = ctrl_state.adapt_int + Ki * error * dt;
ctrl_state.adapt_int = max(-0.5 * P_eq, min(0.5 * P_eq, ctrl_state.adapt_int));

P_cmd_raw = P_eq + Kp * error + ctrl_state.adapt_int;

% Saturation against driver demand and limits
P_max_allowed = min(p.brake.P_max, P_driver);
P_cmd = max(p.brake.P_min, min(P_max_allowed, P_cmd_raw));

end
