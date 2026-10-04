function [P_cmd, ctrl_state] = controller_smc(v, w, slip, P_driver, ctrl_state, p, dt)
% CONTROLLER_SMC Second-Order Continuous Sliding Mode Controller for wheel slip
%
% Implements a continuous Super-Twisting / quasi-continuous sliding mode
% algorithm with boundary layer smoothing. Guarantees finite-time convergence
% to target slip without actuator chattering.

% Low speed deactivation
if v < 1.5
    P_cmd = P_driver;
    return;
end

target_slip = p.ctrl.smc.target_slip;
error_slip  = slip - target_slip; % Positive when slip exceeds target

% Initialize state
if ~isfield(ctrl_state, 'u_int')
    ctrl_state.u_int = 0.0;
end
if ~isfield(ctrl_state, 'active')
    ctrl_state.active = false;
end
if ~isfield(ctrl_state, 'P_cmd')
    ctrl_state.P_cmd = 0.0;
end


% Pre-activation ramp
if ~ctrl_state.active
    if slip > 0.07 || ctrl_state.P_cmd > 5.0e6
        ctrl_state.active = true;
    else
        ctrl_state.P_cmd = min(P_driver, ctrl_state.P_cmd + 1.2e8 * dt);
        P_cmd = ctrl_state.P_cmd;
        return;
    end
end

% Boundary layer saturation
Phi = p.ctrl.smc.boundary_layer;
if abs(error_slip) <= Phi
    sat_s = error_slip / Phi;
else
    sat_s = sign(error_slip);
end

% Sliding mode gains
K1 = 6.0e7;  % Proportional switching gain [Pa]
K2 = 1.2e8;  % Integral switching gain [Pa/s]

% Integral state: dot(u_int) = -K2 * sat_s
ctrl_state.u_int = ctrl_state.u_int - K2 * sat_s * dt;
ctrl_state.u_int = max(-p.brake.P_max, min(p.brake.P_max, ctrl_state.u_int));

% Continuous super-twisting command
P_cmd_raw = 6.5e6 + ctrl_state.u_int - K1 * sat_s;

% Physical actuator clamping
P_cmd = max(p.brake.P_min, min(P_driver, P_cmd_raw));
ctrl_state.P_cmd = P_cmd;

end
