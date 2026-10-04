function [P_cmd, ctrl_state] = controller_pid(slip, P_driver, ctrl_state, p, dt)
% CONTROLLER_PID Velocity-form discrete PID slip tracking controller
%
% Regulates slip to target_slip (~0.16) using an incremental PID algorithm
% that prevents integrator windup, automatically converges to surface-specific
% equilibrium brake pressure, and provides robust closed-loop stability.

target_slip = p.ctrl.pid.target_slip;
error = target_slip - slip;

% Initialize state
if ~isfield(ctrl_state, 'P_cmd')
    ctrl_state.P_cmd = 0.0;
end
if ~isfield(ctrl_state, 'e_prev')
    ctrl_state.e_prev = error;
end
if ~isfield(ctrl_state, 'e_prev2')
    ctrl_state.e_prev2 = error;
end
if ~isfield(ctrl_state, 'active')
    ctrl_state.active = false;
end


% Pre-activation: ramp pressure towards driver request until slip approaches target
if ~ctrl_state.active
    if slip > 0.07 || ctrl_state.P_cmd > 5.0e6
        ctrl_state.active = true;
    else
        ctrl_state.P_cmd = min(P_driver, ctrl_state.P_cmd + 1.2e8 * dt);
        P_cmd = ctrl_state.P_cmd;
        ctrl_state.e_prev  = error;
        ctrl_state.e_prev2 = error;
        return;
    end
end

% Gains for incremental PID
% e > 0 means slip < target -> increase pressure
% e < 0 means slip > target -> decrease pressure
Kp = 4.5e7;  % Proportional gain
Ki = 1.2e8;  % Integral gain
Kd = 6.0e5;  % Derivative gain

% Incremental change in control command:
% delta_P = Kp*(e - e_prev) + Ki*e*dt + Kd*(e - 2*e_prev + e_prev2)/dt
delta_P = Kp * (error - ctrl_state.e_prev) + ...
          Ki * error * dt + ...
          Kd * (error - 2 * ctrl_state.e_prev + ctrl_state.e_prev2) / max(dt, 1e-4);

% Update history
ctrl_state.e_prev2 = ctrl_state.e_prev;
ctrl_state.e_prev  = error;

% Rate-limit the delta to prevent unphysical command spikes
delta_P = max(-p.brake.rate_dump * dt, min(p.brake.rate_build * dt, delta_P));

% Accumulate command
ctrl_state.P_cmd = ctrl_state.P_cmd + delta_P;

% Saturate against driver demand and physical limits
P_cmd = max(p.brake.P_min, min(P_driver, ctrl_state.P_cmd));
ctrl_state.P_cmd = P_cmd;

end
