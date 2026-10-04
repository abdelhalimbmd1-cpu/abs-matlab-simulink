function [v_ref, est_state] = estimate_reference_speed(w_wheels, est_state, p, dt)
% ESTIMATE_REFERENCE_SPEED Real-time production vehicle reference velocity estimator
%
% During heavy 4-wheel braking, all wheels experience slip, so true vehicle
% speed cannot be measured directly by any wheel. This estimator reconstructs
% the reference vehicle velocity v_ref by tracking the fastest wheel speed
% while enforcing a maximum physical deceleration gradient constraint.
%
% Inputs:
%   w_wheels  - 4-element vector of measured wheel angular speeds [rad/s]
%   est_state - Estimator state struct (tracks previous v_ref and filtered decel)
%   p         - Parameter struct
%   dt        - Sample time [s]
%
% Outputs:
%   v_ref     - Estimated vehicle longitudinal reference speed [m/s]
%   est_state - Updated estimator state struct

R = p.wheel.R;
vw = w_wheels * R; % Linear peripheral speed of each wheel [m/s]

% Find maximum wheel speed (the wheel with least slip)
vw_max = max(vw);

% Initialize state if needed
if ~isfield(est_state, 'v_ref') || isempty(est_state.v_ref)
    est_state.v_ref      = vw_max;
    est_state.filt_decel = 8.0; % Initial assumed deceleration [m/s^2]
end

v_prev = est_state.v_ref;

% Maximum physically plausible vehicle deceleration slope (dry road limit ~ 1.1 g)
a_decel_max = p.sensor.ref_decel_max; % ~11.0 m/s^2

% If the fastest wheel is moving faster than the previous reference speed,
% update immediately upward (wheels cannot spin faster than the vehicle under braking)
if vw_max >= v_prev
    v_ref = vw_max;
else
    % Wheel speed is below reference speed: ramp down reference speed at estimated slope
    % Calculate allowed deceleration step
    delta_v_allowed = a_decel_max * dt;
    
    % The reference speed cannot drop faster than physical vehicle deceleration
    v_ref = max(vw_max, v_prev - delta_v_allowed);
end

% Prevent negative speed
v_ref = max(0.0, v_ref);
est_state.v_ref = v_ref;

end
