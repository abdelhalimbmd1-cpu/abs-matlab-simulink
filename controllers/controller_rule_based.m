function [P_cmd, ctrl_state] = controller_rule_based(v, w, a_wheel, slip, P_driver, P_act, ctrl_state, p, dt)
% CONTROLLER_RULE_BASED Industrial Bosch 3-phase threshold ABS controller
%
% Accurately models hydraulic modulation:
%   - Phase 0: BUILD (Pressure ramps up towards peak grip)
%   - Phase 1: HOLD  (Maintains pressure while wheel accelerates back)
%   - Phase 2: DUMP  (Releases pressure to relieve lockup without losing deceleration)

% Low speed deactivation (allow full lockup below 1.5 m/s for final vehicle stop)
if v < 1.5
    P_cmd = P_driver;
    ctrl_state.phase = 0;
    return;
end

% Ensure all fields exist
if ~isfield(ctrl_state, 't_phase')
    ctrl_state.phase   = 0;
    ctrl_state.P_cmd   = 0;
    ctrl_state.P_lock  = 7.0e6;
    ctrl_state.P_hold  = 0;
    ctrl_state.t_phase = 0;
    ctrl_state.active  = false;
end

phase = ctrl_state.phase;
ctrl_state.t_phase = ctrl_state.t_phase + dt;

% Combined lockup detection
is_locking = (slip > 0.19) || (slip > 0.09 && a_wheel < -25.0);

if ~ctrl_state.active
    if is_locking
        ctrl_state.active  = true;
        phase              = 2; % Jump to DUMP
        ctrl_state.P_lock  = P_act;
        ctrl_state.P_hold  = 0.78 * P_act;
        ctrl_state.P_cmd   = ctrl_state.P_hold;
        ctrl_state.t_phase = 0;
    else
        ctrl_state.P_cmd = P_driver;
        P_cmd = ctrl_state.P_cmd;
        return;
    end
end

% State Transitions
switch phase
    case 2 % DUMP PHASE
        if (a_wheel > 3.0) || (slip < 0.15 && a_wheel > 0) || (ctrl_state.t_phase > 0.030)
            phase = 1; % Switch to HOLD
            ctrl_state.P_hold  = ctrl_state.P_cmd;
            ctrl_state.t_phase = 0;
        end
        
    case 1 % HOLD PHASE
        if ((a_wheel < 1.5) && (slip < 0.14)) || (ctrl_state.t_phase > 0.025)
            phase = 0; % Switch to BUILD
            ctrl_state.t_phase = 0;
        elseif is_locking
            phase = 2; % Re-entering lockup
            ctrl_state.P_lock  = P_act;
            ctrl_state.P_hold  = 0.78 * P_act;
            ctrl_state.P_cmd   = ctrl_state.P_hold;
            ctrl_state.t_phase = 0;
        end
        
    case 0 % BUILD PHASE
        if is_locking
            phase = 2; % Switch to DUMP
            ctrl_state.P_lock  = P_act;
            ctrl_state.P_hold  = 0.78 * P_act;
            ctrl_state.P_cmd   = ctrl_state.P_hold;
            ctrl_state.t_phase = 0;
        end
end

ctrl_state.phase = phase;

% Output pressure generation
switch phase
    case 2 % DUMP: Controlled fast venting
        dump_rate = 9.0e7; % [Pa/s]
        target_dump = 0.72 * ctrl_state.P_lock;
        ctrl_state.P_cmd = max(target_dump, ctrl_state.P_cmd - dump_rate * dt);
        
    case 1 % HOLD: Maintain pressure
        ctrl_state.P_cmd = ctrl_state.P_hold;
        
    case 0 % BUILD: Stepped / pulsed build towards peak
        build_rate = 8.5e7; % 850 bar/s build rate [Pa/s]
        ctrl_state.P_cmd = min(P_driver, ctrl_state.P_cmd + build_rate * dt);
end

P_cmd = max(p.brake.P_min, min(P_driver, ctrl_state.P_cmd));

end
