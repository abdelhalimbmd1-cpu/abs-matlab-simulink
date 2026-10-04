function m = compute_metrics(sim_out, p)
% COMPUTE_METRICS Evaluates standard ABS performance and safety metrics
%
% Metrics computed:
%   - Stopping distance [m]
%   - Stopping time [s]
%   - Mean deceleration [m/s^2]
%   - Peak slip ratio (during active braking v > 2 m/s)
%   - Mean slip ratio (during active braking v > 2 m/s)
%   - Slip RMSE vs optimal surface slip
%   - Time spent locked (slip > 0.85 while v > 2 m/s) [s]
%   - Hydraulic control effort (integral of pressure) [MPa*s]
%   - Pressure chatter index (mean absolute pressure derivative) [MPa/s]
%   - Max yaw angle & max yaw rate (for 4-wheel / split-mu runs)

if nargin < 2 || isempty(p)
    p = init_params();
end

m = struct();

% Basic stopping metrics
m.stop_dist = sim_out.stop_dist;
m.stop_time = sim_out.stop_time;
if isfield(sim_out, 'v0')
    m.v0 = sim_out.v0;
else
    m.v0 = sim_out.v(1);
end

% Mean deceleration: a_mean = (v0 - v_final) / stop_time
m.mean_decel = m.v0 / max(0.1, sim_out.stop_time);


% Active braking mask (exclude initial 0.15s pedal transient and final stop < 2 m/s)
mask = (sim_out.t > 0.15) & (sim_out.v > 2.0);

if any(mask)
    t_act = sim_out.t(mask);
    v_act = sim_out.v(mask);
    
    % If 4-wheel simulation, average across wheels
    if size(sim_out.slip, 2) > 1
        slip_act = mean(sim_out.slip(mask, :), 2);
        P_act    = mean(sim_out.P_act(mask, :), 2);
    else
        slip_act = sim_out.slip(mask);
        P_act    = sim_out.P_act(mask);
    end
    
    m.peak_slip = max(slip_act);
    m.mean_slip = mean(slip_act);
    
    % Optimal slip target for comparison
    surf = sim_out.surface;
    if ischar(surf) && isfield(p.tire, surf)
        opt_slip = p.tire.(surf).opt_slip;
    else
        opt_slip = 0.16;
    end
    m.slip_rmse = sqrt(mean((slip_act - opt_slip).^2));
    
    % Time spent locked (slip > 0.85 while moving)
    dt = mean(diff(sim_out.t));
    m.time_locked = sum(slip_act > 0.85) * dt;
    
    % Control effort: integral of pressure [MPa * s]
    m.control_effort = trapz(t_act, P_act * 1e-6);
    
    % Chatter index: mean absolute pressure derivative [MPa / s]
    if length(t_act) > 1
        dP_dt = diff(P_act * 1e-6) ./ diff(t_act);
        m.chatter_index = mean(abs(dP_dt));
    else
        m.chatter_index = 0.0;
    end
else
    m.peak_slip      = 0.0;
    m.mean_slip      = 0.0;
    m.slip_rmse      = 0.0;
    m.time_locked    = 0.0;
    m.control_effort = 0.0;
    m.chatter_index  = 0.0;
end

% Yaw metrics if available
if isfield(sim_out, 'max_yaw')
    m.max_yaw_deg      = sim_out.max_yaw * (180 / pi);
    m.max_yaw_rate_dps = sim_out.max_yaw_rate * (180 / pi);
else
    m.max_yaw_deg      = 0.0;
    m.max_yaw_rate_dps = 0.0;
end

end
