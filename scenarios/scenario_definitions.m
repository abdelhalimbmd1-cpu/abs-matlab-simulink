function scen_list = scenario_definitions()
% SCENARIO_DEFINITIONS Generates the standard test scenario matrix for ABS evaluation.
%
% Optimized integration step dt = 0.001s (1 kHz) provides exceptional numerical
% fidelity while ensuring high-throughput batch execution.

scen_list = [];

speeds_kmh = [60, 100, 130];
surfaces   = {'dry', 'wet', 'snow', 'ice'};
drivers    = {'step', 'ramp', 'panic'};

% 1. Core Speed x Surface Matrix (Step driver)
for s_idx = 1:length(surfaces)
    surf = surfaces{s_idx};
    for v_idx = 1:length(speeds_kmh)
        spd = speeds_kmh(v_idx);
        
        sc.id          = sprintf('%s_%dkmh_step', surf, spd);
        sc.description = sprintf('%s surface at %d km/h (Step braking)', upper(surf), spd);
        sc.v0          = spd / 3.6;
        sc.surface     = surf;
        sc.surface_L   = surf;
        sc.surface_R   = surf;
        sc.driver_type = 'step';
        sc.is_split_mu = false;
        sc.use_ymc     = false;
        if strcmp(surf, 'ice')
            sc.t_end = 25.0;
        elseif strcmp(surf, 'snow')
            sc.t_end = 14.0;
        else
            sc.t_end = 7.0;
        end
        sc.dt          = 0.001; % 1 ms step
        
        scen_list = [scen_list; sc];
    end
end

% 2. Driver Profile Matrix (100 km/h on Dry and Wet)
for d_idx = 1:length(drivers)
    drv = drivers{d_idx};
    if strcmp(drv, 'step'), continue; end
    
    for surf_cell = {'dry', 'wet'}
        surf = surf_cell{1};
        sc.id          = sprintf('%s_100kmh_%s', surf, drv);
        sc.description = sprintf('%s surface at 100 km/h (%s braking)', upper(surf), drv);
        sc.v0          = 100 / 3.6;
        sc.surface     = surf;
        sc.surface_L   = surf;
        sc.surface_R   = surf;
        sc.driver_type = drv;
        sc.is_split_mu = false;
        sc.use_ymc     = false;
        sc.t_end       = 7.0;
        sc.dt          = 0.001;
        
        scen_list = [scen_list; sc];
    end
end

% 3. Surface Transitions (100 km/h)
% 3.1 Transition: Dry -> Ice at t = 1.0s
sc.id          = 'transition_dry_ice_100kmh';
sc.description = 'Mid-braking Transition: Dry -> Ice at t=1.0s (100 km/h)';
sc.v0          = 100 / 3.6;
sc.surface     = 'transition';
sc.surface_L   = 'transition';
sc.surface_R   = 'transition';
sc.driver_type = 'step';
sc.is_split_mu = false;
sc.use_ymc     = false;
sc.t_end       = 14.0;
sc.dt          = 0.001;
scen_list = [scen_list; sc];

% 3.2 Transition: Wet -> Dry at t = 1.0s
sc.id          = 'transition_wet_dry_100kmh';
sc.description = 'Mid-braking Transition: Wet -> Dry at t=1.0s (100 km/h)';
sc.v0          = 100 / 3.6;
sc.surface     = 'transition_wet_dry';
sc.surface_L   = 'transition_wet_dry';
sc.surface_R   = 'transition_wet_dry';
sc.driver_type = 'step';
sc.is_split_mu = false;
sc.use_ymc     = false;
sc.t_end       = 6.0;
sc.dt          = 0.001;
scen_list = [scen_list; sc];

% 4. Split-mu Scenario (Left: Dry, Right: Ice at 100 km/h)
sc.id          = 'split_mu_dry_ice_100kmh';
sc.description = 'Split-mu Surface (Left: Dry, Right: Ice) at 100 km/h';
sc.v0          = 100 / 3.6;
sc.surface     = 'dry';
sc.surface_L   = 'dry';
sc.surface_R   = 'ice';
sc.driver_type = 'step';
sc.is_split_mu = true;
sc.use_ymc     = true;
sc.t_end       = 8.0;
sc.dt          = 0.001;
scen_list = [scen_list; sc];

end
