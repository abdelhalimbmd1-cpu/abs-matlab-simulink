function all_results = batch_runner()
% BATCH_RUNNER Executes the exhaustive batch simulation sweep across all
% controller x scenario combinations, computes standardized metrics,
% generates summary tables, and exports publication-ready figures.

p = init_params();
scen_list = scenario_definitions();
controllers = {'none', 'rule_based', 'pid', 'smc', 'adaptive'};
ctrl_names  = {'No-ABS', 'Rule-Based', 'PID', 'SMC', 'Adaptive'};

results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
if ~exist(results_dir, 'dir'), mkdir(results_dir); end

fprintf('=================================================================\n');
fprintf('  STARTING COMPREHENSIVE ABS BATCH SWEEP MATRIX (%d Scenarios)\n', length(scen_list));
fprintf('=================================================================\n');

record_list = [];

for s_idx = 1:length(scen_list)
    scen = scen_list(s_idx);
    fprintf('\nScenario [%02d/%02d]: %s\n', s_idx, length(scen_list), scen.description);
    fprintf('-----------------------------------------------------------------\n');
    
    no_abs_dist = 0;
    
    for c_idx = 1:length(controllers)
        c_type = controllers{c_idx};
        
        % Select simulation engine based on scenario type
        if scen.is_split_mu
            sim_out = sim_full_car(scen, c_type, p);
        else
            sim_out = sim_quarter_car(scen, c_type, p);
        end
        
        % Compute metrics
        m = compute_metrics(sim_out, p);
        
        if strcmp(c_type, 'none')
            no_abs_dist = m.stop_dist;
            dist_red_pct = 0.0;
        else
            dist_red_pct = ((no_abs_dist - m.stop_dist) / max(0.1, no_abs_dist)) * 100.0;
        end
        
        fprintf('%-12s | Dist: %6.2f m (%+5.1f%%) | Time: %5.2f s | Decel: %4.2f m/s^2 | Slip: %.3f\n', ...
            ctrl_names{c_idx}, m.stop_dist, dist_red_pct, m.stop_time, m.mean_decel, m.mean_slip);
        
        % Pack into record
        rec.scenario_id   = scen.id;
        rec.description   = scen.description;
        rec.v0_kmh        = round(scen.v0 * 3.6);
        rec.surface       = scen.surface;
        rec.driver        = scen.driver_type;
        rec.controller    = c_type;
        rec.ctrl_name     = ctrl_names{c_idx};
        rec.stop_dist     = m.stop_dist;
        rec.stop_time     = m.stop_time;
        rec.mean_decel    = m.mean_decel;
        rec.peak_slip     = m.peak_slip;
        rec.mean_slip     = m.mean_slip;
        rec.slip_rmse     = m.slip_rmse;
        rec.time_locked   = m.time_locked;
        rec.control_effort = m.control_effort;
        rec.chatter_index = m.chatter_index;
        rec.max_yaw_deg   = m.max_yaw_deg;
        rec.dist_red_pct  = dist_red_pct;
        
        record_list = [record_list; rec];
        
        % Save detailed raw simulation trace for key reference runs
        if strcmp(scen.id, 'dry_100kmh_step') || strcmp(scen.id, 'wet_100kmh_step') || ...
           strcmp(scen.id, 'split_mu_dry_ice_100kmh') || strcmp(scen.id, 'transition_dry_ice_100kmh')
            trace_file = fullfile(results_dir, sprintf('trace_%s_%s.mat', scen.id, c_type));
            save(trace_file, 'sim_out');
        end
    end
end

% Save complete workspace mat file
batch_mat = fullfile(results_dir, 'batch_results.mat');
save(batch_mat, 'record_list', 'scen_list', 'p');
fprintf('\nSaved complete batch results to: %s\n', batch_mat);

% Generate formatted CSV table
tbl_file = fullfile(results_dir, 'summary_table.csv');
fid = fopen(tbl_file, 'w');
fprintf(fid, 'Scenario_ID,Surface,Speed_kmh,Driver,Controller,StopDist_m,StopTime_s,MeanDecel_ms2,MeanSlip,PeakSlip,SlipRMSE,TimeLocked_s,Chatter_MPas,DistReduction_pct\n');
for i = 1:length(record_list)
    r = record_list(i);
    fprintf(fid, '%s,%s,%d,%s,%s,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.1f\n', ...
        r.scenario_id, r.surface, r.v0_kmh, r.driver, r.ctrl_name, ...
        r.stop_dist, r.stop_time, r.mean_decel, r.mean_slip, r.peak_slip, ...
        r.slip_rmse, r.time_locked, r.chatter_index, r.dist_red_pct);
end
fclose(fid);
fprintf('Saved summary CSV table to: %s\n', tbl_file);

% Call Plotting Engine to generate all Phase 6 figures
generate_phase6_figures(record_list, results_dir, p);

all_results = record_list;
fprintf('\n=================================================================\n');
fprintf('  BATCH SWEEP COMPLETED SUCCESSFULLY!\n');
fprintf('=================================================================\n');

end
