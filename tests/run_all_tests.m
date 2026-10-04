function all_passed = run_all_tests()
% RUN_ALL_TESTS Comprehensive automated validation test harness
%
% Executes:
%   1. test_phase1_baseline (Quarter-car baseline & wheel lockup)
%   2. test_phase2_controllers (Comparative controller evaluation)
%   3. test_phase3_full_car (4-wheel full car, pitch load transfer, split-mu)
%   4. test_phase4_robustness (Sensor noise, speed estimator, parameter sweeps)
%   5. test_energy_and_physics_sanity (Energy conservation, NaN check, physical bounds)
%   6. test_textbook_benchmarks (Automotive textbook deceleration & stopping distance bounds)
%   7. test_simulink_models (Programmatic verification of abs_quarter_car.slx and abs_full_car.slx)

fprintf('=================================================================\n');
fprintf('  STARTING COMPREHENSIVE ABS AUTOMATED TEST SUITE\n');
fprintf('=================================================================\n\n');

set(groot, 'defaultFigureRenderer', 'painters');
p = init_params();
test_results = struct();

%% Test 1: Phase 1 Baseline Verification
fprintf('>>> TEST 1: Phase 1 Baseline & Lockup Verification...\n');
try
    res1 = test_phase1_baseline();
    close all;
    test_results.phase1 = true;
    fprintf('    [PASS] Test 1: Baseline Quarter-Car Lockup\n\n');
catch me
    close all;
    test_results.phase1 = false;
    fprintf('    [FAIL] Test 1 Error: %s\n\n', me.message);
end

%% Test 2: Phase 2 ABS Controllers
fprintf('>>> TEST 2: Phase 2 Controller Comparative Evaluation...\n');
try
    res2 = test_phase2_controllers();
    close all;
    test_results.phase2 = true;
    fprintf('    [PASS] Test 2: Controller Benchmark\n\n');
catch me
    close all;
    test_results.phase2 = false;
    fprintf('    [FAIL] Test 2 Error: %s\n\n', me.message);
end

%% Test 3: Phase 3 Full Car & Split-mu
fprintf('>>> TEST 3: Phase 3 4-Wheel Full Car & Split-mu Dynamics...\n');
try
    res3 = test_phase3_full_car();
    close all;
    test_results.phase3 = true;
    fprintf('    [PASS] Test 3: 4-Wheel Dynamics & Split-mu Yaw Stability\n\n');
catch me
    close all;
    test_results.phase3 = false;
    fprintf('    [FAIL] Test 3 Error: %s\n\n', me.message);
end

%% Test 4: Phase 4 Realism & Robustness
fprintf('>>> TEST 4: Phase 4 Realism & Robustness Sweeps...\n');
try
    res4 = test_phase4_robustness();
    close all;
    test_results.phase4 = true;
    fprintf('    [PASS] Test 4: Sensor Realism & Robustness Sweeps\n\n');
catch me
    close all;
    test_results.phase4 = false;

    fprintf('    [FAIL] Test 4 Error: %s\n\n', me.message);
end

%% Test 5: Energy Conservation & Numerical Sanity
fprintf('>>> TEST 5: Energy Conservation & Numerical Sanity Checks...\n');
try
    scen_test.v0 = 100 / 3.6;
    scen_test.surface = 'dry';
    scen_test.driver_type = 'step';
    scen_test.t_end = 4.0;
    scen_test.dt = 0.0005;
    
    sim_t = sim_quarter_car(scen_test, 'rule_based', p);
    
    % Check no NaNs or Infs
    assert(~any(isnan(sim_t.v)), 'NaN detected in vehicle speed!');
    assert(~any(isnan(sim_t.w)), 'NaN detected in wheel speed!');
    assert(~any(isnan(sim_t.slip)), 'NaN detected in slip ratio!');
    assert(~any(isnan(sim_t.P_act)), 'NaN detected in brake pressure!');
    assert(~any(isinf(sim_t.v)), 'Inf detected in vehicle speed!');
    
    % Check physical bounds
    assert(all(sim_t.slip >= 0.0 & sim_t.slip <= 1.0), 'Slip ratio must remain within [0, 1]!');
    assert(all(sim_t.v >= -1e-5), 'Vehicle speed must remain non-negative!');
    assert(all(sim_t.w >= -1e-5), 'Wheel speed must remain non-negative!');
    assert(all(sim_t.P_act >= 0 & sim_t.P_act <= p.brake.P_max * 1.05), 'Pressure within physical limits!');
    
    % Kinetic energy analysis
    m_q = p.veh.mass_quarter;
    J_w = p.wheel.Jw;
    E_k_initial = 0.5 * m_q * sim_t.v0^2 + 0.5 * J_w * (sim_t.v0 / p.wheel.R)^2;
    
    % Total work done by tire braking force
    W_tire = trapz(sim_t.t, sim_t.Fx .* sim_t.v);
    
    energy_ratio = W_tire / E_k_initial;
    fprintf('    Initial Kinetic Energy: %.1f J | Dissipated Tire Work: %.1f J (Ratio: %.3f)\n', ...
        E_k_initial, W_tire, energy_ratio);
    assert(abs(energy_ratio - 1.0) < 0.05, 'Energy conservation violation: Work must balance initial Ek within 5%!');
    
    test_results.energy_sanity = true;
    fprintf('    [PASS] Test 5: Energy Conservation & Bounds Verified\n\n');
catch me
    test_results.energy_sanity = false;
    fprintf('    [FAIL] Test 5 Error: %s\n\n', me.message);
end

%% Test 6: Automotive Textbook Benchmark Comparisons
fprintf('>>> TEST 6: Automotive Textbook Benchmark Deceleration & Distance Checks...\n');
try
    results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
    batch_file = fullfile(results_dir, 'batch_results.mat');
    
    if ~exist(batch_file, 'file')
        % If not already run, run batch runner
        batch_runner();
    end
    load(batch_file, 'record_list');
    
    % Dry 100 km/h deceleration check (Textbook range: 7.5 - 9.8 m/s^2 for high-performance ABS)
    idx_dry = find(strcmp({record_list.scenario_id}, 'dry_100kmh_step') & strcmp({record_list.controller}, 'pid'));
    r_dry = record_list(idx_dry);
    fprintf('    Dry Asphalt (PID): Deceleration = %.2f m/s^2 (Expected: 7.5 - 9.8 m/s^2)\n', r_dry.mean_decel);
    assert(r_dry.mean_decel >= 7.5 && r_dry.mean_decel <= 9.8, 'Dry deceleration outside textbook range!');
    
    % Wet 100 km/h deceleration check (Textbook range: 5.5 - 7.5 m/s^2)
    idx_wet = find(strcmp({record_list.scenario_id}, 'wet_100kmh_step') & strcmp({record_list.controller}, 'pid'));
    r_wet = record_list(idx_wet);
    fprintf('    Wet Asphalt (PID): Deceleration = %.2f m/s^2 (Expected: 5.5 - 7.5 m/s^2)\n', r_wet.mean_decel);
    assert(r_wet.mean_decel >= 5.5 && r_wet.mean_decel <= 7.5, 'Wet deceleration outside textbook range!');
    
    % Check that ABS stopping distance is shorter than No-ABS across all homogeneous surfaces
    test_surfs = {'dry', 'wet', 'snow', 'ice'};
    for s = 1:length(test_surfs)
        surf_name = test_surfs{s};
        sc_id = sprintf('%s_100kmh_step', surf_name);
        d_no_abs = record_list(strcmp({record_list.scenario_id}, sc_id) & strcmp({record_list.controller}, 'none')).stop_dist;
        d_pid    = record_list(strcmp({record_list.scenario_id}, sc_id) & strcmp({record_list.controller}, 'pid')).stop_dist;
        d_rb     = record_list(strcmp({record_list.scenario_id}, sc_id) & strcmp({record_list.controller}, 'rule_based')).stop_dist;
        
        fprintf('    %5s 100 km/h: No-ABS = %5.1f m | Rule-Based = %5.1f m | PID = %5.1f m\n', ...
            upper(surf_name), d_no_abs, d_rb, d_pid);
        assert(d_pid < d_no_abs, sprintf('PID must stop shorter than No-ABS on %s!', surf_name));
        assert(d_rb < d_no_abs, sprintf('Rule-Based must stop shorter than No-ABS on %s!', surf_name));
    end
    
    test_results.textbook_benchmarks = true;
    fprintf('    [PASS] Test 6: Textbook Bounds & Deceleration Benchmarks Verified\n\n');
catch me
    test_results.textbook_benchmarks = false;
    fprintf('    [FAIL] Test 6 Error: %s\n\n', me.message);
end

%% Test 7: Simulink Models Compilation and Execution
fprintf('>>> TEST 7: Simulink Models Verification (.slx)...\n');
try
    models_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'models');
    
    % Check Quarter Car Model
    load_system(fullfile(models_dir, 'abs_quarter_car.slx'));
    simOut_q = sim('abs_quarter_car');
    close_system('abs_quarter_car', 0);
    assert(~isempty(simOut_q.dist_m), 'Quarter car model output missing!');
    fprintf('    Quarter-Car SLX: Verified! Stop dist = %.2f m\n', simOut_q.dist_m(end));
    
    % Check Full Car Model
    load_system(fullfile(models_dir, 'abs_full_car.slx'));
    simOut_f = sim('abs_full_car');
    close_system('abs_full_car', 0);
    assert(~isempty(simOut_f.dist_m), 'Full car model output missing!');
    fprintf('    Full-Car SLX:    Verified! Stop dist = %.2f m, Max yaw = %.2f deg\n', ...
        simOut_f.dist_m(end), max(abs(simOut_f.yaw_deg)));
    
    test_results.simulink = true;
    fprintf('    [PASS] Test 7: Simulink Models Verified\n\n');
catch me
    test_results.simulink = false;
    fprintf('    [FAIL] Test 7 Error: %s\n\n', me.message);
end

%% Summary
fprintf('=================================================================\n');
fprintf('  TEST SUITE EXECUTION SUMMARY\n');
fprintf('=================================================================\n');
test_fields = fieldnames(test_results);
all_passed = true;
for i = 1:length(test_fields)
    f = test_fields{i};
    status = test_results.(f);
    if status
        str_status = 'PASS';
    else
        str_status = 'FAIL';
        all_passed = false;
    end
    fprintf('  %-25s : [%s]\n', f, str_status);
end
fprintf('=================================================================\n');

if all_passed
    fprintf('  >>> ALL 7 AUTOMATED VERIFICATION SUITES PASSED! <<<\n');
else
    fprintf('  >>> WARNING: Some tests encountered issues. <<<\n');
end
fprintf('=================================================================\n\n');

end
