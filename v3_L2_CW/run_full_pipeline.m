% run_full_pipeline.m
%
% End-to-end driver for the C-V2X CW ISAC simulation.
%
%   Stage 1 - L2_CW_sim.m    : run the MAC-layer simulation (data SPS + CW).
%   Stage 2 - generate_cw_baseband : synthesize the per-vehicle CW baseband
%                                    in low-rate de-hopped mode (1 MHz).
%   Stage 3 - mDopPx_speedUp_allPRBs : recover per-subframe Doppler from the
%                                      .dat file and compare to the
%                                      ground-truth linear chirp.
%
% The single parameter to tweak here is the vehicle ID whose baseband is
% generated and analysed. All other configuration lives inside
% L2_CW_sim.m (CW mode, sim duration, number of vehicles, etc.).

% -------- Stage 1: run the MAC simulation --------
run('L2_CW_sim.m');   % populates L2out, CWout in this workspace

% -------- Stage 2 + Stage 3: baseband + micro-Doppler --------
pipeline_vehicle_id = 1;

[mdop_results, bb_info] = run_baseband_mdop( ...
    L2out, CWout, pipeline_vehicle_id, ...
    'Fs_out',       1e6, ...               % 1 MHz de-hopped
    'DopplerRange', [-1000, 1000], ...     % ground-truth linear chirp
    'BurstMode',    true);                 % per-ms RMSE comparison
