function [results, info] = run_baseband_mdop(L2out, CWout, vehicle_id, varargin)
% RUN_BASEBAND_MDOP  Generate CW baseband for one vehicle and extract
% micro-Doppler, comparing against the ground-truth linear chirp.
%
%   [results, info] = run_baseband_mdop(L2out, CWout, vehicle_id)
%   [results, info] = run_baseband_mdop(..., 'Name', Value, ...)
%
% Pipeline:
%   1. Call generate_cw_baseband() in low-rate de-hopped mode (default 1 MHz).
%      This synthesizes CW tones at fixed V frequencies with a linear
%      Doppler chirp sweeping from f_start to f_end across the sim duration.
%   2. Call mDopPx_speedUp_allPRBs() in burst mode to recover per-subframe
%      Doppler estimates from the .dat file.
%   3. Overlay recovered Doppler vs ground-truth chirp and report RMSE.
%
% Name-Value Options:
%   'Fs_out'         - Output sample rate (default: 1e6)
%   'DopplerRange'   - [f_start, f_end] Hz for ground-truth chirp
%                      (default: [-1000, 1000])
%   'Duration'       - Sim seconds to generate (default: L2out.params.simseconds)
%   'BurstMode'      - true = 1 ms burst processing, false = STFT
%                      (default: true, recommended for chirp RMSE)
%   'MicroDopplerHz' - Optional sinusoidal micro-Doppler amplitude
%   'MicroDopplerRate' - Micro-Doppler rotation rate (default: 10)
%   Any other option is forwarded to generate_cw_baseband.
%
% Output:
%   results - struct from mDopPx_speedUp_allPRBs (includes rmse_Hz)
%   info    - struct from generate_cw_baseband (filepath, V_frequencies, ...)

    p = inputParser; p.KeepUnmatched = true;
    addParameter(p, 'Fs_out', 1e6);
    addParameter(p, 'DopplerRange', [-1000, 1000]);
    addParameter(p, 'Duration', []);
    addParameter(p, 'BurstMode', true);
    addParameter(p, 'MicroDopplerHz', 0);
    addParameter(p, 'MicroDopplerRate', 10);
    parse(p, varargin{:});

    if isempty(p.Results.Duration)
        duration = L2out.params.simseconds;
    else
        duration = p.Results.Duration;
    end

    % Forward extra options (e.g. SNR_dB, CWPower_dB) to generator
    extra = namedargs2cell(p.Unmatched);

    fprintf('\n==== Stage 1/2: Generate CW baseband ====\n');
    info = generate_cw_baseband(L2out, CWout, vehicle_id, ...
        'Fs_out',           p.Results.Fs_out, ...
        'Duration',         duration, ...
        'DopplerRange',     p.Results.DopplerRange, ...
        'MicroDopplerHz',   p.Results.MicroDopplerHz, ...
        'MicroDopplerRate', p.Results.MicroDopplerRate, ...
        extra{:});

    fprintf('\n==== Stage 2/2: Micro-Doppler processing ====\n');
    gt = struct( ...
        'f_start', p.Results.DopplerRange(1), ...
        'f_end',   p.Results.DopplerRange(2), ...
        'T_total', info.duration_s);

    results = mDopPx_speedUp_allPRBs( ...
        info.filepath, info.V_frequencies, info.fs_Hz, ...
        p.Results.BurstMode, gt);

    fprintf('\n==== Pipeline complete ====\n');
    fprintf('  Vehicle:     %d\n', vehicle_id);
    fprintf('  CW mode:     %s\n', info.sensing_mode);
    fprintf('  Baseband:    %s\n', info.filepath);
    if ~isnan(results.rmse_Hz)
        fprintf('  Doppler RMSE vs linear chirp: %.2f Hz\n', results.rmse_Hz);
        fprintf('  (Ground truth: %.0f -> %.0f Hz over %.1f s)\n', ...
            p.Results.DopplerRange(1), p.Results.DopplerRange(2), ...
            info.duration_s);
    end
end
