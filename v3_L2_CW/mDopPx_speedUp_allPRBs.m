function results = mDopPx_speedUp_allPRBs(filepath, V, Fs, burst_mode, gt_params)
% MDOPPX_SPEEDUP_ALLPRBS  Extract micro-Doppler from a baseband .dat file
% and compare against a ground-truth linear chirp.
%
%   results = mDopPx_speedUp_allPRBs(filepath, V, Fs, burst_mode, gt_params)
%
% Modes:
%   burst_mode = false  -> standard STFT (250 ms windows, 50 ms advance)
%   burst_mode = true   -> per-subframe CW burst processing (1 ms resolution,
%                          zero-padded FFT + parabolic interpolation)
%
% Inputs:
%   filepath   - Path to complex baseband .dat (interleaved float32 I/Q)
%   V          - CW reference tone frequencies (Hz). Detected peak is
%                matched to nearest V(k); Doppler = peak - V(k).
%   Fs         - Sampling rate (Hz)
%   burst_mode - logical, true for burst mode, false for STFT
%   gt_params  - (optional) struct with fields f_start, f_end, T_total
%                describing the ground-truth linear chirp. If provided,
%                an RMSE is computed and the chirp is overlaid on plots.
%                Pass [] to skip comparison.
%
% Output:
%   results - struct with fields:
%     .filepath, .Fs, .V
%     .burst_results           (burst mode only)
%     .FTDP_Window_Doppler, .f_FTDP, .t_FTDP, .detected_prb_freqs
%                              (STFT mode only)
%     .num_windows, .num_bursts
%     .rmse_Hz                 RMSE of detected Doppler vs ground truth
%     .gt_params               echoed ground-truth params

    %% === Constants ===
    f0 = 5.860e9;
    lambda = physconst('LightSpeed') / f0;  %#ok<NASGU>  % for reference

    if nargin < 4 || isempty(burst_mode), burst_mode = true; end
    if nargin < 5, gt_params = []; end

    % Locate file (accept bare filename if in same directory)
    if ~isfile(filepath)
        error('mDopPx_speedUp_allPRBs:fileNotFound', ...
              'Baseband file not found: %s', filepath);
    end
    [~, fname_base, ~] = fileparts(filepath);
    fname = fname_base;

    %% === Load ===
    x = read_complex_binary(filepath);
    L = length(x);
    fprintf('Loaded %d samples (%.3f s at %.0f Hz) from %s\n', ...
        L, L/Fs, Fs, fname);

    %% === Initialize ===
    results = struct('filepath', filepath, 'Fs', Fs, 'V', V, ...
        'burst_results', [], ...
        'FTDP_Window_Doppler', [], 'f_FTDP', [], 't_FTDP', [], ...
        'detected_prb_freqs', [], ...
        'num_windows', 0, 'num_bursts', 0, ...
        'rmse_Hz', NaN, 'gt_params', gt_params);

    burst_results = struct('times', [], 'doppler', [], 'peak_freqs', [], ...
                           'matched_V', [], 'energy', [], 'num_bursts', 0);

    if burst_mode
        %% =============================================================
        %  PER-BURST CW PROCESSING (1 ms resolution)
        %  =============================================================
        fprintf('Processing in CW burst mode...\n');
        tic;

        sf_samps = floor(Fs * 1e-3);
        num_sf = floor(L / sf_samps);

        % Per-subframe mean power
        energy = zeros(num_sf, 1);
        for i = 1:num_sf
            seg = x((i-1)*sf_samps+1 : i*sf_samps);
            energy(i) = sum(abs(seg).^2) / sf_samps;
        end

        noise_power = median(energy);
        threshold = noise_power * 5;
        is_tx = energy > threshold;
        tx_indices = find(is_tx);
        num_tx = length(tx_indices);

        fprintf('  Detected %d TX bursts of %d subframes (%.1f%% duty)\n', ...
            num_tx, num_sf, 100*num_tx/num_sf);
        fprintf('  Noise floor: %.2e, Threshold: %.2e\n', ...
            noise_power, threshold);

        if num_tx == 0
            warning('mDopPx_speedUp_allPRBs:noBursts', ...
                'No TX bursts detected (threshold too high?)');
            return;
        end

        %% Per-burst frequency estimation
        nfft_burst = 16384;
        f_burst = ((-nfft_burst/2 : nfft_burst/2-1) * Fs / nfft_burst)';
        freq_res = Fs / nfft_burst;
        dc_remove_hz = 4000;
        dc_remove_bins_burst = round(dc_remove_hz / freq_res);
        center_burst = nfft_burst/2 + 1;

        burst_times = zeros(num_tx, 1);
        burst_peak_freqs = zeros(num_tx, 1);
        burst_doppler = zeros(num_tx, 1);
        burst_matched_V = zeros(num_tx, 1);

        win = hann(sf_samps);
        for i = 1:num_tx
            sf_idx = tx_indices(i);
            seg = x((sf_idx-1)*sf_samps+1 : sf_idx*sf_samps);
            Fx = fftshift(fft(win .* seg, nfft_burst));
            MFx = abs(Fx);
            MFx(center_burst - dc_remove_bins_burst : ...
                center_burst + dc_remove_bins_burst) = 0;
            [~, loc] = max(MFx);
            peak_freq = f_burst(loc);

            % Parabolic interpolation
            if loc > 1 && loc < nfft_burst
                a = MFx(loc-1);  b = MFx(loc);  c = MFx(loc+1);
                denom = a - 2*b + c;
                if abs(denom) > 0
                    delta = 0.5 * (a - c) / denom;
                    peak_freq = peak_freq + delta * freq_res;
                end
            end

            [~, pos] = min(abs(peak_freq - V));
            burst_times(i) = (sf_idx - 0.5) * sf_samps / Fs;
            burst_peak_freqs(i) = peak_freq;
            burst_matched_V(i) = V(pos);
            burst_doppler(i) = peak_freq - V(pos);
        end

        fprintf('  Doppler range: [%.1f, %.1f] Hz\n', ...
            min(burst_doppler), max(burst_doppler));
        fprintf('Processed %d bursts in %.2f s\n', num_tx, toc);

        burst_results.times       = burst_times;
        burst_results.doppler     = burst_doppler;
        burst_results.peak_freqs  = burst_peak_freqs;
        burst_results.matched_V   = burst_matched_V;
        burst_results.energy      = energy;
        burst_results.num_bursts  = num_tx;

        results.burst_results = burst_results;
        results.num_bursts    = num_tx;

        %% RMSE vs ground truth
        if ~isempty(gt_params)
            gt_doppler = ground_truth_chirp(burst_times, gt_params);
            results.rmse_Hz = rms(burst_doppler - gt_doppler);
            fprintf('  Ground-truth RMSE: %.2f Hz\n', results.rmse_Hz);
        end

        %% Plots
        plot_burst_results(burst_results, fname, gt_params, results.rmse_Hz);

    else
        %% =============================================================
        %  STFT PROCESSING
        %  =============================================================
        fprintf('Processing with STFT approach...\n');
        tic;

        wind_dur = 0.25;
        FTDP_adv_dur = 0.05;
        wind_samps = floor(wind_dur * Fs);
        FTDP_adv_samps = floor(FTDP_adv_dur * Fs);
        num_windows = floor((L - wind_samps) / FTDP_adv_samps) + 1;

        f = ((-wind_samps/2 : wind_samps/2 - 1) * Fs / wind_samps)';
        doppler_range = 2000;
        doppler_bins = floor(doppler_range / (Fs/wind_samps));
        center_bin = wind_samps/2 + 1;
        doppler_start = center_bin - doppler_bins;
        doppler_end = center_bin + doppler_bins;
        f_FTDP = ((-doppler_bins:doppler_bins) * Fs / wind_samps)';

        FTDP_Window_Doppler = zeros(num_windows, length(f_FTDP));
        detected_prb_freqs = zeros(num_windows, 1);
        detected_peak_freqs = zeros(num_windows, 1);

        blk = blackman(wind_samps);
        hn  = hann(wind_samps);
        for i = 1:num_windows
            window_start = (i-1) * FTDP_adv_samps + 1;
            window_end   = window_start + wind_samps - 1;
            segment = x(window_start:window_end);

            Fx = fftshift(fft(blk .* segment));
            MFxPos = abs(Fx);
            dc_remove_bins = 1000;
            center_idx = length(Fx)/2 + 1;
            MFxPos(center_idx - dc_remove_bins : center_idx + dc_remove_bins) = 0;
            [~, loc] = max(MFxPos);
            detected_peak_freq = f(loc);

            [~, pos] = min(abs(detected_peak_freq - V));
            detected_prb_freqs(i) = V(pos);
            detected_peak_freqs(i) = detected_peak_freq;

            time_vec = (0:length(segment)-1)' / Fs;
            cfo_corrected = segment .* exp(-1i*2*pi*detected_peak_freq*time_vec);

            Fx_final = fftshift(fft(hn .* cfo_corrected));
            FTDP_Window_Doppler(i, :) = abs(Fx_final(doppler_start:doppler_end));
        end

        t_FTDP = (0:(num_windows-1)) * FTDP_adv_dur;
        fprintf('Processed %d windows in %.2f s\n', num_windows, toc);

        results.FTDP_Window_Doppler = FTDP_Window_Doppler;
        results.f_FTDP              = f_FTDP;
        results.t_FTDP              = t_FTDP;
        results.detected_prb_freqs  = detected_prb_freqs;
        results.num_windows         = num_windows;

        %% RMSE vs ground truth (using detected peak - matched V per window)
        if ~isempty(gt_params)
            detected_doppler = detected_peak_freqs - detected_prb_freqs;
            gt_doppler = ground_truth_chirp(t_FTDP(:), gt_params);
            results.rmse_Hz = rms(detected_doppler - gt_doppler);
            fprintf('  Ground-truth RMSE: %.2f Hz\n', results.rmse_Hz);
        end

        plot_stft_results(results, fname, gt_params);
    end

    fprintf('Processing complete.\n');
end


%% =====================================================================
%  Helpers
%  =====================================================================

function f = ground_truth_chirp(t, gtp)
    % Linear chirp: f_start -> f_end over [0, T_total]
    t_clip = min(max(t, 0), gtp.T_total);
    f = gtp.f_start + (gtp.f_end - gtp.f_start) * (t_clip / gtp.T_total);
end


function plot_burst_results(br, fname, gt_params, rmse_Hz)
    if br.num_bursts == 0, return; end
    t_min = min(br.times);
    t_max = max(br.times);

    %% Fig 1: 2-D micro-Doppler spectrogram
    figure('Name','CW Micro-Doppler Spectrogram'); clf;
    dt_bin = 0.05;
    t_edges = t_min : dt_bin : t_max + dt_bin;
    t_centers = t_edges(1:end-1) + dt_bin/2;
    df_bin = 5;
    f_edges = -1500 : df_bin : 1500;
    f_centers = f_edges(1:end-1) + df_bin/2;
    [spec, ~, ~] = histcounts2(br.times, br.doppler, t_edges, f_edges);
    spec = spec';
    ampmin = max(spec(:)) / 1000;
    spec_dB = 20*log10(max(spec, ampmin) / max(ampmin, eps));
    imagesc(t_centers, f_centers, spec_dB);
    axis xy; colorbar; colormap(jet);
    title(['CW Micro-Doppler Spectrogram: ' fname], 'Interpreter', 'none');
    xlabel('Time (s)'); ylabel('Doppler (Hz)');
    ylim([-1500 1500]);

    %% Fig 2: Detected Doppler vs ground truth
    figure('Name','Doppler vs Ground Truth'); clf;
    plot(br.times, br.doppler, '.', 'MarkerSize', 5); hold on;
    if ~isempty(gt_params)
        t_gt = linspace(t_min, t_max, 500);
        plot(t_gt, ground_truth_chirp(t_gt, gt_params), 'r--', 'LineWidth', 2);
        legend('Measured', 'Ground-truth chirp', 'Location', 'best');
        ttl = sprintf('CW Burst Doppler — RMSE = %.1f Hz', rmse_Hz);
    else
        legend('Measured', 'Location', 'best');
        ttl = 'CW Burst Doppler Extraction';
    end
    hold off; grid on;
    title([ttl ' | ' fname], 'Interpreter', 'none');
    xlabel('Time (s)'); ylabel('Doppler (Hz)');
    ylim([-1500 1500]);

    %% Fig 3: Raw detected peaks
    figure('Name','Detected Peak Frequencies'); clf;
    plot(br.times, br.peak_freqs/1e3, '.', 'MarkerSize', 4);
    title('Detected CW Peak Frequencies');
    xlabel('Time (s)'); ylabel('Peak (kHz)'); grid on;

    %% Fig 4: TX energy profile
    figure('Name','Per-Subframe Energy'); clf;
    num_sf = length(br.energy);
    t_sf = ((1:num_sf) - 0.5) * 1e-3;
    plot(t_sf, 10*log10(br.energy + eps));
    title('Per-Subframe Energy (TX Burst Detection)');
    xlabel('Time (s)'); ylabel('Power (dB)'); grid on;

    %% Fig 5: Doppler error
    if ~isempty(gt_params)
        figure('Name','Doppler Error vs Ground Truth'); clf;
        gt = ground_truth_chirp(br.times, gt_params);
        err = br.doppler - gt;
        plot(br.times, err, '.', 'MarkerSize', 4);
        title(sprintf('Doppler Error vs Ground Truth (RMSE = %.1f Hz)', rmse_Hz));
        xlabel('Time (s)'); ylabel('Error (Hz)'); grid on;
    end
end


function plot_stft_results(r, fname, gt_params)
    if r.num_windows == 0, return; end

    figure('Name','Micro-Doppler Profile (STFT)'); clf;
    typ = r.FTDP_Window_Doppler;
    if mean(r.f_FTDP) < 0, typ = fliplr(typ); end
    ampmin = max(abs(typ(:))) / 1000;
    imagesc(r.t_FTDP, r.f_FTDP, 20*log10(max(abs(typ.'), ampmin)/max(ampmin,eps)));
    title(['Micro-Doppler Profile: ' fname], 'Interpreter', 'none');
    xlabel('Time (s)'); ylabel('Doppler (Hz) rel. CW center');
    axis xy; colorbar;
    if ~isempty(gt_params)
        hold on;
        t_gt = linspace(0, gt_params.T_total, 500);
        plot(t_gt, ground_truth_chirp(t_gt, gt_params), 'w--', 'LineWidth', 1.5);
        legend('', 'Ground-truth chirp', 'Location', 'best');
        hold off;
    end

    figure('Name','Detected CW Tones Over Time'); clf;
    plot(r.t_FTDP, r.detected_prb_freqs/1e3, 'o-', 'LineWidth', 1.5);
    title('Detected CW Tone Over Time');
    xlabel('Time (s)'); ylabel('Tone (kHz)'); grid on;

    figure('Name','Detected Tone Histogram'); clf;
    histogram(r.detected_prb_freqs/1000, 'BinMethod', 'auto');
    title('Detected CW Tone Distribution');
    xlabel('Tone (kHz)'); ylabel('Windows'); grid on;
end
