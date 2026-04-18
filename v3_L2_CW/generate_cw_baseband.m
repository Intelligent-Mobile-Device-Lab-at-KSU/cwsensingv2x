function info = generate_cw_baseband(L2out, CWout, vehicle_id, varargin)
% GENERATE_CW_BASEBAND  Generate complex baseband .dat from L2 CW sim output.
%
%   info = generate_cw_baseband(L2out, CWout, vehicle_id)
%   info = generate_cw_baseband(L2out, CWout, vehicle_id, 'Name', Value, ...)
%
%   Creates a per-vehicle complex baseband file containing:
%     - CW sensing tones during TX subframes
%     - SC-FDMA data signals during TX subframes (full-rate) or modeled as
%       bandlimited noise (low-rate)
%     - AWGN noise during idle subframes
%     - Linear Doppler sweep across full simulation duration
%
%   TWO OPERATING MODES:
%
%     Full-rate (default): Generates at native C-V2X sample rate (15.36 MHz
%       for 10 MHz BW). Produces proper SC-FDMA waveforms with DFT precoding,
%       localized subcarrier mapping, IFFT, and cyclic prefix. CW tones at
%       their physical sensing bin frequencies. Large files (~12 GB / 100s).
%
%     Low-rate (set 'Fs_out'): Generates at a lower sample rate (e.g., 1 MHz)
%       with de-hopped CW tones. Since the transmitter knows its own CW bin
%       hopping pattern, all CW elements are placed at fixed output
%       frequencies regardless of physical bin changes. SC-FDMA contribution
%       is modeled as noise (it's just spectral leakage at CW bandwidth).
%       Files are ~15x smaller and generation is ~15x faster.
%       De-hopping means frequency hops are invisible — phase-continuous
%       CW tones at fixed V frequencies for direct mDopPx processing.
%
%   Inputs:
%     L2out      - L2 output struct from L2_CW_sim
%     CWout      - CW output struct from L2_CW_sim (with v3 extension fields)
%     vehicle_id - Integer vehicle/RSU ID to generate baseband for
%
%   Name-Value Options:
%     'Duration'      - Duration in seconds (default: L2out.params.simseconds)
%     'SNR_dB'        - SNR in dB for AWGN noise floor (default: 20)
%     'DopplerRange'  - [f_start, f_end] Hz for linear sweep (default: [-1000, 1000])
%     'CWPower_dB'    - CW tone power relative to noise (default: -10 full-rate,
%                       20 low-rate). In full-rate mode, relative to SC-FDMA RMS.
%                       In low-rate mode, relative to noise floor (higher = cleaner CW).
%     'OutputDir'     - Output directory (default: script directory)
%     'Filename'      - Custom output filename (default: auto-generated)
%     'Fs_out'        - Output sample rate in Hz (default: [] = native rate).
%                       Set to e.g. 1e6 for low-rate de-hopped mode.
%     'ToneOffset'    - First CW tone output frequency in Hz (default: 10000).
%                       Must be > 4000 to avoid mDopPx DC notch.
%                       Only used in low-rate mode.
%     'ToneSpacing'   - Spacing between CW tones in Hz (default: 10000).
%                       Must be > 4000 to avoid Doppler window overlap.
%                       Only used in low-rate mode (Mode 2: multiple CW elements).
%     'CWContinuous'  - If true, CW tones are generated in ALL subframes
%                       (not just TX subframes). Default: true for low-rate
%                       mode, false for full-rate. Continuous CW avoids the
%                       sinc spectral spreading from 1ms pulsing (which
%                       smears Doppler over ~±500 Hz). TX subframes still
%                       have additional SC-FDMA noise contribution.
%     'MicroDopplerHz'  - Maximum micro-Doppler frequency shift in Hz
%                         (default: 0 = disabled). Adds sinusoidal Doppler
%                         modulation on top of the linear sweep, simulating
%                         rotating/vibrating target components (e.g., spinning
%                         wheel, pedestrian limb swing). Creates the
%                         characteristic "butterfly" pattern in spectrograms.
%                         Typical values: 50-500 Hz.
%     'MicroDopplerRate' - Rotation/vibration rate in Hz (default: 10).
%                          Controls how fast the micro-Doppler oscillates.
%                          Examples: ~10-15 Hz for vehicle wheel at highway
%                          speed, ~1-2 Hz for pedestrian gait cycle.
%
%   Output:
%     info - Struct with generation metadata:
%       .filepath         - Full path to .dat file
%       .fs_Hz            - Output sampling rate
%       .total_samples    - Total samples written
%       .duration_s       - Duration in seconds
%       .cw_freq_offsets  - CW sensing bin frequency offsets (physical, Hz)
%       .cw_bins_used     - Unique sensing bin indices used
%       .doppler_range    - [f_start, f_end] applied
%       .vehicle_id       - Vehicle ID
%       .sensing_mode     - CW sensing mode string
%       .V_frequencies    - CW frequencies for mDopPx V parameter
%       .mode             - 'full-rate' or 'low-rate-dehopped'
%       .N_cw             - Number of CW elements for this vehicle
%
%   Examples:
%     % Full-rate (15.36 MHz, SC-FDMA + CW at physical frequencies):
%     info = generate_cw_baseband(L2out, CWout, 1, 'Duration', 10);
%
%     % Low-rate de-hopped (1 MHz, CW at fixed output tones):
%     info = generate_cw_baseband(L2out, CWout, 1, 'Fs_out', 1e6, 'Duration', 10);
%     % Then in mDopPx: Fs = 1e6; V = info.V_frequencies;
%
%     % With micro-Doppler (spinning wheel at 12 Hz, ±200 Hz max shift):
%     info = generate_cw_baseband(L2out, CWout, 1, 'Fs_out', 1e6, ...
%            'Duration', 10, 'MicroDopplerHz', 200, 'MicroDopplerRate', 12);
%
%   Dependencies:
%     ../common/write_complex_binary.m
%     ../common/cw_phy_params.m (via CWout.cw_params)

    addpath('../common');

    %% ===== PARSE INPUTS =====
    p = inputParser;
    addRequired(p, 'L2out', @isstruct);
    addRequired(p, 'CWout', @isstruct);
    addRequired(p, 'vehicle_id', @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'Duration', [], @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'SNR_dB', 20, @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'DopplerRange', [-1000, 1000], @(x) isnumeric(x) && length(x) == 2);
    addParameter(p, 'CWPower_dB', [], @(x) isnumeric(x) && isscalar(x));
    addParameter(p, 'OutputDir', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Filename', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Fs_out', [], @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'ToneOffset', 10000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'ToneSpacing', 10000, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'CWContinuous', [], @(x) islogical(x) || (isnumeric(x) && isscalar(x)));
    addParameter(p, 'MicroDopplerHz', 0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
    addParameter(p, 'MicroDopplerRate', 10, @(x) isnumeric(x) && isscalar(x) && x > 0);
    parse(p, L2out, CWout, vehicle_id, varargin{:});

    opts = p.Results;

    %% ===== EXTRACT PHY PARAMETERS =====
    if ~isfield(CWout, 'cw_params')
        error('generate_cw_baseband:missingParams', ...
              'CWout.cw_params not found. Run L2_CW_sim (not L2_sim_SPS) first.');
    end
    cw_params = CWout.cw_params;

    M_data = cw_params.M_data;           % 1024 (10MHz) or 2048 (20MHz)
    Fs_native = cw_params.fs_Hz;         % 15.36e6 or 30.72e6
    Nrbs = cw_params.Nrbs;
    Nslch = cw_params.Nslch;

    % Determine operating mode
    if ~isempty(opts.Fs_out) && opts.Fs_out < Fs_native
        gen_mode = 'low-rate';
        Fs = opts.Fs_out;
        samples_per_sf = round(Fs * 1e-3);  % e.g., 1000 at 1 MHz
    else
        gen_mode = 'full-rate';
        Fs = Fs_native;
        samples_per_sf = M_data * 15;       % 15360 (10MHz) or 30720 (20MHz)
    end

    % Default CW power depends on mode
    if isempty(opts.CWPower_dB)
        if strcmp(gen_mode, 'full-rate')
            opts.CWPower_dB = -10;  % CW is 10 dB below SC-FDMA
        else
            opts.CWPower_dB = 20;   % CW is 20 dB above noise floor
        end
    end

    % Default CWContinuous: true for low-rate (avoids sinc spreading from
    % 1ms pulsing), false for full-rate (match physical TX schedule)
    if isempty(opts.CWContinuous)
        cw_continuous = strcmp(gen_mode, 'low-rate');
    else
        cw_continuous = logical(opts.CWContinuous);
    end

    %% ===== DURATION AND SUBFRAME COUNT =====
    Tdur = L2out.params.Tdur;
    simseconds = L2out.params.simseconds;

    if ~isempty(opts.Duration)
        total_subframes = min(floor(opts.Duration * 1000), Tdur);
    else
        total_subframes = Tdur;
    end
    T_total = total_subframes / 1000;  % Total duration in seconds

    %% ===== VEHICLE TX SCHEDULE LOOKUP =====
    if vehicle_id > L2out.params.numEntities
        error('generate_cw_baseband:invalidVehicle', ...
              'vehicle_id %d exceeds numEntities (%d).', ...
              vehicle_id, L2out.params.numEntities);
    end

    % TX subframes (simulation-relative, 1-based)
    tx_subframes = CWout.txSubframes{vehicle_id};
    tx_subchannels = CWout.txSubchannels{vehicle_id};
    tx_L_subCH = CWout.txL_subCH{vehicle_id};

    % Limit to requested duration
    valid_mask = tx_subframes <= total_subframes;
    tx_subframes = tx_subframes(valid_mask);
    tx_subchannels = tx_subchannels(valid_mask);
    tx_L_subCH = tx_L_subCH(valid_mask);

    % Build fast lookup
    is_tx_sf = false(total_subframes, 1);
    is_tx_sf(tx_subframes) = true;

    tx_idx_map = zeros(total_subframes, 1);
    for k = 1:length(tx_subframes)
        tx_idx_map(tx_subframes(k)) = k;
    end

    %% ===== CW BIN LOOKUP =====
    % Reconstruct [subframe, bin] pairs from CW_RH (bins x Tdur, sim-period).
    % Cell value == vehicle_id at (b, sf) means that vehicle transmitted CW
    % on bin b at subframe sf. Collision cells (negative) are not recovered
    % here because they lose the vehicle identity on write.
    cw_at_tx = [];
    if isfield(CWout, 'CW_RH') && ~isempty(CWout.CW_RH)
        [bins, sfs] = find(CWout.CW_RH == vehicle_id);
        if ~isempty(bins)
            cw_at_tx = [double(sfs), double(bins)];
            cw_at_tx = cw_at_tx(cw_at_tx(:,1) <= total_subframes, :);
        end
    elseif strcmp(gen_mode, 'full-rate')
        warning('generate_cw_baseband:noCWRH', ...
            ['CWout.CW_RH is empty and full-rate mode requires physical ' ...
             'bin assignments. Set record_cw_rh_always=true in L2_CW_sim.m.']);
    end

    % Unique physical bins and their frequencies
    if ~isempty(cw_at_tx)
        unique_bins = unique(cw_at_tx(:,2));
    else
        unique_bins = [];
    end

    cw_freq_offsets = zeros(length(unique_bins), 1);
    for k = 1:length(unique_bins)
        cw_freq_offsets(k) = cw_params.bin_freq_offset_Hz(unique_bins(k));
    end

    % Determine N_cw for this vehicle
    if isfield(CWout, 'N_cw_per_vehicle')
        N_cw = CWout.N_cw_per_vehicle(vehicle_id);
    else
        % Infer from max simultaneous bins in any TX subframe
        if ~isempty(cw_at_tx) && ~isempty(tx_subframes)
            N_cw = max(histcounts(cw_at_tx(:,1), [tx_subframes(:); inf]));
            N_cw = max(N_cw, 1);
        else
            N_cw = 1;
        end
    end

    %% ===== DOPPLER PARAMETERS =====
    f_start = opts.DopplerRange(1);
    f_end = opts.DopplerRange(2);
    delta_f_doppler = f_end - f_start;

    % Micro-Doppler: sinusoidal modulation on top of linear sweep
    %   Models rotating/vibrating target components (e.g., spinning wheel).
    %   f_d(t) = f_bulk(t) + f_md * sin(2π * f_rot * t)
    %   where f_md = MicroDopplerHz (max micro-Doppler shift)
    %         f_rot = MicroDopplerRate (rotation/vibration frequency)
    f_md = opts.MicroDopplerHz;
    f_rot = opts.MicroDopplerRate;

    %% ===== NOISE AND CW AMPLITUDE =====
    noise_sigma = 1 / sqrt(2 * 10^(opts.SNR_dB / 10));
    A_cw = 10^(opts.CWPower_dB / 20);

    %% ===== OUTPUT FILE SETUP =====
    if isempty(opts.OutputDir)
        output_dir = fileparts(mfilename('fullpath'));
    else
        output_dir = opts.OutputDir;
    end
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    sensing_mode = 'unknown';
    if isfield(CWout, 'sensing_mode')
        sensing_mode = CWout.sensing_mode;
    end

    if isempty(opts.Filename)
        if strcmp(gen_mode, 'low-rate')
            out_filename = sprintf('cw_baseband_v%d_mode%s_%.0fkHz_%.0fs.dat', ...
                vehicle_id, sensing_mode, Fs/1e3, T_total);
        else
            out_filename = sprintf('cw_baseband_v%d_mode%s_%.0fs.dat', ...
                vehicle_id, sensing_mode, T_total);
        end
    else
        out_filename = opts.Filename;
    end
    output_path = fullfile(output_dir, out_filename);

    %% ===== COMPUTE OUTPUT V FREQUENCIES =====
    if strcmp(gen_mode, 'low-rate')
        % De-hopped: CW tones at fixed output frequencies
        V_out = opts.ToneOffset + (0 : N_cw - 1) * opts.ToneSpacing;
    else
        % Full-rate: CW tones at physical bin frequencies
        V_out = cw_freq_offsets(:)';
    end

    %% ===== PRINT CONFIGURATION =====
    fprintf('\n========================================\n');
    fprintf('  generate_cw_baseband  [%s]\n', upper(gen_mode));
    fprintf('========================================\n');
    fprintf('  Vehicle ID:       %d\n', vehicle_id);
    fprintf('  CW Mode:          %s\n', sensing_mode);
    fprintf('  N_cw:             %d\n', N_cw);
    fprintf('  Duration:         %.1f s (%d subframes)\n', T_total, total_subframes);
    fprintf('  Output Fs:        %.2f MHz\n', Fs / 1e6);
    if strcmp(gen_mode, 'full-rate')
        fprintf('  FFT Size:         %d (data), %d (sensing)\n', M_data, cw_params.M_sensing);
    end
    fprintf('  Samples/subframe: %d\n', samples_per_sf);
    fprintf('  TX subframes:     %d (of %d total)\n', length(tx_subframes), total_subframes);
    fprintf('  CW bins (phys):   %d unique\n', length(unique_bins));
    for k = 1:min(length(unique_bins), 10)
        fprintf('    Bin %d -> %.1f Hz\n', unique_bins(k), cw_freq_offsets(k));
    end
    if length(unique_bins) > 10
        fprintf('    ... (%d more)\n', length(unique_bins) - 10);
    end
    if strcmp(gen_mode, 'low-rate')
        fprintf('  De-hopped V:      [%s] Hz\n', ...
            strjoin(arrayfun(@(x) sprintf('%.0f', x), V_out, ...
            'UniformOutput', false), ', '));
    end
    fprintf('  Doppler sweep:    [%.0f, %.0f] Hz\n', f_start, f_end);
    fprintf('  SNR:              %.1f dB\n', opts.SNR_dB);
    fprintf('  CW Power:         %.1f dB (rel. %s)\n', opts.CWPower_dB, ...
        ternary(strcmp(gen_mode, 'full-rate'), 'SC-FDMA', 'noise'));
    fprintf('  CW Continuous:    %s\n', ternary(cw_continuous, 'YES (all subframes)', 'NO (TX only)'));
    if f_md > 0
        fprintf('  Micro-Doppler:    ±%.0f Hz at %.1f Hz rotation rate\n', f_md, f_rot);
    else
        fprintf('  Micro-Doppler:    OFF (linear sweep only)\n');
    end
    fprintf('  Output:           %s\n', output_path);
    fprintf('  Est. file size:   %.1f MB\n', ...
        total_subframes * samples_per_sf * 8 / 1e6);
    fprintf('----------------------------------------\n');

    %% ===== OPEN OUTPUT FILE =====
    fid = fopen(output_path, 'w', 'ieee-le');
    if fid < 0
        error('generate_cw_baseband:fileError', ...
              'Could not create output file: %s', output_path);
    end

    %% ===== ROUTE TO GENERATION MODE =====
    if strcmp(gen_mode, 'low-rate')
        [total_samples_written, tx_sf_generated, idle_sf_generated, elapsed_total] = ...
            generate_lowrate(fid, total_subframes, samples_per_sf, Fs, T_total, ...
            is_tx_sf, cw_at_tx, N_cw, A_cw, noise_sigma, ...
            f_start, delta_f_doppler, V_out, cw_continuous, f_md, f_rot);
    else
        [total_samples_written, tx_sf_generated, idle_sf_generated, elapsed_total] = ...
            generate_fullrate(fid, total_subframes, samples_per_sf, Fs, T_total, ...
            M_data, cw_params, is_tx_sf, tx_idx_map, tx_subchannels, tx_L_subCH, ...
            Nrbs, cw_at_tx, A_cw, noise_sigma, ...
            f_start, delta_f_doppler, cw_continuous, f_md, f_rot);
    end

    fclose(fid);

    %% ===== BUILD OUTPUT INFO STRUCT =====
    info = struct();
    info.filepath = output_path;
    info.fs_Hz = Fs;
    info.total_samples = total_samples_written;
    info.duration_s = T_total;
    info.cw_freq_offsets = cw_freq_offsets;
    info.cw_bins_used = unique_bins;
    info.doppler_range = [f_start, f_end];
    info.vehicle_id = vehicle_id;
    info.sensing_mode = sensing_mode;
    info.V_frequencies = V_out;
    info.mode = ternary(strcmp(gen_mode, 'low-rate'), 'low-rate-dehopped', 'full-rate');
    info.N_cw = N_cw;
    info.M_data = M_data;
    info.M_sensing = cw_params.M_sensing;
    info.samples_per_subframe = samples_per_sf;
    info.SNR_dB = opts.SNR_dB;
    info.CWPower_dB = opts.CWPower_dB;
    info.cw_continuous = cw_continuous;
    if strcmp(gen_mode, 'low-rate')
        info.tone_offset = opts.ToneOffset;
        info.tone_spacing = opts.ToneSpacing;
    end
    info.micro_doppler_hz = f_md;
    info.micro_doppler_rate = f_rot;

    %% ===== PRINT SUMMARY =====
    file_info = dir(output_path);
    file_size_MB = file_info.bytes / 1e6;

    fprintf('\n========================================\n');
    fprintf('  Generation Complete  [%s]\n', upper(gen_mode));
    fprintf('========================================\n');
    fprintf('  File:             %s\n', output_path);
    fprintf('  File size:        %.1f MB\n', file_size_MB);
    fprintf('  Total samples:    %d\n', total_samples_written);
    fprintf('  Duration:         %.3f s\n', T_total);
    fprintf('  TX subframes:     %d\n', tx_sf_generated);
    fprintf('  Idle subframes:   %d\n', idle_sf_generated);
    fprintf('  Generation time:  %.1f s (%.0fx realtime)\n', ...
        elapsed_total, T_total / max(elapsed_total, 0.001));
    fprintf('\n  --- For mDopPx Configuration ---\n');
    fprintf('  Fs = %.0f;  %% Sampling rate (Hz)\n', Fs);
    fprintf('  V = [%s];  %% CW tone frequencies (Hz)\n', ...
        strjoin(arrayfun(@(x) sprintf('%.0f', x), V_out, ...
        'UniformOutput', false), ', '));
    if strcmp(gen_mode, 'low-rate')
        fprintf('  %% De-hopped: tones at fixed freqs, hops are transparent\n');
    end
    fprintf('========================================\n\n');

end


%% =====================================================================
%  LOW-RATE DE-HOPPED GENERATION
%  =====================================================================
%  CW tones placed at fixed output frequencies (de-hopped).
%  SC-FDMA modeled as bandlimited noise (spectral leakage at CW bandwidth).
%  No FFT/IFFT needed — just sinusoids + noise.
%  ~15x faster and ~15x smaller files than full-rate.

function [total_written, tx_count, idle_count, elapsed] = ...
    generate_lowrate(fid, total_subframes, samples_per_sf, Fs, T_total, ...
    is_tx_sf, cw_at_tx, N_cw, A_cw, noise_sigma, ...
    f_start, delta_f_doppler, V_out, cw_continuous, f_md, f_rot)

    total_written = 0;
    tx_count = 0;
    idle_count = 0;
    global_sample_idx = 0;

    % SC-FDMA leakage adds ~3 dB noise during TX subframes
    % (OFDM sidelobe at 7.5 kHz offset from nearest subcarrier ≈ -3.9 dB)
    tx_noise_boost = sqrt(2);  % +3 dB during TX subframes

    t_start = tic;
    progress_interval = max(1, floor(total_subframes / 20));

    for sf = 1:total_subframes

        % Progress
        if mod(sf, progress_interval) == 0 || sf == total_subframes
            el = toc(t_start);
            fprintf('  [%5.1f%%] SF %d/%d  (%.0fs elapsed, ~%.0fs remaining)\n', ...
                100*sf/total_subframes, sf, total_subframes, el, ...
                el/sf*(total_subframes-sf));
        end

        % Global time vector (phase-continuous)
        t_sf = ((global_sample_idx) : (global_sample_idx + samples_per_sf - 1))' / Fs;

        if is_tx_sf(sf)
            % === TX SUBFRAME: CW tones + boosted noise ===
            tx_count = tx_count + 1;
            signal = zeros(samples_per_sf, 1);

            % Generate CW tones at de-hopped output frequencies
            for k = 1:N_cw
                f_out = V_out(k);
                signal = signal + A_cw * exp(1j * 2 * pi * f_out * t_sf);
            end

            % Noise: AWGN + SC-FDMA leakage boost
            signal = signal + tx_noise_boost * noise_sigma * ...
                (randn(samples_per_sf, 1) + 1i * randn(samples_per_sf, 1));
        else
            % === IDLE SUBFRAME: CW (if continuous) + noise ===
            idle_count = idle_count + 1;

            if cw_continuous
                % CW always on — avoids sinc spectral spreading from pulsing
                signal = zeros(samples_per_sf, 1);
                for k = 1:N_cw
                    f_out = V_out(k);
                    signal = signal + A_cw * exp(1j * 2 * pi * f_out * t_sf);
                end
                signal = signal + noise_sigma * ...
                    (randn(samples_per_sf, 1) + 1i * randn(samples_per_sf, 1));
            else
                % Pulsed CW — only during TX (realistic but causes spectral spreading)
                signal = noise_sigma * ...
                    (randn(samples_per_sf, 1) + 1i * randn(samples_per_sf, 1));
            end
        end

        % Doppler shift: linear sweep + micro-Doppler (continuous phase)
        % f_d(t) = f_start + Δf*t/T + f_md*sin(2π*f_rot*t)
        % φ(t) = 2π*[f_start*t + Δf*t²/(2T)] - (f_md/f_rot)*cos(2π*f_rot*t)
        doppler_phase = 2 * pi * (f_start * t_sf + ...
                        delta_f_doppler * t_sf.^2 / (2 * T_total));
        if f_md > 0
            doppler_phase = doppler_phase - ...
                (f_md / f_rot) * cos(2 * pi * f_rot * t_sf);
        end
        signal = signal .* exp(1j * doppler_phase);

        % Write to file
        interleaved = zeros(2 * samples_per_sf, 1, 'single');
        interleaved(1:2:end) = single(real(signal));
        interleaved(2:2:end) = single(imag(signal));
        fwrite(fid, interleaved, 'float32');

        global_sample_idx = global_sample_idx + samples_per_sf;
        total_written = total_written + samples_per_sf;
    end

    elapsed = toc(t_start);
end


%% =====================================================================
%  FULL-RATE GENERATION (native C-V2X sample rate)
%  =====================================================================
%  Proper SC-FDMA waveforms with DFT precoding, localized subcarrier
%  mapping, IFFT, and cyclic prefix. CW tones at physical sensing bin
%  frequencies. 14 symbols per subframe with LTE Normal CP.

function [total_written, tx_count, idle_count, elapsed] = ...
    generate_fullrate(fid, total_subframes, samples_per_sf, Fs, T_total, ...
    M_data, cw_params, is_tx_sf, tx_idx_map, tx_subchannels, tx_L_subCH, ...
    Nrbs, cw_at_tx, A_cw, noise_sigma, ...
    f_start, delta_f_doppler, cw_continuous, f_md, f_rot)

    total_written = 0;
    tx_count = 0;
    idle_count = 0;
    global_sample_idx = 0;

    % LTE Normal CP lengths
    if M_data == 1024
        cp_first = 80; cp_other = 72;
    else
        cp_first = 160; cp_other = 144;
    end
    cp_lengths = zeros(14, 1);
    cp_lengths(1) = cp_first;
    cp_lengths(2:7) = cp_other;
    cp_lengths(8) = cp_first;
    cp_lengths(9:14) = cp_other;

    assert(sum(cp_lengths) + 14 * M_data == samples_per_sf, ...
        'CP + IFFT samples must equal %d per subframe', samples_per_sf);

    t_start = tic;
    progress_interval = max(1, floor(total_subframes / 20));

    for sf = 1:total_subframes

        % Progress
        if mod(sf, progress_interval) == 0 || sf == total_subframes
            el = toc(t_start);
            fprintf('  [%5.1f%%] SF %d/%d  (%.0fs elapsed, ~%.0fs remaining)\n', ...
                100*sf/total_subframes, sf, total_subframes, el, ...
                el/sf*(total_subframes-sf));
        end

        % Global time vector
        t_sf = ((global_sample_idx) : (global_sample_idx + samples_per_sf - 1))' / Fs;

        if is_tx_sf(sf)
            % === TX SUBFRAME: SC-FDMA + CW ===
            tx_count = tx_count + 1;
            k = tx_idx_map(sf);

            % Allocation
            start_sch = tx_subchannels(k);
            L_sch = tx_L_subCH(k);
            N_alloc_sc = L_sch * Nrbs * 12;

            start_prb = (start_sch - 1) * Nrbs;
            start_sc = start_prb * 12;

            % Precompute FFT bin indices (vectorized)
            sc_indices = start_sc + (0 : N_alloc_sc - 1);
            fft_bins = mod(sc_indices - cw_params.total_data_sc/2 + M_data, M_data) + 1;

            % Generate 14 SC-FDMA symbols
            scfdma_signal = zeros(samples_per_sf, 1);
            sample_offset = 0;

            for sym_idx = 0:13
                cp_len = cp_lengths(sym_idx + 1);

                % Random QPSK
                qpsk_data = (2*randi([0,1], N_alloc_sc, 1) - 1 + ...
                             1i*(2*randi([0,1], N_alloc_sc, 1) - 1)) / sqrt(2);

                % DFT precoding (SC-FDMA)
                dft_precoded = fft(qpsk_data);

                % Subcarrier mapping (localized, vectorized)
                freq_grid = zeros(M_data, 1);
                freq_grid(fft_bins) = dft_precoded;

                % IFFT + cyclic prefix
                time_domain = ifft(freq_grid) * sqrt(M_data);
                sym_with_cp = [time_domain(end - cp_len + 1 : end); time_domain];

                sym_len = cp_len + M_data;
                scfdma_signal(sample_offset + 1 : sample_offset + sym_len) = sym_with_cp;
                sample_offset = sample_offset + sym_len;
            end

            % CW tone(s)
            cw_signal = zeros(samples_per_sf, 1);
            if ~isempty(cw_at_tx)
                cw_mask = cw_at_tx(:,1) == sf;
                bins_this_sf = cw_at_tx(cw_mask, 2);
                for b = 1:length(bins_this_sf)
                    f_cw = cw_params.bin_freq_offset_Hz(bins_this_sf(b));
                    cw_signal = cw_signal + A_cw * exp(1j * 2 * pi * f_cw * t_sf);
                end
            end

            % Combine
            noise = noise_sigma * (randn(samples_per_sf, 1) + 1i * randn(samples_per_sf, 1));
            signal = scfdma_signal + cw_signal + noise;

        else
            % === IDLE SUBFRAME ===
            idle_count = idle_count + 1;

            if cw_continuous && ~isempty(cw_at_tx)
                % CW always on: generate CW tones using most recent bin assignment
                % Find the latest CW bins from before this subframe
                prior_mask = cw_at_tx(:,1) < sf;
                if any(prior_mask)
                    last_sf = max(cw_at_tx(prior_mask, 1));
                    last_bins = sort(cw_at_tx(cw_at_tx(:,1) == last_sf, 2));
                else
                    % No prior TX yet — use first known bins
                    first_sf = min(cw_at_tx(:,1));
                    last_bins = sort(cw_at_tx(cw_at_tx(:,1) == first_sf, 2));
                end
                signal = zeros(samples_per_sf, 1);
                for b = 1:length(last_bins)
                    f_cw = cw_params.bin_freq_offset_Hz(last_bins(b));
                    signal = signal + A_cw * exp(1j * 2 * pi * f_cw * t_sf);
                end
                signal = signal + noise_sigma * ...
                    (randn(samples_per_sf, 1) + 1i * randn(samples_per_sf, 1));
            else
                signal = noise_sigma * ...
                    (randn(samples_per_sf, 1) + 1i * randn(samples_per_sf, 1));
            end
        end

        % Doppler shift: linear sweep + micro-Doppler (continuous phase)
        doppler_phase = 2 * pi * (f_start * t_sf + ...
                        delta_f_doppler * t_sf.^2 / (2 * T_total));
        if f_md > 0
            doppler_phase = doppler_phase - ...
                (f_md / f_rot) * cos(2 * pi * f_rot * t_sf);
        end
        signal = signal .* exp(1j * doppler_phase);

        % Write to file
        interleaved = zeros(2 * samples_per_sf, 1, 'single');
        interleaved(1:2:end) = single(real(signal));
        interleaved(2:2:end) = single(imag(signal));
        fwrite(fid, interleaved, 'float32');

        global_sample_idx = global_sample_idx + samples_per_sf;
        total_written = total_written + samples_per_sf;
    end

    elapsed = toc(t_start);
end


%% ===== HELPER =====
function val = ternary(cond, true_val, false_val)
    if cond
        val = true_val;
    else
        val = false_val;
    end
end
