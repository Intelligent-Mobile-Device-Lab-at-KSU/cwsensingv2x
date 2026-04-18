function params = cw_phy_params(channel_bw_MHz, Nslch, Nrbs)
% CW_PHY_PARAMS  Compute CW sensing element PHY parameters from C-V2X config.
%
% Computes the number, spacing, and subchannel mapping of CW sensing
% elements (bins placed at midpoints between OFDM data subcarriers).
%
% Uses 2xM FFT to achieve 7.5 kHz sensing bin spacing between 15 kHz
% data subcarriers. Each sensing bin falls exactly at the frequency
% midpoint between two adjacent data subcarriers.
%
% Inputs:
%   channel_bw_MHz  - Channel bandwidth: 10 or 20 (MHz)
%   Nslch           - Number of sidelink sub-channels
%   Nrbs            - PRBs per sub-channel
%
% Output:
%   params          - Struct with CW PHY parameters (see fields below)
%
% References:
%   3GPP TS 36.211 Section 9 (SC-FDMA for sidelink)
%   LTE PHY: 15 kHz subcarrier spacing, 12 subcarriers per PRB

    % ====== Validate Inputs ======
    assert(ismember(channel_bw_MHz, [10, 20]), ...
        'channel_bw_MHz must be 10 or 20');
    total_PRB = Nslch * Nrbs;
    if channel_bw_MHz == 10
        assert(total_PRB == 50, ...
            '10 MHz requires Nslch*Nrbs = 50 PRBs (got %d)', total_PRB);
    else
        assert(total_PRB == 100, ...
            '20 MHz requires Nslch*Nrbs = 100 PRBs (got %d)', total_PRB);
    end

    % ====== Core PHY Parameters ======
    params.channel_bw_MHz = channel_bw_MHz;
    params.Nslch = Nslch;
    params.Nrbs = Nrbs;
    params.total_PRB = total_PRB;

    % Data OFDM parameters
    params.delta_f_data = 15e3;          % 15 kHz data subcarrier spacing
    params.sc_per_prb = 12;              % Subcarriers per PRB (LTE standard)
    params.total_data_sc = total_PRB * 12;  % Total data subcarriers
    params.data_sc_per_subchannel = Nrbs * 12;  % Data subcarriers per subchannel

    % FFT sizes
    if channel_bw_MHz == 10
        params.M_data = 1024;            % Data FFT size (10 MHz)
        params.fs_Hz = 15.36e6;          % Sampling rate (10 MHz)
    else
        params.M_data = 2048;            % Data FFT size (20 MHz)
        params.fs_Hz = 30.72e6;          % Sampling rate (20 MHz)
    end

    % ====== CW Sensing Parameters ======
    % 2xM FFT gives 7.5 kHz spacing (midpoints between 15 kHz data SCs)
    params.M_sensing = 2 * params.M_data;
    params.delta_f_sensing = params.fs_Hz / params.M_sensing;  % = 7.5 kHz

    % Sensing bins are the midpoints between adjacent data subcarriers
    % N data subcarriers -> N-1 midpoint sensing bins
    params.total_sensing_bins = params.total_data_sc - 1;

    % ====== Bin-to-Subchannel Mapping ======
    % Each subchannel spans data_sc_per_subchannel data subcarriers.
    % Sensing bin b sits between data subcarrier b and b+1.
    % If both subcarriers are in the same subchannel, the bin belongs there.
    % Boundary bins (between subchannels) are assigned to the lower subchannel.
    %
    % Data SC layout (1-based):
    %   Subchannel 1: data SCs 1 .. data_sc_per_subchannel
    %   Subchannel 2: data SCs (data_sc_per_subchannel+1) .. 2*data_sc_per_subchannel
    %   ...
    % Sensing bin b is between data SC b and b+1.
    % Subchannel of data SC b: ceil(b / data_sc_per_subchannel)

    sc_per_sch = params.data_sc_per_subchannel;
    bin_indices = 1:params.total_sensing_bins;

    % Subchannel of the LEFT data subcarrier of each bin
    left_sc_subchannel = ceil(bin_indices / sc_per_sch);

    % Assign bin to its left data subcarrier's subchannel
    % (boundary bins between subchannels go to the lower subchannel)
    params.bin_to_subchannel = left_sc_subchannel;

    % ====== Per-Subchannel Bin Ranges ======
    % For each subchannel, find the range of sensing bins that map to it
    params.subchannel_bin_range = zeros(Nslch, 2);  % [start_bin, end_bin]
    params.bins_per_subchannel = zeros(Nslch, 1);

    for s = 1:Nslch
        bin_mask = (params.bin_to_subchannel == s);
        if any(bin_mask)
            params.subchannel_bin_range(s, 1) = find(bin_mask, 1, 'first');
            params.subchannel_bin_range(s, 2) = find(bin_mask, 1, 'last');
            params.bins_per_subchannel(s) = sum(bin_mask);
        end
    end

    % ====== Frequency Mapping ======
    % Absolute frequency of sensing bin b (relative to channel center):
    %   f_bin(b) = (b - total_data_sc/2) * delta_f_data + delta_f_data/2
    % Simplified: sensing bin b is at the midpoint between data SC b and b+1
    % This is precomputed as an anonymous function for downstream use
    center_sc = params.total_data_sc / 2;  % Center data subcarrier index
    params.bin_freq_offset_Hz = @(b) (b - center_sc) * params.delta_f_data + ...
                                     params.delta_f_data / 2;

    % ====== Summary ======
    params.summary = sprintf(['CW PHY: %d MHz, M_data=%d, M_sensing=%d, ' ...
        'Δf_sensing=%.1f kHz\n  %d data SCs, %d sensing bins, ' ...
        '%d subchannels (avg %.0f bins/subch)'], ...
        channel_bw_MHz, params.M_data, params.M_sensing, ...
        params.delta_f_sensing/1e3, params.total_data_sc, ...
        params.total_sensing_bins, Nslch, mean(params.bins_per_subchannel));
end
