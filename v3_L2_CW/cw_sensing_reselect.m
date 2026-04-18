function chosen_bins = cw_sensing_reselect(CW_RH, vehicleIdx, currSF, N_cw, candidate_bins)
% CW_SENSING_RESELECT  SPS-like sensing and selection for CW sensing bins
%
% Mirrors the SB-SPS Mode 4 resource selection algorithm (3GPP TS 36.213
% Section 14.1.1.6) adapted for CW (Continuous Wave) sensing elements
% instead of data subchannels. Used by Mode 1b, 1c, 2b, 2c.
%
% This function handles BIN SELECTION only. Dwell counter management is
% the caller's responsibility (managed in L2_CW_sim.m). CW sensing has
% no SCI, no packet type, and no RRI — it operates in a separate
% scheduling domain from DATA-SPS.
%
% Vehicles sense the CW resource history (CW_RH matrix) over up to the
% last 1000 subframes (1ms each) to find bins with the lowest occupancy,
% select from the bottom 20% of candidates (lowest interference), then
% randomly pick N_cw bins.
%
% Inputs:
%   CW_RH          - CW resource history matrix, total_sensing_bins x
%                     total_subframes. Values: 0 = empty, vehicleID =
%                     occupied by that vehicle, negative = collision.
%   vehicleIdx     - ID of the vehicle performing reselection (used for
%                     half-duplex exclusion to avoid self-bias).
%   currSF         - Current subframe number (1-based column index in CW_RH).
%   N_cw           - Number of CW elements to select (1 for Mode 1, 2-8
%                     for Mode 2).
%   candidate_bins - Vector of eligible sensing bin indices (row indices
%                     into CW_RH). For Mode 1b/2b: bins within allocated
%                     PRBs. For Mode 1c/2c: all bins (1:total_sensing_bins).
%
% Outputs:
%   chosen_bins    - 1 x N_cw vector of selected sensing bin indices.
%                     Empty if candidate_bins is empty.
%
% Algorithm (mirrors 3GPP TS 36.213 Sec 14.1.1.6 adapted for CW bins):
%   1. Define sensing window: up to last 1000 subframes before currSF
%   2a. Validate candidate_bins against CW_RH row range
%   2b. Extract CW_RH submatrix for valid candidates over sensing window
%   3. Half-duplex exclusion: mask out entries where this vehicle was
%      already transmitting CW (to avoid self-bias in occupancy count)
%   4. Compute occupancy per candidate bin (count non-zero entries)
%   5. Sort candidates by occupancy ascending (lowest first)
%   6. Select bottom 20% (at least 1 candidate)
%   7. Randomly pick N_cw unique bins from the best candidates

    % ====== Edge Case: Empty candidate list ======
    if isempty(candidate_bins)
        chosen_bins = [];
        return;
    end

    % ====== Edge Case: No history available ======
    % If CW_RH is empty or the sensing window clamps to zero-length,
    % there is no history to evaluate. Fall back to random selection.
    [total_bins, total_sf] = size(CW_RH);
    if total_bins == 0 || total_sf == 0
        N_pick = min(N_cw, length(candidate_bins));
        perm = randperm(length(candidate_bins), N_pick);
        chosen_bins = candidate_bins(perm);
        chosen_bins = chosen_bins(:)';  % Ensure row vector
        return;
    end

    % ====== STEP 1: Define Sensing Window (up to last 1000 subframes) ======
    sw_start = max(1, currSF - 1000);
    sw_end   = currSF - 1;
    sensing_cols = sw_start : sw_end;

    % Clamp to valid column range. Also catches currSF <= 1 where
    % sw_end = 0 produces an empty range.
    sensing_cols = sensing_cols(sensing_cols >= 1 & sensing_cols <= total_sf);
    if isempty(sensing_cols)
        % No valid sensing columns; fall back to random
        N_pick = min(N_cw, length(candidate_bins));
        perm = randperm(length(candidate_bins), N_pick);
        chosen_bins = candidate_bins(perm);
        chosen_bins = chosen_bins(:)';
        return;
    end

    % ====== STEP 2a: Validate candidate_bins against CW_RH row range ======
    valid_mask = candidate_bins >= 1 & candidate_bins <= total_bins;
    candidate_bins = candidate_bins(valid_mask);
    if isempty(candidate_bins)
        chosen_bins = [];
        return;
    end

    % ====== STEP 2b: Extract CW_RH submatrix ======
    % Rows = candidate bins, cols = sensing window subframes
    CW_sub = CW_RH(candidate_bins, sensing_cols);

    % ====== STEP 3: Half-Duplex Exclusion ======
    % Mask out entries where this vehicle was transmitting CW to avoid
    % inflating its own occupancy count (self-bias exclusion).
    % Set those entries to 0 so they don't count as occupied.
    self_mask = (CW_sub == vehicleIdx);
    CW_sub(self_mask) = 0;

    % ====== STEP 4: Compute Occupancy per Candidate Bin ======
    % Count non-zero entries across the sensing window for each candidate.
    % Non-zero includes both positive (single occupant) and negative (collision).
    occupancy = sum(CW_sub ~= 0, 2);  % N_candidates x 1

    % ====== STEP 5: Sort Candidates by Occupancy (ascending) ======
    [~, sort_idx] = sort(occupancy, 'ascend');

    % ====== STEP 6: Select Bottom 20% (lowest interference) ======
    num_candidates = length(candidate_bins);
    bottom20_count = max(1, floor(0.20 * num_candidates));
    best_indices = sort_idx(1:bottom20_count);
    best_bins = candidate_bins(best_indices);

    % ====== STEP 7: Random Selection of N_cw Unique Bins ======
    N_pick = min(N_cw, length(best_bins));
    perm = randperm(length(best_bins), N_pick);
    chosen_bins = best_bins(perm);
    chosen_bins = chosen_bins(:)';  % Ensure 1 x N_cw row vector

end
