function [NextRS, NextreTXRS, newRsc] = sps_reselect_cw(RH, CW_subchannel_RH, vehicleIdx, currSF, Nslch, re_tx, RRI_ms, L_subCH, Global_SCI_Memory)
% SPS_RESELECT_CW  Perform SB-SPS Mode 4 resource reselection with CW-aware RSSI scoring
%
% Implements the sensing-based semi-persistent scheduling resource selection
% procedure per 3GPP TS 36.213 Section 14.1.1.6, extended for multi-subchannel
% transmissions where each TX occupies L_subCH contiguous subchannels.
%
% This variant incorporates CW (continuous wave / jammer) occupancy into the
% RSSI measurement so that subchannels occupied by CW interference are scored
% higher (worse), steering selection away from jammed resources.
%
% =========================================================================
% ALGORITHM PSEUDOCODE (same as sps_reselect, with CW-aware Step 5)
% =========================================================================
%
%   STEP 1 — WINDOWS & CANDIDATES (identical to sps_reselect)
%   STEP 2 — HALF-DUPLEX EXCLUSION (phase-corrected absolute projection)
%   STEP 3 — SCI RESERVATION EXCLUSION (Global_SCI_Memory lookup)
%   STEP 4 — 20% THRESHOLD RULE (preserves HD exclusions on fallback)
%   STEP 5 — RSSI RANKING: combines data + CW occupancy via OR logic
%   STEP 6 — RANDOM SELECTION from bottom 20%
%
% Inputs:
%   RH                - Resource history matrix, Nslch x totalSubframes
%   CW_subchannel_RH  - CW occupancy at subchannel level, Nslch x totalSubframes
%                       (non-zero values indicate CW active in that subchannel/subframe)
%   vehicleIdx        - ID of the vehicle performing reselection
%   currSF            - Current subframe number (1-based column index in RH)
%   Nslch             - Number of sidelink sub-channels
%   re_tx             - Retransmission offset in subframes
%   RRI_ms            - Resource reservation interval in ms (100 for 10Hz, 1000 for 1Hz)
%   L_subCH           - Number of contiguous subchannels per transmission
%   Global_SCI_Memory - 3D logical array (numEntities x Nslch x totalSF).
%                       Entry (v, sc, sf) = true means vehicle v successfully
%                       decoded the SCI transmitted at (sc, sf).
%
% Outputs:
%   NextRS      - Linear index of selected primary resource (start subchannel)
%   NextreTXRS  - Linear index of retransmission resource (same start subchannel)
%   newRsc      - New SL_RESOURCE_RESELECTION_COUNTER value
%
% References:
%   3GPP TS 36.213 Section 14.1.1.6
%   3GPP TS 36.321 Section 5.14.1.1
%   3GPP TS 36.213 Section 14.1.1.4c (SCI Format 1)

    % ====== STEP 1: Define Sensing and Selection Windows ======
    sw_start = max(1, currSF - 1000);
    sw_end   = currSF - 1;
    sensingWindow = sw_start : sw_end;

    T1 = 1;
    T2 = 100;
    selectionWindow = (currSF + T1) : (currSF + T2);

    numStartPositions = max(1, Nslch - L_subCH + 1);
    totalCandidates   = T2 * numStartPositions;
    candidateList     = 1 : totalCandidates;

    % ====== STEP 2: Half-Duplex Exclusion (Phase-Corrected) ======
    RH_sensing = RH(:, sensingWindow);
    tx_sf_cols = find(any(RH_sensing == vehicleIdx, 1));

    excludedList = [];
    if ~isempty(tx_sf_cols)
        abs_tx_sf = sensingWindow(tx_sf_cols);
        future_hd_sf = abs_tx_sf + RRI_ms;

        valid_hd_idx = find(future_hd_sf >= selectionWindow(1) & ...
                            future_hd_sf <= selectionWindow(end));

        if ~isempty(valid_hd_idx)
            sel_offsets = future_hd_sf(valid_hd_idx) - selectionWindow(1);
            exclude_matrix = bsxfun(@plus, sel_offsets(:) * numStartPositions, 1:numStartPositions);
            excludedList = exclude_matrix(:)';
        end
    end

    HD_excludedList = unique(excludedList);

    % ====== STEP 3: SCI Reservation Exclusion (Memory Lookup) ======
    decoded_sci_mask = squeeze(Global_SCI_Memory(vehicleIdx, :, sensingWindow));
    [sci_sc_idx, sci_sf_idx] = find(decoded_sci_mask);

    sci_excludedList = [];
    if ~isempty(sci_sc_idx)
        abs_sensing_sf = sensingWindow(sci_sf_idx);
        projected_sf = abs_sensing_sf + RRI_ms;

        in_window = projected_sf >= selectionWindow(1) & ...
                    projected_sf <= selectionWindow(end);
        proj_sc = sci_sc_idx(in_window);
        proj_sf_offset = projected_sf(in_window) - selectionWindow(1);

        for ii = 1:length(proj_sc)
            reserved_k = proj_sc(ii);
            t_offset   = proj_sf_offset(ii);
            start_min = max(1, reserved_k - L_subCH + 1);
            start_max = min(numStartPositions, reserved_k);
            invalid_linear = (start_min:start_max) + t_offset * numStartPositions;
            sci_excludedList = [sci_excludedList, invalid_linear]; %#ok<AGROW>
        end
    end

    excludedList = unique([HD_excludedList, sci_excludedList]);
    validCandidates = setdiff(candidateList, excludedList);

    % ====== STEP 4: 20% Threshold Rule (preserves HD exclusions) ======
    if length(validCandidates) < ceil(0.20 * totalCandidates)
        validCandidates = setdiff(candidateList, HD_excludedList);
    end

    % ====== STEP 5: RSSI Ranking (CW-Aware, Vectorized with cumsum) ======
    % Combine data occupancy and CW occupancy using OR logic so that
    % subframes where BOTH data and CW are present are counted only once.
    CW_sensing = CW_subchannel_RH(:, sensingWindow);
    combined_occupancy = sum((RH_sensing ~= 0) | (CW_sensing ~= 0), 2);  % Nslch x 1

    cumsum_occ = [0; cumsum(combined_occupancy)];
    rssi_per_start = cumsum_occ(L_subCH+1 : Nslch+1) - cumsum_occ(1 : numStartPositions);

    cand_start_scs = mod(validCandidates - 1, numStartPositions) + 1;
    rssi_scores = rssi_per_start(cand_start_scs)';

    [~, sortIdx]    = sort(rssi_scores, 'ascend');
    bottom20_count  = max(1, floor(0.20 * length(validCandidates)));
    best_candidates = validCandidates(sortIdx(1:bottom20_count));

    % ====== STEP 6: Random Selection ======
    chosen_candidate = best_candidates(randi(length(best_candidates)));

    chosen_start_sc = mod(chosen_candidate - 1, numStartPositions) + 1;
    chosen_sf_offset = floor((chosen_candidate - 1) / numStartPositions);
    absolute_sf = selectionWindow(1) + chosen_sf_offset;

    NextRS = idx_helpers.to_linear(chosen_start_sc, absolute_sf, Nslch);
    NextreTXRS = NextRS + re_tx * Nslch;
    newRsc = idx_helpers.draw_reselection_counter(RRI_ms);

end
