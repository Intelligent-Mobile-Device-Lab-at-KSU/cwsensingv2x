function [NextRS, NextreTXRS, newRsc] = sps_reselect(RH, vehicleIdx, currSF, Nslch, re_tx, RRI_ms, L_subCH, Global_SCI_Memory)
% SPS_RESELECT  Perform SB-SPS Mode 4 resource reselection (multi-subchannel)
%
% Implements the sensing-based semi-persistent scheduling resource selection
% procedure per 3GPP TS 36.213 Section 14.1.1.6, extended for multi-subchannel
% transmissions where each TX occupies L_subCH contiguous subchannels.
%
% =========================================================================
% ALGORITHM PSEUDOCODE
% =========================================================================
%
%   STEP 1 — WINDOWS & CANDIDATES
%     sensingWindow  = [currSF-1000 ... currSF-1]   (look back 1 second)
%     selectionWindow = [currSF+1 ... currSF+100]    (look ahead T2=100 sf)
%     candidates = all (startSubchannel, subframeOffset) pairs
%                  where startSubchannel in [1, Nslch-L_subCH+1]
%
%   STEP 2 — HALF-DUPLEX EXCLUSION
%     Find subframes in sensingWindow where this vehicle transmitted.
%     Project each forward by RRI_ms using absolute subframe arithmetic:
%       future_hd_sf = abs_tx_sf + RRI_ms
%     If future_hd_sf lands inside selectionWindow, exclude ALL start
%     positions at that subframe (we will be transmitting — can't receive).
%     Save as HD_excludedList (physical constraint, NEVER relaxed).
%
%     CRITICAL: Do NOT use modulo arithmetic for this projection. The gap
%     between sensingWindow(1) and selectionWindow(1) is 1001 subframes,
%     which is not divisible by typical RRI values (100, 200, 1000).
%     Modulo introduces a phase-shift error.
%
%   STEP 3 — SCI RESERVATION EXCLUSION (memory lookup)
%     Read this vehicle's causal decode memory from Global_SCI_Memory.
%     (Decode decisions were made once at TX time with a flat PDR draw —
%      no rand() calls here, so results are deterministic and causal.)
%     For each decoded SCI at (subchannel K, absolute subframe T_sci):
%       Project forward: T_sci + RRI_ms
%       If it lands in selectionWindow, exclude any candidate whose
%       L_subCH-wide block overlaps subchannel K:
%         invalidate starts from max(1, K-L_subCH+1) to K
%     Save as sci_excludedList.
%
%   STEP 4 — 20% THRESHOLD RULE
%     validCandidates = candidateList minus (HD + SCI exclusions)
%     If |validCandidates| < 20% of total:
%       Drop SCI exclusions but KEEP HD exclusions.
%       (Half-duplex is a hardware constraint; SCI is a threshold-based
%        exclusion that the spec allows relaxing by raising 3 dB.)
%
%   STEP 5 — RSSI PROXY RANKING
%     Score each start position by historical occupancy in the sensing
%     window (cumsum trick over L_subCH contiguous subchannels).
%     Sort validCandidates ascending, keep bottom 20% (quietest slots).
%
%   STEP 6 — RANDOM SELECTION
%     Pick one candidate randomly from the bottom-20% pool.
%     NextRS     = linear RH index of chosen (startSubchannel, subframe)
%     NextreTXRS = NextRS + re_tx * Nslch  (same subchannel, re_tx sf later)
%     newRsc     = draw SL_RESOURCE_RESELECTION_COUNTER per TS 36.321
%
% =========================================================================
% THE ROLE OF RRI_ms
% =========================================================================
%
%   RRI_ms (Resource Reservation Interval, in milliseconds) is the heartbeat
%   of SPS. A vehicle that transmits at subframe T will transmit again at
%   T + RRI_ms. This periodicity drives two key mechanisms:
%
%     - Step 2 (half-duplex): If we transmitted at T_past, we predict we
%       will transmit again at T_past + RRI_ms. We cannot receive during
%       our own TX, so that future subframe is a blind spot — exclude it.
%
%     - Step 3 (SCI exclusion): If we decoded another vehicle's SCI at
%       T_sci, that vehicle will occupy the same resource at T_sci + RRI_ms.
%       We should avoid picking that slot to prevent a collision.
%
%   Typical values: 100 ms (10 Hz BSM), 200 ms (5 Hz), 1000 ms (1 Hz).
%   The value is per (vehicle, packetType) and passed by the caller.
%
% =========================================================================
%
% Inputs:
%   RH                - Resource history matrix, Nslch x totalSubframes.
%                       Each entry is a vehicle ID (>0), collision marker
%                       (<0), or empty (0).
%   vehicleIdx        - ID of the vehicle performing reselection (1-based)
%   currSF            - Current subframe number (1-based column index in RH)
%   Nslch             - Number of sidelink sub-channels
%   re_tx             - Retransmission offset in subframes
%   RRI_ms            - Resource reservation interval in ms. The SPS repeat
%                       period: a TX at subframe T repeats at T + RRI_ms.
%                       (100 for 10 Hz, 200 for 5 Hz, 1000 for 1 Hz)
%   L_subCH           - Number of contiguous subchannels per transmission
%   Global_SCI_Memory - 3D logical array (numEntities x Nslch x totalSF).
%                       Entry (v, sc, sf) = true means vehicle v successfully
%                       decoded the SCI transmitted at (sc, sf). Written once
%                       at TX time in L2_sim_SPS.m with a flat PDR draw.
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
    T2 = 100;   % Selection window: next 100 subframes (common implementation choice)
    selectionWindow = (currSF + T1) : (currSF + T2);

    % Candidate list: groups of L_subCH contiguous subchannels x T2 subframes
    % A candidate is identified by (start_subchannel, subframe_offset)
    % Valid start subchannels: 1 to (Nslch - L_subCH + 1)
    numStartPositions = max(1, Nslch - L_subCH + 1);
    totalCandidates   = T2 * numStartPositions;
    candidateList     = 1 : totalCandidates;

    % ====== STEP 2: Half-Duplex Exclusion (Vectorized and Phase-Corrected) ======
    % Extract sensing window submatrix: Nslch x len(sensingWindow)
    RH_sensing = RH(:, sensingWindow);

    % Find columns (subframes in sensing window) where this vehicle transmitted
    tx_sf_cols = find(any(RH_sensing == vehicleIdx, 1));

    excludedList = [];
    if ~isempty(tx_sf_cols)
        % Get absolute subframe numbers of past transmissions
        abs_tx_sf = sensingWindow(tx_sf_cols);

        % Project forward by RRI_ms using absolute subframe arithmetic
        % (avoids phase-shift bug from modulo on relative indices —
        %  the 1001-subframe gap between sensingWindow(1) and selectionWindow(1)
        %  is not divisible by typical RRI values)
        future_hd_sf = abs_tx_sf + RRI_ms;

        % Keep only the blind spots that actually land inside our selection window
        valid_hd_idx = find(future_hd_sf >= selectionWindow(1) & ...
                            future_hd_sf <= selectionWindow(end));

        if ~isempty(valid_hd_idx)
            % Calculate 0-based offset relative to the start of the selection window
            sel_offsets = future_hd_sf(valid_hd_idx) - selectionWindow(1);

            % Exclude ALL start positions at those time offsets
            exclude_matrix = bsxfun(@plus, sel_offsets(:) * numStartPositions, 1:numStartPositions);
            excludedList = exclude_matrix(:)';
        end
    end

    % Save half-duplex exclusions (physical constraint — never relaxed)
    HD_excludedList = unique(excludedList);

    % ====== STEP 3: SCI Reservation Exclusion (Memory Lookup) ======
    % Read this vehicle's permanent decode memory for the sensing window.
    % Global_SCI_Memory is written at TX time with a flat PDR draw, so
    % decode decisions are causal and deterministic — no re-rolling here.
    decoded_sci_mask = squeeze(Global_SCI_Memory(vehicleIdx, :, sensingWindow));
    % decoded_sci_mask is Nslch x len(sensingWindow)

    % Find (subchannel, sensing-window-column) of decoded SCIs
    [sci_sc_idx, sci_sf_idx] = find(decoded_sci_mask);

    sci_excludedList = [];
    if ~isempty(sci_sc_idx)
        % Project forward by RRI_ms using absolute subframe numbers
        abs_sensing_sf = sensingWindow(sci_sf_idx);
        projected_sf = abs_sensing_sf + RRI_ms;

        % Filter to reservations landing inside selection window
        in_window = projected_sf >= selectionWindow(1) & ...
                    projected_sf <= selectionWindow(end);
        proj_sc = sci_sc_idx(in_window);
        proj_sf_offset = projected_sf(in_window) - selectionWindow(1);  % 0-based

        % For each decoded SCI at subchannel K, invalidate candidate starts
        % from max(1, K-L_subCH+1) to min(numStartPositions, K)
        for ii = 1:length(proj_sc)
            reserved_k = proj_sc(ii);
            t_offset   = proj_sf_offset(ii);
            start_min = max(1, reserved_k - L_subCH + 1);
            start_max = min(numStartPositions, reserved_k);
            invalid_linear = (start_min:start_max) + t_offset * numStartPositions;
            sci_excludedList = [sci_excludedList, invalid_linear]; %#ok<AGROW>
        end
    end

    % Combine HD exclusions and SCI exclusions
    excludedList = unique([HD_excludedList, sci_excludedList]);

    % ====== Apply Exclusions ======
    validCandidates = setdiff(candidateList, excludedList);

    % ====== STEP 4: 20% Threshold Rule ======
    % Per 3GPP TS 36.213 Sec 14.1.1.6:
    %   If remaining candidates < 20% of total, increase RSSI/SCI exclusion
    %   threshold by 3 dB and retry. Half-duplex exclusions are physical
    %   constraints and must never be relaxed.
    if length(validCandidates) < ceil(0.20 * totalCandidates)
        % Fall back to HD-only exclusions (relax SCI threshold)
        validCandidates = setdiff(candidateList, HD_excludedList);
    end

    % ====== STEP 5: RSSI Ranking (Vectorized with cumsum) ======
    % Compute occupancy count per subchannel over the entire sensing window
    occupancy_per_subchannel = sum(RH_sensing ~= 0, 2);  % Nslch x 1

    % Efficient sum over L_subCH contiguous subchannels using cumulative sum
    % rssi_per_start(k) = sum of occupancy for subchannels k to k+L_subCH-1
    cumsum_occ = [0; cumsum(occupancy_per_subchannel)];
    rssi_per_start = cumsum_occ(L_subCH+1 : Nslch+1) - cumsum_occ(1 : numStartPositions);
    % rssi_per_start is numStartPositions x 1

    % Map each candidate to its start subchannel and look up group RSSI
    cand_start_scs = mod(validCandidates - 1, numStartPositions) + 1;  % 1-based
    rssi_scores = rssi_per_start(cand_start_scs)';

    % Sort ascending (lowest interference first), select bottom 20%
    [~, sortIdx]    = sort(rssi_scores, 'ascend');
    bottom20_count  = max(1, floor(0.20 * length(validCandidates)));
    best_candidates = validCandidates(sortIdx(1:bottom20_count));

    % ====== STEP 6: Random Selection ======
    chosen_candidate = best_candidates(randi(length(best_candidates)));

    % Convert local candidate index to global RH linear index
    chosen_start_sc = mod(chosen_candidate - 1, numStartPositions) + 1;
    chosen_sf_offset = floor((chosen_candidate - 1) / numStartPositions);  % 0-based
    absolute_sf = selectionWindow(1) + chosen_sf_offset;

    % NextRS is the linear index of the START subchannel
    % The transmission occupies subchannels chosen_start_sc to chosen_start_sc + L_subCH - 1
    NextRS = idx_helpers.to_linear(chosen_start_sc, absolute_sf, Nslch);

    % Retransmission: same start subchannel, re_tx subframes later
    % Per 3GPP TS 36.213 Sec 14.1.1.4c (SCI Format 1)
    NextreTXRS = NextRS + re_tx * Nslch;

    % Draw new reselection counter per 3GPP TS 36.321 Sec 5.14.1.1
    newRsc = idx_helpers.draw_reselection_counter(RRI_ms);

end
