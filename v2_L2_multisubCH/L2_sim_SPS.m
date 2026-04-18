% ============================================================================
% L2_sim_SPS.m - 3GPP Rel 14 PC5 Sidelink Mode 4 SPS Resource Scheduling Sim
% ============================================================================
% Simulates Semi-Persistent Scheduling (SPS) Mode 4 resource selection for
% C-V2X vehicular communications with MULTI-SUBCHANNEL support (L_subCH >= 1).
% Each transmission occupies L_subCH contiguous subchannels determined by
% SAE message size, MCS index, and subchannel PRB configuration.
%
% Outputs resource allocation data for downstream CW sensing and microDoppler
% simulations.
%
% Bug fixes applied (vs original):
%   Bug 1: Trigger condition off-by-one on higher subchannels
%   Bug 2: RSSI scoring column-major indexing error
%   Bug 3: Half-duplex exclusion conflating subchannel/subframe dims
%   Bug 4: Retransmission on random subchannel (now same subchannel per spec)
%   Bug 5: Missing 1Hz reselection logic
%   Bug 6: Reselection counter range not parameterized by RRI
%   Bug 7: 20% rule incorrectly counting as packet drop
%   Bug 8: Inconsistent collision handling and missing retx writes to RH
%   Bug 9: Hardcoded Nslch-dependent offsets
%
% Multi-subchannel enhancements:
%   - SAE J2735 message size model per message type
%   - TBS-based L_subCH computation per 3GPP TS 36.213 Table 7.1.7.2.1-1
%   - Contiguous subchannel group selection in sps_reselect
%   - Multi-row RH writes with per-subchannel collision detection
%
% Dependencies:
%   idx_helpers.m        - Index conversions + L_subCH computation
%   sps_reselect.m       - SB-SPS Mode 4 reselection (multi-subchannel)
%   L2_plot_results.m    - Visualization (8 graphs)
%   L2_output_interface.m - Output struct construction and file export
%   L2_interactive_grid.m - Interactive resource grid viewer
%
% References:
%   3GPP TS 36.213 Section 14.1.1.6 (Resource selection for V2X sidelink)
%   3GPP TS 36.321 Section 5.14.1.1 (SL_RESOURCE_RESELECTION_COUNTER)
%   3GPP TS 36.213 Section 14.1.1.4c (SCI Format 1)
%   3GPP TS 36.300 Section 23.14 (V2X sidelink communication)
%   3GPP TS 36.213 Table 7.1.7.2.1-1 (TBS determination)
%   SAE J2735 (V2X message set dictionary)
% ============================================================================

clear; clc; close all;
tic;

%% ===== 1. SIMULATION PARAMETERS =====

Tsense_history = 1000;   % Sensing window history (ms) = 1000 subframes
Tepoch = 100;            % Epoch period (ms)
simseconds = 100;        % Simulation duration in seconds
Tdur = 1000 * simseconds;  % Simulation duration in subframes

% PHY parameters
Nslch = 10;              % Number of sidelink sub-channels
Nrbs = 5;                % Resource blocks per sub-channel
PRB = Nslch * Nrbs;      % Total PRBs per sub-frame (10MHz=50, 20MHz=100)

% NOTE: With Nrbs=5 and MCS 7, BSMs need L_subCH=3 (3 contiguous subchannels).
% For lower L_subCH, consider Nrbs=10 (Nslch=5) or higher MCS.
% 10 MHz channel: Nslch=10, Nrbs=5 OR Nslch=5, Nrbs=10 are both valid.
% 20 MHz channel: Nslch=20, Nrbs=5 OR Nslch=10, Nrbs=10, etc.

% System model
% Nr_epoch and Nepochs_in_history removed — no longer needed without pre-sim

% Network
Vnum = 30;              % Number of vehicles (OBUs)
RSUnum = 2;              % Number of roadside units (RSUs)
re_tx = 3;               % Retransmission offset in subframes
PDR_flat = 0.9;          % Flat packet delivery ratio for SCI decode (Step 3)

% RH matrix padding (accommodates selection window T2=100 + retx)
RH_padding = 200;

% --- Multi-Subchannel / TBS Parameters ---
MCS_index = 7;            % MCS index (7 = QPSK, moderate code rate)
bits_per_prb = 260;       % Approx bits per PRB at MCS 7 (3GPP TS 36.213 Table 7.1.7.2.1-1)

% SAE J2735 typical message sizes (bytes, including MAC/RLC/PDCP headers ~12B)
% OBU:   BSM    SRM    PVD    RSA    TMSG
obu_msg_bytes = [300,   400,   500,   300,   200];

% RSU:   SPAT   MAP    SSM    TIM    RTCM   PDM    RSA    TMSG
rsu_msg_bytes = [600,   2000,  400,   1500,  800,   500,   300,   200];

% Compute L_subCH for each message type
obu_L_subCH_table = zeros(1, length(obu_msg_bytes));
for k = 1:length(obu_msg_bytes)
    obu_L_subCH_table(k) = idx_helpers.compute_L_subCH(obu_msg_bytes(k), Nrbs, bits_per_prb);
    obu_L_subCH_table(k) = min(obu_L_subCH_table(k), Nslch);
end

rsu_L_subCH_table = zeros(1, length(rsu_msg_bytes));
for k = 1:length(rsu_msg_bytes)
    rsu_L_subCH_table(k) = idx_helpers.compute_L_subCH(rsu_msg_bytes(k), Nrbs, bits_per_prb);
    rsu_L_subCH_table(k) = min(rsu_L_subCH_table(k), Nslch);
end

fprintf('=== Multi-Subchannel Configuration ===\n');
fprintf('  Nslch=%d, Nrbs=%d, MCS=%d (%d bits/PRB)\n', Nslch, Nrbs, MCS_index, bits_per_prb);
obu_names_short = {'BSM','SRM','PVD','RSA','TMSG'};
for k = 1:length(obu_msg_bytes)
    fprintf('  OBU %-4s: %4d bytes -> L_subCH = %d\n', obu_names_short{k}, obu_msg_bytes(k), obu_L_subCH_table(k));
end
rsu_names_short = {'SPAT','MAP','SSM','TIM','RTCM','PDM','RSA','TMSG'};
for k = 1:length(rsu_msg_bytes)
    fprintf('  RSU %-4s: %4d bytes -> L_subCH = %d\n', rsu_names_short{k}, rsu_msg_bytes(k), rsu_L_subCH_table(k));
end
fprintf('\n');

%% ===== 2. SAE MESSAGE TYPES & RATES =====

% --- OBU SAE J2735 message types ---
BSM = 1;       % Basic Safety Message
SRM = 2;       % Signal Request Message
PVD = 3;       % Probe Vehicle Data
RSA_OBU = 4;   % Roadside Alert (OBU-originated)
TMSG_OBU = 5;  % Test Message (OBU)

obu_sae_m = [BSM SRM PVD RSA_OBU TMSG_OBU];
obu_sae_m_names = {'BSM','SRM','PVD','RSA_obu','TMSG_obu'};
obu_m_rate = [10 1 1 1 1];  % Hz

% --- RSU SAE J2735 message types ---
SPAT = 1;       % Signal Phase and Timing
MAP = 2;        % Map Data
SSM = 3;        % Signal Status Message
TIM = 4;        % Traveler Information Message
RTCM = 5;       % Differential Correction
PDM = 6;        % Probe Data Management
RSA_RSU = 7;    % Roadside Alert (RSU-originated)
TMSG_RSU = 8;   % Test Message (RSU)

rsu_sae_m = [SPAT MAP SSM TIM RTCM PDM RSA_RSU TMSG_RSU];
rsu_sae_m_names = {'SPAT','MAP','SSM','TIM','RTCM','PDM','RSA_rsu','TMSG_rsu'};
rsu_m_rate = [10 1 1 1 1 1 1 1];  % Hz

% Randomize which optional messages each vehicle/RSU sends
obu_m_select = [ones(Vnum,1) ones(Vnum,1) randi(2,Vnum,length(obu_sae_m)-2)-1];
obu_m = obu_m_rate .* obu_m_select;

rsu_m_select = [ones(RSUnum,1) ones(RSUnum,1) randi(2,RSUnum,length(rsu_sae_m)-2)-1];
rsu_m_select_num_of_TIMs = ones(RSUnum, length(rsu_m_rate));
rsu_m_select_num_of_TIMs(:,4) = randi(8,RSUnum,1);
rsu_m = rsu_m_rate .* (rsu_m_select .* rsu_m_select_num_of_TIMs);

% SAE J2735 message dependency logic:
%   No SRM from OBUs -> RSUs don't need SSM
%   No RSA from OBUs -> RSUs don't need RSA_RSU
%   No PDM from RSUs -> OBUs don't need PVD
if ~any(obu_m(:,SRM)),     rsu_m(:,SSM) = 0;     end
if ~any(obu_m(:,RSA_OBU)), rsu_m(:,RSA_RSU) = 0;  end
if ~any(rsu_m(:,PDM)),     obu_m(:,PVD) = 0;      end

%% ===== 3. BUILD ENTITY TABLE =====
% Minimal rh struct: packet types, rates, L_subCH per entity.
% No pre-sim history generation — the main loop runs a 1000ms warmup
% from subframe 1 to populate the sensing window naturally.

rh = [];

% --- OBU entities ---
for i = 1:Vnum
    entity = struct();
    pkt_types = find(obu_m(i,:) > 0);
    entity.packetTypes = pkt_types;

    rates = zeros(1, length(pkt_types));
    lsc   = zeros(1, length(pkt_types));
    for k = 1:length(pkt_types)
        if pkt_types(k) == TMSG_OBU
            rates(k) = (randi(2) == 2) * 9 + 1;  % 1 or 10 Hz
        else
            rates(k) = obu_m_rate(pkt_types(k));
        end
        lsc(k) = obu_L_subCH_table(pkt_types(k));
    end
    entity.packetRate = rates;
    entity.L_subCH = lsc;
    entity.Rsc = cell(1, length(pkt_types));
    entity.selectedHistoryResources = cell(1, length(pkt_types));
    entity.initialResource = [];
    entity.initialEpoch = [];

    rh = [rh entity];
end
obu_r = Vnum;  % needed by L2_output_interface

% --- RSU entities ---
for i = 1:RSUnum
    entity = struct();
    pkt_types = find(rsu_m(i,:) > 0);

    % TIM expansion (same logic as before)
    if any(pkt_types == TIM) && rsu_m(i, TIM) > 1
        extra_TIMs = rsu_m(i, TIM) - 1;
        pkt_types = [pkt_types, ones(1, extra_TIMs) * TIM];
    end

    entity.packetTypes = pkt_types;

    rates = zeros(1, length(pkt_types));
    lsc   = zeros(1, length(pkt_types));
    for k = 1:length(pkt_types)
        if pkt_types(k) == TMSG_RSU
            rates(k) = (randi(2) == 2) * 9 + 1;
        else
            rates(k) = rsu_m_rate(pkt_types(k));
        end
        lsc(k) = rsu_L_subCH_table(pkt_types(k));
    end
    entity.packetRate = rates;
    entity.L_subCH = lsc;
    entity.Rsc = cell(1, length(pkt_types));
    entity.selectedHistoryResources = cell(1, length(pkt_types));
    entity.initialResource = [];
    entity.initialEpoch = [];

    rh = [rh entity];
end

%% ===== 4. ALLOCATE RH MATRIX & GLOBAL SCI MEMORY =====

RH = zeros(Nslch, Tdur + Tsense_history + RH_padding);
totalSF_RH = size(RH, 2);

% Global SCI Memory: causal decode record — written once at TX time with PDR draw.
% Prevents "Decoder Amnesia" where per-call rand() re-rolls decode outcomes.
% Dimensions: numEntities x Nslch x totalSF_RH (logical ~32 MB for defaults)
Global_SCI_Memory = false(Vnum + RSUnum, Nslch, totalSF_RH);

%% ===== 5. INITIALIZE SCHEDULING TABLE =====
% Each (entity, packetType) gets a random initial TX time within its first
% RRI period, spreading vehicles across time to avoid thundering herd.
% currentRsc = 0 forces reselection on first TX (no pre-sim history).

numEntities = length(rh);
maxPktTypes = max(arrayfun(@(x) length(x.packetTypes), rh));

nextTxSF      = inf(numEntities, maxPktTypes);
pktRRI_sf     = zeros(numEntities, maxPktTypes);
pktRRI_ms     = zeros(numEntities, maxPktTypes);
lastPrimaryRS = zeros(numEntities, maxPktTypes);
currentRsc    = zeros(numEntities, maxPktTypes);  % 0 = force reselection
L_subCH_all   = ones(numEntities, maxPktTypes);

for i = 1:numEntities
    for j = 1:length(rh(i).packetTypes)
        rate = rh(i).packetRate(j);
        if rate >= 10
            pktRRI_sf(i,j) = 100;
            pktRRI_ms(i,j) = 100;
        elseif rate >= 1
            pktRRI_sf(i,j) = 1000;
            pktRRI_ms(i,j) = 1000;
        else
            continue;
        end
        L_subCH_all(i,j) = rh(i).L_subCH(j);

        % Random initial TX within first RRI period (avoids thundering herd)
        nextTxSF(i,j) = randi(pktRRI_sf(i,j));
        currentRsc(i,j) = 0;  % force reselection on first TX
    end
end

% Warmup: start from subframe 1 — first 1000 sf populate the sensing window
simStart = 1;
simEnd   = Tsense_history + Tdur;  % 101000

%% ===== 6. MAIN SIMULATION LOOP (OPTIMIZED, MULTI-SUBCHANNEL) =====

% Pre-allocate event logs
estTotalTx = numEntities * maxPktTypes * ceil(Tdur / 100) * 2;
txLog = zeros(min(estTotalTx, 5000000), 8, 'int32');
% txLog columns: [vehicleID, pktType, subframe, startSubchannel, isReTX,
%                  isCollision, isReselection, L_subCH]
txLogCount = 0;

reselLog = zeros(50000, 5, 'int32');
% reselLog columns: [vehicleID, pktType, subframe, oldSubchannel, newSubchannel]
reselLogCount = 0;

collisionCount = zeros(1, Tdur);
CBR = zeros(1, Tdur);

sbsps_cnt = 0;
progress_interval = 10000;

fprintf('Simulating %d seconds (%d subframes) with %d entities...\n', ...
    simseconds, Tdur, numEntities);

for currSF = simStart : simEnd

    % Progress reporting
    sf_rel = currSF - Tsense_history;
    if mod(sf_rel, progress_interval) == 0
        fprintf('  t = %d s (%.0f%%)\n', sf_rel/1000, sf_rel/Tdur*100);
    end

    % --- Find all (vehicle, packetType) pairs scheduled to TX this subframe ---
    [activeV, activeP] = find(nextTxSF == currSF);

    for k = 1:length(activeV)
        i = activeV(k);
        j = activeP(k);

        if currentRsc(i,j) < 0
            continue;
        end

        L = L_subCH_all(i,j);
        wasReselection = false;
        oldSubch = idx_helpers.to_subchannel(lastPrimaryRS(i,j), Nslch);

        if currentRsc(i,j) == 0
            % =============================================
            % RESELECTION: Call SB-SPS Mode 4 procedure
            % Passes L_subCH for multi-subchannel group selection
            % =============================================
            sbsps_cnt = sbsps_cnt + 1;
            wasReselection = true;

            [NextRS, NextreTXRS, newRsc] = sps_reselect( ...
                RH, i, currSF, Nslch, re_tx, pktRRI_ms(i,j), L, Global_SCI_Memory);

            currentRsc(i,j) = newRsc;
        else
            % =============================================
            % PERSISTENT: Same start subchannel, RRI later
            % =============================================
            NextRS      = lastPrimaryRS(i,j) + pktRRI_sf(i,j) * Nslch;
            NextreTXRS  = NextRS + re_tx * Nslch;
            currentRsc(i,j) = currentRsc(i,j) - 1;
        end

        % --- Write PRIMARY TX to RH (multi-subchannel, collision detection) ---
        isCollision_primary = int32(0);
        nextSF = idx_helpers.to_subframe(NextRS, Nslch);
        nextSC = idx_helpers.to_subchannel(NextRS, Nslch);

        for sch_off = 0:(L-1)
            lin_idx = NextRS + sch_off;
            if lin_idx >= 1 && lin_idx <= numel(RH)
                if RH(lin_idx) == 0
                    RH(lin_idx) = i;
                else
                    RH(lin_idx) = -abs(lin_idx);  % Negative = collision marker; filtered by (tx_in_sf > 0) in heard_mask
                    isCollision_primary = int32(1);
                end
            end
        end
        if isCollision_primary
            rel_sf = nextSF - Tsense_history;
            if rel_sf >= 1 && rel_sf <= Tdur
                collisionCount(rel_sf) = collisionCount(rel_sf) + 1;
            end
        end

        % --- Record SCI decode events for primary TX ---
        % Causal: decode decision made once at TX time, stored permanently.
        if nextSF >= 1 && nextSF <= totalSF_RH
            heard_mask = rand(numEntities, 1) <= PDR_flat;
            heard_mask(i) = false;  % can't hear own SCI
            % Half-duplex: vehicles transmitting in this subframe can't decode
            tx_in_sf = RH(:, nextSF);
            active_tx_vehicles = unique(tx_in_sf(tx_in_sf > 0));
            heard_mask(active_tx_vehicles) = false;
            % Write to permanent memory for all L occupied subchannels
            % sps_reselect guarantees nextSC + L - 1 <= Nslch (no truncation)
            sc_range = nextSC : min(nextSC + L - 1, Nslch);
            Global_SCI_Memory(heard_mask, sc_range, nextSF) = true;
        end

        % --- Write RETRANSMISSION to RH (multi-subchannel, collision detection) ---
        isCollision_retx = int32(0);
        retxSF = idx_helpers.to_subframe(NextreTXRS, Nslch);
        retxSC = idx_helpers.to_subchannel(NextreTXRS, Nslch);

        for sch_off = 0:(L-1)
            lin_idx = NextreTXRS + sch_off;
            if lin_idx >= 1 && lin_idx <= numel(RH)
                if RH(lin_idx) == 0
                    RH(lin_idx) = i;
                else
                    RH(lin_idx) = -abs(lin_idx);  % Negative = collision marker
                    isCollision_retx = int32(1);
                end
            end
        end
        if isCollision_retx
            rel_sf_retx = retxSF - Tsense_history;
            if rel_sf_retx >= 1 && rel_sf_retx <= Tdur
                collisionCount(rel_sf_retx) = collisionCount(rel_sf_retx) + 1;
            end
        end

        % --- Record SCI decode events for retransmission ---
        if retxSF >= 1 && retxSF <= totalSF_RH
            heard_mask = rand(numEntities, 1) <= PDR_flat;
            heard_mask(i) = false;
            tx_in_sf = RH(:, retxSF);
            active_tx_vehicles = unique(tx_in_sf(tx_in_sf > 0));
            heard_mask(active_tx_vehicles) = false;
            % sps_reselect guarantees retxSC + L - 1 <= Nslch (no truncation)
            sc_range = retxSC : min(retxSC + L - 1, Nslch);
            Global_SCI_Memory(heard_mask, sc_range, retxSF) = true;
        end

        % --- Log TX events ---
        % Primary TX
        txLogCount = txLogCount + 1;
        if txLogCount > size(txLog, 1)
            txLog = [txLog; zeros(size(txLog, 1), 8, 'int32')];
        end
        txLog(txLogCount, :) = [int32(i), int32(j), int32(nextSF), int32(nextSC), ...
                                int32(0), isCollision_primary, int32(wasReselection), int32(L)];

        % Retransmission TX
        txLogCount = txLogCount + 1;
        if txLogCount > size(txLog, 1)
            txLog = [txLog; zeros(size(txLog, 1), 8, 'int32')];
        end
        txLog(txLogCount, :) = [int32(i), int32(j), int32(retxSF), int32(retxSC), ...
                                int32(1), isCollision_retx, int32(0), int32(L)];

        % Reselection log
        if wasReselection
            newSubch = idx_helpers.to_subchannel(NextRS, Nslch);
            reselLogCount = reselLogCount + 1;
            if reselLogCount > size(reselLog, 1)
                reselLog = [reselLog; zeros(size(reselLog, 1), 5, 'int32')];
            end
            reselLog(reselLogCount, :) = [int32(i), int32(j), int32(currSF), ...
                                          int32(oldSubch), int32(newSubch)];
        end

        % --- Update scheduling state ---
        lastPrimaryRS(i,j) = NextRS;
        nextTxSF(i,j) = idx_helpers.to_subframe(NextRS, Nslch) + pktRRI_sf(i,j);
    end
end

% Trim TX log and reselection log to actual size
txLog    = txLog(1:txLogCount, :);
reselLog = reselLog(1:reselLogCount, :);

% --- Compute CBR post-simulation (vectorized) ---
fprintf('Computing CBR (vectorized)...\n');
sim_RH = RH(:, (Tsense_history+1):(Tsense_history+Tdur));
occupancy_per_sf = sum(sim_RH ~= 0, 1) / Nslch;
CBR = movmean(occupancy_per_sf, [99 0]);

% --- Reconstruct rh struct from txLog (backward compatibility) ---
fprintf('Reconstructing resource histories...\n');
for i = 1:numEntities
    for j = 1:length(rh(i).packetTypes)
        veh_mask = txLog(:,1) == i & txLog(:,2) == j;
        veh_events = txLog(veh_mask, :);
        if isempty(veh_events), continue; end

        new_resources = zeros(1, size(veh_events, 1));
        for ev = 1:size(veh_events, 1)
            new_resources(ev) = idx_helpers.to_linear( ...
                veh_events(ev, 4), veh_events(ev, 3), Nslch);
        end
        rh(i).selectedHistoryResources{j} = [rh(i).selectedHistoryResources{j}, new_resources];
    end
    for j = 1:length(rh(i).packetTypes)
        rh(i).Rsc{j} = [rh(i).Rsc{j}, currentRsc(i,j)];
    end
end

elapsed = toc;
fprintf('\nSimulation complete in %.2f seconds.\n', elapsed);
fprintf('Total SPS reselection events: %d\n', sbsps_cnt);
fprintf('Total TX events logged: %d\n', txLogCount);
fprintf('Total collisions during simulation: %d\n', sum(collisionCount));
fprintf('Mean CBR: %.4f (%.1f%%)\n', mean(CBR(CBR>0)), mean(CBR(CBR>0))*100);

%% ===== 7. POST-PROCESSING: BUILD OUTPUT STRUCTURES =====

[L2out, CWout] = L2_output_interface(rh, RH, txLog, reselLog, ...
    collisionCount, CBR, Nslch, Tsense_history, Tdur, obu_r, simseconds, ...
    L_subCH_all, obu_L_subCH_table, rsu_L_subCH_table, ...
    obu_msg_bytes, rsu_msg_bytes, MCS_index, bits_per_prb, Nrbs);

%% ===== 8. VISUALIZATION =====

L2_plot_results(L2out, CWout, Nslch, Tsense_history, Tdur, simseconds);

%% ===== 8b. INTERACTIVE RESOURCE GRID VIEWER =====
% Opens interactive GUI for exploring resource allocations per vehicle/message
% Usage after sim: L2_interactive_grid(L2out, CWout, rh)
L2_interactive_grid(L2out, CWout, rh);

%% ===== 9. SAVE OUTPUTS =====

outputDir = fileparts(mfilename('fullpath'));
matFile = fullfile(outputDir, 'L2_SPS_output.mat');
csvFile = fullfile(outputDir, 'vehicle_tx_schedule.csv');

fprintf('\nSaving outputs...\n');
save(matFile, 'L2out', 'CWout', 'rh', 'RH', '-v7.3');
fprintf('  Saved: %s\n', matFile);

% Export CSV: vehicleID, subframe, startSubchannel, L_subCH, isCollision, isRetransmission
primaryTx = txLog(txLog(:,5) == 0, :);
csvTable = table( ...
    primaryTx(:,1), primaryTx(:,3) - Tsense_history, primaryTx(:,4), ...
    primaryTx(:,8), primaryTx(:,6), ...
    'VariableNames', {'vehicleID', 'subframe', 'startSubchannel', 'L_subCH', 'isCollision'});
writetable(csvTable, csvFile);
fprintf('  Saved: %s (%d rows)\n', csvFile, height(csvTable));

fprintf('\nDone. Total elapsed time: %.2f seconds.\n', toc);
