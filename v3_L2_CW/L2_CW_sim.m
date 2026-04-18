% ============================================================================
% L2_CW_sim.m - C-V2X Mode 4 SPS + CW Sensing Simulation (v3)
% ============================================================================
% Extends the v2 L2 SPS simulation with integrated CW (Continuous Wave)
% sensing transmissions. Vehicles transmit CW tones at 5.9 GHz during data
% subframes, placed at 7.5 kHz midpoints between 15 kHz OFDM data subcarriers
% (using 2xM FFT). Six sensing modes control CW element assignment:
%
%   Mode 1a: 1 CW, global random, fixed entire sim
%   Mode 1b: 1 CW, within allocated PRBs, hops with data SPS
%   Mode 1c: 1 CW, global SPS-like, SPS-counter dwell then reselect
%   Mode 2a: 2-8 CWs, global random, fixed entire sim
%   Mode 2b: 2-8 CWs, within allocated PRBs, hops with data SPS
%   Mode 2c: 2-8 CWs, global SPS-like, SPS-counter dwell then reselect
%
% CW does NOT corrupt data at PHY (experimentally validated). CW occupancy
% in subchannels outside the data allocation increases perceived RSSI/CBR,
% which can influence SPS resource selection.
%
% Dependencies:
%   ../common/idx_helpers.m   - Index conversions + L_subCH computation
%   ../common/cw_phy_params.m - CW sensing PHY parameter computation
%   sps_reselect.m            - Plain SPS Mode 4 reselection (1b/2b)
%   sps_reselect_cw.m         - CW-aware SPS Mode 4 reselection (1a/1c/2a/2c)
%   cw_assign.m               - CW sensing element assignment (all modes)
%   cw_sensing_reselect.m     - SPS-like reselection for CW bins
%   L2_plot_results.m         - Extended visualization
%   L2_output_interface.m     - Output with CW assignment history
%   L2_interactive_grid.m     - Interactive resource grid viewer
%
% References:
%   3GPP TS 36.213 Section 14.1.1.6 (Resource selection for V2X sidelink)
%   3GPP TS 36.321 Section 5.14.1.1 (SL_RESOURCE_RESELECTION_COUNTER)
%   3GPP TS 36.213 Section 14.1.1.4c (SCI Format 1)
%   3GPP TS 36.211 Section 9 (SC-FDMA for sidelink)
% ============================================================================

clear; clc; close all;
tic;
addpath('../common');

%% ===== 1. SIMULATION PARAMETERS =====

Tsense_history = 1000;   % Sensing window history (ms) = 1000 subframes
Tepoch = 100;            % Epoch period = 100ms
simseconds = 100;        % Simulation duration in seconds
Tdur = 1000 * simseconds;  % Simulation duration in subframes

% PHY parameters
channel_bw_MHz = 10;     % Channel bandwidth: 10 or 20 MHz
Nslch = 10;              % Number of sidelink sub-channels
Nrbs = 5;                % Resource blocks per sub-channel
PRB = Nslch * Nrbs;      % Total PRBs per sub-frame (10MHz=50, 20MHz=100)

% --- CW Sensing Mode Configuration ---
cw_mode = '2b';          % Sensing mode: '1a','1b','1c','2a','2b','2c'
N_cw = 'random';         % CW elements per vehicle (Mode 2: 2-8 or 'random')
                          % Ignored for Mode 1 (always 1)
record_cw_rh_always = true; % Set true to record bin-level CW history for all modes
cw_dwell_range = [5, 15]; % Mode 1c/2c: CW dwell counter range [min, max]
                          % Counter drawn uniformly from this range.
                          % Unit = subframes (1ms each). Decrements every
                          % subframe since global CW is continuous.
                          % [5,15] → 5-15ms dwell (very fast hopping).
                          % Decoupled from DATA-SPS RRI — CW has no SCI.
                          % Tune via Monte Carlo under varying node densities.

% NOTE: With Nrbs=5 and MCS 7, BSMs need L_subCH=3 (3 contiguous subchannels).
% For lower L_subCH, consider Nrbs=10 (Nslch=5) or higher MCS.
% 10 MHz channel: Nslch=10, Nrbs=5 OR Nslch=5, Nrbs=10 are both valid.
% 20 MHz channel: Nslch=20, Nrbs=5 OR Nslch=10, Nrbs=10, etc.

% System model
PDR_flat = 0.9;  % Flat probability of decoding another vehicle's SCI

% Network
Vnum = 30;              % Number of vehicles (OBUs)
RSUnum = 1;              % Number of roadside units (RSUs)
re_tx = 3;               % Retransmission offset in subframes

% RH matrix padding (accommodates selection window T2=100 + retx + margin)
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

%% ===== 2b. CW SENSING PHY PARAMETERS =====

cw_params = cw_phy_params(channel_bw_MHz, Nslch, Nrbs);
fprintf('\n=== CW Sensing Configuration ===\n');
fprintf('  %s\n', cw_params.summary);
fprintf('  Mode: %s, N_cw: %s\n', cw_mode, num2str(N_cw));
fprintf('\n');

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

%% ===== 5b. CW SENSING INITIALIZATION =====

% Initialize CW assignment state
cw_state = cw_assign(cw_mode, numEntities, N_cw, cw_params);
fprintf('CW assignment initialized (mode %s):\n', cw_mode);
fprintf('  Requires CW_RH: %d, Integrated loop: %d\n', ...
    cw_state.requires_cw_rh, cw_state.requires_integrated_loop);
fprintf('  N_cw per vehicle: min=%d, max=%d\n', ...
    min(cw_state.N_cw_per_vehicle), max(cw_state.N_cw_per_vehicle));
if ismember(cw_mode, {'1c', '2c'})
    fprintf('  CW dwell range: [%d, %d] (subframes)\n', ...
        cw_dwell_range(1), cw_dwell_range(2));
end

% Mode 1b/2b: Expand CW assignment to per-(node, packetType).
% Each packet type has its own subchannel allocation, so CW bins must track
% independently. Global modes (1a/1c/2a/2c) remain per-node (1D cell).
if ismember(cw_mode, {'1b', '2b'})
    cw_state.assignment = cell(numEntities, maxPktTypes);
end

% CW subchannel-level occupancy matrix (for RSSI scoring in SPS).
% Lightweight representation: Nslch x total_subframes.
%
% Collision encoding: -1 is a "multi-vehicle occupancy" flag — NOT a PHY
% collision. This grid tracks subchannel-level CW presence for CBR/RSSI
% feedback to DATA-SPS, where any CW energy in the subchannel counts as
% occupancy (two vehicles on different bins in the same subchannel are
% flagged as -1 here but are orthogonal at RF).
%
% For true bin-level PHY collisions (two vehicles on the same 7.5 kHz
% bin), see CW_RH below. Data RH uses -abs(lin_idx) for collision
% encoding because each data TX spans multiple subchannels, and the
% absolute linear index preserves collision position info.
total_RH_cols = totalSF_RH;
CW_subchannel_RH = zeros(Nslch, total_RH_cols, 'int16');

% Full CW bin-level resource history (for modes that need SPS-like CW sensing)
if cw_state.requires_cw_rh || record_cw_rh_always
    % Use int16 to save memory: 599 x 101200 x 2 bytes = ~121 MB
    CW_RH = zeros(cw_params.total_sensing_bins, total_RH_cols, 'int16');
    fprintf('  CW_RH allocated: %d bins x %d subframes (%.0f MB)\n', ...
        cw_params.total_sensing_bins, total_RH_cols, ...
        cw_params.total_sensing_bins * total_RH_cols * 2 / 1e6);
else
    CW_RH = [];  % Not needed for Mode 1a/2a
end

% CW dwell counters (for Mode 1c/2c)
% Unit = subframes. Drawn from cw_dwell_range (configurable in Section 1).
% Decrements once per subframe in the continuous CW block.
cw_dwell_counter = zeros(numEntities, 1, 'int32');
if ismember(cw_mode, {'1c', '2c'})
    for i = 1:numEntities
        cw_dwell_counter(i) = randi(cw_dwell_range);
    end
end

% CW event log for tracking assignment changes (not TX occupancy).
% For TX occupancy/collisions, see CW_RH and cwCollisionCount.
cwEventLog = zeros(500000, 5, 'int32');
% Columns: [vehicleID, subframe, sensing_bin, event_type, reserved]
% event_type: 3=reselect (1c/2c dwell expiry), 4=hop_with_data (1b/2b)
cwEventLogCount = 0;

% CW collision counter per subframe
cwCollisionCount = zeros(1, Tdur);

fprintf('\n');

%% ===== 6. MAIN SIMULATION LOOP (OPTIMIZED, MULTI-SUBCHANNEL + CW) =====

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

    % --- Find all (node, packetType) pairs scheduled to TX this subframe ---
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
            % Uses CW-aware RSSI if CW mode affects SPS sensing
            % =============================================
            sbsps_cnt = sbsps_cnt + 1;
            wasReselection = true;

            if cw_state.requires_integrated_loop
                [NextRS, NextreTXRS, newRsc] = sps_reselect_cw( ...
                    RH, CW_subchannel_RH, i, currSF, Nslch, re_tx, pktRRI_ms(i,j), L, Global_SCI_Memory);
            else
                [NextRS, NextreTXRS, newRsc] = sps_reselect( ...
                    RH, i, currSF, Nslch, re_tx, pktRRI_ms(i,j), L, Global_SCI_Memory);
            end

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
                    RH(lin_idx) = -abs(lin_idx);
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
                    RH(lin_idx) = -abs(lin_idx);
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

        % =============================================
        % CW SENSING (DATA-COUPLED): Mode 1b/2b only
        % CW transmits within allocated PRBs, hops with data SPS.
        % Only active during data TX subframes.
        % =============================================

        % --- Mode 1b/2b: Assign CW within allocated PRBs ---
        %   Triggers on: (1) first TX when assignment is still empty,
        %                (2) every data SPS reselection (new allocation)
        if ismember(cw_mode, {'1b', '2b'}) && ...
                (wasReselection || isempty(cw_state.assignment{i,j}))
            new_start_sc = idx_helpers.to_subchannel(NextRS, Nslch);
            % Candidate bins (cand_bins) = all bins within the new L_subCH subchannel range
            cand_bins = [];
            for sc_idx = new_start_sc : min(new_start_sc + L - 1, Nslch)
                r = cw_params.subchannel_bin_range(sc_idx, :);
                cand_bins = [cand_bins, r(1):r(2)];
            end
            n_cw_i = cw_state.N_cw_per_vehicle(i);
            if ~isempty(CW_RH)
                new_bins = cw_sensing_reselect( ...
                    CW_RH, i, currSF, n_cw_i, cand_bins);
            else
                % Fallback: random selection from candidates
                perm = randperm(length(cand_bins), min(n_cw_i, length(cand_bins)));
                new_bins = cand_bins(perm);
            end
            cw_state.assignment{i,j} = new_bins;

            % Log CW hop event
            for cb = 1:length(new_bins)
                cwEventLogCount = cwEventLogCount + 1;
                if cwEventLogCount > size(cwEventLog, 1)
                    cwEventLog = [cwEventLog; zeros(size(cwEventLog, 1), 5, 'int32')];
                end
                cwEventLog(cwEventLogCount, :) = [int32(i), int32(currSF), ...
                    int32(new_bins(cb)), int32(4), int32(0)];
            end
        end

        % --- Mode 1b/2b: Write CW occupancy for data TX subframes only ---
        if ismember(cw_mode, {'1b', '2b'})
            cw_bins_i = cw_state.assignment{i,j};
            if ~isempty(cw_bins_i)
                % Primary TX subframe
                if nextSF >= 1 && nextSF <= total_RH_cols
                    for cb = 1:length(cw_bins_i)
                        bin = cw_bins_i(cb);
                        cw_sch = cw_params.bin_to_subchannel(bin);
                        if CW_subchannel_RH(cw_sch, nextSF) == 0
                            CW_subchannel_RH(cw_sch, nextSF) = int16(i);
                        elseif CW_subchannel_RH(cw_sch, nextSF) ~= int16(i)
                            CW_subchannel_RH(cw_sch, nextSF) = int16(-1);
                        end
                        if ~isempty(CW_RH)
                            if CW_RH(bin, nextSF) == 0
                                CW_RH(bin, nextSF) = int16(i);
                            else
                                CW_RH(bin, nextSF) = int16(-abs(bin));
                                rel_sf_cw = nextSF - Tsense_history;
                                if rel_sf_cw >= 1 && rel_sf_cw <= Tdur
                                    cwCollisionCount(rel_sf_cw) = cwCollisionCount(rel_sf_cw) + 1;
                                end
                            end
                        end
                    end
                end
                % Retransmission subframe
                if retxSF >= 1 && retxSF <= total_RH_cols
                    for cb = 1:length(cw_bins_i)
                        bin = cw_bins_i(cb);
                        cw_sch = cw_params.bin_to_subchannel(bin);
                        if CW_subchannel_RH(cw_sch, retxSF) == 0
                            CW_subchannel_RH(cw_sch, retxSF) = int16(i);
                        elseif CW_subchannel_RH(cw_sch, retxSF) ~= int16(i)
                            CW_subchannel_RH(cw_sch, retxSF) = int16(-1);
                        end
                        if ~isempty(CW_RH)
                            if CW_RH(bin, retxSF) == 0
                                CW_RH(bin, retxSF) = int16(i);
                            else
                                CW_RH(bin, retxSF) = int16(-abs(bin));
                                rel_sf_cw2 = retxSF - Tsense_history;
                                if rel_sf_cw2 >= 1 && rel_sf_cw2 <= Tdur
                                    cwCollisionCount(rel_sf_cw2) = cwCollisionCount(rel_sf_cw2) + 1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end  % end data TX inner loop

    % =============================================
    % CW SENSING (CONTINUOUS): Mode 1a/1c/2a/2c
    % CW transmits EVERY subframe, independent of data TX.
    % Dwell counter (1c/2c) decrements once per subframe.
    % =============================================
    if ismember(cw_mode, {'1a', '1c', '2a', '2c'}) && ...
            currSF >= 1 && currSF <= total_RH_cols

        % --- Mode 1c/2c: Decrement dwell counter, reselect if expired ---
        if ismember(cw_mode, {'1c', '2c'})
            for i = 1:numEntities
                cw_dwell_counter(i) = cw_dwell_counter(i) - 1;
                if cw_dwell_counter(i) <= 0
                    n_cw_i = cw_state.N_cw_per_vehicle(i);
                    all_bins = 1:cw_params.total_sensing_bins;
                    if ~isempty(CW_RH)
                        new_bins = cw_sensing_reselect( ...
                            CW_RH, i, currSF, n_cw_i, all_bins);
                    else
                        perm = randperm(length(all_bins), min(n_cw_i, length(all_bins)));
                        new_bins = all_bins(perm);
                    end
                    cw_state.assignment{i} = new_bins;
                    cw_dwell_counter(i) = randi(cw_dwell_range);

                    % Log CW reselection event
                    for cb = 1:length(new_bins)
                        cwEventLogCount = cwEventLogCount + 1;
                        if cwEventLogCount > size(cwEventLog, 1)
                            cwEventLog = [cwEventLog; zeros(size(cwEventLog, 1), 5, 'int32')];
                        end
                        cwEventLog(cwEventLogCount, :) = [int32(i), int32(currSF), ...
                            int32(new_bins(cb)), int32(3), int32(0)];
                    end
                end
            end
        end

        % --- Write continuous CW occupancy for ALL vehicles this subframe ---
        for i = 1:numEntities
            cw_bins_i = cw_state.assignment{i};
            if isempty(cw_bins_i), continue; end

            for cb = 1:length(cw_bins_i)
                bin = cw_bins_i(cb);
                cw_sch = cw_params.bin_to_subchannel(bin);

                % Write to subchannel-level CW grid (for RSSI scoring in sps_reselect_cw)
                if CW_subchannel_RH(cw_sch, currSF) == 0
                    CW_subchannel_RH(cw_sch, currSF) = int16(i);
                elseif CW_subchannel_RH(cw_sch, currSF) ~= int16(i)
                    CW_subchannel_RH(cw_sch, currSF) = int16(-1);
                end

                % Write to bin-level CW grid (for CW SPS-like sensing)
                if ~isempty(CW_RH)
                    if CW_RH(bin, currSF) == 0
                        CW_RH(bin, currSF) = int16(i);
                    else
                        CW_RH(bin, currSF) = int16(-abs(bin));
                        rel_sf_cw = currSF - Tsense_history;
                        if rel_sf_cw >= 1 && rel_sf_cw <= Tdur
                            cwCollisionCount(rel_sf_cw) = cwCollisionCount(rel_sf_cw) + 1;
                        end
                    end
                end
            end
        end
    end

end  % end main subframe loop

% Trim TX log and reselection log to actual size
txLog    = txLog(1:txLogCount, :);
reselLog = reselLog(1:reselLogCount, :);

% Trim CW event log
cwEventLog = cwEventLog(1:cwEventLogCount, :);

% --- Compute CBR post-simulation (vectorized) ---
% NOTE: `CBR` (no suffix) is the combined data+CW CBR — the default reported
% and exported value. For data-only channel occupancy, use `CBR_data`.
fprintf('Computing CBR (vectorized)...\n');
sim_RH = RH(:, (Tsense_history+1):(Tsense_history+Tdur));
sim_CW_RH = CW_subchannel_RH(:, (Tsense_history+1):(Tsense_history+Tdur));

% Data-only CBR
occupancy_data = sum(sim_RH ~= 0, 1) / Nslch;
CBR_data = movmean(occupancy_data, [99 0]);

% Combined data+CW CBR (OR logic to avoid double-counting)
occupancy_combined = sum((sim_RH ~= 0) | (sim_CW_RH ~= 0), 1) / Nslch;
CBR = movmean(occupancy_combined, [99 0]);

fprintf('  Data-only mean CBR: %.4f (%.1f%%)\n', mean(CBR_data(CBR_data>0)), mean(CBR_data(CBR_data>0))*100);
fprintf('  Data+CW  mean CBR: %.4f (%.1f%%)\n', mean(CBR(CBR>0)), mean(CBR(CBR>0))*100);
fprintf('  CW collisions: %d\n', sum(cwCollisionCount));

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

% --- Extend CWout with v3 CW sensing data ---
fprintf('Adding CW extension fields to CWout...\n');
CWout.sensing_mode = cw_mode;
CWout.cw_params = cw_params;
CWout.N_cw_per_vehicle = cw_state.N_cw_per_vehicle;
CWout.assignment_final = cw_state.assignment;  % Final CW bin assignment per vehicle
CWout.cwEventLog = cwEventLog;
CWout.cwCollisionCount = cwCollisionCount;
CWout.CBR_data_only = CBR_data;
CWout.CBR_data_cw = CBR;
CWout.CW_subchannel_RH = sim_CW_RH;  % Nslch x Tdur (simulation period only)
if ~isempty(CW_RH)
    CWout.CW_RH = CW_RH(:, (Tsense_history+1):(Tsense_history+Tdur));  % bins x Tdur
else
    CWout.CW_RH = [];
end

% Build per-vehicle hop/reselection history from cwEventLog.
% For TX occupancy see CW_RH; for collision counts see cwCollisionCount.
CWout.hop_events = cell(numEntities, 1);

for i = 1:numEntities
    veh_mask = cwEventLog(:,1) == i;
    veh_events = cwEventLog(veh_mask, :);

    % Hop/reselection events (event_type == 3 or 4)
    hop_mask = veh_events(:,4) == 3 | veh_events(:,4) == 4;
    CWout.hop_events{i} = veh_events(hop_mask, [2 3 4]);  % [subframe, bin, type]
end

%% ===== 8. VISUALIZATION =====

L2_plot_results(L2out, CWout, Nslch, Tsense_history, Tdur, simseconds);

% --- CW-Specific Plots ---
fprintf('\nGenerating CW sensing plots...\n');

% Plot 9: CBR Comparison (data-only vs data+CW)
figure('Name', 'CBR Comparison: Data vs Data+CW');
t_sec = (1:Tdur) / 1000;
plot(t_sec, CBR_data, 'b-', 'LineWidth', 0.5); hold on;
plot(t_sec, CBR, 'r-', 'LineWidth', 0.5);
legend('Data Only', 'Data + CW', 'Location', 'best');
xlabel('Time (s)'); ylabel('CBR');
title(sprintf('CBR Comparison — CW Mode %s', cw_mode));
grid on;

% Plot 10: CW Bin Assignment Map
if ~isempty(cwEventLog)
    figure('Name', 'CW Bin Assignments');
    init_events = cwEventLog(cwEventLog(:,4) == 1, :);  % TX events
    if ~isempty(init_events)
        % Show first occurrence per vehicle
        [unique_vehs, first_idx] = unique(init_events(:,1), 'first');
        first_bins = init_events(first_idx, 3);
        first_schs = cw_params.bin_to_subchannel(first_bins);
        scatter(double(unique_vehs), double(first_bins), 20, double(first_schs), 'filled');
        xlabel('Vehicle ID'); ylabel('Sensing Bin Index');
        title(sprintf('CW Bin Assignments — Mode %s', cw_mode));
        cb = colorbar; cb.Label.String = 'Subchannel';
        grid on;
    end
end

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
