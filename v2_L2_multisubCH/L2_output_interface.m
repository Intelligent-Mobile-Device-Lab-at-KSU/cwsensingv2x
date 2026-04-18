function [L2out, CWout] = L2_output_interface(rh, RH, txLog, reselLog, ...
    collisionCount, CBR, Nslch, Tsense_history, Tdur, obu_r, simseconds, ...
    L_subCH_all, obu_L_subCH_table, rsu_L_subCH_table, ...
    obu_msg_bytes, rsu_msg_bytes, MCS_index, bits_per_prb, Nrbs)
% L2_OUTPUT_INTERFACE  Build L2 and CW output structures (multi-subchannel)
%
% Constructs two output structures:
%   L2out - L2 performance metrics (collisions, CBR, reselections, IPG, etc.)
%   CWout - Downstream CW sensing / microDoppler interface data
%
% txLog columns: [vehicleID, pktType, subframe, startSubchannel, isReTX,
%                  isCollision, isReselection, L_subCH]
%
% Inputs:
%   rh                - Struct array of all entities with resource histories
%   RH                - Resource history matrix (Nslch x totalSubframes)
%   txLog             - TX event log (int32, 8 columns)
%   reselLog          - Reselection event log (int32, 5 columns)
%   collisionCount    - Collisions per simulation subframe (1 x Tdur)
%   CBR               - Channel Busy Ratio per simulation subframe (1 x Tdur)
%   Nslch             - Number of sidelink sub-channels
%   Tsense_history    - Sensing history duration in subframes
%   Tdur              - Simulation duration in subframes
%   obu_r             - Number of OBU entities
%   simseconds        - Simulation duration in seconds
%   L_subCH_all       - L_subCH per (entity, packetType) matrix
%   obu_L_subCH_table - L_subCH per OBU message type
%   rsu_L_subCH_table - L_subCH per RSU message type
%   obu_msg_bytes     - OBU message sizes in bytes
%   rsu_msg_bytes     - RSU message sizes in bytes
%   MCS_index         - MCS index used
%   bits_per_prb      - Bits per PRB at chosen MCS
%   Nrbs              - PRBs per subchannel
%
% Outputs:
%   L2out - struct with L2 performance fields
%   CWout - struct with CW sensing interface fields

    fprintf('Building output structures...\n');

    numEntities = length(rh);

    %% ===== L2out: L2 Performance Metrics =====
    L2out = struct();
    L2out.txLog = txLog;
    L2out.reselLog = reselLog;
    L2out.collisionCount = collisionCount;
    L2out.CBR = CBR;

    % Simulation parameters for reference
    L2out.params.Nslch = Nslch;
    L2out.params.Nrbs = Nrbs;
    L2out.params.MCS_index = MCS_index;
    L2out.params.bits_per_prb = bits_per_prb;
    L2out.params.Tsense_history = Tsense_history;
    L2out.params.Tdur = Tdur;
    L2out.params.simseconds = simseconds;
    L2out.params.numOBUs = obu_r;
    L2out.params.numRSUs = numEntities - obu_r;
    L2out.params.numEntities = numEntities;
    L2out.params.obu_msg_bytes = obu_msg_bytes;
    L2out.params.rsu_msg_bytes = rsu_msg_bytes;
    L2out.params.obu_L_subCH = obu_L_subCH_table;
    L2out.params.rsu_L_subCH = rsu_L_subCH_table;
    L2out.params.L_subCH_all = L_subCH_all;

    % --- Inter-Packet Gap (IPG) per vehicle ---
    L2out.IPG = cell(numEntities, 1);

    primaryMask = txLog(:,5) == 0;
    primaryLog = txLog(primaryMask, :);

    for i = 1:numEntities
        veh_mask = primaryLog(:,1) == i;
        veh_subframes = sort(double(primaryLog(veh_mask, 3)));
        if length(veh_subframes) > 1
            L2out.IPG{i} = diff(veh_subframes);
        else
            L2out.IPG{i} = [];
        end
    end

    % --- Per-subchannel utilization ---
    sim_RH = RH(:, (Tsense_history+1) : (Tsense_history+Tdur));
    L2out.subchannelUtil = sum(sim_RH ~= 0, 2) / Tdur;  % Nslch x 1

    % --- Subchannel-subframe resource consumption ---
    % With multi-subchannel, count total subchannel-subframes consumed per TX
    if ~isempty(primaryLog)
        total_subch_sf_consumed = sum(double(primaryLog(:,8)));  % sum of L_subCH
        available_subch_sf = Nslch * Tdur;
        L2out.stats.resourceUtilization = total_subch_sf_consumed / available_subch_sf;
    else
        L2out.stats.resourceUtilization = 0;
    end

    % --- Summary statistics ---
    all_ipg = cell2mat(L2out.IPG);
    if ~isempty(all_ipg)
        L2out.stats.meanIPG = mean(all_ipg);
        L2out.stats.stdIPG  = std(all_ipg);
        L2out.stats.medianIPG = median(all_ipg);
        L2out.stats.minIPG  = min(all_ipg);
        L2out.stats.maxIPG  = max(all_ipg);
    end
    L2out.stats.totalReselections = size(reselLog, 1);
    L2out.stats.totalCollisions = sum(collisionCount);
    L2out.stats.meanCBR = mean(CBR(CBR > 0));
    L2out.stats.totalPrimaryTx = sum(primaryMask);
    L2out.stats.collisionRate = L2out.stats.totalCollisions / max(L2out.stats.totalPrimaryTx, 1);

    % L_subCH distribution summary
    if ~isempty(primaryLog)
        L2out.stats.L_subCH_distribution = histcounts(double(primaryLog(:,8)), ...
            0.5:(max(double(primaryLog(:,8)))+0.5));
    end

    fprintf('  L2out: %d primary TX, %d collisions (%.2f%%), %d reselections\n', ...
        L2out.stats.totalPrimaryTx, L2out.stats.totalCollisions, ...
        L2out.stats.collisionRate * 100, L2out.stats.totalReselections);
    fprintf('  Resource utilization: %.1f%% (accounts for multi-subchannel)\n', ...
        L2out.stats.resourceUtilization * 100);

    %% ===== CWout: CW Sensing / MicroDoppler Interface =====
    CWout = struct();

    % CW sensing parameters
    CWout.params.carrier_freq_Hz    = 5.9e9;
    CWout.params.sensing_bw_Hz      = 20e6;
    CWout.params.subcarrier_spacing = 100;
    CWout.params.total_subcarriers  = 200000;
    CWout.params.subcarriers_per_subchannel = 200000 / Nslch;
    CWout.params.subframe_duration_s = 1e-3;
    CWout.params.Nslch = Nslch;
    CWout.params.Nrbs = Nrbs;
    CWout.params.MCS_index = MCS_index;

    % Per-vehicle TX schedule (primary TX only)
    CWout.txSubframes   = cell(numEntities, 1);
    CWout.txSubchannels = cell(numEntities, 1);
    CWout.txGaps        = cell(numEntities, 1);
    CWout.txL_subCH     = cell(numEntities, 1);

    % Binary occupancy and collision matrices
    % occupancyMatrix(vehicleID, simSubframe) = true if transmitting
    % collisionMatrix(vehicleID, simSubframe) = true if TX collided
    CWout.occupancyMatrix = false(numEntities, Tdur);
    CWout.collisionMatrix = false(numEntities, Tdur);

    % Multi-subchannel occupancy: tracks which subchannels are used per vehicle per subframe
    % Stored sparsely via txLog rather than dense matrix (would be too large)

    for i = 1:numEntities
        veh_primary = primaryLog(primaryLog(:,1) == i, :);

        if isempty(veh_primary), continue; end

        subframes   = double(veh_primary(:,3)) - Tsense_history;
        subchannels = double(veh_primary(:,4));
        isCollision = logical(veh_primary(:,6));
        lsubch      = double(veh_primary(:,8));

        valid = subframes >= 1 & subframes <= Tdur;
        subframes   = subframes(valid);
        subchannels = subchannels(valid);
        isCollision = isCollision(valid);
        lsubch      = lsubch(valid);

        [subframes, sortIdx] = sort(subframes);
        subchannels = subchannels(sortIdx);
        isCollision = isCollision(sortIdx);
        lsubch      = lsubch(sortIdx);

        CWout.txSubframes{i}   = subframes;
        CWout.txSubchannels{i} = subchannels;
        CWout.txL_subCH{i}     = lsubch;

        if length(subframes) > 1
            CWout.txGaps{i} = diff(subframes);
        else
            CWout.txGaps{i} = [];
        end

        % Populate occupancy matrix (vehicle transmits during these subframes)
        CWout.occupancyMatrix(i, subframes) = true;

        % Populate collision matrix
        CWout.collisionMatrix(i, subframes(isCollision)) = true;
    end

    % --- CW Sub-Carrier Mapping Info ---
    % Documents how C-V2X subchannels map to CW sensing sub-carriers.
    %
    % 20MHz = 200,000 sub-carriers at 100Hz spacing
    % With Nslch subchannels, each spans (200000/Nslch) sub-carriers.
    %
    % For multi-subchannel TX (L_subCH > 1), the CW sub-carrier could be:
    %   Mode 1 (Fixed): Vehicle uses one CW sub-carrier throughout
    %          -> Phase coherent across all TX events (ideal for microDoppler)
    %   Mode 2 (Frequency hopping): CW sub-carrier follows start subchannel
    %          -> Sub-carrier changes on reselection events
    %
    % For Mode 2, the start subchannel maps to a CW sub-carrier:
    %   cw_subcarrier = (start_subchannel - 1) * subcarriers_per_subchannel + offset
    CWout.subchannel_to_subcarrier = @(sch, offset) ...
        (sch - 1) * CWout.params.subcarriers_per_subchannel + offset;

    % Summary
    active_vehicles = sum(cellfun(@length, CWout.txSubframes) > 0);
    total_occupancy = sum(CWout.occupancyMatrix(:));
    fprintf('  CWout: %d active vehicles, %.1f%% overall occupancy\n', ...
        active_vehicles, total_occupancy / (numEntities * Tdur) * 100);

end
