function cw_state = cw_assign(mode, numEntities, N_cw, cw_params)
% CW_ASSIGN  Assign CW sensing elements to vehicles for C-V2X simulation.
%
%   cw_state = cw_assign(mode, numEntities, N_cw, cw_params)
%
%   Assigns CW (continuous wave) tones to sensing bins for each vehicle
%   according to the specified sensing mode. CW tones are placed at the
%   midpoints between 15 kHz OFDM data subcarriers (7.5 kHz spacing).
%
%   INPUTS
%     mode         - String: '1a','1b','1c','2a','2b','2c'
%     numEntities  - Total number of nodes (OBUs + RSUs)
%     N_cw         - Number of CW elements per node.
%                      Mode 1: always forced to 1 (input ignored).
%                      Mode 2: scalar integer (same for all vehicles)
%                               or the string 'random' (each vehicle
%                               draws Uniform[2,8]).
%     cw_params    - Struct from cw_phy_params(channel_bw_MHz, Nslch, Nrbs) with fields:
%                      cw_params.total_sensing_bins, cw_params.bins_per_subchannel,
%                      cw_params.bin_to_subchannel, cw_params.subchannel_bin_range
%
%   OUTPUT
%     cw_state     - Struct with fields:
%       .mode                    - Mode string
%       .N_cw_per_vehicle        - 1 x numEntities vector of CW counts
%       .assignment              - cell(numEntities,1), each cell is a
%                                  1 x N_cw vector of sensing bin indices
%       .bin_taken               - logical(1, total_sensing_bins) tracking
%                                  which bins are occupied (Modes 1a/2a)
%       .requires_cw_rh          - true for modes needing CW resource
%                                  history (1b, 1c, 2b, 2c)
%       .requires_integrated_loop- true for modes that feed back into
%                                  SPS CBR (1a, 1c, 2a, 2c)
%
%   MODE DESCRIPTIONS
%     1a - Each vehicle gets 1 randomly chosen unique bin at init. Fixed
%          for the entire simulation. Collision avoidance via bin_taken.
%     1b - Deferred assignment. Bins are assigned during simulation when
%          a vehicle first transmits, within its allocated PRBs.
%     1c - Same initial random unique bin as 1a, but bins will be
%          periodically reselected during the simulation.
%     2a - Each vehicle gets N_cw randomly chosen unique bins at init.
%          Fixed for the entire simulation.
%     2b - Deferred assignment, same as 1b but with N_cw > 1.
%     2c - Same initial assignment as 2a, but multiple bins (not single bin) will be periodically
%          reselected during the simulation.

    % ---- Input validation ------------------------------------------------
    valid_modes = {'1a','1b','1c','2a','2b','2c'};
    if ~ischar(mode) && ~isstring(mode)
        error('cw_assign:invalidMode', ...
              'mode must be a string or char array.');
    end
    mode = char(mode);
    if ~ismember(mode, valid_modes)
        error('cw_assign:invalidMode', ...
              'mode must be one of: %s. Got ''%s''.', ...
              strjoin(valid_modes, ', '), mode);
    end

    if ~isnumeric(numEntities) || ~isscalar(numEntities) || ...
       numEntities < 1 || floor(numEntities) ~= numEntities
        error('cw_assign:invalidNumEntities', ...
              'numEntities must be a positive integer.');
    end

    if ~isstruct(cw_params) || ~isfield(cw_params, 'total_sensing_bins')
        error('cw_assign:invalidParams', ...
              'cw_params must be a struct from cw_phy_params().');
    end

    total_bins = cw_params.total_sensing_bins;

    % ---- Determine if this is a Mode-1 or Mode-2 variant -----------------
    is_mode1 = mode(1) == '1';

    % ---- Resolve N_cw per vehicle ----------------------------------------
    N_cw_per_vehicle = zeros(1, numEntities);

    if is_mode1
        % Mode 1: always exactly 1 CW element per vehicle
        N_cw_per_vehicle(:) = 1;
    else
        % Mode 2: scalar integer or 'random'
        if ischar(N_cw) || isstring(N_cw)
            if strcmpi(N_cw, 'random')
                % Each vehicle draws independently from Uniform[2,8]
                N_cw_per_vehicle = randi([2, 8], 1, numEntities);
            else
                error('cw_assign:invalidNcw', ...
                      'N_cw string must be ''random''. Got ''%s''.', ...
                      char(N_cw));
            end
        elseif isnumeric(N_cw) && isscalar(N_cw) && N_cw >= 1 && ...
               floor(N_cw) == N_cw
            N_cw_per_vehicle(:) = N_cw;
        else
            error('cw_assign:invalidNcw', ...
                  'N_cw must be a positive integer or ''random''.');
        end
    end

    % ---- Pre-check capacity for immediate-assignment modes ---------------
    total_needed = sum(N_cw_per_vehicle);
    is_immediate = ismember(mode, {'1a','1c','2a','2c'});
    if is_immediate && total_needed > total_bins
        error('cw_assign:insufficientBins', ...
              ['Total CW elements requested (%d) exceeds available ' ...
               'sensing bins (%d).'], total_needed, total_bins);
    end

    % ---- Initialise output struct ----------------------------------------
    cw_state.mode                    = mode;
    cw_state.N_cw_per_vehicle        = N_cw_per_vehicle;
    cw_state.assignment              = cell(numEntities, 1);
    cw_state.bin_taken               = false(1, total_bins);
    cw_state.requires_cw_rh          = false;
    cw_state.requires_integrated_loop = false;

    % ---- Set mode-dependent flags ----------------------------------------
    % requires_cw_rh: modes that need CW resource history for SPS-like
    % sensing (1b, 1c, 2b, 2c)
    if ismember(mode, {'1b','1c','2b','2c'})
        cw_state.requires_cw_rh = true;
    end

    % requires_integrated_loop: modes whose CW assignments feed back into
    % the SPS CBR calculation (1a, 1c, 2a, 2c)
    if ismember(mode, {'1a','1c','2a','2c'})
        cw_state.requires_integrated_loop = true;
    end

    % ---- Perform assignment based on mode --------------------------------
    switch mode

        % ------------------------------------------------------------------
        % MODE 1a: Random unique bin, fixed for entire simulation
        %   Each vehicle picks 1 sensing bin that no other vehicle uses.
        %   Collision avoidance is enforced at initialisation via bin_taken.
        % ------------------------------------------------------------------
        case '1a'
            for i = 1:numEntities
                bin = randi(total_bins);
                while cw_state.bin_taken(bin)
                    bin = randi(total_bins);
                end
                cw_state.assignment{i} = bin;
                cw_state.bin_taken(bin) = true;
            end

        % ------------------------------------------------------------------
        % MODE 1b: Deferred assignment
        %   CW bins are NOT assigned at initialisation. They will be
        %   assigned during the simulation when a vehicle first transmits,
        %   constrained to fall within the vehicle's allocated PRBs.
        % ------------------------------------------------------------------
        case '1b'
            for i = 1:numEntities
                cw_state.assignment{i} = [];
            end

        % ------------------------------------------------------------------
        % MODE 1c: Random unique bin (like 1a), but reselected periodically
        %   Initial assignment uses the same collision-free random draw as
        %   Mode 1a. During simulation the bin will be reselected, so this
        %   is just a starting point.
        % ------------------------------------------------------------------
        case '1c'
            for i = 1:numEntities
                bin = randi(total_bins);
                while cw_state.bin_taken(bin)
                    bin = randi(total_bins);
                end
                cw_state.assignment{i} = bin;
                cw_state.bin_taken(bin) = true;
            end

        % ------------------------------------------------------------------
        % MODE 2a: Each vehicle picks N_cw unique unused bins, fixed
        %   Same collision-avoidance approach as 1a but repeated N_cw
        %   times per vehicle. All chosen bins remain fixed for the
        %   entire simulation.
        % ------------------------------------------------------------------
        case '2a'
            for i = 1:numEntities
                n = N_cw_per_vehicle(i);
                bins = zeros(1, n);
                for k = 1:n
                    bin = randi(total_bins);
                    while cw_state.bin_taken(bin)
                        bin = randi(total_bins);
                    end
                    bins(k) = bin;
                    cw_state.bin_taken(bin) = true;
                end
                cw_state.assignment{i} = bins;
            end

        % ------------------------------------------------------------------
        % MODE 2b: Deferred assignment (like 1b, but N_cw > 1)
        %   Bins will be assigned during the simulation within the
        %   vehicle's allocated PRBs, similar to Mode 1b.
        % ------------------------------------------------------------------
        case '2b'
            for i = 1:numEntities
                cw_state.assignment{i} = [];
            end

        % ------------------------------------------------------------------
        % MODE 2c: Random unique bins (like 2a, but multiple sensing bins), reselected periodically
        %   Initial assignment follows the same procedure as Mode 2a.
        %   During simulation the bins will be periodically reselected.
        % ------------------------------------------------------------------
        case '2c'
            for i = 1:numEntities
                n = N_cw_per_vehicle(i);
                bins = zeros(1, n);
                for k = 1:n
                    bin = randi(total_bins);
                    while cw_state.bin_taken(bin)
                        bin = randi(total_bins);
                    end
                    bins(k) = bin;
                    cw_state.bin_taken(bin) = true;
                end
                cw_state.assignment{i} = bins;
            end

    end  % switch mode

end  % function cw_assign
