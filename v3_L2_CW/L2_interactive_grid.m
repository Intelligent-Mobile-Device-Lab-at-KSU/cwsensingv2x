function L2_interactive_grid(L2out, CWout, rh)
% L2_INTERACTIVE_GRID  Interactive resource grid viewer with CW visualization
%
% View modes: Data Grid, CW Subchannel Grid, CW Sensing Bins
% Controls: vehicle selection, message type filter, time range,
%           retransmission/collision toggles, CW overlay on data grid.
%
% Usage:
%   L2_interactive_grid(L2out, CWout, rh)

    % === Extract simulation parameters ===
    txLog          = L2out.txLog;
    Nslch          = L2out.params.Nslch;
    Tsense_history = L2out.params.Tsense_history;
    Tdur           = L2out.params.Tdur;
    numEntities    = L2out.params.numEntities;
    obu_r          = L2out.params.numOBUs;

    % === CW data extraction (robust — checks multiple fields) ===
    has_cw = isstruct(CWout) && ...
        (isfield(CWout, 'sensing_mode') || ...
         isfield(CWout, 'CW_subchannel_RH') || ...
         isfield(CWout, 'cwEventLog'));
    if has_cw
        if isfield(CWout, 'cwEventLog')
            cwEventLog = CWout.cwEventLog;
        else
            cwEventLog = zeros(0, 5, 'int32');
        end
        if isfield(CWout, 'cw_params')
            cw_params = CWout.cw_params;
        else
            cw_params = struct();
        end
        if isfield(CWout, 'CW_subchannel_RH')
            CW_subchannel_RH = CWout.CW_subchannel_RH;
        else
            CW_subchannel_RH = [];
        end
        if isfield(CWout, 'CW_RH')
            CW_RH_sim = CWout.CW_RH;
        else
            CW_RH_sim = [];
        end
        if isfield(CWout, 'sensing_mode')
            cw_mode_str = CWout.sensing_mode;
        else
            cw_mode_str = 'unknown';
        end
        if isfield(CWout, 'assignment_final')
            cw_assignment = CWout.assignment_final;
        else
            cw_assignment = {};
        end
        fprintf('L2_interactive_grid: CW data detected (mode: %s)\n', cw_mode_str);
    else
        cwEventLog       = zeros(0, 5, 'int32');
        cw_params        = struct();
        CW_subchannel_RH = [];
        CW_RH_sim        = [];
        cw_mode_str      = 'none';
        cw_assignment    = {};
        fprintf('L2_interactive_grid: No CW data found in CWout.\n');
        if isstruct(CWout)
            fprintf('  CWout fields: %s\n', strjoin(fieldnames(CWout), ', '));
        else
            fprintf('  CWout is empty or not a struct.\n');
        end
    end

    % SAE message name tables
    obu_names = {'BSM','SRM','PVD','RSA','TMSG'};
    rsu_names = {'SPAT','MAP','SSM','TIM','RTCM','PDM','RSA','TMSG'};

    % Build resolved message name for each (vehicleID, pktTypeIdx)
    msg_names = cell(numEntities, 1);
    for i = 1:numEntities
        nPkt = length(rh(i).packetTypes);
        msg_names{i} = cell(1, nPkt);
        for j = 1:nPkt
            sae_type = rh(i).packetTypes(j);
            if i <= obu_r
                if sae_type <= length(obu_names)
                    msg_names{i}{j} = obu_names{sae_type};
                else
                    msg_names{i}{j} = sprintf('OBU_T%d', sae_type);
                end
            else
                if sae_type <= length(rsu_names)
                    msg_names{i}{j} = rsu_names{sae_type};
                else
                    msg_names{i}{j} = sprintf('RSU_T%d', sae_type);
                end
            end
        end
    end

    % Collect unique message names
    all_msg_flat = {};
    for i = 1:numEntities
        all_msg_flat = [all_msg_flat, msg_names{i}];
    end
    unique_msg_names = unique(all_msg_flat);

    % Precompute message name per txLog row
    txLog_msg_names = cell(size(txLog, 1), 1);
    for r = 1:size(txLog, 1)
        vid = txLog(r, 1);
        pid = txLog(r, 2);
        if vid >= 1 && vid <= numEntities && pid >= 1 && pid <= length(msg_names{vid})
            txLog_msg_names{r} = msg_names{vid}{pid};
        else
            txLog_msg_names{r} = '?';
        end
    end

    % Entity labels
    entity_labels = cell(numEntities, 1);
    for i = 1:numEntities
        if i <= obu_r
            entity_labels{i} = sprintf('OBU %d', i);
        else
            entity_labels{i} = sprintf('RSU %d', i - obu_r);
        end
    end

    % === Create Figure ===
    fig = figure('Name', 'Interactive Resource Grid Viewer', ...
                 'NumberTitle', 'off', ...
                 'Position', [50 50 1500 700], ...
                 'Color', [0.94 0.94 0.94]);

    % =====================================================================
    % CONTROL PANEL (left side)
    % =====================================================================
    px = 0.01;  pw = 0.17;

    % --- View Mode ---
    uicontrol(fig, 'Style', 'text', 'String', 'VIEW MODE', ...
        'Units', 'normalized', 'Position', [px 0.93 pw 0.05], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    view_options = {'Data Grid', 'CW Subchannel Grid', 'CW Sensing Bins'};
    h_view_mode = uicontrol(fig, 'Style', 'popupmenu', ...
        'String', view_options, ...
        'Units', 'normalized', 'Position', [px 0.89 pw 0.04], ...
        'FontSize', 10, 'Value', 1, ...
        'Callback', @refresh_callback);

    % --- Filters ---
    uicontrol(fig, 'Style', 'text', 'String', 'FILTERS', ...
        'Units', 'normalized', 'Position', [px 0.84 pw 0.04], ...
        'FontSize', 11, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    % --- Vehicle Selection ---
    uicontrol(fig, 'Style', 'text', 'String', 'Vehicle IDs:', ...
        'Units', 'normalized', 'Position', [px 0.80 pw 0.03], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    default_veh = sprintf('1:%d', min(numEntities, 5));
    h_veh_edit = uicontrol(fig, 'Style', 'edit', ...
        'String', default_veh, ...
        'Units', 'normalized', 'Position', [px 0.76 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontSize', 10, ...
        'Callback', @refresh_callback);

    bw = pw/3 - 0.002;
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'OBUs', ...
        'Units', 'normalized', 'Position', [px 0.72 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', obu_r)));

    if numEntities > obu_r
        uicontrol(fig, 'Style', 'pushbutton', 'String', 'RSUs', ...
            'Units', 'normalized', 'Position', [px+bw+0.002 0.72 bw 0.04], ...
            'FontSize', 8, ...
            'Callback', @(~,~) quick_select(sprintf('%d:%d', obu_r+1, numEntities)));
    end

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'All', ...
        'Units', 'normalized', 'Position', [px+2*(bw+0.002) 0.72 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', numEntities)));

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'First 5', ...
        'Units', 'normalized', 'Position', [px 0.68 bw 0.035], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', min(5, numEntities))));

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'First 20', ...
        'Units', 'normalized', 'Position', [px+bw+0.002 0.68 bw 0.035], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', min(20, numEntities))));

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'First 50', ...
        'Units', 'normalized', 'Position', [px+2*(bw+0.002) 0.68 bw 0.035], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', min(50, numEntities))));

    % --- Message Type Selection ---
    uicontrol(fig, 'Style', 'text', 'String', 'Message Types:', ...
        'Units', 'normalized', 'Position', [px 0.63 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    uicontrol(fig, 'Style', 'text', 'String', '(Data Grid only)', ...
        'Units', 'normalized', 'Position', [px 0.60 pw 0.03], ...
        'HorizontalAlignment', 'left', 'FontSize', 8, ...
        'ForegroundColor', [0.5 0.5 0.5], ...
        'BackgroundColor', [0.94 0.94 0.94]);

    h_msg_list = uicontrol(fig, 'Style', 'listbox', ...
        'String', unique_msg_names, ...
        'Units', 'normalized', 'Position', [px 0.42 pw 0.18], ...
        'Max', length(unique_msg_names), 'Min', 0, ...
        'Value', 1:length(unique_msg_names), ...
        'FontSize', 10);

    % --- Time Range ---
    uicontrol(fig, 'Style', 'text', 'String', 'Time Range (subframes):', ...
        'Units', 'normalized', 'Position', [px 0.37 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    uicontrol(fig, 'Style', 'text', 'String', 'Start:', ...
        'Units', 'normalized', 'Position', [px 0.33 0.04 0.03], ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.94 0.94 0.94]);
    h_time_start = uicontrol(fig, 'Style', 'edit', 'String', '1', ...
        'Units', 'normalized', 'Position', [px+0.04 0.33 pw-0.04 0.04], ...
        'FontSize', 10, 'Callback', @refresh_callback);

    uicontrol(fig, 'Style', 'text', 'String', 'End:', ...
        'Units', 'normalized', 'Position', [px 0.29 0.04 0.03], ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.94 0.94 0.94]);
    h_time_end = uicontrol(fig, 'Style', 'edit', 'String', '1000', ...
        'Units', 'normalized', 'Position', [px+0.04 0.29 pw-0.04 0.04], ...
        'FontSize', 10, 'Callback', @refresh_callback);

    uicontrol(fig, 'Style', 'pushbutton', 'String', '1s', ...
        'Units', 'normalized', 'Position', [px 0.25 bw 0.035], ...
        'FontSize', 8, 'Callback', @(~,~) quick_time(1, 1000));
    uicontrol(fig, 'Style', 'pushbutton', 'String', '5s', ...
        'Units', 'normalized', 'Position', [px+bw+0.002 0.25 bw 0.035], ...
        'FontSize', 8, 'Callback', @(~,~) quick_time(1, 5000));
    uicontrol(fig, 'Style', 'pushbutton', 'String', '10s', ...
        'Units', 'normalized', 'Position', [px+2*(bw+0.002) 0.25 bw 0.035], ...
        'FontSize', 8, 'Callback', @(~,~) quick_time(1, 10000));

    % --- Options ---
    h_show_retx = uicontrol(fig, 'Style', 'checkbox', ...
        'String', 'Show Retransmissions', ...
        'Units', 'normalized', 'Position', [px 0.20 pw 0.04], ...
        'Value', 1, 'BackgroundColor', [0.94 0.94 0.94]);

    h_show_coll = uicontrol(fig, 'Style', 'checkbox', ...
        'String', 'Highlight Collisions', ...
        'Units', 'normalized', 'Position', [px 0.16 pw 0.04], ...
        'Value', 1, 'BackgroundColor', [0.94 0.94 0.94]);

    h_cw_overlay = uicontrol(fig, 'Style', 'checkbox', ...
        'String', 'Overlay CW on Data', ...
        'Units', 'normalized', 'Position', [px 0.12 pw 0.04], ...
        'Value', 0, 'BackgroundColor', [0.94 0.94 0.94]);

    % --- CW Info ---
    h_cw_info = uicontrol(fig, 'Style', 'text', 'String', '', ...
        'Units', 'normalized', 'Position', [px 0.07 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontSize', 8, ...
        'ForegroundColor', [0.1 0.4 0.7], ...
        'BackgroundColor', [0.94 0.94 0.94]);
    if has_cw
        set(h_cw_info, 'String', sprintf('CW Mode: %s', upper(cw_mode_str)));
    end

    % --- Refresh Button ---
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'REFRESH', ...
        'Units', 'normalized', 'Position', [px 0.01 pw 0.05], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.9], 'ForegroundColor', 'w', ...
        'Callback', @refresh_callback);

    % --- Status bar ---
    h_info = uicontrol(fig, 'Style', 'text', 'String', '', ...
        'Units', 'normalized', 'Position', [0.20 0.00 0.78 0.04], ...
        'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], ...
        'FontSize', 9);

    % =====================================================================
    % AXES
    % =====================================================================
    ax = axes('Parent', fig, 'Position', [0.22 0.10 0.73 0.84]);

    % === Initial render ===
    refresh_callback([], []);

    % =====================================================================
    % NESTED: Quick-select helpers
    % =====================================================================
    function quick_select(str)
        set(h_veh_edit, 'String', str);
        refresh_callback([], []);
    end

    function quick_time(t1, t2)
        set(h_time_start, 'String', num2str(t1));
        set(h_time_end, 'String', num2str(min(t2, Tdur)));
        refresh_callback([], []);
    end

    % =====================================================================
    % NESTED: Build colormap (collision, empty, vehicle colors)
    % =====================================================================
    function cmap = build_colormap(nSel)
        cmap = zeros(2 + nSel, 3);
        cmap(1, :) = [0.85 0.1 0.1];   % index 1 -> collision (-1)
        cmap(2, :) = [1 1 1];           % index 2 -> empty (0)
        if nSel <= 20
            sel_colors = lines(nSel);
        else
            sel_colors = parula(nSel);
        end
        cmap(3:end, :) = sel_colors;
    end

    % =====================================================================
    % NESTED: Build colorbar with vehicle labels
    % =====================================================================
    function build_colorbar(ax_h, nSel, vids)
        cb = colorbar(ax_h);
        if nSel <= 20
            tick_vals = [-1, 0];
            tick_labels = {'Collision', 'Empty'};
            for vi = 1:nSel
                tick_vals(end+1) = vi; %#ok<AGROW>
                tick_labels{end+1} = entity_labels{vids(vi)}; %#ok<AGROW>
            end
            cb.Ticks = tick_vals;
            cb.TickLabels = tick_labels;
            cb.FontSize = 8;
        else
            cb.Label.String = 'Vehicle index (-1=collision)';
        end
    end

    % =====================================================================
    % NESTED: CW bin info string for status bar
    % =====================================================================
    function s = cw_bin_info_str(vids)
        s = '';
        if isempty(cw_assignment), return; end
        if ~isfield(cw_params, 'bin_to_subchannel'), return; end
        if length(vids) > 5, return; end
        parts = {};
        for vi = 1:length(vids)
            vid = vids(vi);
            if vid <= length(cw_assignment) && ~isempty(cw_assignment{vid})
                bins = cw_assignment{vid};
                schs = unique(cw_params.bin_to_subchannel(bins));
                parts{end+1} = sprintf('%s: bins[%s](SCH %s)', ...
                    entity_labels{vid}, num2str(bins), num2str(schs)); %#ok<AGROW>
            end
        end
        if ~isempty(parts)
            s = strjoin(parts, '   ');
        end
    end

    % =====================================================================
    % NESTED: Main refresh callback
    % =====================================================================
    function refresh_callback(~, ~)
        try
            view_mode = get(h_view_mode, 'Value');

            % --- Parse vehicle selection ---
            veh_str = strtrim(get(h_veh_edit, 'String'));
            if isempty(veh_str) || strcmpi(veh_str, 'all')
                veh_ids = 1:numEntities;
            else
                veh_ids = eval(['[' veh_str ']']);
            end
            veh_ids = unique(veh_ids(veh_ids >= 1 & veh_ids <= numEntities));
            if isempty(veh_ids)
                set(h_info, 'String', 'No valid vehicle IDs selected.');
                return;
            end

            % --- Parse time range ---
            t_start = max(1, round(str2double(get(h_time_start, 'String'))));
            t_end   = min(Tdur, round(str2double(get(h_time_end, 'String'))));
            if isnan(t_start), t_start = 1; end
            if isnan(t_end),   t_end = 1000; end
            if t_end < t_start, t_end = t_start + 999; end
            t_end = min(t_end, Tdur);
            display_duration = t_end - t_start + 1;

            % --- Common state ---
            show_coll = get(h_show_coll, 'Value');
            nSel = length(veh_ids);
            veh_set = false(numEntities, 1);
            veh_set(veh_ids) = true;
            veh_to_color = zeros(numEntities, 1);
            for vi = 1:nSel
                veh_to_color(veh_ids(vi)) = vi;
            end

            % --- Title vehicle string ---
            if nSel <= 8
                veh_labels = arrayfun(@(v) entity_labels{v}, veh_ids, ...
                    'UniformOutput', false);
                veh_title = strjoin(veh_labels, ', ');
            else
                veh_title = sprintf('%d vehicles (%s - %s)', nSel, ...
                    entity_labels{veh_ids(1)}, entity_labels{veh_ids(end)});
            end

            % =============================================================
            switch view_mode
            % =============================================================
            case 1  % ======== DATA GRID ========
                msg_sel_idx = get(h_msg_list, 'Value');
                sel_msg_names = unique_msg_names(msg_sel_idx);
                show_retx = get(h_show_retx, 'Value');
                show_cw_ovl = get(h_cw_overlay, 'Value') && ...
                    ~isempty(cwEventLog) && size(cwEventLog, 1) > 0;

                % Filter txLog
                veh_mask = veh_set(txLog(:,1));
                if show_retx
                    retx_mask = true(size(txLog, 1), 1);
                else
                    retx_mask = txLog(:,5) == 0;
                end
                abs_start = t_start + Tsense_history;
                abs_end   = t_end + Tsense_history;
                time_mask = txLog(:,3) >= int32(abs_start) & ...
                            txLog(:,3) <= int32(abs_end);
                pre_mask = veh_mask & retx_mask & time_mask;
                pre_indices = find(pre_mask);

                msg_mask_pre = false(length(pre_indices), 1);
                for qi = 1:length(pre_indices)
                    r = pre_indices(qi);
                    msg_mask_pre(qi) = ismember(txLog_msg_names{r}, sel_msg_names);
                end
                final_indices = pre_indices(msg_mask_pre);
                filtered_log = txLog(final_indices, :);
                filtered_msg = txLog_msg_names(final_indices);

                % Build grid
                grid_data = zeros(Nslch, display_duration);
                for ev = 1:size(filtered_log, 1)
                    sf_rel = double(filtered_log(ev, 3)) - Tsense_history ...
                             - t_start + 1;
                    if sf_rel < 1 || sf_rel > display_duration, continue; end
                    start_sc = double(filtered_log(ev, 4));
                    L        = double(filtered_log(ev, 8));
                    veh_id   = double(filtered_log(ev, 1));
                    is_coll  = filtered_log(ev, 6);
                    color_id = veh_to_color(veh_id);
                    for sc = start_sc : min(start_sc + L - 1, Nslch)
                        if show_coll && is_coll
                            grid_data(sc, sf_rel) = -1;
                        elseif grid_data(sc, sf_rel) == 0
                            grid_data(sc, sf_rel) = color_id;
                        elseif show_coll
                            grid_data(sc, sf_rel) = -1;
                        end
                    end
                end

                % Render
                cla(ax); hold(ax, 'off');
                imagesc(ax, t_start:t_end, 1:Nslch, grid_data);

                msg_title = strjoin(sel_msg_names, ', ');
                title(ax, sprintf('Data Grid  |  %s  |  %s', ...
                    veh_title, msg_title), 'FontSize', 11);
                xlabel(ax, sprintf('Subframe (%d - %d)   [%.1fs - %.1fs]', ...
                    t_start, t_end, t_start/1000, t_end/1000));
                ylabel(ax, 'Subchannel');

                colormap(ax, build_colormap(nSel));
                caxis(ax, [-1 nSel]);
                build_colorbar(ax, nSel, veh_ids);
                set(ax, 'YDir', 'normal');
                yticks(ax, 1:Nslch);
                grid(ax, 'on');
                set(ax, 'GridAlpha', 0.3);

                % --- CW overlay markers ---
                n_cw_markers = 0;
                if show_cw_ovl
                    cw_tx_mask = cwEventLog(:,4) == 1;
                    cw_tx = cwEventLog(cw_tx_mask, :);
                    cw_tx = cw_tx(ismember(cw_tx(:,1), veh_ids), :);
                    cw_sf = double(cw_tx(:,2)) - Tsense_history;
                    cw_tm = cw_sf >= t_start & cw_sf <= t_end;
                    cw_tx = cw_tx(cw_tm, :);
                    cw_sf = cw_sf(cw_tm);

                    if ~isempty(cw_tx)
                        cw_schs = double(cw_params.bin_to_subchannel( ...
                            double(cw_tx(:,3))));
                        % Color markers by vehicle
                        cw_cids = arrayfun(@(v) veh_to_color(v), ...
                            double(cw_tx(:,1)));
                        if nSel <= 20
                            sc = lines(nSel);
                        else
                            sc = parula(nSel);
                        end
                        cw_rgb = sc(cw_cids, :);
                        hold(ax, 'on');
                        scatter(ax, cw_sf, cw_schs, 14, cw_rgb, 'x', ...
                            'LineWidth', 0.8);
                        hold(ax, 'off');
                        n_cw_markers = size(cw_tx, 1);
                    end
                end

                % Status
                n_primary = sum(filtered_log(:,5) == 0);
                n_retx    = sum(filtered_log(:,5) == 1);
                n_coll    = sum(filtered_log(:,6) == 1);
                msg_counts = {};
                for mi = 1:length(sel_msg_names)
                    cnt = sum(strcmp(filtered_msg, sel_msg_names{mi}));
                    if cnt > 0
                        msg_counts{end+1} = sprintf('%s:%d', ...
                            sel_msg_names{mi}, cnt); %#ok<AGROW>
                    end
                end
                status = sprintf('Data: %d events (%d pri, %d retx, %d coll)   |   %s', ...
                    size(filtered_log, 1), n_primary, n_retx, n_coll, ...
                    strjoin(msg_counts, '  '));
                if show_cw_ovl
                    status = [status sprintf('   |   CW: %d markers', n_cw_markers)];
                elseif get(h_cw_overlay, 'Value')
                    status = [status '   |   CW overlay: no CW event data'];
                end
                set(h_info, 'String', status);

            % =============================================================
            case 2  % ======== CW SUBCHANNEL GRID ========
            % =============================================================
                if isempty(CW_subchannel_RH)
                    cla(ax);
                    axis(ax, [0 1 0 1]);
                    text(0.5, 0.5, {'No CW subchannel data available.', ...
                        'Run v3 simulation with CW to populate CWout.'}, ...
                        'Parent', ax, 'HorizontalAlignment', 'center', ...
                        'FontSize', 13);
                    set(h_info, 'String', 'CW_subchannel_RH not found in CWout.');
                    return;
                end

                % Extract CW subchannel data for time window
                cw_slice = double(CW_subchannel_RH(:, t_start:t_end));
                grid_data = zeros(size(cw_slice));

                % Map selected vehicles to color indices
                for vi = 1:nSel
                    grid_data(cw_slice == veh_ids(vi)) = vi;
                end
                % Collisions
                if show_coll
                    grid_data(cw_slice == -1) = -1;
                end

                % Render
                cla(ax); hold(ax, 'off');
                imagesc(ax, t_start:t_end, 1:Nslch, grid_data);

                title(ax, sprintf('CW Subchannel Grid (Mode %s)  |  %s', ...
                    upper(cw_mode_str), veh_title), 'FontSize', 11);
                xlabel(ax, sprintf('Subframe (%d - %d)   [%.1fs - %.1fs]', ...
                    t_start, t_end, t_start/1000, t_end/1000));
                ylabel(ax, 'Subchannel');

                colormap(ax, build_colormap(nSel));
                caxis(ax, [-1 nSel]);
                build_colorbar(ax, nSel, veh_ids);
                set(ax, 'YDir', 'normal');
                yticks(ax, 1:Nslch);
                grid(ax, 'on');
                set(ax, 'GridAlpha', 0.3);

                % Status
                n_occ  = sum(grid_data(:) > 0);
                n_coll = sum(grid_data(:) == -1);
                bi_str = cw_bin_info_str(veh_ids);
                status = sprintf('CW subchannel: %d occupied, %d collisions', ...
                    n_occ, n_coll);
                if ~isempty(bi_str)
                    status = [status '   |   ' bi_str];
                end
                set(h_info, 'String', status);

            % =============================================================
            case 3  % ======== CW SENSING BINS ========
            % =============================================================
                if isempty(CW_RH_sim)
                    cla(ax);
                    axis(ax, [0 1 0 1]);
                    text(0.5, 0.5, {'No CW bin history available.', ...
                        'Enable record_cw_rh_always in L2_CW_sim.m.'}, ...
                        'Parent', ax, 'HorizontalAlignment', 'center', ...
                        'FontSize', 13);
                    set(h_info, 'String', 'CWout.CW_RH not found.');
                    return;
                end

                % Slice CW_RH_sim (bins x Tdur) to time window
                ts = max(1, t_start);
                te = min(size(CW_RH_sim, 2), t_end);
                if te < ts
                    cla(ax); axis(ax, [0 1 0 1]);
                    text(0.5, 0.5, 'Invalid time range', 'Parent', ax, ...
                        'HorizontalAlignment', 'center', 'FontSize', 12);
                    return;
                end
                rh_slice = double(CW_RH_sim(:, ts:te));  % total_bins x W

                % Mask to only selected vehicles (collisions flagged as -bin
                % don't carry vehicle ID — keep them wherever selected
                % vehicles have any presence in that bin).
                veh_mask = ismember(rh_slice, veh_ids);
                bin_has_veh = any(veh_mask, 2);
                coll_mask = rh_slice < 0;
                if show_coll
                    keep_rows = bin_has_veh | any(coll_mask & bin_has_veh, 2);
                else
                    keep_rows = bin_has_veh;
                end
                used_bins = find(keep_rows);
                if isempty(used_bins)
                    cla(ax); axis(ax, [0 1 0 1]);
                    text(0.5, 0.5, 'No CW activity for selected vehicles', ...
                        'Parent', ax, 'HorizontalAlignment', 'center', ...
                        'FontSize', 12);
                    set(h_info, 'String', 'No CW bins used by selection.');
                    return;
                end

                % Build compact display grid: nBins x W
                sub = rh_slice(used_bins, :);
                nBins = length(used_bins);
                W = size(sub, 2);
                cw_bin_grid = zeros(nBins, W);
                for r = 1:nBins
                    for c = 1:W
                        v = sub(r, c);
                        if v > 0 && ismember(v, veh_ids)
                            cw_bin_grid(r, c) = veh_to_color(v);
                        elseif show_coll && v < 0
                            cw_bin_grid(r, c) = -1;
                        end
                    end
                end

                % Stub for downstream code that may still reference cw_tx
                cw_tx = zeros(0, 5);
                unique_bins = used_bins;

                % Render
                cla(ax); hold(ax, 'off');
                imagesc(ax, ts:te, 1:nBins, cw_bin_grid);

                title(ax, sprintf(...
                    'CW Sensing Bins (Mode %s)  |  %s  |  %d bins', ...
                    upper(cw_mode_str), veh_title, nBins), 'FontSize', 11);
                xlabel(ax, sprintf('Subframe (%d - %d)   [%.1fs - %.1fs]', ...
                    t_start, t_end, t_start/1000, t_end/1000));
                ylabel(ax, 'Sensing Bin (subchannel)');

                colormap(ax, build_colormap(nSel));
                caxis(ax, [-1 nSel]);
                build_colorbar(ax, nSel, veh_ids);
                set(ax, 'YDir', 'normal');

                % Y-axis labels: bin index + subchannel
                if nBins <= 40
                    yticks(ax, 1:nBins);
                    ylabels = cell(nBins, 1);
                    for bi = 1:nBins
                        b = unique_bins(bi);
                        sch = cw_params.bin_to_subchannel(b);
                        ylabels{bi} = sprintf('%d (S%d)', b, sch);
                    end
                    yticklabels(ax, ylabels);
                    set(ax, 'FontSize', 8);
                else
                    ytick_step = max(1, floor(nBins / 25));
                    ytick_pos = 1:ytick_step:nBins;
                    yticks(ax, ytick_pos);
                    ylabels = cell(length(ytick_pos), 1);
                    for ti = 1:length(ytick_pos)
                        b = unique_bins(ytick_pos(ti));
                        sch = cw_params.bin_to_subchannel(b);
                        ylabels{ti} = sprintf('%d (S%d)', b, sch);
                    end
                    yticklabels(ax, ylabels);
                end

                grid(ax, 'on');
                set(ax, 'GridAlpha', 0.3);

                % Draw subchannel boundary lines
                hold(ax, 'on');
                for s = 1:Nslch-1
                    bnd = cw_params.subchannel_bin_range(s, 2);
                    row_above = find(unique_bins <= bnd, 1, 'last');
                    row_below = find(unique_bins > bnd, 1, 'first');
                    if ~isempty(row_above) && ~isempty(row_below)
                        y_line = (row_above + row_below) / 2;
                        line(ax, [ts te], [y_line y_line], ...
                            'Color', [0.3 0.3 0.3], 'LineWidth', 1.2, ...
                            'LineStyle', '--');
                    end
                end
                hold(ax, 'off');

                % Status
                n_occ = sum(cw_bin_grid(:) > 0);
                n_coll = sum(cw_bin_grid(:) == -1);
                bi_str = cw_bin_info_str(veh_ids);
                status = sprintf('CW bins: %d bins, %d occupied cells, %d collisions', ...
                    nBins, n_occ, n_coll);
                if ~isempty(bi_str)
                    status = [status '   |   ' bi_str];
                end
                set(h_info, 'String', status);

            end  % switch

        catch ME
            set(h_info, 'String', ['Error: ' ME.message]);
            fprintf('Interactive grid error: %s\n', ME.message);
            for si = 1:length(ME.stack)
                fprintf('  %s line %d\n', ME.stack(si).name, ME.stack(si).line);
            end
        end
    end

end
