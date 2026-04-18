function L2_interactive_grid(L2out, CWout, rh)
% L2_INTERACTIVE_GRID  Interactive resource grid viewer with filtering
%
% Opens a GUI with controls to explore the SPS Mode 4 resource grid:
%   - Select vehicles by ID (supports MATLAB range syntax: "1:5", "1,3,257")
%   - Filter by SAE message type (BSM, SPAT, MAP, etc.)
%   - Adjust time window (start/end subframes)
%   - Toggle retransmission and collision visibility
%   - Quick-select buttons for All OBUs, All RSUs, etc.
%
% Usage (after running L2_sim_SPS.m):
%   L2_interactive_grid(L2out, CWout, rh)
%
% Controls:
%   Vehicle IDs edit box: Enter MATLAB expression (e.g., "1:5", "[1 3 257:258]")
%   Message Types listbox: Ctrl+click or Shift+click to multi-select
%   Time Range: Start and end subframes (relative to simulation start)
%   Press ENTER in any edit box or click REFRESH to update the plot.

    % === Extract simulation parameters ===
    txLog          = L2out.txLog;
    Nslch          = L2out.params.Nslch;
    Tsense_history = L2out.params.Tsense_history;
    Tdur           = L2out.params.Tdur;
    numEntities    = L2out.params.numEntities;
    obu_r          = L2out.params.numOBUs;

    % SAE message name tables (must match L2_sim_SPS.m definitions)
    obu_names = {'BSM','SRM','PVD','RSA','TMSG'};
    rsu_names = {'SPAT','MAP','SSM','TIM','RTCM','PDM','RSA','TMSG'};

    % === Build resolved message name for each (vehicleID, pktTypeIdx) ===
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

    % Collect all unique message type names for the listbox
    all_msg_flat = {};
    for i = 1:numEntities
        all_msg_flat = [all_msg_flat, msg_names{i}];
    end
    unique_msg_names = unique(all_msg_flat);

    % === Precompute resolved message name per txLog row (fast lookup) ===
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

    % === Precompute entity labels for display ===
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
                 'Position', [50 50 1500 650], ...
                 'Color', [0.94 0.94 0.94]);

    % =====================================================================
    % CONTROL PANEL (left side, 19% width)
    % =====================================================================
    px = 0.01;  pw = 0.17;

    uicontrol(fig, 'Style', 'text', 'String', 'FILTERS', ...
        'Units', 'normalized', 'Position', [px 0.93 pw 0.05], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    % --- Vehicle Selection ---
    uicontrol(fig, 'Style', 'text', 'String', 'Vehicle IDs:', ...
        'Units', 'normalized', 'Position', [px 0.88 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    default_veh = sprintf('1:%d', min(numEntities, 5));
    h_veh_edit = uicontrol(fig, 'Style', 'edit', ...
        'String', default_veh, ...
        'Units', 'normalized', 'Position', [px 0.83 pw 0.05], ...
        'HorizontalAlignment', 'left', 'FontSize', 10, ...
        'Callback', @refresh_callback);

    % Quick-select buttons (3 per row)
    bw = pw/3 - 0.002;
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'OBUs', ...
        'Units', 'normalized', 'Position', [px 0.77 bw 0.05], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', obu_r)));

    if numEntities > obu_r
        uicontrol(fig, 'Style', 'pushbutton', 'String', 'RSUs', ...
            'Units', 'normalized', 'Position', [px+bw+0.002 0.77 bw 0.05], ...
            'FontSize', 8, ...
            'Callback', @(~,~) quick_select(sprintf('%d:%d', obu_r+1, numEntities)));
    end

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'All', ...
        'Units', 'normalized', 'Position', [px+2*(bw+0.002) 0.77 bw 0.05], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', numEntities)));

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'First 5', ...
        'Units', 'normalized', 'Position', [px 0.72 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', min(5, numEntities))));

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'First 20', ...
        'Units', 'normalized', 'Position', [px+bw+0.002 0.72 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', min(20, numEntities))));

    uicontrol(fig, 'Style', 'pushbutton', 'String', 'First 50', ...
        'Units', 'normalized', 'Position', [px+2*(bw+0.002) 0.72 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_select(sprintf('1:%d', min(50, numEntities))));

    % --- Message Type Selection ---
    uicontrol(fig, 'Style', 'text', 'String', 'Message Types:', ...
        'Units', 'normalized', 'Position', [px 0.66 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    uicontrol(fig, 'Style', 'text', 'String', '(Ctrl+click multi)', ...
        'Units', 'normalized', 'Position', [px 0.63 pw 0.03], ...
        'HorizontalAlignment', 'left', 'FontSize', 8, ...
        'ForegroundColor', [0.5 0.5 0.5], ...
        'BackgroundColor', [0.94 0.94 0.94]);

    h_msg_list = uicontrol(fig, 'Style', 'listbox', ...
        'String', unique_msg_names, ...
        'Units', 'normalized', 'Position', [px 0.40 pw 0.23], ...
        'Max', length(unique_msg_names), 'Min', 0, ...
        'Value', 1:length(unique_msg_names), ...
        'FontSize', 10);

    % --- Time Range ---
    uicontrol(fig, 'Style', 'text', 'String', 'Time Range (subframes):', ...
        'Units', 'normalized', 'Position', [px 0.34 pw 0.04], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
        'BackgroundColor', [0.94 0.94 0.94]);

    uicontrol(fig, 'Style', 'text', 'String', 'Start:', ...
        'Units', 'normalized', 'Position', [px 0.29 0.04 0.04], ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.94 0.94 0.94]);
    h_time_start = uicontrol(fig, 'Style', 'edit', 'String', '1', ...
        'Units', 'normalized', 'Position', [px+0.04 0.29 pw-0.04 0.05], ...
        'FontSize', 10, 'Callback', @refresh_callback);

    uicontrol(fig, 'Style', 'text', 'String', 'End:', ...
        'Units', 'normalized', 'Position', [px 0.24 0.04 0.04], ...
        'HorizontalAlignment', 'left', ...
        'BackgroundColor', [0.94 0.94 0.94]);
    h_time_end = uicontrol(fig, 'Style', 'edit', 'String', '1000', ...
        'Units', 'normalized', 'Position', [px+0.04 0.24 pw-0.04 0.05], ...
        'FontSize', 10, 'Callback', @refresh_callback);

    % Quick time buttons
    uicontrol(fig, 'Style', 'pushbutton', 'String', '1s', ...
        'Units', 'normalized', 'Position', [px 0.19 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_time(1, 1000));
    uicontrol(fig, 'Style', 'pushbutton', 'String', '5s', ...
        'Units', 'normalized', 'Position', [px+bw+0.002 0.19 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_time(1, 5000));
    uicontrol(fig, 'Style', 'pushbutton', 'String', '10s', ...
        'Units', 'normalized', 'Position', [px+2*(bw+0.002) 0.19 bw 0.04], ...
        'FontSize', 8, ...
        'Callback', @(~,~) quick_time(1, 10000));

    % --- Options ---
    h_show_retx = uicontrol(fig, 'Style', 'checkbox', ...
        'String', 'Show Retransmissions', ...
        'Units', 'normalized', 'Position', [px 0.14 pw 0.04], ...
        'Value', 1, 'BackgroundColor', [0.94 0.94 0.94]);

    h_show_coll = uicontrol(fig, 'Style', 'checkbox', ...
        'String', 'Highlight Collisions', ...
        'Units', 'normalized', 'Position', [px 0.10 pw 0.04], ...
        'Value', 1, 'BackgroundColor', [0.94 0.94 0.94]);

    % --- Refresh Button ---
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'REFRESH', ...
        'Units', 'normalized', 'Position', [px 0.03 pw 0.06], ...
        'FontSize', 12, 'FontWeight', 'bold', ...
        'BackgroundColor', [0.2 0.6 0.9], 'ForegroundColor', 'w', ...
        'Callback', @refresh_callback);

    % --- Status bar ---
    h_info = uicontrol(fig, 'Style', 'text', 'String', '', ...
        'Units', 'normalized', 'Position', [0.20 0.00 0.78 0.04], ...
        'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.94 0.94], ...
        'FontSize', 9);

    % =====================================================================
    % AXES (right side, 78% width)
    % =====================================================================
    ax = axes('Parent', fig, 'Position', [0.22 0.10 0.73 0.84]);

    % === Initial render ===
    refresh_callback([], []);

    % =====================================================================
    % NESTED: Quick-select vehicle helper
    % =====================================================================
    function quick_select(str)
        set(h_veh_edit, 'String', str);
        refresh_callback([], []);
    end

    % =====================================================================
    % NESTED: Quick-select time range helper
    % =====================================================================
    function quick_time(t1, t2)
        set(h_time_start, 'String', num2str(t1));
        set(h_time_end, 'String', num2str(min(t2, Tdur)));
        refresh_callback([], []);
    end

    % =====================================================================
    % NESTED: Main refresh callback - filters txLog and redraws grid
    % =====================================================================
    function refresh_callback(~, ~)
        try
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

            % --- Parse message type selection ---
            msg_sel_idx = get(h_msg_list, 'Value');
            sel_msg_names = unique_msg_names(msg_sel_idx);

            % --- Parse time range ---
            t_start = max(1, round(str2double(get(h_time_start, 'String'))));
            t_end   = min(Tdur, round(str2double(get(h_time_end, 'String'))));
            if isnan(t_start), t_start = 1; end
            if isnan(t_end),   t_end = 1000; end
            if t_end < t_start, t_end = t_start + 999; end
            display_duration = t_end - t_start + 1;

            % --- Options ---
            show_retx = get(h_show_retx, 'Value');
            show_coll = get(h_show_coll, 'Value');

            % --- Filter txLog ---
            % Step 1: Vehicle filter
            veh_set = false(numEntities, 1);
            veh_set(veh_ids) = true;
            veh_mask = veh_set(txLog(:,1));

            % Step 2: Retransmission filter
            if show_retx
                retx_mask = true(size(txLog, 1), 1);
            else
                retx_mask = txLog(:,5) == 0;
            end

            % Step 3: Time filter (absolute subframes)
            abs_start = t_start + Tsense_history;
            abs_end   = t_end + Tsense_history;
            time_mask = txLog(:,3) >= int32(abs_start) & txLog(:,3) <= int32(abs_end);

            % Step 4: Pre-filter (vehicle + retx + time) before expensive msg check
            pre_mask = veh_mask & retx_mask & time_mask;
            pre_indices = find(pre_mask);

            % Step 5: Message type filter (only on pre-filtered rows)
            msg_mask_pre = false(length(pre_indices), 1);
            for qi = 1:length(pre_indices)
                r = pre_indices(qi);
                msg_mask_pre(qi) = ismember(txLog_msg_names{r}, sel_msg_names);
            end
            final_indices = pre_indices(msg_mask_pre);
            filtered_log = txLog(final_indices, :);
            filtered_msg = txLog_msg_names(final_indices);

            % --- Build grid_data using sequential vehicle color indices ---
            grid_data = zeros(Nslch, display_duration);
            % Map: vehicle ID -> sequential color index (1..nSel)
            veh_to_color = zeros(numEntities, 1);
            for vi = 1:length(veh_ids)
                veh_to_color(veh_ids(vi)) = vi;
            end

            for ev = 1:size(filtered_log, 1)
                sf_rel = double(filtered_log(ev, 3)) - Tsense_history - t_start + 1;
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
                        grid_data(sc, sf_rel) = -1;  % Overlap from filtered set
                    end
                end
            end

            % --- Render ---
            cla(ax);
            imagesc(ax, t_start:t_end, 1:Nslch, grid_data);

            % Build descriptive title
            if length(veh_ids) <= 8
                veh_labels = arrayfun(@(v) entity_labels{v}, veh_ids, 'UniformOutput', false);
                veh_title = strjoin(veh_labels, ', ');
            else
                veh_title = sprintf('%d vehicles (%s %d - %s %d)', ...
                    length(veh_ids), entity_labels{veh_ids(1)}, veh_ids(1), ...
                    entity_labels{veh_ids(end)}, veh_ids(end));
            end
            msg_title = strjoin(sel_msg_names, ', ');
            title(ax, sprintf('%s  |  %s', veh_title, msg_title), 'FontSize', 11);
            xlabel(ax, sprintf('Subframe (%d - %d)   [%.1fs - %.1fs]', ...
                t_start, t_end, t_start/1000, t_end/1000));
            ylabel(ax, 'Subchannel');

            % Colormap: -1=red(collision), 0=white(empty), 1..nSel=vehicle colors
            nSel = length(veh_ids);
            cmap_grid = zeros(2 + nSel, 3);
            cmap_grid(1, :) = [0.85 0.1 0.1];   % collision -> red
            cmap_grid(2, :) = [1 1 1];           % empty -> white
            if nSel <= 20
                sel_colors = lines(nSel);        % distinctive for small sets
            else
                sel_colors = parula(nSel);        % gradient for large sets
            end
            cmap_grid(3:end, :) = sel_colors;
            colormap(ax, cmap_grid);
            caxis(ax, [-1 nSel]);

            % Colorbar with vehicle labels
            cb = colorbar(ax);
            if nSel <= 20
                tick_vals = [-1, 0];
                tick_labels = {'Collision', 'Empty'};
                for vi = 1:nSel
                    tick_vals(end+1) = vi;
                    tick_labels{end+1} = entity_labels{veh_ids(vi)};
                end
                cb.Ticks = tick_vals;
                cb.TickLabels = tick_labels;
                cb.FontSize = 8;
            else
                cb.Label.String = 'Sequential vehicle index (-1=collision)';
            end

            set(ax, 'YDir', 'normal');
            yticks(ax, 1:Nslch);
            grid(ax, 'on');
            set(ax, 'GridAlpha', 0.3);

            % --- Status bar ---
            n_primary = sum(filtered_log(:,5) == 0);
            n_retx    = sum(filtered_log(:,5) == 1);
            n_coll    = sum(filtered_log(:,6) == 1);

            % Count unique message types in filtered set
            msg_counts = {};
            for mi = 1:length(sel_msg_names)
                cnt = sum(strcmp(filtered_msg, sel_msg_names{mi}));
                if cnt > 0
                    msg_counts{end+1} = sprintf('%s:%d', sel_msg_names{mi}, cnt);
                end
            end

            status = sprintf('Showing %d events (%d primary, %d retx, %d collisions)   |   %s', ...
                size(filtered_log, 1), n_primary, n_retx, n_coll, strjoin(msg_counts, '  '));
            set(h_info, 'String', status);

        catch ME
            set(h_info, 'String', ['Error: ' ME.message]);
            fprintf('Interactive grid error: %s\n', ME.message);
            for si = 1:length(ME.stack)
                fprintf('  %s line %d\n', ME.stack(si).name, ME.stack(si).line);
            end
        end
    end

end
