function L2_plot_results(L2out, CWout, Nslch, Tsense_history, Tdur, simseconds)
% L2_PLOT_RESULTS  Generate L2 performance visualization graphs
%
% Generates 8 figures showing SPS Mode 4 simulation performance with
% multi-subchannel (L_subCH) visualization support:
%   1. Resource Grid Heatmap (first 1 second) - shows multi-subchannel blocks
%   2. CBR vs Time
%   3. Collision Rate vs Time
%   4. Reselection Rate vs Time
%   5. Per-Vehicle TX Timeline (sample of 20 vehicles)
%   6. Inter-Packet Gap Histogram
%   7. Subchannel Utilization
%   8. Packet Success Rate vs Time
%
% Inputs:
%   L2out          - L2 performance output struct (from L2_output_interface)
%   CWout          - CW sensing output struct (from L2_output_interface)
%   Nslch          - Number of sidelink sub-channels
%   Tsense_history - Sensing history duration in subframes
%   Tdur           - Simulation duration in subframes
%   simseconds     - Simulation duration in seconds

    fprintf('Generating performance plots...\n');

    txLog = L2out.txLog;
    primaryMask = txLog(:,5) == 0;
    primaryLog = txLog(primaryMask, :);

    %% ===== Figure 1: Resource Grid Heatmap (first 1 second) =====
    % Shows multi-subchannel blocks: each TX occupies L_subCH rows
    figure('Name', 'Resource Grid Visualization', 'NumberTitle', 'off', ...
           'Position', [0 100 1200 400]);

    display_duration = min(1000, Tdur);
    grid_data = zeros(Nslch, display_duration);

    for ev = 1:size(primaryLog, 1)
        sf = double(primaryLog(ev, 3)) - Tsense_history;
        if sf < 1 || sf > display_duration, continue; end

        start_sc = double(primaryLog(ev, 4));
        L = double(primaryLog(ev, 8));
        veh_id = double(primaryLog(ev, 1));
        is_coll = primaryLog(ev, 6);

        for sc = start_sc : min(start_sc + L - 1, Nslch)
            if is_coll
                grid_data(sc, sf) = -1;  % Collision marker
            elseif grid_data(sc, sf) == 0
                grid_data(sc, sf) = veh_id;
            else
                grid_data(sc, sf) = -1;  % Multiple vehicles = collision
            end
        end
    end

    % Also show retransmissions in the grid
    retxLog = txLog(txLog(:,5) == 1, :);
    for ev = 1:size(retxLog, 1)
        sf = double(retxLog(ev, 3)) - Tsense_history;
        if sf < 1 || sf > display_duration, continue; end

        start_sc = double(retxLog(ev, 4));
        L = double(retxLog(ev, 8));
        veh_id = double(retxLog(ev, 1));
        is_coll = retxLog(ev, 6);

        for sc = start_sc : min(start_sc + L - 1, Nslch)
            if is_coll
                grid_data(sc, sf) = -1;
            elseif grid_data(sc, sf) == 0
                grid_data(sc, sf) = veh_id;
            else
                grid_data(sc, sf) = -1;
            end
        end
    end

    imagesc(1:display_duration, 1:Nslch, grid_data);
    xlabel('Subframe (first 1s)');
    ylabel('Subchannel');
    title(sprintf('Resource Grid: Vehicle Occupancy (first 1s, L_{subCH} shown as blocks)'));
    cb = colorbar;
    cb.Label.String = 'Vehicle ID (-1 = collision)';

    % Custom colormap: maps data values to colors via caxis([-1, nVeh])
    %   Value -1 -> colormap row 1 (red = collision)
    %   Value  0 -> colormap row 2 (white = empty)
    %   Value  k -> colormap row 2+k (distinct vehicle color)
    nVeh = L2out.params.numEntities;
    nColors = 2 + max(nVeh, 1);
    cmap = zeros(nColors, 3);
    cmap(1, :)   = [0.8 0 0];        % row 1: collision (value=-1) -> red
    cmap(2, :)   = [1 1 1];          % row 2: empty (value=0) -> white
    veh_colors   = parula(max(nVeh, 1));
    cmap(3:end, :) = veh_colors;     % rows 3+: vehicle IDs -> parula
    colormap(gca, cmap);
    caxis([-1 max(nVeh, 1)]);
    set(gca, 'YDir', 'normal');
    yticks(1:Nslch);
    grid on;

    %% ===== Figure 2: CBR vs Time =====
    figure('Name', 'Channel Busy Ratio', 'NumberTitle', 'off', ...
           'Position', [50 100 800 400]);

    time_s = (1:Tdur) / 1000;
    cbr_smooth = movmean(L2out.CBR, 1000);
    plot(time_s, L2out.CBR, '-', 'LineWidth', 0.3, 'Color', [0.7 0.7 1]);
    hold on;
    plot(time_s, cbr_smooth, 'b-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('CBR');
    title(sprintf('Channel Busy Ratio vs Time (multi-subCH, mean=%.1f%%)', ...
          mean(L2out.CBR)*100));
    legend('Per-subframe', '1s moving average', 'Location', 'best');
    grid on;
    ylim([0 min(max(L2out.CBR)*1.2, 1)]);

    %% ===== Figure 3: Collision Rate vs Time =====
    figure('Name', 'Collision Rate', 'NumberTitle', 'off', ...
           'Position', [50 300 800 400]);

    coll_rate_smooth = movmean(L2out.collisionCount, 1000);
    plot(time_s, coll_rate_smooth, 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Collisions per subframe (1s avg)');
    title(sprintf('Collision Rate vs Time (total: %d)', L2out.stats.totalCollisions));
    grid on;

    %% ===== Figure 4: Reselection Rate vs Time =====
    figure('Name', 'Reselection Rate', 'NumberTitle', 'off', ...
           'Position', [50 200 800 400]);

    if ~isempty(L2out.reselLog)
        resel_subframes = double(L2out.reselLog(:,3)) - Tsense_history;
        resel_per_second = histcounts(resel_subframes, 0:1000:Tdur);
        bar(0:(length(resel_per_second)-1), resel_per_second, 1, ...
            'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'none');
    end
    xlabel('Time (s)');
    ylabel('Reselections per second');
    title(sprintf('SPS Reselection Rate (total: %d)', L2out.stats.totalReselections));
    grid on;

    %% ===== Figure 5: Per-Vehicle TX Timeline =====
    figure('Name', 'Vehicle TX Timeline', 'NumberTitle', 'off', ...
           'Position', [50 100 1200 500]);

    numSample = min(20, L2out.params.numEntities);
    sample_vehicles = 1:numSample;
    colors = lines(numSample);

    for vi = 1:numSample
        v = sample_vehicles(vi);
        if isempty(CWout.txSubframes{v}), continue; end
        txSFs = double(CWout.txSubframes{v}) / 1000;
        plot(txSFs, vi * ones(size(txSFs)), '.', 'MarkerSize', 2, 'Color', colors(vi,:));
        hold on;
    end
    xlabel('Time (s)');
    ylabel('Vehicle Index');
    title(sprintf('Per-Vehicle TX Timeline (first %d vehicles)', numSample));
    yticks(1:numSample);
    ylim([0.5, numSample + 0.5]);
    grid on;

    %% ===== Figure 6: Inter-Packet Gap Histogram =====
    figure('Name', 'Inter-Packet Gap Distribution', 'NumberTitle', 'off', ...
           'Position', [50 100 600 400]);

    all_ipg = cell2mat(L2out.IPG);
    if ~isempty(all_ipg)
        histogram(all_ipg, 'BinWidth', 1, 'Normalization', 'probability', ...
                  'FaceColor', [0.3 0.7 0.4], 'EdgeColor', 'none');
        hold on;
        xline(100, 'r--', 'LineWidth', 2, 'Label', 'Nominal 10Hz (100 sf)');
        if any(all_ipg > 500)
            xline(1000, 'm--', 'LineWidth', 2, 'Label', 'Nominal 1Hz (1000 sf)');
        end
        xlabel('Inter-Packet Gap (subframes)');
        ylabel('Probability');
        title('Inter-Packet Gap Distribution (primary TX)');
        xlim([0, min(max(all_ipg)+10, 1200)]);
        grid on;

        stats_str = sprintf('Mean: %.1f sf\nMedian: %.0f sf\nStd: %.1f sf', ...
            mean(all_ipg), median(all_ipg), std(all_ipg));
        text(0.95, 0.95, stats_str, 'Units', 'normalized', ...
            'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
            'BackgroundColor', 'w', 'EdgeColor', 'k', 'FontSize', 9);
    else
        text(0.5, 0.5, 'No IPG data available', 'Units', 'normalized', ...
            'HorizontalAlignment', 'center');
    end

    %% ===== Figure 7: Subchannel Utilization =====
    figure('Name', 'Subchannel Utilization', 'NumberTitle', 'off', ...
           'Position', [50 100 600 400]);

    bar(1:Nslch, L2out.subchannelUtil * 100, 0.6, ...
        'FaceColor', [0.2 0.5 0.8], 'EdgeColor', 'none');
    xlabel('Subchannel');
    ylabel('Utilization (%)');
    title(sprintf('Resource Utilization per Subchannel (multi-subCH, resource util=%.1f%%)', ...
          L2out.stats.resourceUtilization * 100));
    xticks(1:Nslch);
    grid on;
    ylim([0, max(L2out.subchannelUtil * 100) * 1.2 + 1]);

    hold on;
    yline(mean(L2out.subchannelUtil) * 100, 'r--', 'LineWidth', 1.5, ...
          'Label', sprintf('Mean: %.1f%%', mean(L2out.subchannelUtil)*100));

    %% ===== Figure 8: Packet Success Rate vs Time =====
    figure('Name', 'Packet Success Rate', 'NumberTitle', 'off', ...
           'Position', [50 300 600 400]);

    total_tx_per_sec = zeros(1, simseconds);
    successful_tx_per_sec = zeros(1, simseconds);

    for s = 1:simseconds
        sf_lo = (s-1)*1000 + 1;
        sf_hi = s * 1000;

        sec_mask = double(primaryLog(:,3)) - Tsense_history >= sf_lo & ...
                   double(primaryLog(:,3)) - Tsense_history <= sf_hi;
        total_tx_per_sec(s) = sum(sec_mask);
        successful_tx_per_sec(s) = sum(sec_mask & primaryLog(:,6) == 0);
    end

    success_rate = successful_tx_per_sec ./ max(total_tx_per_sec, 1) * 100;
    plot(1:simseconds, success_rate, 'g-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Successful TX Rate (%)');
    title(sprintf('Packet Success Rate (mean: %.1f%%)', mean(success_rate)));
    grid on;
    ylim([max(0, min(success_rate)-5), 100]);

    hold on;
    yline(mean(success_rate), 'r--', 'LineWidth', 1, ...
          'Label', sprintf('Mean: %.1f%%', mean(success_rate)));

    fprintf('  8 figures generated.\n');

end
