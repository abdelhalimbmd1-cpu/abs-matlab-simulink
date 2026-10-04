function fig = plot_tire_curves(p, save_path)
% PLOT_TIRE_CURVES Plots the Pacejka Magic Formula mu-slip curves for 4 road surfaces
% (Dry Asphalt, Wet Asphalt, Snow, Ice) and marks their peak points.

if nargin < 1 || isempty(p)
    p = init_params();
end
if nargin < 2
    save_path = fullfile(fileparts(mfilename('fullpath')), '..', 'results');
end
if ~exist(save_path, 'dir')
    mkdir(save_path);
end

lambda_range = linspace(0, 1.0, 500);

surfaces = {'dry', 'wet', 'snow', 'ice'};
names    = {'Dry Asphalt', 'Wet Asphalt', 'Packed Snow', 'Smooth Ice'};
colors   = {[0.15 0.65 0.15], [0.10 0.45 0.85], [0.85 0.55 0.10], [0.85 0.15 0.15]};

fig = figure('Name', 'Pacejka Tire Curves', 'Color', 'w', 'Position', [100, 100, 850, 550], 'Visible', 'off');
hold on; grid on; box on;

for i = 1:length(surfaces)
    s = surfaces{i};
    tp = p.tire.(s);
    mu = pacejka_tire(lambda_range, 1.0, tp);
    
    [mu_peak, idx_peak] = max(mu);
    lam_peak = lambda_range(idx_peak);
    
    plot(lambda_range, mu, 'LineWidth', 2.2, 'Color', colors{i}, ...
        'DisplayName', sprintf('%s (\\mu_{peak}=%.2f, \\lambda_{peak}=%.2f)', names{i}, mu_peak, lam_peak));
    
    % Peak point marker
    plot(lam_peak, mu_peak, 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors{i}, ...
        'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    
    % Text label at peak
    text(lam_peak + 0.02, mu_peak + 0.02, sprintf('\\mu=%.2f', mu_peak), ...
        'FontSize', 9, 'FontWeight', 'bold', 'Color', colors{i});
end

% Highlight unstable brake lockup region vs stable region
yl = ylim;
patch([0.20 1.0 1.0 0.20], [0 0 1.15 1.15], [0.9 0.9 0.9], 'FaceAlpha', 0.25, ...
    'EdgeColor', 'none', 'DisplayName', 'Unstable Slip Zone (\lambda > 0.20)');

xlabel('Longitudinal Slip Ratio \lambda = (v - \omega R) / v [-]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Tire-Road Friction Coefficient \mu [-]', 'FontSize', 11, 'FontWeight', 'bold');
title('Pacejka Magic Formula Tire Friction vs. Longitudinal Slip', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 10);
xlim([0 1.0]);
ylim([0 1.15]);

% Annotate optimal ABS operating window
text(0.12, 1.08, '\leftarrow Typical ABS Target Range (\lambda \approx 0.10 - 0.20)', ...
    'FontSize', 10, 'FontAngle', 'italic', 'Color', [0.2 0.2 0.2]);

% Save figure in both PNG and FIG formats
png_file = fullfile(save_path, 'pacejka_tire_curves.png');
fig_file = fullfile(save_path, 'pacejka_tire_curves.fig');
try
    exportgraphics(fig, png_file, 'Resolution', 150);
catch
    saveas(fig, png_file);
end
savefig(fig, fig_file, 'compact');
fprintf('Saved tire curves to %s and %s\n', png_file, fig_file);

end

