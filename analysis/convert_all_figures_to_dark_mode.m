function convert_all_figures_to_dark_mode()
% CONVERT_ALL_FIGURES_TO_DARK_MODE
% Transforms all figures in results/ to a high-contrast dark theme:
% Pitch-black screen background, dark slate axes, and crisp bright-white text.

project_root = fullfile(fileparts(mfilename('fullpath')), '..');
results_dir = fullfile(project_root, 'results');

fig_files = dir(fullfile(results_dir, '*.fig'));
fprintf('Found %d .fig files in %s\n', length(fig_files), results_dir);

% Color palette for Dark Theme
dark_fig_bg  = [0.06, 0.06, 0.06];  % Deep pitch-black canvas
dark_axes_bg = [0.11, 0.11, 0.11];  % Charcoal axes interior
white_text   = [1.00, 1.00, 1.00];  % Pure bright white text
tick_color   = [0.75, 0.75, 0.75];  % Crisp light-gray ticks and frame
grid_color   = [0.25, 0.25, 0.25];  % Subtle dark grid
legend_bg    = [0.15, 0.15, 0.15];  % Legend card background

for k = 1:length(fig_files)
    fig_name = fig_files(k).name;
    fig_path = fullfile(results_dir, fig_name);
    [~, base_name, ~] = fileparts(fig_name);
    png_path = fullfile(results_dir, [base_name, '.png']);
    
    fprintf('Processing [%d/%d]: %s... ', k, length(fig_files), fig_name);
    try
        fig = openfig(fig_path, 'invisible');
        set(fig, 'Color', dark_fig_bg);
        set(fig, 'InvertHardcopy', 'off'); % PREVENTS MATLAB FORCING WHITE CANVAS ON EXPORT
        
        % 1. Supertitle / sgtitle
        all_sg = findall(fig, 'Type', 'subplottext');
        for s = all_sg'
            set(s, 'Color', white_text, 'FontWeight', 'bold');
        end
        
        % 2. Axes
        axes_list = findall(fig, 'Type', 'axes');
        for ax = axes_list'
            set(ax, 'Color', dark_axes_bg);
            set(ax, 'XColor', tick_color);
            set(ax, 'YColor', tick_color);
            set(ax, 'ZColor', tick_color);
            set(ax, 'GridColor', grid_color);
            set(ax, 'MinorGridColor', grid_color);
            set(ax, 'GridAlpha', 0.6);
            
            if isprop(ax, 'Title') && ~isempty(ax.Title)
                set(ax.Title, 'Color', white_text, 'FontWeight', 'bold');
            end
            if isprop(ax, 'XLabel') && ~isempty(ax.XLabel)
                set(ax.XLabel, 'Color', white_text);
            end
            if isprop(ax, 'YLabel') && ~isempty(ax.YLabel)
                set(ax.YLabel, 'Color', white_text);
            end
            if isprop(ax, 'ZLabel') && ~isempty(ax.ZLabel)
                set(ax.ZLabel, 'Color', white_text);
            end
        end
        
        % 3. Text objects (annotations, bar chart value labels)
        texts = findall(fig, 'Type', 'text');
        for t = texts'
            set(t, 'Color', white_text);
        end
        
        % 4. Legends
        legends = findall(fig, 'Type', 'legend');
        for leg = legends'
            set(leg, 'Color', legend_bg);
            set(leg, 'TextColor', white_text);
            set(leg, 'EdgeColor', [0.45, 0.45, 0.45]);
        end
        
        % 5. Lines: Convert black lines to silver/white so they don't disappear on dark bg
        lines = findall(fig, 'Type', 'line');
        for ln = lines'
            c = get(ln, 'Color');
            if (ischar(c) && strcmp(c, 'k')) || (isnumeric(c) && all(c < 0.15))
                set(ln, 'Color', [0.88, 0.88, 0.88]); % High visibility silver
            end
        end
        
        % 6. Bar charts: if edges are black, make them crisp light-gray
        bars = findall(fig, 'Type', 'bar');
        for b = bars'
            set(b, 'EdgeColor', [0.7, 0.7, 0.7]);
        end
        
        % Re-save FIG and PNG
        savefig(fig, fig_path);
        exportgraphics(fig, png_path, 'Resolution', 150, 'BackgroundColor', 'current');
        close(fig);
        fprintf('DONE (Converted to Dark Theme)\n');
    catch me
        fprintf('FAILED: %s\n', me.message);
        if exist('fig', 'var') && isvalid(fig)
            close(fig);
        end
    end
end

fprintf('\nAll figures in results/ successfully updated to Dark Theme!\n');

end
