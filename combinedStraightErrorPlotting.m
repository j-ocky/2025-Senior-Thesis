% combinedStraightErrorPlotting.m
%
% Takes the direct output of swimmingPerpendicularError.m
% (all_perpendicular_errors.csv) and plots the normalized perpendicular
% errors for 3 straight trials on one graph
%
%
%
%
%


close all;
clc;

% Step 1: Load the data from the CSV file
data = readtable('all_perpendicular_errors.csv'); 

% Extract unique file names
unique_files = unique(data.FileName);

% Initialize figure
figure;
hold on;

% Step 2: Plot the intended trajectory as a reference line
yline(0, 'k--', 'LineWidth', 2, 'DisplayName', 'Intended Trajectory');

% Step 3: Loop through each file and plot errors
colors = lines(length(unique_files)); % Generate unique colors for each file
start_x = 0; % Define a common starting x value for all files

for i = 1:length(unique_files)
    % Extract data for the current file
    file_data = data(strcmp(data.FileName, unique_files{i}), :);
    
    % Adjust x-values to start from the same point
    adjusted_x = (1:height(file_data)) + start_x; % Relative x-values starting from start_x
    
    % Plot the signed perpendicular errors
    plot(adjusted_x, file_data.SignedDistance, '-', 'Color', colors(i, :), ...
        'DisplayName', unique_files{i}, 'LineWidth', 1.5);
end

% Step 4: Finalize the plot
xlabel('Relative Index (Aligned Start)');
ylabel('Error from Best-Fit Line (m)');
legend('Intended Trajectory', 'Trial 1', 'Trial 2', 'Trial 3', 'Location', 'best');
ylim([-1, 1]);
title('Perpendicular Errors by File');
legend('show');
grid on;
hold off;