% swimmingPerpendicularError 
%
% *FOR STRAIGHT SWIM TESTS*
%
% Processing 3 trials at a time from the addVelocities.m outputs (i.e. ends
% in "_cleaned_with_speed.csv", this program has 3 main goals:
%
% 1) Calculates the perpendicular errors at every point from the best fit 
%   line of the data
%
% 2) Graphs the position points and best fit line
%
% 3) Outputs all of the positions & corresponding perpendicular errors to
%    one csv file (all_perpenducular_errors.csv)



close all;
clc;

% Step 1: File information
% Specify the common filename ending and the prefixes for the files
file_prefixes = {'1_1', '1_2', '1_3'}; % Replace with actual prefixes
file_suffix = '_cleaned_with_speed.csv'; % Replace with actual suffixes * FROM OUTPUT OF addVelocities.m * 

% Initialize a table to store results from all files
all_results = table();

% Step 2: Loop through each file
for i = 1:length(file_prefixes)
    % Construct the full filename
    file_name = strcat(file_prefixes{i}, file_suffix);
    disp(['Processing file: ', file_name]);

    % Load the data from the current file
    data = readmatrix(file_name);

    % Extract x and y positions
    x = data(:, 1); % First column (x positions)
    y = data(:, 2); % Second column (y positions)

    % Fit a straight line to the data
    p = polyfit(x, y, 1); % Linear fit (degree 1)

    % Calculate signed perpendicular distances
    a = -p(1); % Line equation coefficients
    b = 1;
    c = -p(2);

    % Compute signed and absolute distances
    signed_distances = (a*x + b*y + c) / sqrt(a^2 + b^2);
    distances = abs(signed_distances); % Absolute distances
    directions = signed_distances > 0; % 1 = "left", 0 = "right"

    % Create a table for this file's results
    file_results = table(x, y, signed_distances, distances, directions, ...
        'VariableNames', {'X', 'Y', 'SignedDistance', 'Distance', 'Direction'});
    
    % Add a column for the file identifier
    file_results.FileName = repmat({file_name}, size(file_results, 1), 1);

    % Append to the combined results table
    all_results = [all_results; file_results];
end

% Step 3: Save all results to a consolidated CSV file
output_file_name = 'all_perpendicular_errors.csv';
writetable(all_results, output_file_name);
disp(['Results saved to ', output_file_name]);

% Step 4: Plotting each file 
for i = 1:length(file_prefixes)
    % Extract data for the current file
    file_name = strcat(file_prefixes{i}, file_suffix);
    file_data = all_results(strcmp(all_results.FileName, file_name), :);

    % Plot
    figure;
    plot(file_data.X, file_data.Y, 'o', 'DisplayName', 'Data Points');
    hold on;
    p = polyfit(file_data.X, file_data.Y, 1); % Recalculate the best-fit line
    x_custom = linspace(min(file_data.X), max(file_data.X), 250);
    y_custom = polyval(p, x_custom);
    plot(x_custom, y_custom, '-r', 'DisplayName', 'Best-Fit Line');
    scatter(file_data.X(file_data.Direction == 1), file_data.Y(file_data.Direction == 1), ...
        40, 'b', 'filled', 'DisplayName', 'Left of Line');
    scatter(file_data.X(file_data.Direction == 0), file_data.Y(file_data.Direction == 0), ...
        40, 'b', 'filled', 'DisplayName', 'Right of Line');
    xlabel('X Positions');
    ylabel('Y Positions');
    legend('Data Points', 'Best-Fit Line');
    legend('show');
    title(['Trajectory Data for ', file_name]);
    hold off;
end