% videoCleanup.m
%
% THIS IS STEP 1 OF POSITION DATA CLEANUP PROCESS: filter original data and clean up csv 
% 
% Because the video tracking analysis rate was faster than the video frame  
% rate, some position readings were clumped together while the video frame 
% did not change, leading to issues when calculating speed.
% 
% The resulting data from this cleanup is directly used in position
% tracking.
%
% The next step in cleanup is appending speed measurements, done in
% addVelocities.m


clc
clear all

% List of IDs for the files to process
ids = {'0_1', '1_1', '1_2', '1_3', '2_1', '2_2', '2_3', '3_1', '3_2', '3_3', '3_4', ...
       '4_1', '4_2', '4_3', '5_1', '5_2', '5_3', '6_1', '6_2', '6_3', '7_1', ...
       '7_2', '7_3', '8_1', '8_2', '8_3', '9_1', '9_2', '9_3', '13_1', '13_2', ...
       '13_3', '14_1', '14_2', '14_3', '15_1', '15_2', '15_3', '16_1'};

% Input folder containing the CSV files
inputFolder = '/Users/jaoch/Documents/MATLAB/video data'; % Replace with the folder path
outputFolder = '/Users/jaoch/Documents/MATLAB/cleaned video data'; % Replace with the desired output folder

% Loop through each ID and process the corresponding file
for i = 1:length(ids)
    % Construct the input file name
    inputFile = fullfile(inputFolder, [ids{i}, '_clippedDLC_Resnet50_Asym Tail JoachimNov26shuffle1_snapshot_050_filtered.csv']);
    
    % Read the CSV file, skipping the first two rows
    data = readtable(inputFile, 'HeaderLines', 2);

    % Remove unnecessary columns: "coords" (1st column) and "likelihood" (4th column)
    data(:, [1, 4]) = [];

    % Rename columns for clarity (optional)
    data.Properties.VariableNames = {'x', 'y'};

    % Add a new column for 60 FPS timestamps
    numRows = height(data);
    data.Times = (0:numRows-1)' * 0.01667; % Time step of 1/60 seconds

    % Calculate displacement between consecutive points
    dx = [0; diff(data.x)]; % Change in x
    dy = [0; diff(data.y)]; % Change in y
    data.Displacement = sqrt(dx.^2 + dy.^2); % Euclidean distance for displacement

    % Initialize an array to keep track of rows to include
    includeRow = false(numRows, 1);

    % Always include the first row (time = 0)
    includeRow(1) = true;

    % Loop through the rows to apply the dynamic threshold
    for j = 2:numRows
        % Include the row if its displacement is 5 times larger than the previous row's displacement
        if data.Displacement(j) > 11.5 * (data.Displacement(j-1)+0.0051)
            includeRow(j) = true;
        end
    end

    % Filter the data to retain only rows that meet the condition
    cleanedData = data(includeRow, :);

    % Ensure the first row of data (time = 0) is always included
    if cleanedData.Times(1) ~= 0
        cleanedData = [data(1, :); cleanedData];
    end

    % Exclude the fourth (last) row from the cleaned data
    if height(cleanedData) >= 4
        cleanedData(end, :) = [];
    end

    % Construct the output file name
    outputFile = fullfile(outputFolder, [ids{i}, '_cleaned.csv']);

    % Save the cleaned data to a new CSV file
    writetable(cleanedData, outputFile);
    
    % Display a progress message
    disp(['Cleaned file saved: ', outputFile]);
end

% Display a final confirmation message
disp('All files cleaned');
