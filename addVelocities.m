% addVelocities.m
%
% THIS IS STEP 2 OF POSITION DATA CLEANUP PROCESS: append velocity data to cleaned files
%
% This program uses displacements from videoCleanup.m to append a
% velocities column to the data files.
%
% **Note that the clump cleanups from videoCleanup.m were not 100% perfect,
% so some weird but sparse velocity spikes appear when graphing. I used the
% graphs to identify the timestamp and manually delete that row.**
%
%
%
%


clc;
clear all;

% List of IDs for the cleaned files to process
ids = {'0_1', '1_1', '1_2', '1_3', '2_1', '2_2', '2_3', '3_1', '3_2', '3_3', '3_4', ...
       '4_1', '4_2', '4_3', '5_1', '5_2', '5_3', '6_1', '6_2', '6_3', '7_1', ...
       '7_2', '7_3', '8_1', '8_2', '8_3', '9_1', '9_2', '9_3', '13_1', '13_2', ...
       '13_3', '14_1', '14_2', '14_3', '15_1', '15_2', '15_3', '16_1'};

% Input folder containing the cleaned files
inputFolder = '/Users/jaoch/Documents/MATLAB/cleaned video data'; % Replace with the folder path **MAKE SURE IT'S THE OUTPUT PATH FROM videoCleanup.m**
outputFolder = '/Users/jaoch/Documents/MATLAB/cleaned video data with speeds'; % Replace with the desired output folder

% Scaling factor: pixels per meter
scalingFactor = 176.32;

% Loop through each ID and process the corresponding cleaned file
for i = 1:length(ids)
    % Construct the input file name
    inputFile = fullfile(inputFolder, [ids{i}, '_cleaned.csv']);
    
    % Read the cleaned file
    data = readtable(inputFile);

    % Scale the x, y, and Displacement columns from pixels to meters
    data.x = data.x / scalingFactor; % Scale x column
    data.y = data.y / scalingFactor; % Scale y column
    data.Displacement = data.Displacement / scalingFactor; % Scale Displacement column

    % Calculate the time elapsed between consecutive rows
    timeElapsed = [0; diff(data.Times)]; % Time difference between consecutive rows

    % Calculate speed as displacement divided by elapsed time
    speed = data.Displacement ./ timeElapsed;

    % Replace NaN or Inf values in speed (first row has 0 elapsed time)
    speed(isnan(speed) | isinf(speed)) = 0;

    % Add the speed column to the table
    data.Speed = speed;

    % Construct the output file name
    outputFile = fullfile(outputFolder, [ids{i}, '_cleaned_with_speed.csv']);

    % Save the updated table to a new CSV file
    writetable(data, outputFile);
    
    % Display a progress message
    disp(['Updated file saved: ', outputFile]);
end

% Display a final confirmation message
disp('All files updated with scaled values and speed column successfully.');
