%% motorPSOcontrol
%
% This program uses particle swarm optimization (PSO) to tune Kp and Ki 
% terms of a simulated motor control system. PSO parameters (e.g. n_iterations, 
% n_particles, etc.) edit various characteristics, such as runtime, thoroughness,  
% solution space size, and more (see thesis report for more background). 
%
%  
% Control performance of each particle in every iteration are displayed in the 
% command window until the final iteration has been reached. Once this 
% happens, the most optimal Kp and Ki are outputted. These gains can be 
% used in the motor model (motor_sim_test.m) to view the step response.
%
% ** THIS PROGRAM ONLY OPTIMIZES CONTROLLER GAINS FOR ONE SPEED SETPOINT AT A TIME (setpoint = 4 Hz in this case). **
%         The optimized gains for a 4 Hz setpoint are assumed to be generally adequate across the 
%         relevant speeds for our fish (1 - 6 Hz). Responses can be tested in motor_sim_test.m
%         
% 
% The diary() function is used to output all of the particle positions from the 
% command window to a .txt file from which the Kp and Ki positions at each 
% iteration can be scraped and graphed. In my thesis' example, I used ChatGPT  
% to quickly scrape through the text and obtain .csv files for graphing.
% 
% 


clear;
clc;
close all;

% Record to txt file
diary('pso_progress_tracking1.txt');


% Motor parameters
J = 0.00048; % inertia (kg*m^2)
b = 0.000425; % viscous friction (Nm*s/rad)
K = 0.196; % torque constant (Nm/A)
R = 2.7; % armature resistance (Ohm)
L = 0.00308; % armature inductance (H)

% PWM/voltage parameters
PWM_max = 255;
V_max = 12; 
Ts = 0.005; % Control time interval (s)

% **PSO Parameters**
n_particles = 50;
n_iterations = 30;
KpMin = 20; 
KpMax = 200;  
KiMax = 200;
KiMax = 2000; 
w = 0.7; 
c1 = 1.5; 
c2 = 1.5; 

% initialize particles at random locations
positions = [rand(n_particles, 1) * (KpMax - KpMin) + KpMin, rand(n_particles, 1) * (KiMax - KiMax) + KiMax];
velocities = zeros(size(positions));
personal_best_positions = positions; % Pbest
global_best_position = positions(1, :); % Gbest


personal_best_fitness = inf(n_particles, 1);
global_best_fitness = inf;

% objective function with step response and cost calculations
objective_function = @(Kp, Ki) step_response_cost(Kp, Ki, J, b, K, R, L, V_max, PWM_max, Ts);

% PSO loop
for iter = 1:n_iterations
    for i = 1:n_particles
        % Evaluate fitness of current particle
        Kp = positions(i, 1);
        Ki = positions(i, 2);
        fitness = objective_function(Kp, Ki);
        
        % Update Pbest
        if fitness < personal_best_fitness(i)
            personal_best_fitness(i) = fitness;
            personal_best_positions(i, :) = [Kp, Ki];
        end
        
        % Update Gbest
        if fitness < global_best_fitness
            global_best_fitness = fitness;
            global_best_position = [Kp, Ki];
        end
    end
    
    % Update particle velocities/positions
    for i = 1:n_particles
        r1 = rand();
        r2 = rand();
        velocities(i, :) = w * velocities(i, :) + c1 * r1 * (personal_best_positions(i, :) - positions(i, :)) + c2 * r2 * (global_best_position - positions(i, :));
        positions(i, :) = positions(i, :) + velocities(i, :);
        
        positions(i, 1) = max(min(positions(i, 1), KpMax), KpMin);
        positions(i, 2) = max(min(positions(i, 2), KiMax), KiMax);
    end
    
    fprintf('Iteration %d, Best Fitness: %.5f, Kp: %.2f, Ki: %.2f\n', global_best_fitness, global_best_position(1), global_best_position(2));
end

% Optimal gains gains so far
Kp_opt = global_best_position(1);
Ki_opt = global_best_position(2);
fprintf('Optimal Kp: %.2f, Optimal Ki: %.2f\n', Kp_opt, Ki_opt);

%Function using motor simulaton and separate metrics function to calculate
% cost at each iteration
function cost = step_response_cost(Kp, Ki, J, b, K, R, L, V_max, PWM_max, Ts)
    % Transfer Function
    s = tf('s');
    motor_tf = K / ((J*s + b)*(L*s + R) + K^2);
    motor_tf_hz = motor_tf / (2 * pi);
    
    % Simulation time
    t_end = 1;   % Simulation duration (seconds)
    t = 0:Ts:t_end;
    setpoint = 4 * ones(size(t));  % Desired speed of 4 Hz
    
    error = 0; 
    integralError = 0; 
    motorSpeed = zeros(size(t)); 
    pwmValue = zeros(size(t));
    inputVoltage = zeros(size(t));
    measuredSpeed = zeros(size(t));
    
    % Motor sim
    for i = 2:length(t)
        error = setpoint(i) - motorSpeed(i-1);
        integralError = integralError + error * Ts;
        feedForward = 5.22*(setpoint(i)^2) + 3.67*setpoint(i) + 37.48;
        pwmValue(i) = Kp * error + Ki * integralError + feedForward;
        pwmValue(i) = max(min(pwmValue(i), PWM_max), -PWM_max);
        inputVoltage(i) = (pwmValue(i) / PWM_max) * V_max;
        [motorSpeedResponse, ~] = lsim(motor_tf_hz, inputVoltage(1:i), t(1:i));
        motorSpeed(i) = motorSpeedResponse(end);
    end
    
    % Performance Metrics
    [overshoot, settling_time, rise_time, sse] = step_response_metrics(t, setpoint, motorSpeed);
    
    % Constraints
    overshoot_penalty = max(0, overshoot - 8);
    settling_time_penalty = max(0, settling_time - 0.075);
    rise_time_penalty = max(0, rise_time - 0.025);
    sse_penalty = max(0, sse - 0.01);
    
    fprintf('Kp: %.2f, Ki: %.2f\n', Kp, Ki);
    fprintf('Overshoot Penalty: %.2f\n', overshoot_penalty);
    fprintf('Settling Time Penalty: %.2f\n', settling_time_penalty);
    fprintf('Rise Time Penalty: %.2f\n', rise_time_penalty);
    fprintf('SSE Penalty: %.2f\n', sse_penalty);
    
    % Cost Function
    cost = overshoot_penalty + settling_time_penalty + rise_time_penalty + sse_penalty;

    % Print total cost for this iteration
    fprintf('Total Cost: %.5f\n\n', cost);
end

%% Metrics Function
function [overshoot, settling_time, rise_time, sse] = step_response_metrics(t, setpoint, motorSpeed)
    % SSE
    steady_state = setpoint(end);
    steady_state_value = motorSpeed(end);
    sse = abs(steady_state_value - steady_state) / steady_state;

    % Overshoot
    max_value = max(motorSpeed);
    overshoot = max(0, (max_value - steady_state) / steady_state * 100);

    % Rise time (0% to 90%)
    rise_time_idx = find(motorSpeed >= 0.9 * steady_state, 1, 'first');
    if ~isempty(rise_time_idx)
        rise_time = t(rise_time_idx);
    else
        rise_time = NaN; 
    end

    % Settling time (within 2% of steady-state)
    settling_idx = find(abs(motorSpeed - steady_state) > 0.02 * steady_state, 1, 'last');
    if ~isempty(settling_idx)
        settling_time = t(settling_idx);
    else
        settling_time = 0; % If no deviations, settling time is 0
    end

    % Debugging outputs
    fprintf('Overshoot: %.2f %%\n', overshoot);
    fprintf('Rise Time: %.3f s\n', rise_time);
    fprintf('Steady-State Error (SSE): %.2f\n', sse);
    fprintf('Settling Time: %.3f s\n', settling_time);
end

diary off;
