%% motor_sim_test 
% 
% This program simuates a DC motor response to P, I, D, and feed-forward 
% control parameters after being given a speed setpoint in Hz. 
% 
% This program uses lsim() to simulate the motor's response using its 
% transfer function, and graphs & extracts performance data from the 
% resulting measuredSpeed array.

clear;
clc;
close all;

% Motor Parameters
J = 0.00048; % inertia (kg*m^2)
b = 0.000425; % viscous friction (Nm*s/rad)
K = 0.196; % torque constant (Nm/A)
R = 2.7; % armature resistance (Ohm)
L = 0.00308; % armature inductance (H)

% Controller Gains
Kp = 47.49; % Proportional gain
Ki = 1481.89; % Integral gain
Kd = 0; % Derivative gain
Kff = 0; % Feed-forward gain

% Noise parameters
noiseMean = 0; % Gaussian noise mean and SD
noiseStdDev = 0.0;

% Simulation Time
Ts = 0.005;  % Control interval (s)
t_end = 1; 
t = 0:Ts:t_end;

% Setpoint (Hz)
setpoint = 4 * ones(size(t));  % Desired speed of 4 Hz


PWM_max = 255;
V_max = 12; 

% Motor transfer function
s = tf('s');
motor_tf = K / ((J*s + b)*(L*s + R) + K^2);
motor_tf_hz = motor_tf / (2 * pi);  % Convert to Hz from rad/s


% Misc variables
error = 0; 
integralError = 0; 
prevError = 0;
motorSpeed = zeros(size(t)); 
pwmValue = zeros(size(t));
inputVoltage = zeros(size(t));
measuredSpeed = zeros(size(t)); % Speed with noise



% Sim loop
for i = 2:length(t)
    % Add gaussian noise to measured speed
    noise = noiseMean + noiseStdDev * randn();
    measuredSpeed(i-1) = motorSpeed(i-1) + noise;
    
    % Calculate error with noise
    error = setpoint(i) - measuredSpeed(i-1);

    derivativeError = (error - prevError) / Ts;
    integralError = integralError + error * Ts;
    feedForward = 5.22*(setpoint(i)^2) + 3.67*setpoint(i) + 37.48;

    % Control signal
    pwmValue(i) = Kp * error + Ki * integralError + Kd * derivativeError + Kff * feedForward;


    prevError = error;
    
    pwmValue(i) = max(min(pwmValue(i), PWM_max), -PWM_max);
    
    % Convert PWM value to input voltage
    inputVoltage(i) = (pwmValue(i) / PWM_max) * V_max;
    
    % Simulate motor response using lsim
    [motorSpeedResponse, ~] = lsim(motor_tf_hz, inputVoltage(1:i), t(1:i));
    motorSpeed(i) = motorSpeedResponse(end);
    
    if i == length(t)
        measuredSpeed(i) = motorSpeed(i) + noiseMean + noiseStdDev * randn();
    end
end


% Calling analysis function
step_response_metrics(t, measuredSpeed, setpoint);

%% Plot results

figure;
plot(t(1:end-1), measuredSpeed(1:end-1), 'b-', 'LineWidth', 1.5);
hold on;
plot(t, setpoint, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Speed (Hz)');
title('Measured Speed with Noise');
legend('Measured Speed with Noise', 'Setpoint');
grid on;



%% Analysis

function step_response_metrics(t, measuredSpeed, setpoint)
    % Extract steady-state value
    steady_state = setpoint(end);
    
    % Calculate steady-state error
    SSE = abs(steady_state - measuredSpeed(end));
    
    % Calculate rise time (time to go from 0% to 90% of steady-state value)
    rise_time_idx = find(measuredSpeed >= 0.9 * steady_state, 1, 'first');
    if ~isempty(rise_time_idx)
        rise_time = t(rise_time_idx);
    else
        rise_time = NaN;  % If rise time cannot be determined
    end
    
    % Calculate overshoot percentage
    max_value = max(measuredSpeed);
    overshoot = ((max_value - steady_state) / steady_state) * 100;
    
    % Calculate settling time (time to stay within 2% of steady-state value)
    settling_idx = find(abs(measuredSpeed - steady_state) > 0.02 * steady_state, 1, 'last');
    if ~isempty(settling_idx)
        settling_time = t(settling_idx);
    else
        settling_time = NaN;  % If settling time cannot be determined
    end
    
    % Print Results
    fprintf('SSE: %.2f Hz\n', SSE);
    fprintf('Rise time: %.3f s\n', rise_time);
    fprintf('Overshoot: %.2f %%\n', overshoot);
    fprintf('Settling time: %.3f s\n', settling_time);
end

