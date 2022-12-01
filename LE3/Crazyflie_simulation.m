%% Simulation script for CS1 hands-on exercise with real Crazyflies %%
%% Some pre-processing
close all;
clear;
clc;

%% Enter your controller parameters here
% Lead controller 1
k_lead_1 = 0.67;
alpha_lead_1 = 0.1;
T_lead_1 = 0.79;

% Lead controller 2
k_lead_2 = 0.092;
alpha_lead_2 = 0.1;
T_lead_2 = 2;

% Lag controller
alpha_lag = 20;
T_lag = 7;

%% Definition of transfer functions
s = tf('s');
k_x = 0.02;
m = 0.03;
G_out = feedback(5 / s, 1) * 9.81 / (s * (s + k_x / m));
K_lead_1 = k_lead_1 * (T_lead_1 * s + 1) / (alpha_lead_1 * T_lead_1 * s + 1);
K_lead_2 = k_lead_2 * (T_lead_2 * s + 1) / (alpha_lead_2 * T_lead_2 * s + 1);
K_lag = alpha_lag * (T_lag * s + 1) / (alpha_lag * T_lag * s + 1);
L_lead_1 = K_lead_1 * G_out;
L_lead_2 = K_lead_2 * G_out;
L_lead_lag = K_lag * K_lead_2 * G_out;
CL_lead_1 = feedback(L_lead_1, 1);
CL_lead_2 = feedback(L_lead_2, 1);
CL_lead_lag = feedback(L_lead_lag, 1);

%% Performance comparison of K_lead_1, K_lead_2, K_lag * K_lead_2
L_lead_1_margin = allmargin(L_lead_1);
L_lead_2_margin = allmargin(L_lead_2);
L_lead_lag_margin = allmargin(L_lead_lag);
step_info_lead_1 = stepinfo(CL_lead_1);
step_info_lead_2 = stepinfo(CL_lead_2);
step_info_lead_lag = stepinfo(CL_lead_lag);

fprintf('Controller performance comparison:\n\n');
fprintf('\t\t\t\tK_lead_1\tK_lead_2\tK_lag * K_lead_2\n');
fprintf('Crossover frequency [rad/s]\t%.2f\t\t%.2f\t\t%.2f\n', L_lead_1_margin.PMFrequency, L_lead_2_margin.PMFrequency, L_lead_lag_margin.PMFrequency);
fprintf('Phase margin [deg]\t\t%.2f\t\t%.2f\t\t%.2f\n', L_lead_1_margin.PhaseMargin, L_lead_2_margin.PhaseMargin, L_lead_lag_margin.PhaseMargin);
fprintf('Rise time [s]\t\t\t%.2f\t\t%.2f\t\t%.2f\n', step_info_lead_1.RiseTime, step_info_lead_2.RiseTime, step_info_lead_lag.RiseTime);
fprintf('%% overshoot\t\t\t%.2f\t\t%.2f\t\t%.2f\n', step_info_lead_1.Overshoot, step_info_lead_2.Overshoot, step_info_lead_lag.Overshoot);
fprintf('Max delay [s]\t\t\t%.2f\t\t%.2f\t\t%.2f\n\n', L_lead_1_margin.DelayMargin, L_lead_2_margin.DelayMargin, L_lead_lag_margin.DelayMargin);

%% Performance of K_lead_1, K_lead_2, K_lag * K_lead_2 with model mismatch
% Position measurement delay
Td = 0.2;
CL_lead_1_with_delay = feedback(L_lead_1, exp(-Td * s));
CL_lead_2_with_delay = feedback(L_lead_2, exp(-Td * s));
CL_lead_lag_with_delay = feedback(L_lead_lag, exp(-Td * s));

% Pitch angle measurement error
theta_err = 1 * pi / 180;
CL_lead_1_err = 49.05 / (s * (s + k_x / m) * (s + 5) + 49.05 * K_lead_1);
CL_lead_2_err = 49.05 / (s * (s + k_x / m) * (s + 5) + 49.05 * K_lead_2);
CL_lead_lag_err = 49.05 / (s * (s + k_x / m) * (s + 5) + 49.05 * K_lag * K_lead_2);
t = 0 : 0.02 : 30;
step_lead_1 = step(CL_lead_1, t);
step_err_lead_1 = theta_err * step(CL_lead_1_err, t);
step_lead_1_with_err = step_lead_1 - step_err_lead_1;
step_lead_2 = step(CL_lead_2, t);
step_err_lead_2 = theta_err * step(CL_lead_2_err, t);
step_lead_2_with_err = step_lead_2 - step_err_lead_2;
step_lead_lag = step(CL_lead_lag, t);
step_lead_lag_err = theta_err * step(CL_lead_lag_err, t);
step_lead_lag_with_err = step_lead_lag - step_lead_lag_err;

%% Plots
% Bode plots
fprintf('Figure 1 shows the Bode plots.\n');
figure(1);
fig = gcf;
fig.Position(1:2) = [0, 1000];
bode(L_lead_1, L_lead_2, L_lead_lag);
grid on;
legend('K_{lead,1} G_{out}', 'K_{lead,2} G_{out}', 'K_{lag} K_{lead,2} G_{out}');

% Step responses
fprintf('Figure 2 shows the step responses under different scenarios.\n\n');
figure(2);
fig = gcf;
fig.Position(1:3) = [0, 0, 1500];
subplot(1, 3, 1);
step(CL_lead_1, CL_lead_2, CL_lead_lag);
legend('K_{lead,1}', 'K_{lead,2}', 'K_{lag} K_{lead,2}');
title('Nominal Step Response');
subplot(1, 3, 2)
step(CL_lead_1_with_delay, CL_lead_2_with_delay, CL_lead_lag_with_delay);
legend('K_{lead,1}', 'K_{lead,2}', 'K_{lag} K_{lead,2}');
title('Step Response with 200ms Delay');
subplot(1, 3, 3);
plot(t, step_lead_1_with_err);
hold on;
plot(t, step_lead_2_with_err);
plot(t, step_lead_lag_with_err);
plot(t, ones(length(t), 1), 'k:');
axis tight;
legend('K\_lead,1', 'K\_lead,2', 'K\_lag K\_lead,2');
title('Step Response with 1deg pitch measurement error');
xlabel('Time (seconds)');
ylabel('Amplitude');

fprintf('The step error with 1deg pitch measurement error after 20s is: [cm]\n');
fprintf('K_lead_1:\t\t%.2f\nK_lead_2:\t\t%.2f\nK_lag * K_lead_2:\t%.2f\n\n',...
    (1 - step_lead_1_with_err(t == 20)) * 100,...
    (1 - step_lead_2_with_err(t == 20)) * 100,...
    (1 - step_lead_lag_with_err(t == 20)) * 100);
