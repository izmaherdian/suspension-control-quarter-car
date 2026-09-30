%% export_results.m
% Reproduces the results reported in docs/GA_PID-LQR_QuarterCar_Report.pdf
% without re-running the (random) GA, and exports everything needed by
% tools/make_figures.py and tools/make_animation.py to ../results/.
% (Re-running the GA itself -> main_ga_pid_lqr.m, needs Global Optimization Toolbox.)
%
%   1. Builds the quarter-car state-space model (BMW 530i front axle)
%   2. Designs the baseline PID-LQR  (Q = I6, R = I2)
%   3. Designs the GA-tuned PID-LQR  (Q_opt, R_opt reported in the paper)
%   4. Simulates both in quarter_car_pid_lqr_sim.slx (30 s, bump at 3-5 s)
%
% Run from the src/ folder:  >> export_results

clear; clc; close all;
here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, '..', 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

%% Quarter-car model (front side, BMW 530i)
kk = 340;  kr = 30;  br = 1450;  mc = 408;  mus = 48.3;

A = [0 1 0 0;
    (-kk-kr)/mus -br/mus kr/mus br/mus;
    0 0 0 1;
    kr/mc br/mc -kr/mc -br/mc];
B = [[0; -1/mus; 0; 1/mc], [0; kk/mus; 0; 0]];
C = [0 0 1 0;
     0 0 0 1];
D = zeros(2);

%% Integral-backstepping augmentation
A_aug = [A B; zeros(2,4) zeros(2,2)];
B_aug = [zeros(4,2); eye(2)];
gamma = [C zeros(2,2); C*A C*B; C*A*A C*A*B];

%% Baseline PID-LQR
Q = eye(6);  R = eye(2);
K_base = lqr(A_aug, B_aug, Q, R);
[Kp_base, Ki_base, Kd_base] = pid_from_lqr(K_base, gamma, C, B);

%% GA-tuned PID-LQR (values reported in the paper)
Q_opt = diag([87.4999 47.9164 0.5189 63.4103 99.5451 0.9337]);
R_opt = diag([5.3421 9.3203]);
K_opt = lqr(A_aug, B_aug, Q_opt, R_opt);
[Kp_opt, Ki_opt, Kd_opt] = pid_from_lqr(K_opt, gamma, C, B);

%% Simulink simulation
stime = 30;
simOut = sim(fullfile(here, 'quarter_car_pid_lqr_sim.slx'), 'StopTime', num2str(stime), ...
             'SrcWorkspace', 'current', 'MaxStep', '0.005');

t   = simOut.t_out(:);
ref = simOut.ref(:,1);
T = table(t, ref, ...
    simOut.x_base_out(:,1), simOut.x_base_out(:,2), simOut.x_base_out(:,3), simOut.x_base_out(:,4), ...
    simOut.x_opt_out(:,1),  simOut.x_opt_out(:,2),  simOut.x_opt_out(:,3),  simOut.x_opt_out(:,4), ...
    simOut.y_base_out(:,1), simOut.y_base_out(:,2), simOut.y_opt_out(:,1), simOut.y_opt_out(:,2), ...
    simOut.u_base_out(:,1), simOut.u_base_out(:,end), simOut.u_opt_out(:,1), simOut.u_opt_out(:,end), ...
    'VariableNames', {'t','ref', ...
    'x1_base','x2_base','x3_base','x4_base','x1_opt','x2_opt','x3_opt','x4_opt', ...
    'y1_base','y2_base','y1_opt','y2_opt','u1_base','u2_base','u1_opt','u2_opt'});
writetable(T, fullfile(outDir, 'simulation_response.csv'));

%% Closed-loop poles and gains
eig_base = eig(A_aug - B_aug*K_base);
eig_opt  = eig(A_aug - B_aug*K_opt);
writetable(table(real(eig_base), imag(eig_base), real(eig_opt), imag(eig_opt), ...
    'VariableNames', {'re_base','im_base','re_opt','im_opt'}), ...
    fullfile(outDir, 'closed_loop_poles.csv'));

fid = fopen(fullfile(outDir, 'gains.txt'), 'w');
fprintf(fid, 'K_base =\n%s\n', mat2str(K_base, 5));
fprintf(fid, 'K_opt  =\n%s\n', mat2str(K_opt, 5));
fprintf(fid, 'Kp_base = %s\nKi_base = %s\nKd_base = %s\n', mat2str(Kp_base,5), mat2str(Ki_base,5), mat2str(Kd_base,5));
fprintf(fid, 'Kp_opt  = %s\nKi_opt  = %s\nKd_opt  = %s\n', mat2str(Kp_opt,5), mat2str(Ki_opt,5), mat2str(Kd_opt,5));
fclose(fid);

fprintf('Results exported to %s\n', outDir);

%% ---------------------------------------------------------------------
function [Kp, Ki, Kd] = pid_from_lqr(K, gamma, C, B)
    K_hat = K * pinv(gamma);
    Kd = K_hat(1,5:6) / (1 + K_hat(1,5:6)*C*B);
    Kp = K_hat(1,3:4) * (1 - Kd*C*B);
    Ki = K_hat(1,1:2) * (1 - Kd*C*B);
end
