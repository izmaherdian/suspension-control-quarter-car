%% main_ga_pid_lqr.m
% GA-tuned PID-LQR controller for a quarter-car active suspension.
%
%   1. Quarter-car state-space model (BMW 530i front axle)
%   2. Integral-backstepping augmentation  (A_aug, B_aug, Gamma)
%   3. Genetic Algorithm searches diag(Q) (6) and diag(R) (2)
%   4. LQR gains -> PID gains  (Kp, Ki, Kd) for baseline and GA-tuned designs
%   5. Simulates both controllers in quarter_car_pid_lqr_sim.slx (30 s)
%
% Requires: Control System Toolbox, Simulink, Global Optimization Toolbox (ga)
% Run from the src/ folder:  >> main_ga_pid_lqr
% (No Global Optimization Toolbox? use export_results.m, which reuses the
%  Q_opt / R_opt reported in the paper.)

clear
clc
close all

%%
% Parameter Front Side (BMW)
kk = 340;       % kN/m
kr = 30;        % kN/m
br = 1450;      % Ns/m
mc = 408;       % kg
mus = 48.3;     % kg

%%
% State-space
% Matriks state-space
A = [0 1 0 0; 
    (-kk-kr)/mus -br/mus kr/mus br/mus; 
    0 0 0 1; 
    kr/mc br/mc -kr/mc -br/mc];

B1 = [0; 
    -1/mus; 
    0; 
    1/mc];

B2 = [0; 
    kk/mus; 
    0; 
    0];

B = [B1 B2];  

C = [0 0 1 0;
     0 0 0 1]; 

D = [0 0
     0 0]; 

%%
% Augmentasi Backstepping Integral
A_aug = [A B; 
        zeros(2, 4) zeros(2,2)];  
B_aug = [zeros(4, 2); 
         eye(2,2)];

gamma = [C zeros(2, 2); 
         C*A C*B; 
         C*A*A C*A*B];

%%
% Genetic Algorithm

% Batas untuk variabel optimasi
lb = [0.1 * ones(1, 6), 0.01 * ones(1, 2)];     % Batas bawah yang disesuaikan untuk Q dan R
ub = [100 * ones(1, 6), 10 * ones(1, 2)];       % Batas atas yang disesuaikan untuk Q dan R

% Inisialisasi variabel untuk menyimpan riwayat biaya
global cost_history;
cost_history = [];

% Jalankan algoritma genetika untuk mencari nilai Q dan R optimal
options = optimoptions('ga', 'MaxGenerations', 1000, 'PopulationSize', 20, ...
                       'Display', 'iter', 'OutputFcn', @ga_output_function);
optimal_params = ga(@(params) objective(params, A_aug, B_aug), 8, [], [], [], [], lb, ub, [], options);

%%
% Desain Kontroler LQR Biasa
Q = diag([1, 1, 1, 1, 1, 1]);
R = diag([1, 1]);
[K_base, ~, ~] = lqr(A_aug, B_aug, Q, R);

K_hat_base = K_base*pinv(gamma); 

% Parameter PID
K3_hat_base = K_hat_base(1, 5:6);    
Kd_base = K3_hat_base/(1+K3_hat_base*C*B);  
K2_hat_base = K_hat_base(1, 3:4);      
Kp_base = K2_hat_base*(1-Kd_base*C*B);    
K1_hat_base = K_hat_base(1, 1:2);    
Ki_base = K1_hat_base*(1-Kd_base*C*B);

%%
% Desain Kontroler LQR Genertic Algorithm
Q_opt = diag(optimal_params(1:6));
R_opt = diag(optimal_params(7:8));
[K_opt, ~, ~] = lqr(A_aug, B_aug, Q_opt, R_opt);

K_hat_opt = K_opt*pinv(gamma); 

% Parameter PID
K3_hat_opt = K_hat_opt(1, 5:6);    
Kd_opt = K3_hat_opt/(1+K3_hat_opt*C*B);  
K2_hat_opt = K_hat_opt(1, 3:4);      
Kp_opt = K2_hat_opt*(1-Kd_opt*C*B);    
K1_hat_opt = K_hat_opt(1, 1:2);    
Ki_opt = K1_hat_opt*(1-Kd_opt*C*B);

%%
% Simulasi Menggunakan Simulink
stime = 30;
simOut = sim("quarter_car_pid_lqr_sim.slx", stime);

% Ekstrak Data Waktu
t_out = simOut.t_out;

% Ekstrak Data Referensi
ref = simOut.ref;

% Ekstrak Data State
x_base_out = simOut.x_base_out;
x_opt_out = simOut.x_opt_out;

% Ekstrak Data Output
y_base_out = simOut.y_base_out;
y_opt_out = simOut.y_opt_out;

% Ekstrak Data Input
u_base_out = simOut.u_base_out;
u_opt_out = simOut.u_opt_out;

%%
% Plot Kontrol Base dan Opt

% Plot State (4 states)
figure;
subplot(3, 1, 1);
plot(t_out, x_base_out(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_out, x_opt_out(:,1), 'r--', 'LineWidth', 1.5); % State 1
plot(t_out, x_base_out(:,2), 'g', 'LineWidth', 1.5); 
plot(t_out, x_opt_out(:,2), 'm--', 'LineWidth', 1.5); % State 2
plot(t_out, x_base_out(:,3), 'c', 'LineWidth', 1.5); 
plot(t_out, x_opt_out(:,3), 'y--', 'LineWidth', 1.5); % State 3
plot(t_out, x_base_out(:,4), 'k', 'LineWidth', 1.5); 
plot(t_out, x_opt_out(:,4), 'b--', 'LineWidth', 1.5); % State 4
title('State vs Time');
xlabel('Time (s)');
ylabel('State');
legend('Base State 1', 'Optimal State 1', 'Base State 2', 'Optimal State 2', ...
       'Base State 3', 'Optimal State 3', 'Base State 4', 'Optimal State 4');
grid on;

% Plot Output (2 outputs)
subplot(3, 1, 2);
plot(t_out, y_base_out(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_out, y_opt_out(:,1), 'r--', 'LineWidth', 1.5); % Output 1
plot(t_out, y_base_out(:,2), 'g', 'LineWidth', 1.5); 
plot(t_out, y_opt_out(:,2), 'm--', 'LineWidth', 1.5); % Output 2
plot(t_out, ref(:,1), 'black', 'LineWidth', 0.2); % Referensi
title('Output vs Time');
xlabel('Time (s)');
ylabel('Output');
legend('Base Output 1', 'Optimal Output 1', 'Base Output 2', 'Optimal Output 2', 'Referensi');
grid on;

% Plot Input (2 inputs)
subplot(3, 1, 3);
plot(t_out, u_base_out(:,1), 'b', 'LineWidth', 1.5); hold on;
plot(t_out, u_opt_out(:,1), 'r--', 'LineWidth', 1.5); % Input 1
plot(t_out, u_base_out(:,2), 'g', 'LineWidth', 1.5); 
plot(t_out, u_opt_out(:,2), 'm--', 'LineWidth', 1.5); % Input 2
title('Input vs Time');
xlabel('Time (s)');
ylabel('Input');
legend('Base Input 1', 'Optimal Input 1', 'Base Input 2', 'Optimal Input 2');
grid on;

% Plot biaya vs iterasi
figure;
plot(cost_history, 'LineWidth', 1.5);
title('Biaya vs Iterasi');
xlabel('Iterasi');
ylabel('Biaya');
grid on;

%%
% Fungsi OutputFcn untuk merekam nilai biaya di setiap generasi
function [state, options, optchanged] = ga_output_function(options, state, flag)
    global cost_history;
    if strcmp(flag, 'iter')
        % Simpan biaya terbaik (fitness) pada generasi saat ini
        cost_history = [cost_history; min(state.Score)];
    end
    optchanged = false;
end

% Fungsi objektif untuk optimasi
function cost = objective(params, A, B)
    % Bentuk Q dan R dari parameter optimasi
    Q = diag(params(1:6));  % Disesuaikan untuk 4 status
    R = diag(params(7:8));  % Disesuaikan untuk 2 input

    % Hitung LQR
    K = lqr(A, B, Q, R);

    % Matriks sistem tertutup
    Acl = A - B * K;

    % Set kondisi awal
    x0 = [1; 1; 1; 1; 1; 1];  % Status awal

    % Simulasikan respons sistem tertutup
    [t, x] = ode45(@(t, x) Acl * x, [0, 10], x0);

    % Sinyal referensi adalah nol sepanjang simulasi
    r = zeros(size(x));

    % Hitung error status sepanjang periode waktu
    state_error = x - r; % Error tracking untuk semua status (pada dasarnya hanya x)

    % Hitung overshoot, waktu settling, dan error steady-state untuk setiap status
    overshoot = zeros(6, 1);
    settling_time = zeros(6, 1);
    steady_state_error = zeros(6, 1);

    for i = 1:6
        % Ambil trajektori status untuk status ini
        state_trajectory = x(:, i);

        % Hitung overshoot sebagai persentase deviasi puncak dari nilai steady-state
        steady_value = state_trajectory(end);  % Nilai steady-state terakhir
        peak_value = max(state_trajectory);   % Nilai puncak dari respons
        overshoot(i) = (peak_value - steady_value) / steady_value * 100;

        % Hitung waktu settling (waktu untuk tetap dalam 2% dari nilai steady-state)
        tolerance = 0.02 * abs(steady_value);
        idx_settling = find(abs(state_trajectory - steady_value) <= tolerance, 1, 'last');
        if ~isempty(idx_settling)
            settling_time(i) = t(idx_settling);
        else
            settling_time(i) = NaN; % Jika waktu settling tidak tercapai dalam rentang waktu
        end

        % Hitung error steady-state (nilai error terakhir)
        steady_state_error(i) = abs(state_error(end, i));
    end

    % Integrasikan error kuadrat sepanjang waktu untuk perhitungan biaya
    state_error_squared = sum(state_error.^2, 2);   % Error kuadrat untuk setiap status
    cost = trapz(t, state_error_squared);           % Gunakan integrasi trapezoidal untuk luas di bawah kurva

    % Tambahkan overshoot, waktu settling, dan error steady-state ke biaya
    % (Penalti untuk overshoot tinggi, waktu settling lama, dan error steady-state tinggi)
    cost = cost + sum(overshoot) + sum(settling_time) + 100*sum(steady_state_error);
end