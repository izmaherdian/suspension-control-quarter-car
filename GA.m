clear; 
clc; 
close all;

% Parameter Depan (BMW)
kk = 340;       % kN/m
kr = 30;        % kN/m
br = 1450;      % Ns/m
mc = 408;       % kg
mus = 48.3;     % kg

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

% Batas untuk variabel optimasi
lb = [0.1 * ones(1, 4), 0.01 * ones(1, 2)];     % Batas bawah yang disesuaikan untuk Q dan R
ub = [100 * ones(1, 4), 10 * ones(1, 2)];       % Batas atas yang disesuaikan untuk Q dan R

% Inisialisasi variabel untuk menyimpan riwayat biaya
global cost_history;
cost_history = [];

% Jalankan algoritma genetika untuk mencari nilai Q dan R optimal
options = optimoptions('ga', 'MaxGenerations', 1000, 'PopulationSize', 20, ...
                       'Display', 'iter', 'OutputFcn', @ga_output_function);
optimal_params = ga(@(params) objective(params, A, B), 6, [], [], [], [], lb, ub, [], options);

% Bentuk matriks Q dan R optimal
Q_opt = diag(optimal_params(1:4)); % Disesuaikan untuk 4 status
R_opt = diag(optimal_params(5:6)); % Disesuaikan untuk 2 input

% Hitung LQR dengan Q dan R optimal
K_opt = lqr(A, B, Q_opt, R_opt);

% Plot biaya vs iterasi
figure;
plot(cost_history, 'LineWidth', 1.5);
title('Biaya vs Iterasi');
xlabel('Iterasi');
ylabel('Biaya');
grid on;

% Validasi: Cek Stabilitas
A_cl = A - B * K_opt; % Matriks sistem tertutup
eig_values = eig(A_cl);
disp('Nilai eigen dari sistem tertutup:');
disp(eig_values);

if all(real(eig_values) < 0)
    disp('Sistem stabil.');
else
    disp('Sistem tidak stabil.');
end

% Validasi: Simulasi Respons Closed-Loop
x0 = [1; 1; 1; 1]; % Kondisi awal
[t, x] = ode45(@(t, x) (A_cl * x), [0, 10], x0);

% Validasi: Analisis Upaya Kontrol
u = -K_opt * x'; % Hukum kontrol: u = -Kx
figure;
plot(t, u', 'LineWidth', 1.5);
title('Input Kontrol');
xlabel('Waktu (s)');
ylabel('Upaya Kontrol');
legend('Input 1', 'Input 2');
grid on;

% Validasi: Perbandingan dengan Baseline
Q_baseline = eye(4); % Matriks identitas untuk Q
R_baseline = eye(2); % Matriks identitas untuk R
K_baseline = lqr(A, B, Q_baseline, R_baseline);

% Matriks sistem tertutup untuk baseline
A_cl_baseline = A - B * K_baseline;

% Simulasi respons baseline
[t_base, x_base] = ode45(@(t, x) (A_cl_baseline * x), [0, 10], x0);

% Plot respons yang dioptimalkan vs baseline
figure;
plot(t, x, 'LineWidth', 1.5);
hold on;
plot(t_base, x_base, '--', 'LineWidth', 1.5); % Garis putus-putus untuk baseline
title('Optimalkan vs Biasa');
xlabel('Waktu (s)');
ylabel('Status');
legend('Status Optimalkan 1', 'Status Optimalkan 2', 'Status Optimalkan 3', 'Status Optimalkan 4', ...
       'Status Baseline 1', 'Status Baseline 2', 'Status Baseline 3', 'Status Baseline 4');
grid on;

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
    Q = diag(params(1:4));  % Disesuaikan untuk 4 status
    R = diag(params(5:6));  % Disesuaikan untuk 2 input

    % Hitung LQR
    K = lqr(A, B, Q, R);

    % Matriks sistem tertutup
    Acl = A - B * K;

    % Set kondisi awal
    x0 = [1; 1; 1; 1];  % Status awal

    % Simulasikan respons sistem tertutup
    [t, x] = ode45(@(t, x) Acl * x, [0, 10], x0);

    % Sinyal referensi adalah nol sepanjang simulasi
    r = zeros(size(x));

    % Hitung error status sepanjang periode waktu
    state_error = x - r; % Error tracking untuk semua status (pada dasarnya hanya x)

    % Hitung overshoot, waktu settling, dan error steady-state untuk setiap status
    overshoot = zeros(4, 1);
    settling_time = zeros(4, 1);
    steady_state_error = zeros(4, 1);

    for i = 1:4
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