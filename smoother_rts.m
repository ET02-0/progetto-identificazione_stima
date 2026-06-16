%% SCRIPT 3: RTS_Smoother.m
clear all; close all; clc;
load('EKF_smoother_data.mat'); % Carica log_EKF e log_vars

N = length(log_vars.t);
log_EKF.x_hat_smoothed = zeros(4, N);
log_EKF.P_smoothed = zeros(4, 4, N);

% Inizializzazione: L'ultimo istante coincide con l'ultima correzione EKF
log_EKF.x_hat_smoothed(:, N) = log_EKF.x_hat(:, N);
log_EKF.P_smoothed(:, :, N)  = log_EKF.P_corr(:, :, N);

% Ricorsione all'indietro
for k = N-1:-1:1
    P_corr_k = log_EKF.P_corr(:,:,k);
    P_pred_k1 = log_EKF.P_pred(:,:,k+1);
    F_k1 = log_EKF.F_matrix(:,:,k+1);
    
    % Guadagno dello smoother: C_k = P_{k|k} * F_{k+1}^T * P_{k+1|k}^{-1}
    C_k = P_corr_k * F_k1' / P_pred_k1;
    
    % Update dello stato e della covarianza
    err_x = log_EKF.x_hat_smoothed(:, k+1) - log_EKF.x_hat_pred(:, k+1);
    log_EKF.x_hat_smoothed(:, k) = log_EKF.x_hat(:, k) + C_k * err_x;
    
    err_P = log_EKF.P_smoothed(:,:,k+1) - P_pred_k1;
    log_EKF.P_smoothed(:,:,k) = P_corr_k + C_k * err_P * C_k';
end

%% Plot Confronto RTS vs EKF
figure;
titles = {'\alpha (Pitch)', '\dot{\alpha}', '\beta (Yaw)', '\dot{\beta}'};
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(log_vars.t, log_vars.x_T(i,:), 'k', 'LineWidth', 1.5);
    plot(log_vars.t, log_EKF.x_hat(i,:), 'r--', 'LineWidth', 1.2);
    plot(log_vars.t, log_EKF.x_hat_smoothed(i,:), 'b-', 'LineWidth', 1.2);
    title(titles{i});
    if i==1, legend('Truth', 'EKF', 'RTS Smoother'); end
end