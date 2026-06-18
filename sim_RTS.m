% Esecuzione dello Smoother RTS offline sui dati estratti da Simulink
clear all; close all; clc;

disp('Caricamento dei dati da simulink_smoother_data.mat...');
load('simulink_smoother_data.mat'); % Carica t, N, x_T, log_EKF

% =======================================================
% 1. SMOOTHER DI RAUCH-TUNG-STRIEBEL (RTS) OFFLINE
% =======================================================
disp('Esecuzione dello Smoother RTS...');

log_EKF.x_hat_smoothed = zeros(4, N);
log_EKF.P_smoothed = zeros(4, 4, N);

% Inizializzazione all'ultimo istante (t = N)
log_EKF.x_hat_smoothed(:, N) = log_EKF.x_hat(:, N);
log_EKF.P_smoothed(:, :, N)  = log_EKF.P_corr(:, :, N);

% Ricorsione Backward
for k = N-1:-1:1
    P_corr_k = log_EKF.P_corr(:,:,k);
    P_pred_k1 = log_EKF.P_pred(:,:,k+1);
    F_k1 = log_EKF.F_matrix(:,:,k+1); 
    
    % Guadagno dello smoother (uso di mrdivide per stabilità)
    C_k = P_corr_k * F_k1' / P_pred_k1;
    
    % Update di Stato e Covarianza
    err_x = log_EKF.x_hat_smoothed(:, k+1) - log_EKF.x_hat_pred(:, k+1);
    log_EKF.x_hat_smoothed(:, k) = log_EKF.x_hat(:, k) + C_k * err_x;
    
    err_P = log_EKF.P_smoothed(:,:,k+1) - P_pred_k1;
    log_EKF.P_smoothed(:,:,k) = P_corr_k + C_k * err_P * C_k';
end

% Calcolo Errori per confronto
err_EKF = x_T - log_EKF.x_hat;
err_RTS = x_T - log_EKF.x_hat_smoothed;

% =======================================================
% 2. AREA PLOT AVANZATI (Confronto RTS vs EKF)
% =======================================================
disp('Generazione dei grafici RTS...');
titles = {'$\alpha$ (Pitch) [rad]', '$\dot{\alpha}$ (Pitch Rate) [rad/s]', '$\beta$ (Yaw) [rad]', '$\dot{\beta}$ (Yaw Rate) [rad/s]'};

%% GRAFICO 1: Confronto Traiettorie RTS vs EKF vs Truth
figure('Name', 'RTS Smoother vs EKF Forward', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T(i,:), 'k', 'LineWidth', 1.5);
    plot(t, log_EKF.x_hat(i,:), 'r--', 'LineWidth', 1.2);
    plot(t, log_EKF.x_hat_smoothed(i,:), 'b-', 'LineWidth', 1.2);
    title(titles{i}, 'Interpreter', 'latex');
    if i==1, legend('Truth', 'EKF Forward', 'RTS Backward'); end
end

%% GRAFICO 2: Riduzione dell'Incertezza (Covariance Shrinkage)
figure('Name', 'Riduzione Incertezza (RTS vs EKF)', 'NumberTitle', 'off');
for i=1:4
    sigma_EKF = squeeze(sqrt(log_EKF.P_corr(i,i,:)))';
    sigma_RTS = squeeze(sqrt(log_EKF.P_smoothed(i,i,:)))';
    
    subplot(2,2,i); hold on; grid on;
    plot(t, 3*sigma_EKF, 'r--', 'LineWidth', 1.2);
    plot(t, 3*sigma_RTS, 'b-', 'LineWidth', 1.5);
    title(titles{i}, 'Interpreter', 'latex');
    ylabel('Limite di confidenza a $3\sigma$', 'Interpreter', 'latex');
    if i==1, legend('Incertezza EKF', 'Incertezza RTS Smoother'); end
end

%% GRAFICO 3: Analisi Quantitativa delle Performance (RMSE)
rmse_EKF = sqrt(mean(err_EKF.^2, 2));
rmse_RTS = sqrt(mean(err_RTS.^2, 2));

figure('Name', 'Confronto Quantitativo RMSE (RTS vs EKF)', 'NumberTitle', 'off');
bar_data = [rmse_EKF, rmse_RTS];
bar(bar_data);
set(gca, 'TickLabelInterpreter', 'latex', 'XTickLabel', {'$\alpha$', '$\dot{\alpha}$', '$\beta$', '$\dot{\beta}$'});
ylabel('Root Mean Square Error (RMSE)');
title('Miglioramento dell''Errore Quadratico Medio');
legend('EKF', 'RTS Smoother');

disp('Smoother RTS eseguito con successo! I grafici completi sono pronti.');