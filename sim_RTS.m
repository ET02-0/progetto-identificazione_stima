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

% Inizializzazione all'ultimo istante (t = N)
log_EKF.x_hat_smoothed(:, N) = log_EKF.x_hat(:, N);
log_EKF.P_smoothed(:, :, N)  = log_EKF.P_corr(:, :, N);

for k = N-1:-1:1
    P_k_k = log_EKF.P_corr(:,:,k);
    
    % Se i dati sono allineati, F_k e P_{k+1|k} si trovano all'indice k+1
    P_k1_k = log_EKF.P_pred(:,:,k+1); 
    F_k = log_EKF.F_matrix(:,:,k+1);      
    
    % Calcolo robusto del guadagno C_k
    C_k = P_k_k * F_k' / P_k1_k; 
    
    % Aggiornamento Stato
    err_x = log_EKF.x_hat_smoothed(:, k+1) - log_EKF.x_hat_pred(:, k+1);
    log_EKF.x_hat_smoothed(:, k) = log_EKF.x_hat(:, k) + C_k * err_x;
    
    % Aggiornamento Covarianza e Forzatura della Simmetria
    err_P = log_EKF.P_smoothed(:,:,k+1) - P_k1_k;
    P_smooth_temp = P_k_k + C_k * err_P * C_k';
    log_EKF.P_smoothed(:,:,k) = (P_smooth_temp + P_smooth_temp') / 2; 
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
    subplot(2,2,i); hold on; grid on;
    %plot(t, 3*sigma_EKF, 'r--', 'LineWidth', 1.2);
    plot(t, err_EKF, 'r-', 'LineWidth', 1.2);
    %plot(t, 3*sigma_RTS, 'b--', 'LineWidth', 1.5);
    plot(t, err_RTS, 'b-', 'LineWidth', 1.5);
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

% =======================================================
% 1. SMOOTHER DI RAUCH-TUNG-STRIEBEL (RTS) OFFLINE + WIND
% =======================================================

log_EKF.x_hat_smoothed_W = zeros(4, N);
log_EKF.P_smoothed_W = zeros(4, 4, N);

% Inizializzazione all'ultimo istante (t = N)
log_EKF.x_hat_smoothed_W(:, N) = log_EKF.x_hat_W(:, N);
log_EKF.P_smoothed_W(:, :, N)  = log_EKF.P_corr_W(:, :, N);

% Ricorsione Backward (RTS)
for k = N-1:-1:1
    P_corr_k_W = log_EKF.P_corr_W(:,:,k);
    P_pred_k1_W = log_EKF.P_pred_W(:,:,k+1); % Deve essere P(k+1 | k)
    F_k1_W = log_EKF.F_matrix_W(:,:,k+1);      % Il Jacobiano F valutato al passo k
    
    epsilon = 1e-12; 
    P_pred_reg_W = P_pred_k1_W + eye(4) * epsilon;
    C_k_W = ( P_pred_reg_W \ (F_k1_W * P_corr_k_W) )';
    
    % FIX 2: Aggiornamento Stato
    % Assicurati che x_hat_pred sia la predizione a priori dell'EKF
    err_x_W = log_EKF.x_hat_smoothed_W(:, k+1) - log_EKF.x_hat_pred(:, k+1);
    log_EKF.x_hat_smoothed_W(:, k) = log_EKF.x_hat_W(:, k) + C_k_W * err_x_W;
    
    % FIX 3: Aggiornamento Covarianza e Forzatura della Simmetria
    err_P_W = log_EKF.P_smoothed_W(:,:,k+1) - P_pred_k1_W;
    P_smooth_temp_W = P_corr_k_W + C_k_W * err_P_W * C_k_W';
    
    % Questa riga uccide definitivamente l'origine dei numeri complessi
    log_EKF.P_smoothed_W(:,:,k) = (P_smooth_temp_W + P_smooth_temp_W') / 2; 
end

% Calcolo Errori per confronto
err_EKF_W = x_T_W - log_EKF.x_hat_W;
err_RTS_W = x_T_W - log_EKF.x_hat_smoothed_W;

% =======================================================
% 2. AREA PLOT AVANZATI (Confronto RTS vs EKF)
% =======================================================
disp('Generazione dei grafici RTS + WIND...');
titles = {'$\alpha$ (Pitch) + WIND [rad]', '$\dot{\alpha}$ (Pitch Rate) + WIND [rad/s]', '$\beta$ (Yaw) + WIND [rad]', '$\dot{\beta}$ (Yaw Rate) + WIND [rad/s]'};

%% GRAFICO 4: Confronto Traiettorie RTS vs EKF vs Truth + WIND
figure('Name', 'RTS Smoother vs EKF Forward + WIND', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T_W(i,:), 'k', 'LineWidth', 1.5);
    plot(t, log_EKF.x_hat_W(i,:), 'r--', 'LineWidth', 1.2);
    plot(t, log_EKF.x_hat_smoothed_W(i,:), 'b-', 'LineWidth', 1.2);
    title(titles{i}, 'Interpreter', 'latex');
    if i==1, legend('Truth', 'EKF Forward', 'RTS Backward'); end
end

%% GRAFICO 5: Riduzione dell'Incertezza (Covariance Shrinkage)
figure('Name', 'Riduzione Incertezza (RTS vs EKF)', 'NumberTitle', 'off');
for i=1:4
    sigma_EKF_W = squeeze(sqrt(log_EKF.P_corr_W(i,i,:)))';
    sigma_RTS_W = squeeze(sqrt(log_EKF.P_smoothed_W(i,i,:)))';
    
    subplot(2,2,i); hold on; grid on;
    subplot(2,2,i); hold on; grid on;
    %plot(t, 3*sigma_EKF, 'r--', 'LineWidth', 1.2);
    plot(t, err_EKF_W, 'r-', 'LineWidth', 1.2);
    %plot(t, 3*sigma_RTS, 'b--', 'LineWidth', 1.5);
    plot(t, err_RTS_W, 'b-', 'LineWidth', 1.5);
    title(titles{i}, 'Interpreter', 'latex');
    ylabel('Limite di confidenza a $3\sigma$ + WIND', 'Interpreter', 'latex');
    if i==1, legend('Incertezza EKF', 'Incertezza RTS Smoother'); end
end

%% GRAFICO 6: Analisi Quantitativa delle Performance (RMSE) + WIND
rmse_EKF_W = sqrt(mean(err_EKF_W.^2, 2));
rmse_RTS_W = sqrt(mean(err_RTS_W.^2, 2));

figure('Name', 'Confronto Quantitativo RMSE (RTS vs EKF) + WIND', 'NumberTitle', 'off');
bar_data = [rmse_EKF_W, rmse_RTS_W];
bar(bar_data);
set(gca, 'TickLabelInterpreter', 'latex', 'XTickLabel', {'$\alpha$', '$\dot{\alpha}$', '$\beta$', '$\dot{\beta}$'});
ylabel('Root Mean Square Error (RMSE)');
title('Miglioramento dell''Errore Quadratico Medio + WIND');
legend('EKF', 'RTS Smoother');

disp('Smoother RTS eseguito con successo! I grafici completi sono pronti.');