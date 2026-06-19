% Estrazione dati da Simulink e Analisi Grafica EKF vs UKF
close all; clc;
% --- INSERISCI IL CODICE QUI ---
set(0, 'DefaultFigureColor', 'w');
set(0, 'DefaultAxesColor', 'w');
set(0, 'DefaultAxesXColor', 'k');
set(0, 'DefaultAxesYColor', 'k');
set(0, 'DefaultTextColor', 'k');
set(0, 'DefaultLegendTextColor', 'k');
set(0, 'DefaultLineLineWidth', 1.5);

set(0, 'DefaultLegendColor', 'w');      % Sfondo bianco della legenda
set(0, 'DefaultLegendTextColor', 'k');  % Testo nero della legenda
set(0, 'DefaultLegendEdgeColor', 'k');  % Bordo nero della legenda
% -------------------------------
disp('Estrazione dei dati da Simulink in corso...');

% =======================================================
% 1. ESTRAZIONE E RIALLINEAMENTO DATI
% =======================================================
t = out.tout'; % Vettore del tempo (1 x N)
N = length(t);

% Stato vero
x_T = get_sim_data(out.x_T_sim);

% Dati EKF
log_EKF.x_hat = get_sim_data(out.x_hat_EKF);
log_EKF.P_corr = get_sim_data(out.P_EKF);       
log_EKF.innovation = get_sim_data(out.nu_EKF);
log_EKF.x_hat_pred = get_sim_data(out.x_pred_EKF);  % Serve per RTS
log_EKF.P_pred = get_sim_data(out.P_pred_EKF);      % Serve per RTS
log_EKF.F_matrix = get_sim_data(out.F_EKF);         % Serve per RTS

% Dati UKF
log_UKF.x_hat = get_sim_data(out.x_hat_UKF);
log_UKF.P_corr = get_sim_data(out.P_UKF);
log_UKF.innovation = get_sim_data(out.nu_UKF);

% Salvataggio per lo smoother
save('simulink_smoother_data.mat', 't', 'N', 'x_T', 'log_EKF');

% =======================================================
% 2. CALCOLO ERRORI
% =======================================================
err_EKF = x_T - log_EKF.x_hat;
err_UKF = x_T - log_UKF.x_hat;

% =======================================================
% 3. GENERAZIONE GRAFICI AVANZATI
% =======================================================
disp('Generazione dei grafici EKF vs UKF...');
titles = {'$\alpha$ (Pitch) [rad]', '$\dot{\alpha}$ (Pitch Rate) [rad/s]', '$\beta$ (Yaw) [rad]', '$\dot{\beta}$ (Yaw Rate) [rad/s]'};

%% GRAFICO 1: Traiettorie (Truth vs EKF vs UKF)
figure('Name', 'Confronto Traiettorie', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T(i,:), 'k', 'LineWidth', 1.5, 'DisplayName', 'True State');
    plot(t, log_EKF.x_hat(i,:), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF');
    plot(t, log_UKF.x_hat(i,:), 'b-.', 'LineWidth', 1.2, 'DisplayName', 'UKF');
    title(titles{i}, 'Interpreter', 'latex');
    if i==1, legend('Location', 'best'); end
end

%% GRAFICO 2: Errori e Bound di Incertezza a 3-Sigma
figure('Name', 'Errori e Bounds 3-Sigma (EKF vs UKF)', 'NumberTitle', 'off');
for i=1:4
    sigma_EKF = squeeze(sqrt(log_EKF.P_corr(i,i,:)))';
    sigma_UKF = squeeze(sqrt(log_UKF.P_corr(i,i,:)))';
    
    % Subplot EKF
    subplot(4,2, 2*i-1); hold on; grid on;
    plot(t, err_EKF(i,:), 'r', 'LineWidth', 1);
    plot(t,  3*sigma_EKF, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_EKF, 'k--', 'LineWidth', 1);
    ylabel(titles{i}, 'Interpreter', 'latex');
    if i==1, title('Errore EKF e Bounds $\pm3\sigma$', 'Interpreter', 'latex'); end
    
    % Subplot UKF
    subplot(4,2, 2*i); hold on; grid on;
    plot(t, err_UKF(i,:), 'b', 'LineWidth', 1);
    plot(t,  3*sigma_UKF, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_UKF, 'k--', 'LineWidth', 1);
    if i==1, title('Errore UKF e Bounds $\pm3\sigma$', 'Interpreter', 'latex'); end
end

%% GRAFICO 3: Analisi delle Innovazioni (Residui)
figure('Name', 'Innovazioni del Filtro', 'NumberTitle', 'off');
meas_labels = {'Innovazione Accelerometro', 'Innovazione Magnetometro'};
for i=1:2
    subplot(2,1,i); hold on; grid on;
    plot(t, log_EKF.innovation(i,:), 'r', 'DisplayName', 'EKF');
    plot(t, log_UKF.innovation(i,:), 'b', 'DisplayName', 'UKF');
    ylabel(meas_labels{i});
    legend;
    if i==1, title('Analisi delle Innovazioni nel Tempo'); end
end

%% GRAFICO 4: Analisi Quantitativa (RMSE)
rmse_EKF = sqrt(mean(err_EKF.^2, 2));
rmse_UKF = sqrt(mean(err_UKF.^2, 2));

figure('Name', 'Confronto Quantitativo RMSE', 'NumberTitle', 'off');
bar_data = [rmse_EKF, rmse_UKF];
bar(bar_data);
set(gca, 'TickLabelInterpreter', 'latex', 'XTickLabel', {'$\alpha$', '$\dot{\alpha}$', '$\beta$', '$\dot{\beta}$'});
ylabel('Root Mean Square Error (RMSE)');
title('Confronto Errori Quadratici Medi Globali');
legend('EKF', 'UKF');

disp('Analisi completata con successo! Dati salvati per lo Smoother RTS.');


% Se hai già delle figure aperte, aggiornale tutte in un colpo solo:
h = findall(0, 'Type', 'figure');
for i = 1:length(h)
    set(h(i), 'Color', 'w');
    ax = findall(h(i), 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');
end

% =======================================================
% FUNZIONE HELPER LOCALE
% =======================================================
function data = get_sim_data(sim_var)
    if isa(sim_var, 'timeseries')
        data = sim_var.Data;
    else
        data = sim_var;
    end
    data = squeeze(data);
    if ismatrix(data) && size(data, 1) > size(data, 2)
        data = data';
    end
end