% Estrazione dati da Simulink e Analisi Grafica EKF vs UKF
close all; clc;
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
x_T_W = get_sim_data(out.x_T_sim_W);

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

% Dati EKF + WIND
log_EKF.x_hat_W = get_sim_data(out.x_hat_EKF_W);
log_EKF.P_corr_W = get_sim_data(out.P_EKF_W);       
log_EKF.innovation_W = get_sim_data(out.nu_EKF_W);
log_EKF.x_hat_pred_W = get_sim_data(out.x_pred_EKF_W);  % Serve per RTS
log_EKF.P_pred_W = get_sim_data(out.P_pred_EKF_W);      % Serve per RTS
log_EKF.F_matrix_W = get_sim_data(out.F_EKF_W);         % Serve per RTS

% Dati UKF + WIND
log_UKF.x_hat_W = get_sim_data(out.x_hat_UKF_W);
log_UKF.P_corr_W = get_sim_data(out.P_UKF_W);
log_UKF.innovation_W = get_sim_data(out.nu_UKF_W);

% Salvataggio per lo smoother
save('simulink_smoother_data.mat', 't', 'N', 'x_T', 'x_T_W', 'log_EKF');

% =======================================================
% 2. CALCOLO ERRORI
% =======================================================
err_EKF = x_T - log_EKF.x_hat;
err_UKF = x_T - log_UKF.x_hat;
err_EKF_W = x_T_W - log_EKF.x_hat_W;
err_UKF_W = x_T_W - log_UKF.x_hat_W;

%{
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

disp('Generazione dei grafici EKF vs UKF + WIND...');

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

titles = {'$\alpha$ (Pitch) + WIND [rad]', '$\dot{\alpha}$ (Pitch Rate) + WIND [rad/s]', '$\beta$ (Yaw) + WIND [rad]', '$\dot{\beta}$ (Yaw Rate) + WIND [rad/s]'};


%% GRAFICO 5: Traiettorie (Truth vs EKF vs UKF) + WIND
figure('Name', 'Confronto Traiettorie + WIND', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T_W(i,:), 'k', 'LineWidth', 1.5, 'DisplayName', 'True State');
    plot(t, log_EKF.x_hat_W(i,:), 'r--', 'LineWidth', 1.2, 'DisplayName', 'EKF');
    plot(t, log_UKF.x_hat_W(i,:), 'b-.', 'LineWidth', 1.2, 'DisplayName', 'UKF');
    title(titles{i}, 'Interpreter', 'latex');
    if i==1, legend('Location', 'best'); end
end

%% GRAFICO 6: Errori e Bound di Incertezza a 3-Sigma + WIND
figure('Name', 'Errori e Bounds 3-Sigma (EKF vs UKF) + WIND', 'NumberTitle', 'off');
for i=1:4
    sigma_EKF_W = squeeze(sqrt(log_EKF.P_corr_W(i,i,:)))';
    sigma_UKF_W = squeeze(sqrt(log_UKF.P_corr_W(i,i,:)))';
    
    % Subplot EKF
    subplot(4,2, 2*i-1); hold on; grid on;
    plot(t, err_EKF_W(i,:), 'r', 'LineWidth', 1);
    plot(t,  3*sigma_EKF_W, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_EKF_W, 'k--', 'LineWidth', 1);
    ylabel(titles{i}, 'Interpreter', 'latex');
    if i==1, title('Errore EKF e Bounds $\pm3\sigma$ + WIND', 'Interpreter', 'latex'); end
    
    % Subplot UKF
    subplot(4,2, 2*i); hold on; grid on;
    plot(t, err_UKF_W(i,:), 'b', 'LineWidth', 1);
    plot(t,  3*sigma_UKF_W, 'k--', 'LineWidth', 1);
    plot(t, -3*sigma_UKF_W, 'k--', 'LineWidth', 1);
    if i==1, title('Errore UKF e Bounds $\pm3\sigma$ + WIND', 'Interpreter', 'latex'); end
end

%% GRAFICO 7: Analisi delle Innovazioni (Residui) + WIND
figure('Name', 'Innovazioni del Filtro + WIND', 'NumberTitle', 'off');
meas_labels = {'Innovazione Accelerometro', 'Innovazione Magnetometro'};
for i=1:2
    subplot(2,1,i); hold on; grid on;
    plot(t, log_EKF.innovation_W(i,:), 'r', 'DisplayName', 'EKF');
    plot(t, log_UKF.innovation_W(i,:), 'b', 'DisplayName', 'UKF');
    ylabel(meas_labels{i});
    legend;
    if i==1, title('Analisi delle Innovazioni nel Tempo + WIND'); end
end

%% GRAFICO 8: Analisi Quantitativa (RMSE) + WIND
rmse_EKF_W = sqrt(mean(err_EKF_W.^2, 2));
rmse_UKF_W = sqrt(mean(err_UKF_W.^2, 2));

figure('Name', 'Confronto Quantitativo RMSE + WIND', 'NumberTitle', 'off');
bar_data = [rmse_EKF_W, rmse_UKF_W];
bar(bar_data);
set(gca, 'TickLabelInterpreter', 'latex', 'XTickLabel', {'$\alpha$', '$\dot{\alpha}$', '$\beta$', '$\dot{\beta}$'});
ylabel('Root Mean Square Error (RMSE)');
title('Confronto Errori Quadratici Medi Globali + WIND');
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
%}
% =======================================================
% 3. GENERAZIONE GRAFICI AVANZATI (SCENARIO NOMINALE)
% =======================================================
disp('Generazione dei grafici separati per lo scenario nominale...');
titles = {'$\alpha$ (Pitch) [rad]', '$\dot{\alpha}$ (Pitch Rate) [rad/s]', '$\beta$ (Yaw) [rad]', '$\dot{\beta}$ (Yaw Rate) [rad/s]'};

%% TRAIETTORIE EKF
figure('Name', 'Traiettorie EKF (Nominale)', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T(i,:), 'k', 'DisplayName', 'True State');
    plot(t, log_EKF.x_hat(i,:), 'b--', 'DisplayName', 'EKF'); % EKF IN BLU
    title(titles{i}, 'Interpreter', 'latex');
    if i==1, legend('Location', 'best'); end
end

%% TRAIETTORIE UKF
figure('Name', 'Traiettorie UKF (Nominale)', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T(i,:), 'k', 'DisplayName', 'True State');
    plot(t, log_UKF.x_hat(i,:), 'r--', 'DisplayName', 'UKF'); % UKF IN ROSSO
    title(titles{i}, 'Interpreter', 'latex');
    if i==1, legend('Location', 'best'); end
end

%% ERRORI E BOUNDS EKF
figure('Name', 'Errori e Bounds 3-Sigma EKF (Nominale)', 'NumberTitle', 'off');
for i=1:4
    sigma_EKF = squeeze(sqrt(log_EKF.P_corr(i,i,:)))';
    subplot(2,2,i); hold on; grid on;
    plot(t, err_EKF(i,:), 'b', 'DisplayName', 'Errore EKF'); % BLU
    plot(t,  3*sigma_EKF, 'k--', 'DisplayName', '$\pm 3\sigma$');
    plot(t, -3*sigma_EKF, 'k--', 'HandleVisibility', 'off');
    title(['Errore su ', titles{i}], 'Interpreter', 'latex');
    if i==1, legend('Interpreter', 'latex'); end
end

%% ERRORI E BOUNDS UKF
figure('Name', 'Errori e Bounds 3-Sigma UKF (Nominale)', 'NumberTitle', 'off');
for i=1:4
    sigma_UKF = squeeze(sqrt(log_UKF.P_corr(i,i,:)))';
    subplot(2,2,i); hold on; grid on;
    plot(t, err_UKF(i,:), 'r', 'DisplayName', 'Errore UKF'); % ROSSO
    plot(t,  3*sigma_UKF, 'k--', 'DisplayName', '$\pm 3\sigma$');
    plot(t, -3*sigma_UKF, 'k--', 'HandleVisibility', 'off');
    title(['Errore su ', titles{i}], 'Interpreter', 'latex');
    if i==1, legend('Interpreter', 'latex'); end
end

%% INNOVAZIONI SEPARATE
meas_labels = {'Innovazione Accelerometro', 'Innovazione Magnetometro'};
figure('Name', 'Innovazioni EKF (Nominale)', 'NumberTitle', 'off');
for i=1:2
    subplot(2,1,i); hold on; grid on;
    plot(t, log_EKF.innovation(i,:), 'b'); % BLU
    ylabel(meas_labels{i});
    if i==1, title('Innovazioni EKF'); end
end

figure('Name', 'Innovazioni UKF (Nominale)', 'NumberTitle', 'off');
for i=1:2
    subplot(2,1,i); hold on; grid on;
    plot(t, log_UKF.innovation(i,:), 'r'); % ROSSO
    ylabel(meas_labels{i});
    if i==1, title('Innovazioni UKF'); end
end


% =======================================================
% 4. GENERAZIONE GRAFICI AVANZATI (SCENARIO VENTO)
% =======================================================
disp('Generazione dei grafici separati per lo scenario con VENTO...');
titles_W = {'$\alpha$ (Pitch) + WIND [rad]', '$\dot{\alpha}$ (Pitch Rate) + WIND [rad/s]', '$\beta$ (Yaw) + WIND [rad]', '$\dot{\beta}$ (Yaw Rate) + WIND [rad/s]'};

%% TRAIETTORIE EKF + WIND
figure('Name', 'Traiettorie EKF (+ WIND)', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T_W(i,:), 'k', 'DisplayName', 'True State');
    plot(t, log_EKF.x_hat_W(i,:), 'b--', 'DisplayName', 'EKF');
    title(titles_W{i}, 'Interpreter', 'latex');
    if i==1, legend('Location', 'best'); end
end

%% TRAIETTORIE UKF + WIND
figure('Name', 'Traiettorie UKF (+ WIND)', 'NumberTitle', 'off');
for i=1:4
    subplot(2,2,i); hold on; grid on;
    plot(t, x_T_W(i,:), 'k', 'DisplayName', 'True State');
    plot(t, log_UKF.x_hat_W(i,:), 'r--', 'DisplayName', 'UKF');
    title(titles_W{i}, 'Interpreter', 'latex');
    if i==1, legend('Location', 'best'); end
end

%% ERRORI E BOUNDS EKF + WIND
figure('Name', 'Errori e Bounds 3-Sigma EKF (+ WIND)', 'NumberTitle', 'off');
for i=1:4
    sigma_EKF_W = squeeze(sqrt(log_EKF.P_corr_W(i,i,:)))';
    subplot(2,2,i); hold on; grid on;
    plot(t, err_EKF_W(i,:), 'b', 'DisplayName', 'Errore EKF');
    plot(t,  3*sigma_EKF_W, 'k--', 'DisplayName', '$\pm 3\sigma$');
    plot(t, -3*sigma_EKF_W, 'k--', 'HandleVisibility', 'off');
    title(['Errore su ', titles_W{i}], 'Interpreter', 'latex');
    if i==1, legend('Interpreter', 'latex'); end
end

%% ERRORI E BOUNDS UKF + WIND
figure('Name', 'Errori e Bounds 3-Sigma UKF (+ WIND)', 'NumberTitle', 'off');
for i=1:4
    sigma_UKF_W = squeeze(sqrt(log_UKF.P_corr_W(i,i,:)))';
    subplot(2,2,i); hold on; grid on;
    plot(t, err_UKF_W(i,:), 'r', 'DisplayName', 'Errore UKF');
    plot(t,  3*sigma_UKF_W, 'k--', 'DisplayName', '$\pm 3\sigma$');
    plot(t, -3*sigma_UKF_W, 'k--', 'HandleVisibility', 'off');
    title(['Errore su ', titles_W{i}], 'Interpreter', 'latex');
    if i==1, legend('Interpreter', 'latex'); end
end

%% INNOVAZIONI SEPARATE + WIND
figure('Name', 'Innovazioni EKF (+ WIND)', 'NumberTitle', 'off');
for i=1:2
    subplot(2,1,i); hold on; grid on;
    plot(t, log_EKF.innovation_W(i,:), 'b');
    ylabel(meas_labels{i});
    if i==1, title('Innovazioni EKF (+ WIND)'); end
end

figure('Name', 'Innovazioni UKF (+ WIND)', 'NumberTitle', 'off');
for i=1:2
    subplot(2,1,i); hold on; grid on;
    plot(t, log_UKF.innovation_W(i,:), 'r');
    ylabel(meas_labels{i});
    if i==1, title('Innovazioni UKF (+ WIND)'); end
end


% =======================================================
% 5. GRAFICI DI CONFRONTO (RMSE)
% =======================================================
disp('Generazione dei grafici quantitativi RMSE (Confronto EKF vs UKF)...');

rmse_EKF = sqrt(mean(err_EKF.^2, 2));
rmse_UKF = sqrt(mean(err_UKF.^2, 2));
figure('Name', 'Confronto Quantitativo RMSE (Nominale)', 'NumberTitle', 'off');
b = bar([rmse_EKF, rmse_UKF]);
b(1).FaceColor = 'b'; % EKF Blu
b(2).FaceColor = 'r'; % UKF Rosso
set(gca, 'TickLabelInterpreter', 'latex', 'XTickLabel', {'$\alpha$', '$\dot{\alpha}$', '$\beta$', '$\dot{\beta}$'});
ylabel('Root Mean Square Error (RMSE)');
title('Confronto RMSE (Scenario Nominale)');
legend('EKF', 'UKF');

rmse_EKF_W = sqrt(mean(err_EKF_W.^2, 2));
rmse_UKF_W = sqrt(mean(err_UKF_W.^2, 2));
figure('Name', 'Confronto Quantitativo RMSE (+ WIND)', 'NumberTitle', 'off');
b_w = bar([rmse_EKF_W, rmse_UKF_W]);
b_w(1).FaceColor = 'b'; % EKF Blu
b_w(2).FaceColor = 'r'; % UKF Rosso
set(gca, 'TickLabelInterpreter', 'latex', 'XTickLabel', {'$\alpha$', '$\dot{\alpha}$', '$\beta$', '$\dot{\beta}$'});
ylabel('Root Mean Square Error (RMSE)');
title('Confronto RMSE (Scenario con Vento)');
legend('EKF', 'UKF');

disp('Analisi completata con successo! Dati salvati per lo Smoother RTS.');

% Aggiornamento colori sfondi figure
h = findall(0, 'Type', 'figure');
for i = 1:length(h)
    set(h(i), 'Color', 'w');
    ax = findall(h(i), 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');
end

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