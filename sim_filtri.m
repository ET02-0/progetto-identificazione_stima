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
log_EKF.x_hat_pred = get_sim_data(out.x_pred_EKF);  
log_EKF.P_pred = get_sim_data(out.P_pred_EKF);      
log_EKF.F_matrix = get_sim_data(out.F_EKF);         

% Dati UKF
log_UKF.x_hat = get_sim_data(out.x_hat_UKF);
log_UKF.P_corr = get_sim_data(out.P_UKF);
log_UKF.innovation = get_sim_data(out.nu_UKF);

% Dati EKF + WIND
log_EKF.x_hat_W = get_sim_data(out.x_hat_EKF_W);
log_EKF.P_corr_W = get_sim_data(out.P_EKF_W);       
log_EKF.innovation_W = get_sim_data(out.nu_EKF_W);
log_EKF.x_hat_pred_W = get_sim_data(out.x_pred_EKF_W);  
log_EKF.P_pred_W = get_sim_data(out.P_pred_EKF_W);      
log_EKF.F_matrix_W = get_sim_data(out.F_EKF_W);         

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

% =======================================================
% 6. TEST DI BIANCHEZZA (ANDERSON)
% =======================================================
disp('Esecuzione dei Test di Bianchezza di Anderson sulle innovazioni...');

plot_anderson_whiteness(log_EKF.innovation, 'Test Bianchezza EKF (Nominale)', N);
plot_anderson_whiteness(log_UKF.innovation, 'Test Bianchezza UKF (Nominale)', N);
plot_anderson_whiteness(log_EKF.innovation_W, 'Test Bianchezza EKF (+ WIND)', N);
plot_anderson_whiteness(log_UKF.innovation_W, 'Test Bianchezza UKF (+ WIND)', N);

disp('Analisi completata con successo! Dati salvati per lo Smoother RTS.');

% Aggiornamento colori sfondi figure (Applicato anche ai nuovi grafici)
h = findall(0, 'Type', 'figure');
for i = 1:length(h)
    set(h(i), 'Color', 'w');
    ax = findall(h(i), 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');
end

% =======================================================
% FUNZIONI HELPER LOCALI
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

function plot_anderson_whiteness(innov, title_str, N_samples)
    % Calcola un numero di lag sensato per la visualizzazione (es. max 50)
    max_lag = min(500, round(N_samples/10)); 
    conf_bound = 1.96 / sqrt(N_samples); % Limite di confidenza al 95%
    
    figure('Name', title_str, 'NumberTitle', 'off');
    meas_labels = {'Innovazione Accelerometro', 'Innovazione Magnetometro'};
    
    fprintf('--- Risultati %s ---\n', title_str);
    
    for i = 1:2
        nu = innov(i, :);
        nu = nu - mean(nu); % Assicura media nulla
        
        % Calcolo manuale dell'autocorrelazione
        r = zeros(1, max_lag + 1);
        var_nu = sum(nu.^2);
        if var_nu > 0
            for k = 0:max_lag
                r(k+1) = sum(nu(1:N_samples-k) .* nu(k+1:N_samples)) / var_nu;
            end
        end
        lags = 0:max_lag;
        
        % --- VERIFICA NUMERICA DEL TEST ---
        % Consideriamo i lag da 1 a max_lag (escludiamo il lag 0)
        r_test = r(2:end);
        
        % Contiamo quanti punti sono fuori dal limite di confidenza
        punti_fuori = sum(abs(r_test) > conf_bound);
        percentuale_fuori = (punti_fuori / max_lag) * 100;
        
        % Se i punti fuori sono meno del 5%, il test è passato
        if percentuale_fuori <= 5.0
            esito = 'PASSATO (Innovazione Bianca)';
        else
            esito = 'FALLITO (Presenza di colorazione/correlazione)';
        end
        
        % Stampa a schermo i risultati
        fprintf('%s:\n  Esito: %s\n  Punti fuori limite: %d su %d (%.1f%%)\n', ...
            meas_labels{i}, esito, punti_fuori, max_lag, percentuale_fuori);
        
        % --- PLOT ---
        subplot(2,1,i); hold on; grid on;
        stem(lags, r, 'filled', 'MarkerSize', 4, 'Color', '#0072BD');
        
        % Plot dei limiti di Anderson
        yline(conf_bound, 'r--', 'LineWidth', 1.5, 'DisplayName', '95\% Bound');
        yline(-conf_bound, 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        
        % Estetica e Testi
        title([title_str, ' - ', meas_labels{i}], 'Interpreter', 'none');
        xlabel('Lag $\tau$', 'Interpreter', 'latex'); 
        ylabel('Autocorrelazione $\rho(\tau)$', 'Interpreter', 'latex');
        if i==1, legend('Location', 'best', 'Interpreter', 'latex'); end
        ylim([-max(0.2, conf_bound*2.5), 1.1]);
        xlim([-1, max_lag+1]);
    end
    fprintf('\n');
end