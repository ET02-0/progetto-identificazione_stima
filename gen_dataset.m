%% SCRIPT 1: generate_dataset.m
% Simulazione del sistema Elicottero 2DoF per generare i dati di test

clear all; close all; clc;

% Parametri nominali (da Tabella 1)
p.J_alpha = 0.012; p.J_y = 0.00023; p.J_z = 0.00364; 
p.I_b = 0.00023; p.m = 0.2; p.l = 0.2;
p.c_alpha = 0.01; p.c_beta = 0.01;
p.eps_p = 0.1; p.eps_y = 0.1; p.g = 9.81;

% Setup simulazione
dt = 0.01;
t_max = 10;
t = 0:dt:t_max;
N = length(t);

% Vettori di stato e ingressi reali
x_T = zeros(4, N);
x_T(:,1) = [0.1; 0; 0.1; 0]; % Stato iniziale [alpha, dot_alpha, beta, dot_beta]

% Ingressi di controllo u = [F1; F2] (Usiamo dei segnali eccitanti per testare i filtri)
u = [0.5 * sin(2*t); 0.3 * cos(1.5*t)]; 

% Rumore di processo e misura (Deviazioni standard)
std_w_alpha_dot = 0.05; 
std_w_beta_dot = 0.05;
std_v_acc = 0.2; % Rumore accelerometro
std_v_mag = 0.05; % Rumore magnetometro

y_meas = zeros(2, N);

% Integrazione di Eulero per Ground Truth e Misure
for k = 1:N-1
    alpha = x_T(1,k); alpha_dot = x_T(2,k);
    beta = x_T(3,k);  beta_dot = x_T(4,k);
    F1 = u(1,k);      F2 = u(2,k);
    
    % Dinamica J_beta dipendente da alpha
    J_beta = p.J_y * sin(alpha)^2 + (p.J_z + p.m*p.l^2)*cos(alpha)^2 + p.I_b;
    
    % Equazioni del moto
    alpha_ddot = (-p.c_alpha*alpha_dot - p.m*p.g*p.l*sin(alpha) + p.l*F1*cos(beta) + p.eps_p*p.l*F2*sin(beta)) / p.J_alpha;
    beta_ddot  = (-p.c_beta*beta_dot + p.l*F2*cos(alpha) + p.eps_y*p.l*F1*sin(alpha)) / J_beta;
    
    % Aggiornamento stato reale (con rumore di processo sulle accelerazioni)
    x_T(1,k+1) = alpha + dt * alpha_dot;
    x_T(2,k+1) = alpha_dot + dt * (alpha_ddot + std_w_alpha_dot * randn);
    x_T(3,k+1) = beta + dt * beta_dot;
    x_T(4,k+1) = beta_dot + dt * (beta_ddot + std_w_beta_dot * randn);
    
    % Generazione misure (Accelerometro X, Magnetometro Y) con rumore
    y_meas(1,k+1) = -p.g * sin(x_T(1,k+1)) + std_v_acc * randn;
    y_meas(2,k+1) = cos(x_T(1,k+1)) * sin(x_T(3,k+1)) + std_v_mag * randn;
end

% Salva i dati strutturati come faceva il professore
log_vars.t = t;
log_vars.dt = dt;
log_vars.t_max = t_max;
log_vars.x_T = x_T;
log_vars.u = u;
log_vars.y_meas = y_meas;
log_vars.p = p;

log_vars.Q_true = diag([0, (std_w_alpha_dot*dt)^2, 0, (std_w_beta_dot*dt)^2]);
log_vars.R_true = diag([std_v_acc^2, std_v_mag^2]);

% Inizializzazione per i filtri (leggermente sballata rispetto al vero)
log_vars.x_hat_0 = [0; 0; 0; 0];
log_vars.P_0 = eye(4) * 0.1;

save('dataset_elicottero.mat', 'log_vars');
disp('Dataset generato con successo!');