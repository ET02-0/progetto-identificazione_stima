% Script di gestione per simulazione ed estrazione dataset 2DoF Helicopter
clear all; close all; clc;
% ==========================================
% 1. PARAMETRI GEOMETRICI E MECCANICI (Tabella 1)
% ==========================================
p.J_alpha = 0.012; p.J_y = 0.00023; p.J_z = 0.00364; 
p.I_b = 0.00023; p.m = 0.2; p.l = 0.2;
p.c_alpha = 0.01; p.c_beta = 0.01;
p.eps_p = 0.1; p.eps_y = 0.1; p.g = 9.81;

% Parametri Aerodinamici (letti dal blocco elicottero_dynamics)
p.rho = 1.225; 
p.Cd_A = [0.5; 0.6; 1.2]; 

% Pertutbazione dei parametri per incertezza sulla realtà
p_hat = p; 

% 2. Introduco il mismatch parametrico realistico
p_hat.J_y = p.J_y * 1.15;       
p_hat.J_z = p.J_z * 0.85;       
p_hat.J_alpha = p.J_alpha * 1.15; 
p_hat.I_b = p.I_b * 0.85;

p_hat.c_alpha = p.c_alpha * 1.15; 
p_hat.c_beta  = p.c_beta * 0.85;

p_hat.eps_p = p.eps_p * 1.15;  
p_hat.eps_y = p.eps_y * 0.85;

% ==========================================
% 2. CONFIGURAZIONE TEMPORALE
% ==========================================
dt = 0.001; % Passo di campionamento dei filtri discreti
t_max = 10;
t = 0:dt:t_max;
% ==========================================
% 3. DINAMICA DEGLI ATTUATORI
% ==========================================
omega_n = 30;   
zeta = 0.7;     
tau_d = 0.02;   
num_2nd = [omega_n^2];
den_2nd = [1, 2*zeta*omega_n, omega_n^2];
num_pade = [-tau_d/2, 1];
den_pade = [tau_d/2, 1];

% ==========================================
% 4. MATRICI DI TUNING PER I FILTRI (EKF / UKF)
% ==========================================
std_w_alpha = 0.03; 
std_w_beta = 0.02;  
std_w_alpha_dot = 0.75; % Parametro critico per assorbire ritardi e dinamiche
std_w_beta_dot = 0.25;

std_v_acc = 0.25; 
std_v_mag = 0.05; 

Q = diag([(std_w_alpha*dt)^2, (std_w_alpha_dot*dt)^2, (std_w_beta*dt)^2, (std_w_beta_dot*dt)^2]);
R = diag([std_v_acc^2, std_v_mag^2]);

% Condizioni iniziali per la dinamica e per i filtri
x_0 = [0.1; 0; 0.1; 0];       % Stato iniziale reale del robot
x_hat_0 = [0; 0; 0; 0];       % Stato iniziale stimato (leggermente sballato)
u_0 = [0; 0];                 % Condizione iniziale del ritardo ingressi
P_0 = eye(4) * 0.1;           % Incertezza iniziale sulla stima

% ==========================================
% 6. RACCOLTA DATI ESTRATTI E IMPACCHETTAMENTO
% ==========================================
% Nota: Assumiamo che Simulink sia configurato per salvare i dati come Dataset 
% o come singole strutture nel Workspace di uscita (simOut)

disp('📦 Creazione del file log_vars strutturato...');

log_vars.t = t;
log_vars.dt = dt;
log_vars.t_max = t_max;
log_vars.p = p;
log_vars.Q_true = Q;
log_vars.R_true = R;
log_vars.x_hat_0 = x_hat_0;
log_vars.P_0 = P_0;
