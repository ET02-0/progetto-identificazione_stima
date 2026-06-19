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
% ==========================================
% DINAMICA DEGLI ATTUATORI
% ==========================================
% 1. Dati Nominali Attuatori (Parametri Modificabili)
omega_n = 30;   % [rad/s] Pulsazione naturale (motori veloci)
zeta = 0.7;     % Smorzamento (leggermente sottosmorzato per reattività)
tau_d = 0.02;   % [s] Ritardo di trasporto (20 ms)

% 2. Variabili per il blocco della Dinamica del Secondo Ordine
num_2nd = [omega_n^2];
den_2nd = [1, 2*zeta*omega_n, omega_n^2];

% 3. Variabili per il blocco Approssimazione di Padé del 1° Ordine
num_pade = [-tau_d/2, 1];
den_pade = [tau_d/2, 1];


% Vettori di stato e ingressi reali
x_T = zeros(4, N);
x_T(:,1) = [0.1; 0; 0.1; 0]; % Stato iniziale [alpha, dot_alpha, beta, dot_beta]
% Alpha: Pitch -- Beta: Yaw

% Ingressi di controllo u = [F1; F2] (Usiamo dei segnali eccitanti per
% testare i filtri) ------[Sono abbastanza eccitanti??]------
% Creazione modello attuatori (TF) e filtraggio ingressi (lsim = "simulink via codice")
sys_act = tf(num_2nd, den_2nd) * tf(num_pade, den_pade);
u_raw = [0.5 * sin(2*t); 0.3 * cos(1.5*t)]; 
u_act = zeros(2, N);
u_act(1,:) = lsim(sys_act, u_raw(1,:), t)';
u_act(2,:) = lsim(sys_act, u_raw(2,:), t)';

% Rumore di processo e misura (Deviazioni standard)
std_w_alpha_dot = 0.05; 
std_w_beta_dot = 0.05;
std_v_acc = 0.2; % Rumore accelerometro
std_v_mag = 0.05; % Rumore magnetometro

Q = diag([0, (std_w_alpha_dot*dt)^2, 0, (std_w_beta_dot*dt)^2]);
R = diag([std_v_acc^2, std_v_mag^2]);

y_meas = zeros(2, N);

% Funzione derivata per RK4 (Dinamica Non Lineare)
dynamics = @(x, u_in) [ ...
    x(2); ... % alpha_dot
    (-p.c_alpha*x(2) - p.m*p.g*p.l*sin(x(1)) + p.l*u_in(1)*cos(x(3)) + p.eps_p*p.l*u_in(2)*sin(x(3))) / p.J_alpha; ... % alpha_ddot
    x(4); ... % beta_dot
    (-p.c_beta*x(4) + p.l*u_in(2)*cos(x(1)) + p.eps_y*p.l*u_in(1)*sin(x(1))) / (p.J_y * sin(x(1))^2 + (p.J_z + p.m*p.l^2)*cos(x(1))^2 + p.I_b) ... % beta_ddot
];

% 6. Ciclo RK4
disp('Simulazione RK4 in corso...');
for k = 1:N-1
    u_k = u_act(:,k);
    w_k = [0; std_w_alpha_dot * randn; 0; std_w_beta_dot * randn]; % Rumore di processo
    
    k1 = dynamics(x_T(:,k), u_k);
    k2 = dynamics(x_T(:,k) + 0.5*dt*k1, u_k);
    k3 = dynamics(x_T(:,k) + 0.5*dt*k2, u_k);
    k4 = dynamics(x_T(:,k) + dt*k3, u_k);
    
    x_T(:,k+1) = x_T(:,k) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4) + w_k*sqrt(dt);
    
    % Generazione misure
    y_meas(1,k+1) = -p.g * sin(x_T(1,k+1)) + std_v_acc * randn;  
    y_meas(2,k+1) = cos(x_T(1,k+1)) * sin(x_T(3,k+1)) + std_v_mag * randn;
end
% Salva i dati strutturati come fa Costanzi detto Rick (mi devo ricordare di
% cancellare i commenti idioti)
log_vars.t = t;
log_vars.dt = dt;
log_vars.t_max = t_max;
log_vars.x_T = x_T;
log_vars.u = u_act;
log_vars.y_meas = y_meas;
log_vars.p = p;

log_vars.Q_true = Q;
log_vars.R_true = R;

% Inizializzazione per i filtri (leggermente sballata rispetto al vero)
log_vars.x_hat_0 = [0; 0; 0; 0];
log_vars.P_0 = eye(4) * 0.1;

% Inizializzazione sistema vero
x_0 = [0.1; 0; 0.1; 0]; % Stato iniziale reale
u_0 = [0; 0];           % Condizione iniziale del ritardo ingressi


%% PARAMETRI PER TUNING
% std_w_alpha_dot e std_w_beta_dot: Rappresentano quanta incertezza c'è sul
% modello, più sono alti meno ci fidiamo delle equazioni fisiche e più seguiremo il rumore

% std_v_acc e std_v_mag: Derivano dai datasheet dei sensori scelti. Più sono 
% bassi, più si dice al filtro "i miei sensori sono perfetti", rischiando di far 
% impazzire la stima se il rumore reale è invece alto.

% P_0 = eye(4) * 0.1: Rappresenta l'incertezza sulla condizione iniziale. 
% Visto che lo stato reale parte da [0.1; 0; 0.1; 0] e il filtro da [0; 0; 0; 0], 
% un valore di 0.1 sulla diagonale permette al filtro di correggere rapidamente 
% l'errore iniziale senza divergere.

% --- AGGIUNTA PER IL SALVATAGGIO IN log_vars ---
log_vars.attuatori.omega_n = omega_n;
log_vars.attuatori.zeta    = zeta;
log_vars.attuatori.tau_d   = tau_d;
log_vars.attuatori.num_2nd = num_2nd;
log_vars.attuatori.den_2nd = den_2nd;
log_vars.attuatori.num_pade = num_pade;
log_vars.attuatori.den_pade = den_pade;

disp('✅ Parametri degli attuatori caricati correttamente nel workspace.');

save('dataset_elicottero.mat', 'log_vars');
disp('Dataset generato con successo!');