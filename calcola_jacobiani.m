clear; clc;

% 1. Definizione di TUTTE le variabili simboliche reali
syms alpha alpha_dot beta beta_dot real
syms F1 F2 real
syms dt m l g J_y J_z I_b J_alpha c_alpha c_beta eps_p eps_y rho real
syms Cd_A_x Cd_A_y Cd_A_z real

% Raggruppo gli stati e gli ingressi
x_sym = [alpha; alpha_dot; beta; beta_dot];
u_sym = [F1; F2];
Cd_A = [Cd_A_x; Cd_A_y; Cd_A_z];

% 2. Costruzione della dinamica esatta (f_sym)
J_beta = J_y * sin(alpha)^2 + (J_z + m*l^2)*cos(alpha)^2 + I_b;
omega_body = [-beta_dot * sin(alpha); alpha_dot; beta_dot * cos(alpha)];
v_corpo_body = cross(omega_body, [l; 0; 0]);
v_rel_body = -v_corpo_body;

% Sostituzione di norm() per facilitare la derivazione simbolica
v_rel_norm = sqrt(v_rel_body(1)^2 + v_rel_body(2)^2 + v_rel_body(3)^2 + 1e-5); 

F_drag_body = 0.5 * rho * Cd_A .* v_rel_body * v_rel_norm;
tau_drag_alpha = -F_drag_body(3) * l;
tau_drag_beta  = F_drag_body(2) * l;

alpha_ddot = (-c_alpha*alpha_dot - m*g*l*sin(alpha) + l*F1*cos(beta) + eps_p*l*F2*sin(beta) + tau_drag_alpha) / J_alpha;
beta_ddot  = (-c_beta*beta_dot + l*F2*cos(alpha) + eps_y*l*F1*sin(alpha) + tau_drag_beta) / J_beta;

% Equazioni di stato discrete
f_sym = [alpha + dt * alpha_dot;
         alpha_dot + dt * alpha_ddot;
         beta + dt * beta_dot;
         beta_dot + dt * beta_ddot];

% 3. Calcolo Jacobiano di Stato (F)
F_sym = jacobian(f_sym, x_sym);

% 4. Costruzione equazioni di misura (h_sym) e Calcolo Jacobiano (H)
h_sym = [-g * sin(alpha);
          cos(alpha) * sin(beta)];
H_sym = jacobian(h_sym, x_sym);

% 5. Generazione fisica del file
disp('Generazione del file jacobiani_analitici.m...');
matlabFunction(F_sym, H_sym, 'File', 'jacobiani_analitici', ...
    'Vars', {alpha, alpha_dot, beta, beta_dot, F1, F2, dt, rho, ...
             m, l, g, J_y, J_z, I_b, J_alpha, c_alpha, c_beta, ...
             eps_p, eps_y, Cd_A_x, Cd_A_y, Cd_A_z}, ...
    'Outputs', {'F', 'H'});

disp('Finito! File generato con successo nella cartella corrente.');