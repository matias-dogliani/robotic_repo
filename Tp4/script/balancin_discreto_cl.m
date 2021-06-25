clear all; close all; clc;

% Parámetros de simulación.
J = 100e-3      % Momento de inercia
T = 10          % Tiempo de simulacion
dt = 0.0001        % Intervalo de muestreo
N = T/dt        % Indice maximo para estados discretos
ts = 0:dt:T-dt; % Vector de tiempos discretos
kp = 10; 
Titad = ones(1, N);

% Vector de estado inicial.
% q1 = theta; q2 = theta_punto
q0 = [0; 0];

% Vector de estados e inicialización.
q = zeros(2, N);
q(:, 1) = q0;

% Vector de acciones de control.
%u = zeros(1, N);
% Acciones de control
% u(1, 1:1:N)=1;
u = ones(1, N);

% Matrices del sistema de estados discretizado.
A = [1, dt; 0, 1];
B = [0; dt/J];

% Bucle para cálculo de los estados.
for i = 1 : (N-1)
	q(:, i+1) = A*q(:, i) + B*kp*(Titad(:,i) - q(1,i));
end

figure; 
subplot(3, 1, 1); plot(ts, q(1, :), 'LineWidth',4, 'color','r'); 
grid on; title('\theta vs t '); xlabel('t [s]'); ylabel('q1 (\theta)');
subplot(3, 1, 2); plot(ts, q(2, :)); 
grid on;title('\omega vs t'); xlabel('t [s]'); ylabel('q2 (\omega)');
subplot(3, 1, 3); plot(ts, u(1, :)); 
grid on;title(" \tau vs t"); xlabel('t [s]'); ylabel('u');
