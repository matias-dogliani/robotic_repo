clear all; close all; clc;

% Parametros de simulacion.
J = 100e-3      % Momento de inercia
T = 10          % Tiempo de simulacion
dt = 0.001     % Intervalo de muestreo
N = T/dt        % Indice maximo para estados discretos
ts = 0:dt:T-dt; % Vector de tiempos discretos

%Parametros del sensor 3DM-GX1 (en unidades brindadas por el fabricante ; obtenidos de http://files.microstrain.com/3DM-GX1%20Detailed%20Specs%20-%20Rev%201%20-%20070723.pdf)
DMGX1.arw      = 3.5   .* ones(1,3);   % Angle random walks [X Y Z] (deg/root-hour)
DMGX1.arrw     = zeros(1,3);           % Angle rate random walks [X Y Z] (deg/root-hour/s) 
DMGX1.vrw      = 0.4 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)  [NOISE ACELEROMETRO]
DMGX1.vrrw     = zeros(1,3);           % Velocity rate random walks [X Y Z] (deg/root-hour/s) 
DMGX1.gb_sta   = 0.7   .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)
DMGX1.ab_sta   = 10  .* ones(1,3);     % Acc static biases [X Y Z] (mg)
DMGX1.gb_dyn   = 0.02 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)
DMGX1.ab_dyn   = 0.2 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)
DMGX1.gb_corr  = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
DMGX1.ab_corr  = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
DMGX1.freq     = 1/dt;             % IMU operation frequency [X Y Z] (Hz)
DMGX1.m_psd    = 0 .* ones(1,3);   % Magnetometer noise density [X Y Z] (mgauss/root-Hz) [ESTE NO SALE EN LA HOJA DE DATOS, ASI QUE LO DEJE COMO ESTABA]

imu = imu_si_errors(DMGX1, dt);       % Funcion para convertir las unidades del fabricante a las del Sistema Internacional

%% Simulacion de ruidos

M = [N, 3];

% -------------------------------------------------------------------------
% Simulacion de bias estatico

gb_sta = noise_b_sta (imu.gb_sta, N);

% -------------------------------------------------------------------------
% Simulacion de ruido blanco

wn = randn(M);
g_wn = zeros(M);

for i=1:3

    g_wn(:, i) = imu.g_std(i) .*  wn(:,i);
end

% -------------------------------------------------------------------------
% Simulacion de bias dinamico (inestabilidad de bias)

gb_dyn = noise_b_dyn (imu.gb_corr, imu.gb_dyn, dt, M);

% -------------------------------------------------------------------------
% Simulacion de rate random walk

g_rrw = noise_rrw (imu.arrw, dt, M);

% -------------------------------------------------------------------------
%Error total en cada medicion de velocidad angular
error = g_wn(:,1) + gb_sta(:,1) + gb_dyn(:,1) + g_rrw(:,1); %Nos quedamos unicamente con los errores en X (usamos solo 1 de los giroscopos)

% Vector de estado inicial.
% q(1) = theta (posicion angular) ; q(2) = theta_punto (velocidad angular)
q0 = [0; 0];

% Vector de estados e inicializacion.
q = zeros(2, N);
q(:, 1) = q0;

% Ahora, u es lo que le llega a la planta (luego de restarse a la entrada y
% multiplicar por Kp). Se calcula dentro del bucle for.
u = zeros(1, N);
%u = ones(1, N);

% Vector de theta de referencia (Entrada del sistema)
% Aca se aplican las acciones de control
%ref = zeros(1, N);
ref = ones(1, N);   %Escalon unitario

% Ganancia proporcional
%Kp = 0.1
Kp = 1
%Kp = 10

% Constante del compensador
%Td = 0.1
Td = sqrt(0.4)  %Td critico
%Td = 2

%En la primera iteracion, el punto anterior de la derivada no esta definido
u(1, 1) = ( ref(1, 1) - u(1,1) + Td * ( ref(1, 1) - u(1,1) )/dt ) * Kp; 

% Matrices del sistema de estados discretizado.
A = [1, dt; 0, 1];
B = [0; dt/J];

% Bucle para calculo de los estados.
for i = 1 : (N-1)  
  q(:, i+1) = A*q(:, i) + B*u(:, i);
  q(2, i+1) = q(2, i+1) + error(i);  %Agregamos ruido de medicion a la VELOCIDAD ANGULAR (medida por el giroscopo)
  u(:, i+1) = ( ref(:, i+1) - q(1, i+1) + Td * ( ref(:, i+1) - q(1, i+1) - ( ref(:, i) - q(1, i) ) )/ dt ) * Kp;
end

f1 = figure();
subplot(3, 1, 1); plot(ts, q(1, :), "linewidth", 2); 
grid on; xlabel('t [s]'); ylabel('q1 (\theta)');
title('Estado: Posicion angular')
subplot(3, 1, 2); plot(ts, q(2, :), "linewidth", 2); 
grid on; xlabel('t [s]'); ylabel('q2 (\omega)');
title('Estado: Velocidad angular')
subplot(3, 1, 3); plot(ts, u(1, :), "linewidth", 2); 
grid on; xlabel('t [s]'); ylabel('u');
title('Accion de control: Torque')