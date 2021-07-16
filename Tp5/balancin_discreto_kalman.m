clear all; close all; clc;



addpath ../../ins/
addpath ../../ins-gnss/
addpath ../../simulation/
addpath ../../conversions/
addpath ../../performance_analysis/

% Parametros del modelo de balancin.
J = 100e-3;       % Momento de inercia
T = 10;           % Tiempo de simulacion
dt = 0.0001;      % Intervalo de muestreo
N = T/dt;         % Indice maximo para estados discretos
ts = 0:dt:T-dt;   % Vector de tiempos discretos
kp=1;            
td=0.6324;       
q0 = [0;0];       % Vector de estado inicial.
q   = zeros(2, N);  % Vector de estados e inicialización.
qNoise = zeros(2, N);  % Vector de estados e inicialización.
q(:, 1) = q0;
qIdeal =q; 
u = ones(1, N);   % Vector de acciones de control.
uIdeal = ones(1,N); 
A = [1, dt; 0, 1];% Matriz del sistema de estados discretizado.
B = [0; dt/J];    % Matriz del sistema de estados discretizado.
titaRef = ones(1, N);


%%Ref parameters
ref.freq=1/dt;               %freq: sampling frequency (Hz).
ref.t=dt .* ones(1,1);       %t: Nx1 time vector (seconds).
ref.lat= zeros(1, 1);        %lat: Nx1 latitude (radians).
ref.vel= zeros(1, 2);        %vel: Nx3 NED velocities (m/s).
ref.h=   zeros(1, 1);        %Nx1 altitude (m).
ref.DCMnb_m=zeros(1,9);      %Nx9 matrix with nav-to-body direct cosine matrices (DCM).
D2R = (pi/180);              %degrees to radians
ax_zero= zeros(1, N);        %zero value fot nonused axies
%% A3DMGX IMU ERROR PROFILE con datos EJ 4. 
%Parametros del sensor 3DM-GX1 (en unidades brindadas por el fabricante ; obtenidos de http://files.microstrain.com/3DM-GX1%20Detailed%20Specs%20-%20Rev%201%20-%20070723.pdf)
A3DMGX.arw      = 3.5   .* ones(1,3);   % Angle random walks [X Y Z] (deg/root-hour)
A3DMGX.arrw     = zeros(1,3);           % Angle rate random walks [X Y Z] (deg/root-hour/s) 
A3DMGX.vrw      = 0.4 .* ones(1,3);     % Velocity random walks [X Y Z] (m/s/root-hour)  [NOISE ACELEROMETRO]
A3DMGX.vrrw     = zeros(1,3);           % Velocity rate random walks [X Y Z] (deg/root-hour/s) 
A3DMGX.gb_sta   = 0.7   .* ones(1,3);     % Gyro static biases [X Y Z] (deg/s)  
A3DMGX.ab_sta   = 10  .* ones(1,3);     % Acc static biases [X Y Z] (mg)        
A3DMGX.gb_dyn   = 0.02 .* ones(1,3);   % Gyro dynamic biases [X Y Z] (deg/s)    
A3DMGX.ab_dyn   = 0.2 .* ones(1,3);     % Acc dynamic biases [X Y Z] (mg)       
A3DMGX.gb_corr  = 100 .* ones(1,3);     % Gyro correlation times [X Y Z] (seconds)
A3DMGX.ab_corr  = 100 .* ones(1,3);     % Acc correlation times [X Y Z] (seconds)
A3DMGX.freq     = 1/dt;             % IMU operation frequency [X Y Z] (Hz)      
A3DMGX.m_psd    = 0 .* ones(1,3);   % Magnetometer noise density [X Y Z] (mgauss/root-Hz) [ESTE NO SALE EN LA HOJA DE DATOS, ASI QUE LO DEJE COMO ESTABA]




A3DMGX.t = ref.t;                       % IMU time vector
imu1 = imu_si_errors(A3DMGX, dt);       % IMU manufacturer error units to SI units.
imu1.ini_align_err = [1 1 2] .* D2R;    % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)
imu1.ini_align = [0 0 0];               % Initial attitude align at t(1) (radians).



%Matrices:
F = [1 -dt;0 1];
G = [dt;0];
L = dt;
H = [1 0]; 

%% Matrices de covarianza Q y R
Q = dt^2 * [8.75 0;0 1];           %Covariancia de proceso
R = 398.62;                        %Covariancia de medici�n


%FK / Pred y correccion                                                         
e =titaRef(1) - q(1, 1);  %tita - estado_inicial
u(1, 1) = e+td*kp*((e)/dt);
uIdeal(1, 1) = e+td*kp*((e)/dt);
x_hat= zeros(2, N);                                                  
P = eye(2);                                                                

% Bucle para calculo de los estados.
for i = 1 : (N-1)  
   qIdeal(:, i+1) = A*qIdeal(:, i) + B*uIdeal(:, i);
   uIdeal(:, i+1) = ( titaRef(:, i+1) - qIdeal(1, i+1) + td * ( titaRef(:, i+1) - qIdeal(1, i+1) - ( titaRef(:, i) - qIdeal(1, i) ) )/ dt ) * kp;
end
                   
for i = 2 : N -2                                                                
                                                                                
    %----------Agregado de ruido en la medici�n                                 
    ref.wb = [q(1,i-1);ax_zero(1,1);ax_zero(1,1)]';  %Generacion de tita ruidosa
    titaNoise = gyro_gen(ref, imu1);                                     
    qNoise(1,i-1)=titaNoise(1,1);                                             
    q(1,i-1)  =titaNoise(1,1);                                                
    z =  q(:, i-1);                 %z = tita                                   
    uk = q(:, i-1);                  % u = wm                                   
                                                                                                                                         
  %----------Filtro                                                             
    %Predicción                                                                 
    x_hat(:,i) = F* x_hat(:,i-1) + G*uk(2);                                     
    P_a = F*P*F' + Q;                                                           
    Kk = P_a* H' * inv( H*P_a*H' + R );                                         
    %Correccion                                                                 
    x_hat(:,i) =  x_hat(:,i) + Kk .* (z(1) - H*x_hat(:,i));                     
    P= P_a - Kk * H * P_a;                                                      
                                                                                
    %----------Fin filtro                                                       
    q(1, i-1)=x_hat(1,i-1);                                                     
    q(:, i) = A*q(:, i-1) + B*u(:, i-1);                                        
    e =titaRef(:, i-1)   - q(1, i-1);                                           
    e1=titaRef(:, i) - q(1, i);                                                 
    u(:, i) = (e1+td*((e1-e)/dt))*kp;                                           
                                                                    
end                                                                                
      
    % Colors
blue = [0, 0.4470, 0.7410];
orange = [0.8500, 0.3250, 0.0980];
gray= ones(1,3) * 0.75;

%Comparacion de graficas

fig = figure();
hold on
plot(ts(100:end-10), qIdeal(1,(100:end-10)),'color',blue, 'LineWidth', 2);
plot(ts(100:end-10), qNoise(1,(100:end-10)),'color',gray, 'LineWidth', 1);
plot(ts(100:end-10), q(1,(100:end-10)),'color',orange,'linewidth',2); 
legend('Respuesta ideal','Resuesta con ruido','Respuesta con FK')
grid on; xlabel('ts[s]'); ylabel('Angulo del balancin [rad]');
saveas(fig,"CompPlot.png","png");


