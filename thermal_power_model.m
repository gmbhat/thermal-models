clear variables;
close all;

%% Setup variables
load('rise_data_1800');                                                               

F0_index = 1;
F4_index = 2;
A7_V_index = 3;
A15_V_index = 4;
A7_W_index = 5;
A15_W_index = 6;
MEM_W_index = 7;
GPU_W_index = 8;
T0_index = 9;
T1_index = 10;
T2_index = 11;
T3_index = 12;
TGPU_index = 13;

power_index = A7_W_index:GPU_W_index;
temperature_index = T0_index:TGPU_index;

F0 = data_raw(:,F0_index);                                                      % Read input data
F4 = data_raw(:,F4_index);
A7_V = data_raw(:,A7_V_index);
A15_V = data_raw(:,A15_V_index);
A7_W = data_raw(:,A7_W_index);
A15_W = data_raw(:,A15_W_index);
MEM_W = data_raw(:,MEM_W_index);
GPU_W = data_raw(:,GPU_W_index);
T0 = data_raw(:,T0_index)/1000;
T1 = data_raw(:,T1_index)/1000;
T2 = data_raw(:,T2_index)/1000;
T3 = data_raw(:,T3_index)/1000;
TGPU = data_raw(:,TGPU_index)/1000;
Ts = 0.1;
total_num = length(T0);
t = (0:1:total_num -1)*Ts;
t = t';
up = 0;

%% Thermal and Leakage power parameters
c1 = 0.002488;
c2 = 2660;

Igate = 519*1e-6;

% Thermal model
% Readme
% We use the following A and B matrices to perform the thermal prediction. In
% Matlab we can feed in a vector of temperature and power values to get the
% temperature prediction as
% T[k+1] = A*T[k] + B*P[k]

A =[0.9935    0.0015    0.0022    0.0014 0.0007;
    0.0070    0.9913    0.0001    0.0008 0;
    0.0017    0.0022    0.9911    0.0019 0.0030;
    0.0074         0         0    0.9904  0.0003
    0.0020    0.0019    0.0022    0.0015  0.9896];


B =[0.1646    0.0793         0    0.0228;
    0.1143    0.0730    0.0178         0;
    0.1440    0.0818         0         0;
    0.1160    0.0833    0.1112    0.0154
    0.1332    0.0713    0          0.0991];

%% Generate P and temperature matrices
T_matrix = [T0 T1 T2 T3 TGPU];             % Form T and P matrices
P_matrix = [A7_W A15_W MEM_W GPU_W];

%% Subtract leakage power from the total power
T_matrix_SI = T_matrix + 273;                                                   % Convert temperature to SI units
leakage_power_big = A15_V.*c1.*T_matrix_SI(:,1).^2.*exp(-c2./T_matrix_SI(:,1)) + A15_V*Igate;             % Estimate leakage power from the total power


dyn_power_big = P_matrix(:,2) - leakage_power_big;                              % Get dynamic power

%% Temperature prediction
% Calculate the temperature as predicted by the model
T_prediction = A*T_matrix' + B*P_matrix';

