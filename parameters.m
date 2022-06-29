%% Run this file before simulation
% This file contains the initialization for the simulation model used in
% TTK4900 - Engineering Cybernetics, Master's Thesis
% "Control of power converters for flexible system integration of single battery installation in a two-bus ship power system
% Designed by Håken Sivesindtajet Lunn 2022

clear all
close all
clc

%% Simulation prameters 
T = 1e-4;     % Simulation step size

%% System parameters
V = 690;            % System voltage [Vrms]
fb = 50;            % System frequency [Hz]
V_dc_bat= 1200;     % Battery voltage [Vdc]
 
%% Per Unit system
Vb = V*sqrt(2)/sqrt(3);     % Base phase to zero voltage [V]
Sb = 1e6;                   % Base apperant power [VA]
Ib = 2/3*Sb/Vb;             % Base ac current [A];

Vb_dc = Vb*2;               % Base dc voltage [Vpp]
Ib_dc = Sb/Vb_dc;           % Base ac current [A];

omega_b = 2*pi*fb;          % Angular frequency [Rad/sec]

Rb = Vb/Ib;                 % Base resistance
Lb = Rb/omega_b;            % Base inductance (posetive VAR)
Cb = 1/(Rb*omega_b);        % Base capasitance (negative VAR)

%% Inductor filter parameters
r_lf_pu = 0.003;     % Filter resistans (pu)
l_f_pu = 0.08;       % Filter inductans (pu)
c_f_pu = 0.074;      % Filter conductans (pu)

R_lf = Rb*r_lf_pu;   % Filter resistans
L_f = Lb*l_f_pu;     % Filter inductans
C_f = Cb*c_f_pu;     % Filter conductans

%% DC link parameters
c_dc_pu = 4;          % DC conductans (pu)
C_dc = Cb*c_dc_pu;    % DC conductans

%% PLL
K_vco = 2*pi;       % VCO constant
T_fPLL = 10e-3;     % PLL LP Filter constant <1, 100mS> 
a = 3;              % PLL tuning parameter <2, 10>

K_pPLL = 1/(a*2*pi*T_fPLL);     % Propositional constant PLL PI-controller
K_iPLL = a^2*T_fPLL;            % Integration constant PLL PI-controller

%% VSC model (inner loop)
f_sw_pwm = 5e3; % PWM switching frequency 

K_ffv = 0;      % Feedforward voltage Current Controller
K_ad = 1.5;     % Activ daming gain
omega_ad = 50;  % Active damping low pass cutoff frequency [Rad/s] 

T_1 = L_f/R_lf;                 % Inductorfilter time constant
T_sum = 1/(2*f_sw_pwm);         % Combinded time constant

K_pc = l_f_pu/(4*pi*fb*T_sum);  % Propositional constant CC PI-controller
K_ic = K_pc/T_1;                % Integration constant CC PI-controller 

%% VSM model (QSEM)
% Inertia emulator
Ta = 4;             % Mechanical time constant equal to 2H in traditional SM models
Kd = 40;            % Set the diffrence between feed back dm an the LP-filterd dm signal       
omega_d = 5;        % Low pass feed back cutoff frequency [Rad/s] 

% Frequency control "Governor"
K_omega = 20;       % Frequency control gain

% Electrical model (Quasi-Stationary Electrical Model)
omega_vf = 200;     % Low pass cutoff frequency v_m [Rad/s]

% Voltage Control "AVR"
Kq = 0.1;           % Reactiv power droop gain
omega_qf = 200;     % Low pass feed back cutoff frequency [Rad/s]

K_pv = 0.29;        % Propositional constant VC PI-controller
T_iv = 3.152e-3;    % Time constant constant VC PI-controller
K_iv = K_pv/T_iv;   % Integration constant VC PI-controller   

%% Generator model
% Syncronus Machine parameters
V_SM = V;           % RMS line to line voltage [V]
S_SM = Sb;          % Nominal apparet power [VA]
f_SM = fb;          % Nominal frequency [Hz]

ic_SM = 0.8;        % Inertia constant H(s),  ratio of stored energy @ nominal speed over nominal power (0.3222 SM only) 
ff_SM = 0.01322;    % friction factor [F] (torque[pu]/speed[pu])
pp_SM = 2;          % Number of pole pairs of the synchronous machine

% Stator
rs_SM = 0.01524;    % Stator resistance [pu]
lls_SM = 0.08;      % Stator leakage inductance [pu]
lmd_SM = 2.81;      % d-axis magnetizing inductance [pu]
lmq_SM = 1.64;      % q-axis magnetizing inductance  [pu]
lmq_SM = 1.64;      % q-axis magnetizing inductance  [pu]
lf1f = 0;           % Canay inductance [pu]

% Field 
rfd_SM = 0.004319;  % Field resistance referred to stator[pu] 
lfd_SM = 0.531;     % Field leakage inductance referred to stator [pu]

% Dampers
rkd = 0.2343;       % d-axis resistance referred to stator [pu]
llkd = 2.655;       % d-axis leakage inductance referred to stator [pu]
rkq1 = 0.03365;     % q-axis resistance refered to stator [pu]
llkq1 = 0.2408;     % q-axis leakage inductance referred to stator [pu]

% Frequency Control (Governor)
K_omega_SM = 40;    % Frequency control gain
T_gt_SM = 0.5;      % Prime mover time response

% Automatic Voltage Rgulator (AVR)
Kq_SM = 0.1;           % Reactiv power droop gain
omega_qf_SM = 200;     % Low pass feed back cutoff frequency [Rad/s]

K_pv_SM = 35;          % Proposjonal constant VC PI-controller
T_iv_SM = 0.5;         % Time constant constant VC PI-controller
K_iv_SM = K_pv_SM/T_iv_SM;   % Integration constant VC PI-controller 


%% Variable load model (VSC and Twel pulse rectifier)
v_dc_ref = 1;           % Desired DC voltage in pu
K_pdc = 25;             % Propositional constant DC voltage PI-controller
T_idc = .1;             % Time constant constant DC voltage PI-controller  
K_idc = K_pdc/T_idc;    % Integration constant DC voltage PI-controller


%% Constant impedance load model
p_h = 0.1;               % Active power