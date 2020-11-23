% initialize the mav viewer
addpath('../tools');  

% initial conditions
MAV.pn0    = 0;     % initial North position
MAV.pe0    = 0;     % initial East position
MAV.pd0    = -100;  % initial Down position (negative altitude)
MAV.u0     = 35;     % initial velocity along body x-axis
MAV.v0     = 0;     % initial velocity along body y-axis
MAV.w0     = 0;     % initial velocity along body z-axis
MAV.phi0   = 0;     % initial roll angle
MAV.theta0 = 0;     % initial pitch angle
MAV.psi0   = 0;     % initial yaw angle
%MAV.e0     = cos(MAV.psi0/2)*cos(MAV.theta0/2)*cos(MAV.phi0/2)+sin(MAV.psi0/2)*sin(MAV.theta0/2)*sin(MAV.phi0/2);
%MAV.e1     = cos(MAV.psi0/2)*cos(MAV.theta0/2)*sin(MAV.phi0/2)-sin(MAV.psi0/2)*sin(MAV.theta0/2)*cos(MAV.phi0/2);
%MAV.e2     = cos(MAV.psi0/2)*sin(MAV.theta0/2)*cos(MAV.phi0/2)+sin(MAV.psi0/2)*cos(MAV.theta0/2)*sin(MAV.phi0/2);
%MAV.e3     = sin(MAV.psi0/2)*cos(MAV.theta0/2)*cos(MAV.phi0/2)-cos(MAV.psi0/2)*sin(MAV.theta0/2)*sin(MAV.phi0/2);
MAV.p0     = 0;     % initial body frame roll rate
MAV.q0     = 0;     % initial body frame pitch rate
MAV.r0     = 0;     % initial body frame yaw rate
   
%physical parameters of airframe
MAV.gravity = 9.81;
MAV.mass = 13.5;
MAV.Jx   = 0.824;
MAV.Jy   = 1.135;
MAV.Jz   = 1.759;
MAV.Jxz  = 0.120;
MAV.S_wing        = 0.55;
MAV.b             = 2.90;
MAV.c             = 0.19;
MAV.S_prop        = 0.2027;
MAV.C_prop        = 1;
MAV.rho           = 1.2682;
MAV.e             = 0.9;
MAV.AR            = MAV.b^2/MAV.S_wing;

% Gamma parameters from uavbook page 36
MAV.Gamma  = MAV.Jx*MAV.Jz-MAV.Jxz^2;
MAV.Gamma1 = (MAV.Jxz*(MAV.Jx-MAV.Jy+MAV.Jz))/MAV.Gamma;
MAV.Gamma2 = (MAV.Jz*(MAV.Jz-MAV.Jy)+MAV.Jxz*MAV.Jxz)/MAV.Gamma;
MAV.Gamma3 = MAV.Jz/MAV.Gamma;
MAV.Gamma4 = MAV.Jxz/MAV.Gamma;
MAV.Gamma5 = (MAV.Jz-MAV.Jx)/MAV.Jy;
MAV.Gamma6 = MAV.Jxz/MAV.Jy;
MAV.Gamma7 = (MAV.Jx*(MAV.Jx-MAV.Jy)+MAV.Jxz*MAV.Jxz)/MAV.Gamma;
MAV.Gamma8 = MAV.Jx/MAV.Gamma;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% aerodynamic coefficients


MAV.M             = 50;
MAV.alpha0        = 0.47;
MAV.epsilon       = 0.16;

MAV.C_L_0         = 0.28;
MAV.C_L_alpha     = 3.45;
MAV.C_L_q         = 0.0;
MAV.C_L_delta_e   = -0.36;
MAV.C_D_0         = 0.03;
MAV.C_D_alpha     = 0.30;
MAV.C_D_p         = 0.0437;
MAV.C_D_q         = 0.0;
MAV.C_D_delta_e   = 0.0;
MAV.C_m_0         = -0.02338;
MAV.C_m_alpha     = -0.38;
MAV.C_m_q         = -3.6;
MAV.C_m_delta_e   = -0.5;

MAV.C_Y_0         = 0.0;
MAV.C_Y_beta      = -0.98;
MAV.C_Y_p         = 0.0;
MAV.C_Y_r         = 0.0;
MAV.C_Y_delta_a   = 0.0;
MAV.C_Y_delta_r   = -0.17;
MAV.C_ell_0       = 0.0;
MAV.C_ell_beta    = -0.12;
MAV.C_ell_p       = -0.26;
MAV.C_ell_r       = 0.14;
MAV.C_ell_delta_a = 0.08;
MAV.C_ell_delta_r = 0.105;
MAV.C_n_0         = 0.0;
MAV.C_n_beta      = 0.25;
MAV.C_n_p         = 0.022;
MAV.C_n_r         = -0.35;
MAV.C_n_delta_a   = 0.06;
MAV.C_n_delta_r   = -0.032;
MAV.C_prop        = 1.0;
MAV.M             = 50;
MAV.epsilon       = 0.1592;
MAV.alpha0        = 0.4712;

% Parameters for propulsion thrust and torque models
MAV.D_prop = 0.508;     % prop diameter in m

% Motor parameters
MAV.K_V = 145;                    % from datasheet RPM/V
MAV.KQ = (1/MAV.K_V)*60/(2*pi);   % KQ in N-m/A, V-s/rad
MAV.R_motor = 0.042;              % ohms
MAV.i0 = 1.5;                     % no-load (zero-torque) current (A)
MAV.k_motor = 80;
MAV.k_T_P = 0;
MAV.k_Omega = 0;
MAV.e = 0.9;


% Inputs
MAV.ncells = 12;
MAV.V_max = 3.7*MAV.ncells;       % max voltage for specified number of battery cells

% Coeffiecients from prop_data fit
MAV.C_Q2 = -0.01664;
MAV.C_Q1 = 0.004970;
MAV.C_Q0 = 0.005230;

MAV.C_T2 = -0.1079;
MAV.C_T1 = -0.06044;
MAV.C_T0 = 0.09357;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MAV.gamma = MAV.Jx*MAV.Jz-MAV.Jxz^2;
MAV.gamma_1 = (MAV.Jxz*(MAV.Jx-MAV.Jy+MAV.Jz))/MAV.gamma;
MAV.gamma_2 = (MAV.Jz*(MAV.Jz-MAV.Jy)+MAV.Jxz^2)/MAV.gamma;
MAV.gamma_3 = MAV.Jz/MAV.gamma;
MAV.gamma_4 = MAV.Jxz/MAV.gamma;
MAV.gamma_5 = (MAV.Jz-MAV.Jx)/MAV.Jy;
MAV.gamma_6 = MAV.Jxz/MAV.Jy;
MAV.gamma_7 = ((MAV.Jx-MAV.Jy)*MAV.Jx+MAV.Jxz^2)/MAV.gamma;
MAV.gamma_8 = MAV.Jx/MAV.gamma;



