% x_trim is the trimmed state,
% u_trim is the trimmed input
  
Va_trim = sqrt(x_trim(4)^2 + x_trim(5)^2 + x_trim(6)^2) ;
theta_trim = x_trim(8) ;
alpha_trim = atan(x_trim(6)/x_trim(4)) ;
delta_e_trim = u_trim(1) ;
delta_t_trim = u_trim(4) ;

C_P_p = MAV.gamma_3*MAV.C_ell_p + MAV.gamma_4*MAV.C_n_p;
a_phi1 = -0.25*MAV.rho*Va_trim*MAV.S_wing*(MAV.b^2)*C_P_p;
C_P_delta_a = MAV.gamma_3*MAV.C_ell_delta_a + MAV.gamma_4*MAV.C_n_delta_a;
a_phi2 = 0.5*MAV.rho*(Va_trim^2)*MAV.S_wing*MAV.b*C_P_delta_a;

a_theta1 = -0.25*MAV.rho*Va_trim*(MAV.c^2)*MAV.S_wing*MAV.C_m_q/MAV.Jy;
a_theta2 = -0.5*MAV.rho*(Va_trim^2)*MAV.c*MAV.S_wing*MAV.C_m_alpha/MAV.Jy;
a_theta3 = 0.5*MAV.rho*(Va_trim^2)*MAV.c*MAV.S_wing*MAV.C_m_delta_e/MAV.Jy;

a_V1 = (MAV.rho*Va_trim*MAV.S_wing/MAV.mass)*(MAV.C_D_0 + MAV.C_D_alpha*alpha_trim...
    + MAV.C_D_delta_e*delta_e_trim) + (MAV.rho*MAV.S_prop*MAV.e*Va_trim/MAV.mass);
a_V2 = MAV.rho*MAV.S_prop*MAV.e*(MAV.k_motor^2)*delta_t_trim/MAV.mass;
a_V3 = MAV.gravity*cos(theta_trim - alpha_trim);

a_beta1 = -0.5*MAV.rho*Va_trim*MAV.S_wing*MAV.C_Y_beta/MAV.mass;
a_beta2 = 0.5*MAV.rho*Va_trim*MAV.S_wing*MAV.C_Y_delta_r/MAV.mass;

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([MAV.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([a_beta2],[1,a_beta1]);
