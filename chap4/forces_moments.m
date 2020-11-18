% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, MAV)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind data in NED
    R_roll = [...
              1, 0, 0;...
              0, cos(phi), sin(phi);...
              0, -sin(phi), cos(phi)];
    R_pitch = [...
              cos(theta), 0, -sin(theta);...
              0, 1, 0;...
              sin(theta), 0, cos(theta)];
    R_yaw = [...
              cos(psi), sin(psi), 0;...
              -sin(psi), cos(psi), 0;...
              0, 0, 1];
    R = R_roll*R_pitch*R_yaw;   
    
    w_i = [w_ns, w_es, w_ds]' + R' * [u_wg, v_wg, w_wg]';
    w_n = w_i(1);
    w_e = w_i(2);
    w_d = w_i(3);
    
    w_b = R*[w_ns, w_es, w_ds]' + [u_wg, v_wg, w_wg]';
    u_r = u - w_b(1);
    v_r = v - w_b(2);
    w_r = w - w_b(3);
    
    % compute air data
    Va = (u_r^2 + v_r^2 + w_r^2)^0.5;
    alpha = atan(w_r/u_r);
    beta = asin(v_r/(u_r^2 + v_r^2 + w_r^2)^0.5);
    
    % aerodynamic coefficients
    
    sigma = (1+exp(-MAV.M*(alpha-MAV.alpha0))+exp(MAV.M*(alpha+MAV.alpha0)))/...
            ((1+exp(-MAV.M*(alpha-MAV.alpha0)))*(1+exp(MAV.M*(alpha+MAV.alpha0))));
    Cl = (1-sigma)*(MAV.C_L_0+MAV.C_L_alpha*alpha) + ...
          sigma*(2*sign(alpha)*sin(alpha)^2*cos(alpha));
    Cd = MAV.C_D_p + ((MAV.C_L_0+MAV.C_L_alpha*alpha)^2)/(pi*MAV.e*MAV.b^2/MAV.S_wing);
    Cx = -Cd*cos(alpha)+Cl*sin(alpha);
    Cx_q = -MAV.C_D_q*cos(alpha)+MAV.C_L_q*sin(alpha);
    Cx_delta_e = -MAV.C_D_delta_e*cos(alpha)+MAV.C_L_delta_e*sin(alpha);
    Cz = -Cd*sin(alpha)-Cl*cos(alpha);
    Cz_q = -MAV.C_D_q*sin(alpha)-MAV.C_L_q*cos(alpha);
    Cz_delta_e = -MAV.C_D_delta_e*sin(alpha)-MAV.C_L_delta_e*cos(alpha);
    
    
    % compute external forces and torques on aircraft
    Force(1) = -MAV.mass*MAV.gravity*sin(theta) + ...
        0.5*MAV.rho*Va^2*MAV.S_wing * (Cx + Cx_q*(MAV.c/(2*Va))*q + Cx_delta_e*delta_e) + ...
        0.5*MAV.rho*MAV.S_prop*MAV.C_prop * ((MAV.k_motor*delta_t)^2-Va^2);
    Force(2) = MAV.mass*MAV.gravity*cos(theta)*sin(phi) + ...
        0.5*MAV.rho*Va^2*MAV.S_wing * (MAV.C_Y_0 + MAV.C_Y_beta*beta + MAV.C_Y_p*(MAV.b/(2*Va))*p + ...
        MAV.C_Y_r*(MAV.b/(2*Va))*r + MAV.C_Y_delta_a*delta_a + MAV.C_Y_delta_r*delta_r);
    Force(3) = MAV.mass*MAV.gravity*cos(theta)*cos(phi) + ...
        0.5*MAV.rho*Va^2*MAV.S_wing * (Cz + Cz_q*(MAV.c/(2*Va))*q + Cz_delta_e*delta_e);
    
    Torque(1) =  0.5*MAV.rho*Va^2*MAV.S_wing * MAV.b * ...
       (MAV.C_ell_0 + MAV.C_ell_beta*beta + MAV.C_ell_p*(MAV.b/(2*Va))*p + MAV.C_ell_r*(MAV.b/(2*Va))*r +...
        MAV.C_ell_delta_a*delta_a + MAV.C_ell_delta_r*delta_r) - ...
        MAV.k_T_P*(MAV.k_Omega*delta_t)^2;
    Torque(2) = 0.5*MAV.rho*Va^2*MAV.S_wing * MAV.c * ...
        (MAV.C_m_0 + MAV.C_m_alpha*alpha + MAV.C_m_q*(MAV.c/(2*Va))*q + MAV.C_m_delta_e*delta_e);   
    Torque(3) = 0.5*MAV.rho*Va^2*MAV.S_wing * MAV.b * ...
        (MAV.C_n_0 + MAV.C_n_beta*beta + MAV.C_n_p*(MAV.b/(2*Va))*p + MAV.C_n_r*(MAV.b/(2*Va))*r + ...
         MAV.C_n_delta_a*delta_a + MAV.C_n_delta_r*delta_r);
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end



