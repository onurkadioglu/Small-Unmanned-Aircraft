function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,MAV)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(MAV);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,MAV);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(MAV)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    MAV.pn0;...
    MAV.pe0;...
    MAV.pd0;...
    MAV.u0;...
    MAV.v0;...
    MAV.w0;...
    MAV.phi0;...
    MAV.theta0;...
    MAV.psi0;...
    %MAV.e3;...
    MAV.p0;...
    MAV.q0;...
    MAV.r0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, MAV)

    pn    = x(1);
    pe    = x(2);
    pd    = x(3);
    u     = x(4);
    v     = x(5);
    w     = x(6);
    phi   = x(7);
    theta = x(8);
    psi   = x(9); 
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    
    % Aerosonde Uav parameters
    mass = MAV.mass;
    Jx = MAV.Jx;
    Jy = MAV.Jy;
    Jz = MAV.Jz;
    Jxz = MAV.Jxz;
    
    
    % Angles
    c_the = cos(theta);
    s_the = sin(theta);
    t_the = tan(theta);
    c_psi   = cos(psi);
    s_psi   = sin(psi);
    c_phi   = cos(phi);
    s_phi   = sin(phi);
    
    %Products and Moments of Inertia
    G=Jx*Jz-Jxz^2; 
    G1=(Jxz*(Jx-Jy+Jz))/G; 
    G2=(Jz*(Jz-Jy)+Jxz^2)/G;  
    G3=(Jz)/G; 
    G4=(Jxz)/G; 
    G5=(Jz-Jx)/Jy; 
    G6=(Jxz)/Jy;
    G7=((Jx-Jy)*Jx+Jxz^2)/G; 
    G8=(Jx)/G;
    
    
    % Equations of Motion
    pndot = u*(c_the*c_psi) + v*(s_phi*s_the*c_psi - c_phi*s_psi) + w*(c_phi*s_the*c_psi + s_phi*s_psi);
    pedot = u*(c_the*s_psi) + v*(s_phi*s_the*s_psi + c_phi*c_psi) + w*(c_phi*s_the*s_psi - s_phi*c_psi);
    pddot = u*(-s_the) + v*(s_phi*c_the) + w*(c_phi*c_the);
    
    udot = (r*v-q*w) + fx/mass;
    vdot = (p*w-r*u) + fy/mass;
    wdot = (q*u-p*v) + fz/mass;
    
    phidot = p + q*(s_phi*t_the) + r*(c_phi*t_the);
    thetadot = q*(c_phi) + r*(-s_phi);
    psidot = q*(s_phi/c_the) + r*(c_phi/c_the);
    
    %e0dot = (-1/2)*(p*e1+q*e2+r*e3);
    %e1dot = 1/2*(p*e0+e*e2-q*e3);
    %e2dot = 1/2*(q*e0-r*e1+p*e3);
    %e3dot = 1/2*(r*e0+q*e1-p*e2);
        
    pdot = (G1*p*q - G2*q*r) + (G3*ell + G4*n);
    qdot = (G5*p*r - G6*(p^2 - r^2)) + m/(Jy);
    rdot = (G7*p*r - G1*q*r) + (G4*ell + G8*n);
        

sys = [pndot; pedot; pddot; udot; vdot; wdot; phidot; thetadot; psidot; pdot; qdot; rdot];


% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
    %y = [...
     %   ];
sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
