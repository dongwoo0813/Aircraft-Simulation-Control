function [XDOT] = RCAM_model(X,U)

% Done by Edward Jung for personal interest and development

% References: Dr. Christopher Lum's YouTube Video "A Nonlinear, 6 DOF
% Dynamic Model of an Aircraft: the Research Civil Aircraft Model (RCAM)"
% and "Building a Matlab/Simulink Model of an Aircraft: the Research Civil
% Aircraft Model (RCAM)"
% Also, referred to RCAM document in 1997

%% state and control vectors

x1 = X(1); %u
x2 = X(2); %v
x3 = X(3); %w
x4 = X(4); %p, roll rate
x5 = X(5); %q, pitch rate
x6 = X(6); %r, yaw rate
x7 = X(7); %phi
x8 = X(8); %theta
x9 = X(9); %psi

u1 = U(1); %rad, d_A, aileron
u2 = U(2); %rad, d_T, horizontal stabilizer
u3 = U(3); %rad, d_R, rudder
u4 = U(4); %d_th1, throttle 1
u5 = U(5); %d_th2, throttle 2

%% Constants

% Aircraft Constants
m = 120000;         %kg, Aircraft total mass          ASSUME CONSTANT

cbar = 6.6;         %meter, Mean Aerodynamic Chord
lt = 24.8;          %meter, Distance from AC_Body to AC_tail
S = 260;            %m^2, Wing Planform Area
St = 64;            %m^2, Tail Planform Area

Xcg = 0.23*cbar;    %meter, x-position of CG
Ycg = 0;            %meter, y-position of CG
Zcg = 0.1*cbar;     %meter, z-position of CG

Xac = 0.12*cbar;    %meter, x-position of AC
Yac = 0;            %meter, y-position of AC
Zac = 0;            %meter, z-position of AC

% Engine Constants
Xapt1 = 0;          %meter, x-position of Force exerted by Engine 1
Yapt1 = -7.94;      %meter, y-position of Force exerted by Engine 1
Zapt1 = -1.9;       %meter, z-position of Force exerted by Engine 1

Xapt2 = 0;          %meter, x-position of Force exerted by Engine 2
Yapt2 = 7.94;       %meter, y-position of Force exerted by Engine 2
Zapt2 = -1.9;       %meter, z-position of Force exerted by Engine 2

% Other Constants

rho = 1.225;        %kg/m^3, Air density            ASSUME CONSTANT
g = 9.81;           %m/s^2, Gravitational Acceleration  ASSUME CONSTANT
depsda = 0.25;      %rad/rad, Change in downwash w.r.t. alpha
alpha_L0 = -11.5*pi/180; %rad, zero-lift AoA
n = 5.5;          %Slope of Linear Region of Lift Slope
a3 = -768.5;        %a3 Coefficient of CL_WB
a2 = 609.2;         %a2 Coefficient of CL_WB
a1 = -155.2;        %a1 Coefficient of CL_WB
a0 = 15.212;        %a0 Coefficient of CL_WB

alpha_switch = 14.5*pi/180; %rad, At this AoA angle,
                            %the CL_WB slope changes from linear to nonlinear

%% 1. Control Limits/ Saturation

% Note: These can alternately be enforced in Simulink
% Note: For simplicity of the model, this model assumes that aircraft can
%       instantaneously deflect the control surfaces which rate limits for
%       control surface deflection is neglected.
%       This might alter the fidelity of the model; however, this might be
%       taken into account in the future.

% Saturation of Aileron Deflection, d_A
u1min = -25*pi/180;
u1max =  25*pi/180;

% Saturation of Tailplane (Horizontal Stabilizer) Deflection, d_T
u2min = -25*pi/180;
u2max =  10*pi/180;

% Saturation of Rudder Deflection, d_R
u3min = -30*pi/180;
u3max =  30*pi/180;

% Saturation of Throttle 1, d_TH1
u4min = 0.5*pi/180;
u4max =  10*pi/180;

% Saturation of Throttle 2, d_TH2
u5min = 0.5*pi/180;
u5max =  10*pi/180;

if u1 > u1max
    u1 = u1max;
elseif u1 < u1min
    u1 = u1min;
end

if u2 > u2max
    u2 = u2max;
elseif u2 < u2min
    u2 = u2min;
end

if u3 > u3max
    u3 = u3max;
elseif u3 < u3min
    u3 = u3min;
end

if u4 > u4max
    u4 = u4max;
elseif u4 < u4min
    u4 = u4min;
end

if u5 > u5max
    u5 = u5max;
elseif u5 < u5min
    u5 = u5min;
end

%% 2. Intermediate Variables

Va = sqrt(x1^2 + x2^2 + x3^2);      %m/s, Airspeed

alpha = atan2(x3,x1);               %rad, Angle of Attack
beta = asin(x2/Va);                 %rad, Sideslip Angle

Q = 0.5*rho*Va^2;                   %N/m^2, Dynamic Pressure

omega_B_E_B = [x4;x5;x6];           %rad/s, Angular Velocity of Aircraft
                                    %       in Body Frame w.r.t Earth Frame
                                    %       expressed in Body Frame

V_B = [x1;x2;x3];                   %m/s, Velocity in Body Frame

%% 3.Aerodynamic Force Coefficients

% CL_WB, CL of wing-body
      
%      It is assumed that CL_WB has stall characteristic but no reverse
%      stall characteristic.

if alpha <= alpha_switch
    CL_WB = n*(alpha - alpha_L0);
else
    CL_WB = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end

% CL_t, CL of Tail
epsilon = depsda*(alpha - alpha_L0);                %rad, downwash angle
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;      %rad, AoA of Tail
CL_t = 3.1*St/S*alpha_t;

% Total Lift Coefficient in Stability Axis
CL = CL_WB + CL_t;

% Total Drag Coefficient in Stability Axis (Tail is Neglected)
CD = 0.13 + 0.07*(5.5*alpha+0.654)^2;

% Total Side Force Coefficient in Stability Axis
CY = -1.6*beta + 0.24*u3;
    
%% 4. Dimensional Aerodynamic Forces

% Actual Dimensional Forces in Stability Axis
F_A_S = [-CD; CY; -CL]*Q*S;

C_B_S = [cos(alpha), 0, -sin(alpha); 0,1,0; sin(alpha), 0, cos(alpha)];

% Actual Dimension Forces in Body Frame (Rotated)
F_A_B = C_B_S*F_A_S;

%% 5. Nondimensional Aerodynamic Moment Coefficients about AC in F_B

eta = [-1.4*beta;
       -0.59 - 3.1*(St*lt)/(S*cbar)*(alpha - epsilon);
       (1-alpha*180/(15*pi))*beta];
dCM_dX = [-11, 0, 5;
           0, -4.03*(St*lt^2)/(S*cbar^2), 0;
           1.7, 0, -11.5]*(cbar/Va); % NOT A FULL JACOBIAN, ONLY AERODYNAMICS
       
dCM_dU = [-0.6, 0, 0.22;
            0, (-3.1*(St*lt)/(S*cbar)), 0;
            0, 0, -0.63]; % NOT A FULL JACOBIAN, ONLY AERODYNAMICS

% Coefficient of Aerodynamic Moment about AC expressed in Body Frame
CM_AC_B = eta + dCM_dX*omega_B_E_B + dCM_dU*[u1;u2;u3];

%% 6. Dimensional Aerodynamic Moment about AC in F_B

% Aerodynamic Moment about AC expressed in Body Frame
M_A_AC_B = CM_AC_B*Q*S*cbar;

%% 7. Aerodynamic Moment about CG in F_B (Moment Transfer)

r_CG_B = [Xcg; Ycg; Zcg];
r_AC_B = [Xac; Yac; Zac];

%N*m, Aerodynamic Moment about CG expressed in F_B
M_A_CG_B = M_A_AC_B + cross(F_A_B, (r_CG_B - r_AC_B));

%% 8. Propulsion Effect (Engine Force and Moment)

% Fi = d_th_i*m*g;
F1 = u4*m*g; %N, Force which is exerted by engine 1
F2 = u5*m*g; %N, Force which is exerted by engine 2

% Force Vector due to Engine
F_E_B_1 = [F1;0;0];
F_E_B_2 = [F2;0;0];
F_E_B = [F1+F2; 0; 0];

Mu_B_1 = [Xcg - Xapt1; Yapt1 - Ycg; Zcg - Zapt1];
Mu_B_2 = [Xcg - Xapt2; Yapt2 - Ycg; Zcg - Zapt2];

M_E_CG_B_1 = cross(Mu_B_1, F_E_B_1);
M_E_CG_B_2 = cross(Mu_B_2, F_E_B_2);

% Propulsive/Engine Moment about CG in F_B
M_E_CG_B = M_E_CG_B_1 + M_E_CG_B_2;

%% 9. Gravity Effects
% Force due to gravity expressed in Body Frame
F_G_B = [-sin(x8); cos(x8)*sin(x7); cos(x8)*cos(x7)]*m*g; 

%% 10. Explicit First Order Form
I_B= m*[40.07, 0, -2.0923;
    0, 64, 0;
    -2.0923, 0, 99.92];                 %kg*m^2 Moment of Inertia Matrix

I_B_inv = (1/120000)*[0.0249836, 0, 0.000523151;
            0, 0.015625, 0;
            0.000523151, 0, 0.010019];  %1/m^2/kg Inverse of Moment of Inertia Matrix
% I_B_inv = I_B/eye(3);                   

F_B = F_G_B + F_E_B + F_A_B;            %Total Force

%[dx1;dx2;dx3] = [udot;vdot;wdot]
dx123 = (1/m)*F_B - cross(omega_B_E_B,V_B);

M_CG_B = M_E_CG_B + M_A_CG_B;           %Total Moment

%[dx4;dx5;dx6] = [pdot;qdot;rdot]
dx456 = I_B_inv*(M_CG_B - cross(omega_B_E_B, I_B*omega_B_E_B));

Euler = [1, sin(x7)*tan(x8), cos(x7)*tan(x8);...
         0, cos(x7), -sin(x7);...
         0, sin(x7)/cos(x8), cos(x7)/cos(x8)]; %Euler Kinematic Equations
     
%[dx7;dx8;dx9] = [phidot; thetadot; psidot]
dx789 = Euler*omega_B_E_B;

XDOT = [dx123;dx456;dx789]; %Total Equation of Motions

















