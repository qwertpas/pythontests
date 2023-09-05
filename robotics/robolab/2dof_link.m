% planar manipulator dynamics

nJoints = 2;    % number of joints

% define symbolic variables
%   m:  mass of link
%   Ic: inertia at center of mass of link
%   Lc: distance from parent joint to 
%       center of mass of link
%   Ir: rotor inertia of motor
%   N:  gear ratio of actuator
%   q:  joint angles
%   qd: joint velocity
%   qdd:joint acceleration
%   tau:joint torques
syms N m Ic Lc Ir L q qd qdd tau [nJoints 1] real

% unit vector (global coordinate system)
xhat = [1;0]; yhat = [0;1];

% absolute joint angles
a1 = q1;                 
a2 = q1+q2;

% unit vector of limb directions
e1  = cos(a1)*xhat + sin(a1)*yhat;
e2  = cos(a2)*xhat + sin(a2)*yhat;

% center of mass positions of link
G1  = Lc1*e1;
G2  = L1*e1 + Lc2*e2;

% a neat trick to get symbolic time-derivative
derivative = @(x) jacobian(x, [q;qd])*[qd; qdd];

% time-derivatives of center of mass positions
dG1 = derivative(G1);
dG2 = derivative(G2);

% time-derivatives of absolute joint angles
da1 = derivative(a1);
da2 = derivative(a2);

% Kinetic energy of structural parts
T1  = 0.5*(m1*dot(dG1,dG1)  + m2*dot(dG2,dG2));       % rectilinear
T2  = 0.5*(Ic1*dot(da1,da1) + Ic2*dot(da2,da2));     % rotational

% Kinetic energy of actuators
dr1 = N1*qd1;   % rotors run x(gearRatio) times faster than joints
dr2 = N2*qd2;   
T3  = 0.5*(Ir1*dot(dr1,dr1) + Ir2*dot(dr2,dr2));

% Define Lagrangian
T = T1 + T2 + T3;   % Kinetic Energy
V = 0;              % Potential Energy 
Lag = T-V;          % Lagrangian

% Define Generalized Work
dW = dot(tau, qd);

% Euler-Lagrangian Equation of Motion (EoM)
EoM_LHS = sym(zeros(nJoints,1)); 
Lag = simplify(Lag);
for jj = 1:nJoints
    partial_dq_ = diff(Lag, qd(jj));
    partial_q_ = diff(Lag, q(jj));
    EoM_LHS(jj,1) = derivative(partial_dq_) - partial_q_;
end
EoM_RHS = transpose(jacobian(dW, qd)); 

% Decompose EoM 
% EoM_LHS: M(q)*qdd + c(q,qd) 
M = jacobian(EoM_LHS, qdd);     % hessian(T, qd) also works
c = EoM_LHS - M*qdd;
