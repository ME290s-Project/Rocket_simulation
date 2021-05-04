function f = dynamic_3T(x,u)
% dynamics_approaching - system dynamics for mode "approaching" for the 
%                        spacecraft rendevous benchmark described in [1] 
%
% Syntax:  
%    f = dynamics_approaching(x,u)
%
% Inputs:
%    x - state vector
%    u - input vector
%
% Outputs:
%    f - time-derivate of the system state
%
% References: 
%   [1] N. Chan et al. "Verifying safety of an autonomous spacecraft 
%       rendezvous mission (Benchmark proposal)"  

% Author:       Niklas Kochdumper
% Written:      23-December-2019
% Last update:  ---
% Last revision:---

%------------- BEGIN CODE --------------


g=9.81;
m0=5.5e5;
Ar=100;
Ag=36;
rou=1.22*1.29;
L=70;
ST=46000;
Fs=7.6e6;
N=3;
m=x(7);

f(1,1) = x(4);

f(2,1) = x(5);

f(3,1) = x(6);

f(4,1) = (9*Fs*sind(x(3)-u(1))-rou*Ar*(x(4)^2)/2   -u(2)*(rou*Ag*(x(4)^2)/(2)) )/x(7);

f(5,1) = (9*Fs*cosd(x(3)-u(1))-rou*Ar*(x(5)^2)/2    -u(2)*(rou*Ag*(x(5)^2)/(2))    )/x(7)-g;

f(6,1) =-N*Fs*0.5*sind(u(1))/((1/12)*x(7)*L^2);

f(7,1) =-N*Fs/ST;


end

%------------- END OF CODE --------------