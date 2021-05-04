function f = dynamic_glide(x,u)
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
Ar=100;
Ag=36;
rou=1.22*1.29;
L=70;
ST=46000;
Fs=7.6e6;

N=3;
F=7.6e6;
ST=46000;
% differential equations
% sigma=u(1);
% k=u(2);
% f(1,1) = x(4); % f
% f(2,1) =  x(5); % dy
% f(3,1) = x(6); % d_theta
% f(4,1) = (k*N*F*sind(x(3)+sigma)/m)-(rou*Ar*(x(4)^2)/(2*m)); % df
% f(5,1) = (k*N*F*cosd(x(3)+sigma)/m)-g-(rou*Ar*(x(5)^2)/(2*m)); % ddy
% f(6,1) = -k*N*F*0.5*L*sin(sigma)/((1/12)*m*L^2); % dd_theta
m=x(7);
f(1,1) = x(4); % f
f(2,1) =  x(5); % dy
f(3,1) = x(6); % d_theta
f(4,1) = 0.01*(-(rou*Ar*(x(4)^2)/(2*m))-u(2)*(rou*Ag*(x(4)^2)/(2*m))); % df
f(5,1) = -g-0.01*((rou*Ar*(x(5)^2)/(2*m))-u(2)*(rou*Ag*(x(5)^2)/(2*m))); % ddy
f(6,1) = 0;
f(7,1) = 0;% dm

end

%------------- END OF CODE --------------