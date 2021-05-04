function HA = rocket_levelSet_3()
% spacecraft_levelSet- spacecraft rendevous benchmark described in [1] with
%                      nonlinear guard sets
%
% Syntax:  
%    HA = spacecraft_levelSet()
%
% Inputs:
%    no
%
% Outputs:
%    HA - hybrid automaton object
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
ref=[ 86429.8641152348          114856.549165714          66.5598576030847          889.260095894018          571.431601200797         0.450190824059577          326956.521739155];
benchmark=(ref(7)*g*ref(2)+0.5*ref(7)*(ref(4)^2+ref(5)^2)/ref(7))*0.1;
KL=1e5;
% Mode Approaching --------------------------------------------------------

%       x'  = vx 
%       y'  = vy 
%       vx' = (n^2 + K1_11/m_c)*x + (2*n + K1_14/m_c)*vy + K1_12/m_c * y + K1_13/m_c * vx + mu/r^2 - mu/sqrt((r+x)^2-y^2)^3 * (r+x)
%       vy' = (n^2 + K1_22/m_c)*y + (K1_23/m_c -2*n)*vx + K1_21/m_c * x + K1_24/m_c * vy - mu/sqrt((r+x)^2-y^2)^3 * y

% system dynamics
dynamics1 = nonlinearSys(@dynamics_FT,7,2); 

% invariant:  x^2 + y^2 >= 100^2
syms x y theta vx vy d_theta m;
inv = levelSet(y - KL,[x;y;theta;vx;vy; d_theta; m],'<='); 
% inv = levelSet(g*y+0.5*(vx^2+vy^2)-benchmark,[x;y;theta;vx;vy; d_theta; m],'<='); 
% transition: x^2 + y^2 == 100^2
resetA = eye(7);
resetA(6,6)=0;
resetb = zeros(7,1);
resetb(7)=-96570;
reset1 = struct('A', resetA, 'b', resetb);

guard1 = levelSet(y - KL,[x;y;theta;vx;vy; d_theta; m],'==');

trans{1} = transition(guard1, reset1, 2);

% location
loc{1} = location('S1', inv, trans, dynamics1);


% Mode Rendezvous Attempt -------------------------------------------------

%       x'  = vx
%       y'  = vy 
%       vx' = (n^2 + K2_11/m_c)*x + (2*n + K2_14/m_c)*vy + K2_12/m_c * y + K2_13/m_c * vx + mu/r^2 - mu/sqrt((r+x)^2-y^2)^3 * (r+x)
%       vy' = (n^2 + K2_22/m_c)*y + (K2_23/m_c -2*n)*vx + K2_21/m_c * x + K2_24/m_c * vy - mu/sqrt((r+x)^2-y^2)^3 * y
       
% system dynamics
dynamics2 = nonlinearSys(@dynamic_glide,7,2); 

% invariant: x^2 + y^2 < 100^2
syms x y theta vx vy d_theta m;
inv = levelSet(-y+KL ,[x;y;theta;vx;vy; d_theta; m],'<='); 

% location


resetA = eye(7);
resetA(6,6)=0;
resetA(3,3)=-1;
resetb = zeros(7,1);
reset2 = struct('A', resetA, 'b', resetb);

% guard2 = levelSet(y-9e4,[x;y;theta;vx;vy; d_theta; m],'==');
% guard2 = levelSet(9.81*y+0.5*(vx^2+vy^2)-benchmark,[x;y;theta;vx;vy; d_theta; m],'==');
guard2 = levelSet(y-0.95*KL,[x;y;theta;vx;vy; d_theta; m],'==');
trans2{1} = transition(guard2, reset2, 3);
loc{2} = location('S2', inv, trans2, dynamics2);


% reentry phase
dynamics3 = nonlinearSys(@dynamic_reentry,7,2); 
syms x y theta vx vy d_theta m;
inv = levelSet(-y+2e4,[x;y;theta;vx;vy; d_theta; m],'<='); 
% Hybrid Automaton --------------------------------------------------------
resetA = eye(7);
resetA(6,6)=0;
resetb = zeros(7,1);
reset3 = struct('A', resetA, 'b', resetb);
guard3 = levelSet(y-2e4,[x;y;theta;vx;vy; d_theta; m],'==');
trans3{1} = transition(guard3, reset3, 4);
loc{3} = location('S3', inv, trans3, dynamics3);



% 3T phase
dynamics4 = nonlinearSys(@dynamic_3T,7,2); 
syms x y theta vx vy d_theta m;
inv = levelSet(y-2e4,[x;y;theta;vx;vy; d_theta; m],'<='); 
% Hybrid Automaton --------------------------------------------------------
trans4 = {};
loc{4} = location('S4', inv, trans4, dynamics4);
HA = hybridAutomaton(loc);




end

%------------- END OF CODE --------------