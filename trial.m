
%{
  A matlab implementation with given toolbox 
%}


%% grid 
grid_min = [-100; 0; -100];
grid_max = [100;110;0];
N = [51,51,51];
pdDims = 3; 
g = createGrid(grid_min, grid_max,N,pdDims);

%% target set 
R = 5; 
data0 = shapeCylinder(g,3,[0;100;-20],R);  % also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector 
t0 = 0;
tMax = 5; 
dt = 0.1;
tau = t0:dt:tMax;

%% problem parameters 
% input bounds 
speed = 10;
wMax = 2; 

compTraj = false;
uMode = 'max';

%% Pack problem parameters 
% Define dynamic system 
lander = Lander([0,100,-20],wMax,speed); 

% put grid and dynamic system into schemeData
schemeData.grid = g; 
schemeData.dynSys = lander; 
schemeData.accuracy = 'low';
schemeData.uMode = uMode; 

%% If you have obstacles, compute them here

%% Compute value function

% HJIextraArgs.visualize = true; % show plot 
HJIextraArgs.visualize.valueSet = 1; 
HJIextraArgs.visualize.initialValueSet = 1; 
HJIextraArgs.visualize.figNum = 1;  %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; % delete previous plot as you update 
HJIextraArgs.addGaussianNoiseStandardDeviation = [0.1; 0; 0.5];

% uncomment if you want to see a 2D slice
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

[data, tau2, extraOuts] = HJIPDE_solve(data0, tau, schemeData, 'none',HJIextraArgs);
