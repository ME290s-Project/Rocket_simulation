
%{
  A matlab implementation with given toolbox 
%}

% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then    optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVOverTime' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'minVOverTime' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

%% grid 
grid_min = [-5; -5; -pi];
grid_max = [5; 5; pi];
N = [41;41;41];
pdDims = 3; 
g = createGrid(grid_min, grid_max,N,pdDims);

%% target set 
R = 1; 
data0 = shapeCylinder(g,3,[0;0;0],R);  % also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector 
t0 = 0;
tMax = 2; 
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters 
% input bounds 
speed = 1;
wMax = 1; 

% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory


% 2. Run BRS with goal, then    optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
compTraj = false;
uMode = 'min';

%% Pack problem parameters 
% Define dynamic system 
dCar = DubinsCar([0,0,0],wMax,speed); 

% put grid and dynamic system into schemeData
schemeData.grid = g; 
schemeData.dynSys = dCar; 
schemeData.accuracy = 'low';
schemeData.uMode = uMode; 

% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
dMax = [.25, .25, 0];
if strcmp(uMode, 'min')
  dMode = 'min';
else 
  dMode = 'max';
end 
distCar = DubinsCar([5,5,0],dMax, speed);
schemeDate.distSys = distCar;
schemeData.dMode = dMode;

%% additive random noise
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

%HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
% Try other noise coefficients, like:
%    [0.2; 0; 0]; % Noise on X state
%    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
%    [0.2;0.2;0.5]; % Coupled noise on all states
%    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise

%% If you have obstacles, compute them here

%% Compute value function

% HJIextraArgs.visualize = true; % show plot 
HJIextraArgs.visualize.valueSet = 1; 
HJIextraArgs.visualize.initialValueSet = 1; 
HJIextraArgs.visualize.figNum = 1;  %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; % delete previous plot as you update 

% uncomment if you want to see a 2D slice
% HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; %plot x, y
% HJIextraArgs.visualize.plotData.projpt = [0]; %project at theta = 0
% HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

[data, tau2, extraOuts] = HJIPDE_solve(data0, tau, schemeData, 'none',HJIextraArgs);

if compTraj

    % set initial state 
    xinit = [1,1,pi];
    figure(1);
    clf
    h = visSetIm(g,data(:,:,:,end));
    h.FaceAlpha = 0.3;
    hold on 
    s = scatter3(xinit(1),xinit(2),xinit(3));
    s.SizeData = 70;

    % check if this initial state is in the BRS/BRT
    value = eval_u(g,data(:,:,:,end),xinit);

    if value <= 0  % if initial state is in the BRS/BRT
        % find optimal trajectory 
        dCar.x = xinit;
        TrajExtraArgs.uMode = uMode;
        TrajExtraArgs.visualize = true;
        TrajExtraArgs.dMode = dMode; % added for distance
        TrajExtraArgs.fig_num = 2;  % figure number 

        % we want to see the first 2 dimensions
        TrajExtraArgs.projDim = [1,1,0];

        % flip data time points -> start from the beginning
        dataTraj = flip(data,4);
        [traj, traj_tau] = computeOptTraj(g, dataTraj,tau2,dCar, TrajExtraArgs);
    else
      error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
    end 
end 