
%{
  Toolbox used for rocket landing 
  reference: Silvia, Mo, Ian 
%}

classdef Lander < DynSys
    properties
        wRange % angle bounds
        speed % constant speed 
        dRange % distance
        dims % dimensions
    end

    methods
        function obj = Lander(x,wRange, speed, dRange, dims)
            % Lander class 
            % Dynamics: 
            % x1dot = v * cos(x3) + d1 
            % x2dot = v * sin(x3) + d2 
            % x3dot = u 
            %     u: [-wMax, wMax]
            %     d: [-dMax, dMax]

            % Inputs: 
            % x: state, [x,y]
            % wRange: [thetamin, thetamax]
            % speed: v 
            % dRange: disturbance bounds
            % Output: obj 
            if numel(x) ~= obj.nx
                error('Initial state does not have right dimension!');
            end
            
            if ~iscolumn(x)
                x = x';
            end
            
            if nargin < 2
                wRange = [-1 1];
            end
            
            if nargin < 3
                speed = 5;
            end
            
            if nargin < 4
                dRange = {[0;0;0];[0; 0; 0]};
            end
            
            if nargin < 5
                dims = 1:3;
            end

            if numel(wRange) <2
                wRange = [-wRange; wRange];
            end
            
            if ~iscell(dRange)
                dRange = {-dRange,dRange};
            end

            % Basic vehicle properties
            obj.pdim = [find(dims == 1) find(dims == 2)]; % Position dimensions
            %obj.hdim = find(dims == 3);   % Heading dimensions
            obj.nx = length(dims);
            obj.nu = 1;
            obj.nd = 3;
            
            obj.x = x;
            obj.xhist = obj.x;
            
            obj.wRange = wRange;
            %obj.thetaMax = thetaMax;
            obj.speed = speed;
            obj.dRange = dRange;
            obj.dims = dims;
        end
    end
end