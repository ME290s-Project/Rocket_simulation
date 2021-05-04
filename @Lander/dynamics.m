
%{
  Dynamics for Lander: 
  x' = v (0 first)
  y' = y'
  y'' = -g + N / M * F 

  control: 
  F, (v)
%}

function dx = dynamics(obj, ~, x,u,d)
  % d: initial state 
  if nargin < 5
    d = [0; 0; 0];
  end 

  if iscell(x)
    dx = cell(length(obj.dims),1);

    for i = 1:length(obj.dims)
      dx{i} = dynamics_cell_helper(obj,x,u,d,obj.dims,obj.dims(i));
    end 

  else
    % system dynamics here
    dx = zeros(obj.nx, 1);
    dx(1) = 0 +d(1);
    dx(2) = x(3) +d(2);
    dx(3) = u / 5 + d(3);
  end
end 


function dx = dynamics_cell_helper(obj,x,u,d, dims,dim)
  switch dim 
    case 1
      dx = 0 + d{1};
    case 2 
      dx = x{dims ==3} + d{2}; 
    case 3
      dx = u/ .5 + d{3} -10;
    otherwise
      error('Dimension wrong')
  end 
end 

