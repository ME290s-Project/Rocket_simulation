
%{
  dynamics for Lander
  x' = v* cos(theta) + dx 
  y' = v* sin(theta) + dy 
  theta' = w + dtheta

  control: 
  u = w 
%}

function dx = dynamics(obj, ~, x,u,d)
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
    dx(1) = obj.speed * cos(x(3)) +d(1);
    dx(2) = obj.speed * sin(x(3)) +d(2);
    dx(3) = u + d(3);
  end
end 


function dx = dynamics_cell_helper(obj,x,u,d, dims,dim)
  switch dim 
    case 1
      dx = obj.speed * cos(x{dims ==3}) + d{1};
    case 2 
      dx = obj.speed * sin(x{dims ==3}) + d{2}; 
    case 3
      dx = u + d{3};
    otherwise
      error('Dimension wrong')
  end 
end 

