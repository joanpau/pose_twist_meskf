function handle = plot3c(x,y,z,c,varargin)
% PLOT3C Plot colored line.

handle = surface(
  [x(:), x(:)], ...
  [y(:), y(:)], ...
  [z(:), z(:)], ...
  [c(:), c(:)], ... % the colour vector
  'EdgeColor', 'flat', ...  % colour the edges with flat shading according to the colour vector
  'FaceColor', 'none', ...  % don't colour the face (which is zero area)
  varargin{:}  
);

end
