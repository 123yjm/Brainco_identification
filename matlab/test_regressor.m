addpath('.');
q=rand(7,1)*0.1; qd=rand(7,1)*0.1; qdd=rand(7,1)*0.1;
try
  Y=revoarm_right_getY_cpp(q,qd,qdd);
  fprintf('Y size: %dx%d, max: %.4f, has_nan: %d\n', size(Y,1), size(Y,2), max(abs(Y(:))), any(isnan(Y(:))));
catch e
  fprintf('ERROR at line %d: %s\n', e.stack(1).line, e.message);
end
