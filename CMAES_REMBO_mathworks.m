function CMAES_REMBO_mathworks

rng default

timestamp = datestr(datetime('now'));
dirname=['exps-' timestamp];
mkdir(dirname);
    
max_evals = 5000;
num_of_exps = 1;

global nld
global nhd
global MATRIX_A            % random projection matrix

nld = 20;                    % number of low dims (effective)
nhd = 20;                % number of actual dims

% Define box in high-dim space
global lbhd;
global ubhd;
lb=5;
ub=154;
lbhd = ones(1,nhd)*lb;
ubhd = ones(1,nhd)*ub;

% Assuming the origin is in the box, half the spatial diagonal is the
% furthest in the Euclidean norm that a point can be from the origin.
r = norm(ubhd-lbhd)/2;

% The probability to pierce the box is 1-eps (and it linearly increases the
% search range in each low-dim coordinate).
eps = 0.1;

% Bounds in low-dim space (cf. Wang et al., Theorem 3). We increase them
% because they seemed incorrect in the paper.
lbld = -r/eps*sqrt(nld);
ubld = r/eps*sqrt(nld);

opts.LBounds=lb;
opts.UBounds=ub;
opts.MaxFunEvals=max_evals;

first_false_inds=[];
for exp=1:num_of_exps

MATRIX_A = randn(nhd,nld);

timestamp = datestr(datetime('now'));
log_filename = [dirname '/' timestamp '.txt'];

diary(log_filename);
diary on;

[xmin, ...      % minimum search point of last iteration
          fmin, ...      % function value of xmin
          counteval, ... % number of function evaluations done
          stopflag, ...  % stop criterion reached
          out, ...       % struct with various histories and solutions
          bestever ...   % struct containing overall best solution (for convenience)
    ] = cmaes( ...
    'hello_cmaes_rembo', ...    % name of objective/fitness function
    repmat(lb+ub,nld,1)/2, ...    % objective variables initial point, determines N
    ub-lb, ... % initial coordinate wise standard deviation(s)
    opts);

xmin'
fmin
counteval

diary off;
end

end

