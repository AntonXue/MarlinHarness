function CMAES_mathworks

rng default

timestamp = datestr(datetime('now'));
dirname=['exps-' timestamp];
mkdir(dirname);
    
max_evals = 100;
num_of_exps = 5;

global n;
n = 20;                % number of actual dims

% Define box in high-dim space
global lb;
global ub;
lb = 50;
ub = 150;

opts.LBounds=lb;
opts.UBounds=ub;

first_false_inds=[];
for exp=1:num_of_exps

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
    'hello', ...    % name of objective/fitness function
    repmat(lb+ub,n,1)/2, ...    % objective variables initial point, determines N
    opts)

%obj_min_trace=results.ObjectiveMinimumTrace
%num_false=size(results.ObjectiveMinimumTrace(results.ObjectiveMinimumTrace<0),1)
%a=min(find(results.ObjectiveMinimumTrace<0));
%if isempty(a)
%    first_false_ind=0
%else
%    first_false_ind=a
%end
%first_false_inds=[first_false_inds first_false_ind];
diary off;
end

%results_filename = [dirname '/results.txt'];
%diary(results_filename);
%diary on;
%num_falsifying_experiments=size(find(first_false_inds>0),2)
%first_false_inds
%mean_first_false=mean(first_false_inds(first_false_inds>0))
%std_first_false=std(first_false_inds(first_false_inds>0))
%diary off;

end

%function score = f(x)
%
%global n;
%
%params=[];
%for i=0:n-1
%    varname=sprintf('param%d',i);
%    params=[params eval(['x.' varname])];
%end
%
%score = hello(params);
%
%end


