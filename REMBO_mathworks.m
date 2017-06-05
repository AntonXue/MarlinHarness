function REMBO_mathworks

rng default

timestamp = datestr(datetime('now'));
dirname=['exps-' timestamp];
mkdir(dirname);
    
max_evals = 100;
num_of_exps = 10;

global nld
global nhd
global MATRIX_A            % random projection matrix

nld = 20;                    % number of low dims (effective)
nhd = 20;                % number of actual dims

% Define box in high-dim space
global lbhd;
global ubhd;
lbhd = zeros(1,nhd);
ubhd = ones(1,nhd)*300;

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

variables=[];
for i=0:nld-1
    variables=[variables optimizableVariable(sprintf('param%d', i),[lbld ubld],'Type','real')];
end

first_false_inds=[];
for exp=1:num_of_exps

MATRIX_A = randn(nhd,nld);

timestamp = datestr(datetime('now'));
log_filename = [dirname '/' timestamp '.txt'];

diary(log_filename);
diary on;

results=bayesopt(@f,variables,...
    'AcquisitionFunctionName', 'expected-improvement', ...
    'MaxObjectiveEvaluations', max_evals, ...
    'IsObjectiveDeterministic', true, ...
    'PlotFcn', []);

obj_min_trace=results.ObjectiveMinimumTrace
num_false=size(results.ObjectiveMinimumTrace(results.ObjectiveMinimumTrace<0),1)
a=min(find(results.ObjectiveMinimumTrace<0));
if isempty(a)
    first_false_ind=0
else
    first_false_ind=a
end
first_false_inds=[first_false_inds first_false_ind];
diary off;
end

results_filename = [dirname '/results.txt'];
diary(results_filename);
diary on;
num_falsifying_experiments=size(find(first_false_inds>0),2)
first_false_inds
mean_first_false=mean(first_false_inds(first_false_inds>0))
std_first_false=std(first_false_inds(first_false_inds>0))
diary off;

end

function score = f(x)

global MATRIX_A;
global nld;
global input_param;

params=[];
for i=0:nld-1
    varname=sprintf('param%d',i);
    params=[params eval(['x.' varname])];
end

hd_params = project_into_box(params);
score = hello(hd_params');

end


