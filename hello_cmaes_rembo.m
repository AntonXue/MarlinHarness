function score = hello_cmaes_rembo(x)

global MATRIX_A;
global lbhd;
global ubhd;

hd_params = project_into_box(x');
score = hello(hd_params);

end

