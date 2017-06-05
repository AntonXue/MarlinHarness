function hd_params = project_into_box(params)

global MATRIX_A;
global lbhd;
global ubhd;

% Project point in low-dim space into high-dim space, then into box
hd_params=(MATRIX_A*params')';
hd_params(hd_params<lbhd)=lbhd(hd_params<lbhd);
hd_params(hd_params>ubhd)=ubhd(hd_params>ubhd);

end
