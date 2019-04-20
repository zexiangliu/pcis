% Experiment 1: takeover scenario
%               Compare bnd vel inv and cautious inv
clc;clear all;close all;
time_horizon = 15;
plot_stuff = 1;
is_sp_on = 0; % turn off the supervisor
is_print = 0; % no print
pic_path = '';
lead_dyn = "cau";
Iv = "cau";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 0; % do not update inv set with intention estimation
mpc = "takeover"; % takeover scenario
exp1 = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0,...
                                 'mpc',mpc);
% save exp1.mat exp1

%% Experiment 2: tailgate scenario
%               Compare bnd vel inv and cautious inv
clc;clear all;close all;
time_horizon = 15;
plot_stuff = 1;
is_sp_on = 0; % is supervisor on
is_print = 0; % print the fig or not
pic_path = '';
lead_dyn = "cau";
Iv = "cau";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 0;

exp2 = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0);
save exp2.mat exp2

%% Experiment 3: tailgate scenario
%               Compare bnd vel inv and cautious inv
clc;clear all;close all;
time_horizon = 15;
plot_stuff = 1;
is_sp_on = 1; % is supervisor on
is_print = 0; % print the fig or not
pic_path = '';
lead_dyn = "cau";
Iv = "bnd";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 1;
mpc = "tailgate";

[exp3] = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0,...
                                 'mpc',mpc);
% save exp3.mat exp3


save exp3.mat exp3

%% Experiment 4 "bad" driver with no sp

clc;clear all;close all;
time_horizon = 15;
plot_stuff = 1;
is_sp_on = 0; % is supervisor on
is_print = 0; % print the fig or not
pic_path = '';
lead_dyn = "cau";
Iv = "cau";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 0;

exp4 = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0);
save exp4.mat exp4

%% Experiment 5 "bad" driver with sp

clc;clear all;close all;
time_horizon = 15;
plot_stuff = 1;
is_sp_on = 1; % is supervisor on
is_print = 0; % print the fig or not
pic_path = '';
lead_dyn = "cau";
Iv = "bnd";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 1;

exp5 = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0);
save exp5.mat exp5

%% Experiment 8: tailgate scenario
%               Compare bnd vel inv and cautious inv
clc;clear all;close all;
time_horizon = 30;
plot_stuff = 1;
is_sp_on = 0; % is supervisor on
is_print = 0; % print the fig or not
pic_path = '';
lead_dyn = "agg";
Iv = "bnd";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 0;

exp8 = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0);
save exp8.mat exp8

%% Experiment 9: tailgate scenario
%               Compare bnd vel inv and cautious inv
clc;clear all;close all;
time_horizon = 30;
plot_stuff = 1;
is_sp_on = 1; % is supervisor on
is_print = 0; % print the fig or not
pic_path = '';
lead_dyn = "agg";
Iv = "bnd";
x0 = [30, 0.1, 40, 25]';
sp_with_int = 1;
mpc = "tailgate";

[exp9] = simulate_single('time_horizon',time_horizon,...
                                 'plot_stuff',plot_stuff,...
                                 'is_sp_on',is_sp_on,...
                                 'is_print',is_print,...
                                 'pic_path',pic_path,...
                                 'lead_dyn',lead_dyn,...
                                 'Iv',Iv,...,
                                 'sp_with_int',sp_with_int,...
                                 'x0',x0,...
                                 'mpc',mpc);

save exp9.mat exp9

