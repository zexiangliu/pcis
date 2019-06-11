clear;close all;clc;

load seed_set

%%
rhoPre = 0;
dom = Polyhedron('UB',[5000, 5000, inf, inf],'LB', [-5000, -5000, -inf, -inf]);
dyn = PwDyn(dom,{dom},{dyn2});
CIS = expand(dyn, S, C, rhoPre,'plot_stuff','debug','max_iter',max_iter);