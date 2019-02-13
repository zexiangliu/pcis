% load CIS_cau_XU_R7.mat
xu = [25.9355
        0.6274
       10.1243
       24.7030
        3.0000
        1.8000];

xA3 = xu(1:4);
u = xu(5:6);
containsPolyUnion(CIS_cau,xA3)
containsPolyUnion(preXU_cau,xu)
%%
con.K_cau*xA3-con.K_cau(4)*con.vL_des+0.15
%% 
% [ pwd_A , pwd_C ] = get_takeover_pwd_7regions( );
pwd_C.dyn_list{1}.A*xA3 + pwd_C.dyn_list{1}.B*u + pwd_C.dyn_list{1}.F+pwd_C.dyn_list{1}.Fd{3}*0.15