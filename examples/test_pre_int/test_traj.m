% close all;
% 
figure;
plot(time,vEgo,'linewidth',2); hold on;
plot(time,yEgo,'linewidth',2);
plot(time,h,'linewidth',2);
plot(time,vLead,'linewidth',2);

plot(time(778),vEgo(778),'o'); hold on;
plot(time(778),yEgo(778),'o');
plot(time(778),h(778),'o');
plot(time(778),vLead(778),'o');


legend('v_e','y_e','h','v_l')
xlabel('time');ylabel('state');
% 
% figure;
% plot(h,yEgo);

% % plot(vEgo); hold on;
% plot(yEgo);
% hold on;
% plot(h);
% % plot(vLead);
% % legend('v_e','y_e','h','v_l')
% legend('y_e','h');

%%
% bnd_idx_list = [];
% for i = 1:length(time)-1
%     x = get_from_traj(i);
%     i
%     if ~containsPolyUnion(CIS_cau,x) || ~containsPolyUnion(CIS_ann,x)...
%        || ~containsPolyUnion(CIS_ac,x)
%         bnd_idx_list = [bnd_idx_list,i-1];
%     end
% end

%%

% bnd_list =[];
% 
% for i = 1:length(bnd_idx_list)-1
%     if(bnd_idx_list(i+1) - bnd_idx_list(i)~=1)
%         bnd_list = [bnd_list, bnd_idx_list(i+1)];
%     end
% end

%%

u_cau = cell(length(time)-1,1);
u_ann = cell(length(time)-1,1);
u_ac = cell(length(time)-1,1);
for i = 1:length(time)-1
    x = get_from_traj(i);
    i
    if containsPolyUnion(CIS_cau,x)
        disp("cau");
         u_cau{i} = computeU(x, preXU_cau);
    end
    
    if containsPolyUnion(CIS_ann,x)
        disp("ann");
         u_ann{i} = computeU(x, preXU_ann);
     end
    
%     if containsPolyUnion(CIS_ac,x)
%         disp("ac");
%          u_ac{i} = computeU(x, preXU_ac);
%     end
end


%%

u1_cau = zeros(length(time)-1,2);
u2_cau = u1_cau;
for i = 1:length(u_cau)
    if ~isempty(u_cau{i})
        u1_cau(i,:) = u_cau{i}(1,:);
        u2_cau(i,:) = u_cau{i}(2,:);
    else
        u1_cau(i,:) = zeros(1,2);
        u2_cau(i,:) = zeros(1,2);
    end
end

u1_ann = zeros(length(time)-1,2);
u2_ann = u1_ann;
for i = 1:length(u_ann)
    if ~isempty(u_ann{i})
        u1_ann(i,:) = u_ann{i}(1,:);
        u2_ann(i,:) = u_ann{i}(2,:);
    else
        u1_ann(i,:) = zeros(1,2);
        u2_ann(i,:) = zeros(1,2);
    end
end

figure;
plot(time(1:end-1),u1_cau(:,1))
hold on;
plot(time(1:end-1),u1_cau(:,2))
plot(time(1:end-1),u1_ann(:,1))
plot(time(1:end-1),u1_ann(:,2))
plot(time(1:end-1),aEgo,'x');

figure;
plot(time(1:end-1),u2_cau(:,1))
hold on;
plot(time(1:end-1),u2_cau(:,2))
plot(time(1:end-1),u2_ann(:,1))
plot(time(1:end-1),u2_ann(:,2))
plot(time,steering,'x');

%%

parfor i = 1:length(time)-1
    x = get_from_traj(i);
    i
    if ~containsPolyUnion(CIS_cau,x) && ~containsPolyUnion(CIS_ann,x)
        continue;
    end
        
    if containsPolyUnion(CIS_ac,x)
        disp("ac");
        u1_max = min([u1_cau(i,2);u1_ann(i,2)]);
        u1_min = max([u1_cau(i,1);u1_ann(i,1)]);
        u2_max = min([u2_cau(i,2);u2_ann(i,2)]);
        u2_min = max([u2_cau(i,1);u2_ann(i,1)]);
        u_1 = [(u1_max+u1_min)/2; (u2_max+u2_min)/2];
        u_2 = [u1_max*1/4+u1_min*3/4; u2_max*1/4+u2_min*3/4];
        u_3 = [u1_max*3/4+u1_min*1/4; u2_max*1/4+u2_min*3/4];
        u_4 = [u1_max*3/4+u1_min*1/4; u2_max*3/4+u2_min*1/4];
        u_5 = [u1_max*1/4+u1_min*3/4; u2_max*3/4+u2_min*1/4];
        u = [u_1, u_2, u_3, u_4 ,u_5];
        u_ac{i} = computeU2(x, u, preXU_ac);
    end
end



%%

u1_ac = zeros(length(time)-1,2);
u2_ac = u1_ac;
for i = 1:length(u_ac)
    if ~isempty(u_ac{i})
        u1_ac(i,:) = u_ac{i}(1,:);
        u2_ac(i,:) = u_ac{i}(2,:);
    else
        u1_ac(i,:) = zeros(1,2);
        u2_ac(i,:) = zeros(1,2);
    end
end
%%
figure;
hold on;
% 
% plot(time(1:end-1),u1_cau(:,2),'linewidth',1)
% plot(time(1:end-1),u1_ann(:,2),'linewidth',1)
plot(time(1:end-1),u1_ac(:,2),'-r','linewidth',1)
% 
plot(time(1:end-1),aEgo,'x','markersize',6);
% 
% plot(time(1:end-1),u1_cau(:,1),'linewidth',1)
% % plot(time(1:end-1),u1_ann(:,1),'linewidth',1)
plot(time(1:end-1),u1_ac(:,1),'-b','linewidth',1)
xlabel('time');ylabel('input');
% 


figure;
% plot(time(1:end-1),u2_cau(:,1))
hold on;
% plot(time(1:end-1),u2_cau(:,2))
% plot(time(1:end-1),u2_ann(:,1))
% plot(time(1:end-1),u2_ann(:,2))
plot(time(1:end-1),u2_ac(:,2),'r','linewidth',1)
plot(time(1:end-1),u2_ac(:,1),'b','linewidth',1)
plot(time,steering,'x','markersize',5);
% plot(time,yEgo,'x','markersize',5);
legend("upper bnd","lower bnd","real steering","y_e")
xlabel('time');ylabel('input');


%%


for i = 778
    x = get_from_traj(i);
    i
      plotU(x, preXU_ac);
      plotU(x, preXU_ann);
      plotU(x, preXU_cau);

end