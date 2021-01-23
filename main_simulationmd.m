addpath('include/');

%% load data and add noise
data_location = 'simulation.mat';
load(strcat('data/', data_location));

if ~exist('commonview.mat')
    
    commonview = zeros(length(node_edge.dpos), length(node_edge.dpos));
    %commonview = zeros(max(lm_edge.id1), max(lm_edge.id1));

    tmp = cell(length(node_edge.dpos),1);
    commonviewFrame = cell(length(node_edge.dpos),1);
    for i = 1:length(node_edge.dpos)
        tmp{i} = lm_edge.id2(find(lm_edge.id1==i));
    end

    for i = 1:length(tmp)
        i
        t1 = tmp{i};

        for j = (i+1):length(tmp)

            t2 = tmp{j};
            t3 = intersect(t1, t2);
            if ~isempty(t3)
                commonview(j,i) = 1;
                commonview(i,j) = commonview(j,i);
            end
        end

        commonviewFrame{i} = find(commonview(i,:)==1);
    end
    
    measurements_Commonview = cell(length(lm_edge.id1), 1);
    for i = 1:length(lm_edge.id1) 
        
        idx_frame = lm_edge.id1(i);
        commonviewKeyFrame = find(commonview(idx_frame, :)==1);
            
        measurement_Tmp = [i];
        for j = 1:length(commonviewKeyFrame)
            tmp = find(lm_edge.id1==commonviewKeyFrame(j));
            measurement_Tmp = [measurement_Tmp tmp];
        end
        measurements_Commonview{i} = measurement_Tmp;
    end
    
    
    save commonview commonview
    save commonviewFrame commonviewFrame
    save measurements_Commonview measurements_Commonview
else
    load commonview
    load commonviewFrame
    load measurements_Commonview
end

lm_edge.commonview = commonview;
lm_edge.commonviewFrame = commonviewFrame;
lm_edge.measurements_Commonview = measurements_Commonview;

% Change the random number matrix size to generate according to size of the
% data matrix
data_size = size(lm_edge.dpos);
lm_edge.dpos = lm_edge.dpos + randn(data_size)*0.1;
node_edge.dpos = node_edge.dpos+ randn(size(node_edge.dpos))*0.02;
node_edge.dtheta = node_edge.dtheta + randn(size(node_edge.dtheta))*0.01;

%% plot dataset
fig = figure;
set(fig,'Position', [100, 100, 400, 300]);
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])

plot(truth_traj(:,1),truth_traj(:,2),'k');
hold on; 
        
label={'trajectory'};
for i=1:5
    idx = truth_objects(:,3)==i;
    if sum(idx)>0
        label{end+1}=['class ' num2str(i)];
        switch i
        case 1
            plot(truth_objects(idx,1),truth_objects(idx,2),'bo','MarkerFaceColor','b');            
        case 2
            plot(truth_objects(idx,1),truth_objects(idx,2),'rd','MarkerFaceColor','r');
        case 3
            plot(truth_objects(idx,1),truth_objects(idx,2),'ms','MarkerFaceColor','m');
        case 4
            plot(truth_objects(idx,1),truth_objects(idx,2),'g^','MarkerFaceColor',[0.2 1 0.2]);
        case 5
            plot(truth_objects(idx,1),truth_objects(idx,2),'yp','MarkerFaceColor',[1 0.7 0.3],...
                                        'MarkerSize',15);
        end               
   end
end
axis equal; axis off;
title("True trajectory")
legend(label);
        
%% non-parametric
% pr_DP = Processermd();
% pr_DP = pr_DP.setupobjects(node_edge,lm_edge);
% pr_DP = pr_DP.optimizeDP(6);
% [Iodom_NP, Iobj_NP]=pr.computeEntropy();
% [Eodom_DP, Eobj_DP]=pr_DP.computeError(truth_traj',truth_objects');
% pr_DP.plot();

% pr_HDP = Processermht();
% pr_HDP = pr_HDP.setupobjects(node_edge,lm_edge);
% pr_HDP = pr_HDP.optimizeHDP(6);
% [Iodom_NP, Iobj_NP]=pr.computeEntropy();
% [Eodom_HDP, Eobj_HDP]=pr_HDP.computeError(truth_traj',truth_objects');
% pr_HDP.plot();

pr_MHDP = Processermd();
pr_MHDP = pr_MHDP.setupobjects(node_edge,lm_edge);
pr_MHDP = pr_MHDP.optimizeMHDP(6);
%[Iodom_NP, Iobj_NP]=pr.computeEntropy();
[Eodom_MHDP, Eobj_MHDP]=pr_MHDP.computeError(truth_traj',truth_objects');
pr_MHDP.plot();

pr_MHDP_Chi = Processermd();
pr_MHDP_Chi = pr_MHDP_Chi.setupobjects(node_edge,lm_edge);
pr_MHDP_Chi = pr_MHDP_Chi.optimizeMHDP_Chi(6);
%[Iodom_NP, Iobj_NP]=pr.computeEntropy();
[Eodom_MHDP_Chi, Eobj_MHDP_Chi]=pr_MHDP_Chi.computeError(truth_traj',truth_objects');
pr_MHDP_Chi.plot();

% algo = {'DP';'HDP';'MHDP'};
% mean_odom = mean( sqrt([Eodom_DP;Eodom_HDP;Eodom_MHDP]),2);
% cum_odom = sum( sqrt([Eodom_DP;Eodom_HDP;Eodom_MHDP]),2);
% no_msts = sum( [pr_DP.measurements.obj_id;pr_HDP.measurements.obj_id; ...
%     pr_MHDP.measurements.obj_id]>0,2);
% no_obj = [length(pr_DP.objects); length(pr_HDP.objects); length(pr_MHDP.objects)];
% err_obj = [mean(sqrt(Eobj_DP)); mean(sqrt(Eobj_HDP)); mean(sqrt(Eobj_MHDP))];
% T = table(mean_odom,cum_odom,no_msts,no_obj,err_obj,...
%     'RowNames',algo);


%%
% algo = {'DP';'MHDP'};
% mean_odom = mean( sqrt([Eodom_DP;Eodom_MHDP]),2);
% cum_odom = sum( sqrt([Eodom_DP;Eodom_MHDP]),2);
% no_msts = sum( [pr_DP.measurements.obj_id; ...
%    pr_MHDP.measurements.obj_id]>0,2);
% no_obj = [length(pr_DP.objects); length(pr_MHDP.objects)];
% err_obj = [mean(sqrt(Eobj_DP)); mean(sqrt(Eobj_MHDP))];
% T = table(mean_odom,cum_odom,no_msts,no_obj,err_obj,...
%    'RowNames',algo);


algo = {'MHDP';'MHDP_Chi'};
mean_odom = mean( sqrt([Eodom_MHDP;Eodom_MHDP_Chi]),2);
cum_odom = sum( sqrt([Eodom_MHDP;Eodom_MHDP_Chi]),2);
no_msts = sum( [pr_MHDP.measurements.obj_id; ...
   pr_MHDP_Chi.measurements.obj_id]>0,2);
no_obj = [length(pr_MHDP.objects); length(pr_MHDP_Chi.objects)];
err_obj = [mean(sqrt(Eobj_MHDP)); mean(sqrt(Eobj_MHDP_Chi))];
T = table(mean_odom,cum_odom,no_msts,no_obj,err_obj,...
   'RowNames',algo);


%%
%figure
%set(0,'DefaultLineMarkerSize',10)
%semilogx(no_obj(1), err_obj(1),'ko','MarkerFaceColor','k');hold on;
%xlabel('number of objects');ylabel error;
%legend('NP-Graph','Ground Truth');

%%
%figure
%semilogy(cumsum(sqrt(Eodom_NP(1:20:end))),'k-o');hold on;
%xlabel time;ylabel 'cumulative trajectory error';
%legend('NP-Graph');

