 classdef Processermd
% % 2020.5.15
% % ????????汾???á?large_scene2.mat???????
% % ?????????????DP????????????????????/new_posterior=0.0026
% % ???5??tree????????С??new_posterior????????????track
properties
    odoms
    objects
    measurements
    fp_thre=0
    DP_alpha = 0.9
    HDP_alpha = 0.9
    HDP_gamma = 2
    info_odom = [10 0 0 10 0 10]
    info_lm = [2 0 2]
    track
    delta_thre = 0.05

end
methods
    function obj = setupobjects(obj,node_edge, lm_edge)
        obj.measurements=lm_edge;
        obj.odoms=node_edge;
        theta=0; pos = zeros(2,1);
        for i=1:length(node_edge.dtheta)   % ???766??????
            R=[cos(theta) sin(theta); -sin(theta) cos(theta)];   
            pos=pos+R*obj.odoms.dpos(:,i);       % dpos: ??????????λ?????
            theta=theta + obj.odoms.dtheta(i);   % dtheta: ?????????????仯?????  
            theta = mod(theta+pi,2*pi)-pi;       %mod(a,b)???a????b??????
            obj.odoms.pos(:,i)=pos;              % pos: ?????????λ??
            obj.odoms.theta(i)=theta;            % theta: ????????????   
        end
        obj.measurements.obj_id = 1:length(lm_edge.id1);
        for i=1:length(lm_edge.id1)
            obj.objects(i).id=i;
            obj.objects(i).idPrev=i;
            obj.objects(i).P=zeros(5,1);            % ?????????
            obj.objects(i).P(lm_edge.label(i))=1;   % ???measurement?????????????????1
            obj.objects(i).msts = i;                % ???ID??1098?????
            t = obj.odoms.theta(lm_edge.id1(i));    % ??i??????????????????
            R=[cos(t) sin(t); -sin(t) cos(t)];      
            % ???????λ????????????????????λ????????????????????????????????λ??????
            obj.objects(i).pos = obj.odoms.pos(:,lm_edge.id1(i)) + R*lm_edge.dpos(:,i);
            obj.objects(i).poses = obj.odoms.pos(:,lm_edge.id1(i)) + R*lm_edge.dpos(:,i);
%             obj.all_measurements_pos = [obj.objects.pos];
        end
    end
    
    function [] = write_isam(obj, filename)
        %% write file
        fid = fopen(filename,'w');       
        
        for i=1:length(obj.odoms.dtheta)
            fprintf(fid,'ODOMETRY %d %d %f %f %f',i-1,i,obj.odoms.dpos(1,i),obj.odoms.dpos(2,i),-obj.odoms.dtheta(i));
            for j=1:length(obj.info_odom) 
                fprintf(fid,' %f',obj.info_odom(j)); 
            end
            fprintf(fid,'\n');
        end
        
        for k=1:length(obj.objects)
            if( length(obj.objects(k).msts)>obj.fp_thre)   %dp_prior??????????????????????????????????????????
                for i=obj.objects(k).msts            
                    fprintf(fid,'LANDMARK %d %d %f %f', obj.measurements.id1(i), obj.objects(k).id+1000,...
                        obj.measurements.dpos(1,i), obj.measurements.dpos(2,i));
                    for j=1:length(obj.info_lm) 
                        fprintf(fid,' %f',obj.info_lm(j)); 
                    end
                    fprintf(fid,'\n');
                end
            end
        end
        fclose(fid);
    end
    
    function [obj, Iodom, Iobj] = read_isam(obj, filename)
        fid = fopen(filename);
        tline = fgetl(fid);
        odom = [];
        objects=[];
        entropy=[];
        while ischar(tline)
            if strcmp(tline(1:11),'Pose2d_Node')
                line = tline(13:end);
                line(line=='(')=[]; line(line==')')=[];line(line==';')=',';
                odom = [odom; str2num(line)];
            end
            if strcmp(tline(1:12),'Point2d_Node')
            	line = tline(14:end);
                line(line=='(')=[]; line(line==')')=[];
                objects = [objects; str2num(line)];
            end
            if strcmp(tline(1:7),'entropy')
            	line = tline(9:end);
                entropy = [entropy; str2num(line)];
            end
            tline = fgetl(fid);
        end
        fclose(fid);
        
        N=size(odom,1);
        obj.odoms.pos = odom(:,2:3)';
        obj.odoms.theta = -odom(:,4)';
%         obj.odoms.entropy = entropy(1:N,:);
        
        %         id = [obj.objects.id];
%         id = unique(id,'stable');
        id=[];
        for i=1:length(obj.objects)                     %???????????object
            if( length(obj.objects(i).msts)>obj.fp_thre)
                id=[id obj.objects(i).id];
            end
        end
        for i=1:length(objects)
            obj_idx = [obj.objects.id]==id(i);
            obj.objects(obj_idx).pos = objects(i,2:end)';
%             obj.objects(obj_idx).entropy = entropy(i,2);
        end
    end
   
    function plot(obj)
        odom = obj.odoms.pos;
        fig = figure;
        set(fig,'Position', [100, 100, 400, 300]);
        set(fig,'Units','Inches');
        pos = get(fig,'Position');
        set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
        plot(odom(1,:),odom(2,:),'k');
        hold on; 
        
        label={'tajectory'};
        p=[obj.objects.P];      %[5x1098]
        objpose = [obj.objects.pos];
        for i=1:size(p,1)
            idx = p(i,:)>obj.fp_thre;
            if sum(idx)>0
                label{end+1}=['class ' num2str(i)];
                switch i
                case 1
                    plot(objpose(1,idx),objpose(2,idx),'bo','MarkerFaceColor','b');            
                case 2
                    plot(objpose(1,idx),objpose(2,idx),'rd','MarkerFaceColor','r');
                case 3
                    plot(objpose(1,idx),objpose(2,idx),'ms','MarkerFaceColor','m');
                case 4
                    plot(objpose(1,idx),objpose(2,idx),'g^','MarkerFaceColor',[0.2 1 0.2]);
                case 5
                    plot(objpose(1,idx),objpose(2,idx),'yp','MarkerFaceColor',[1 0.7 0.3],...
                                        'MarkerSize',15);
                end               
            end
        end
        axis equal; axis off;
        legend(label);
    end
    
    %=======        DP     ===========
    function obj = DPsample(obj)
        
        % for each measurement y_t^k
        % ????1089??????
        for k=length(obj.measurements.id1):-1:1    
            
            %% Step 1: Maximize likelihood 
            obj_idx = find([obj.objects.id]==obj.measurements.obj_id(k));
            l=obj.measurements.label(k);
            obj.objects(obj_idx).P(l)=obj.objects(obj_idx).P(l)-1;
            if sum(obj.objects(obj_idx).P)==0
                obj.objects(obj_idx)=[];
            else
                obj.objects(obj_idx).msts(obj.objects(obj_idx).msts==k)=[];
            end
            p = [obj.objects.P];  % ?????????????????????????
            obj_poses = [obj.objects.pos];   % ????????????????????λ?????
            
            N = length(obj.objects);

            % ??k???????????????????
            t = obj.odoms.theta(obj.measurements.id1(k));
            R = [cos(t) sin(t); 
                -sin(t) cos(t)];
            % ???????λ???????????k???????λ??
            pos = obj.odoms.pos(:,obj.measurements.id1(k))+R*obj.measurements.dpos(:,k);
            
            % DP(i)???????
            % ????????????????1????У?????????????????????????????1????????????????????
            dp_prior = p(obj.measurements.label(k),:);   
            % ????????????????????
            sq_dist = sum((obj_poses - repmat(pos,1,N)).^2,1);
            
            
            % Compute posterior p_i of being object i:
            posterior = exp(-4*sq_dist).*dp_prior;   %???????????????????????????????
            
            % y_t^k = argmax(posterior)
                 
            [prob, I]=max(posterior);   % ?????????????????????н????????У? ????? [aa, I]=min(sq_dist)?? 
            
            
            %% STep 2: Assign data association to their maximum likelihood objects
            % y_t is assigned to be the maximum likelihood object if the
            % likelihood is within some certain threshold, 
            % otherwise it is assigned to a new object.
            if(prob>obj.DP_alpha)  % ??????????????????????????????????1?????????????????????б?????????
                obj.objects(I).P(l)=obj.objects(I).P(l)+1;
                obj.objects(I).msts = [obj.objects(I).msts k];
                obj.measurements.obj_id(k)=obj.objects(I).id;
            else
                N_obj = max([obj.objects.id]);
                object.id = N_obj+1;
                object.P = zeros(5,1);object.P(l)=1;
                object.pos = pos;
                object.msts = k;
                object.idPrev = k;
                obj.objects=[obj.objects object];
                obj.measurements.obj_id(k) = N_obj+1;
            end
        end
    end
    
    %=======        HDP     ===========  
    function obj = HDPsample(obj)
        
        % for each measurement y_t^k
        % ????1089??????
        %for k=length(obj.measurements.id1):-1:1  
        for k=length(obj.measurements.id1):-1:1  
            
            %k
            %% Step 1: Maximize likelihood 
            obj_idx = find([obj.objects.id]==obj.measurements.obj_id(k));
            l=obj.measurements.label(k);
            obj.objects(obj_idx).P(l)=obj.objects(obj_idx).P(l)-1;
            if sum(obj.objects(obj_idx).P)==0
                obj.objects(obj_idx)=[];
            else
                obj.objects(obj_idx).msts(obj.objects(obj_idx).msts==k)=[];
            end
            p = [obj.objects.P];  % ?????????????????????????
            obj_poses = [obj.objects.pos];   % ????????????????????λ?????
            
            
            
            % To do
%             idx_frame = obj.measurements.id1(k);
%             commonviewKeyFrame = find(obj.measurements.commonview(idx_frame, :)==1);
%             
%             measurement_Tmp = [];
%             
%             idPrev1 = [obj.objects.idPrev];
%             for i = 1:length(commonviewKeyFrame)
%                 tmp = find(obj.measurements.id1==commonviewKeyFrame(i));
%                 measurement_Tmp = [measurement_Tmp tmp];
%             end
%             
%             measurement_Tmp = [measurement_Tmp k];
            
            measurement_Tmp = obj.measurements.measurements_Commonview{k};

            p2 = [];
            pos1 = [];
            dp_prior = [];
            for i = 1:length(measurement_Tmp)
                tmp = find([obj.objects.idPrev]==measurement_Tmp(i));
                if ~isempty(tmp)
                    pos1 = [pos1 obj_poses(:,tmp)];
                    dp_prior = [dp_prior p(obj.measurements.label(k),tmp)*obj.HDP_gamma];
                    p2 = [p2 tmp];
                end
            end
            
            N = size(pos1,2);

            % ??k???????????????????
            t = obj.odoms.theta(obj.measurements.id1(k));
            R = [cos(t) sin(t); 
                -sin(t) cos(t)];
            % ???????λ???????????k???????λ??
            pos = obj.odoms.pos(:,obj.measurements.id1(k))+R*obj.measurements.dpos(:,k);
            
            % DP(i)???????
            % ????????????????1????У?????????????????????????????1????????????????????
            %dp_prior = p(obj.measurements.label(k),:);   
            % ????????????????????
%             if isempty(pos1)
%                 k
%             end
%             size(pos1)
%             N
            sq_dist = sum((pos1 - repmat(pos,1,N)).^2,1);
            
            % Compute posterior p_i of being object i:
            posterior = exp(-4*sq_dist).*dp_prior;   %???????????????????????????????
            
            % y_t^k = argmax(posterior)
                 
            [prob, I1]=max(posterior);   % ?????????????????????н????????У? ????? [aa, I]=min(sq_dist)?? 
            
            I = p2(I1);
            
            
            %% STep 2: Assign data association to their maximum likelihood objects
            % y_t is assigned to be the maximum likelihood object if the
            % likelihood is within some certain threshold, 
            % otherwise it is assigned to a new object.
            if(prob>obj.HDP_alpha)  % ??????????????????????????????????1?????????????????????б?????????
                obj.objects(I).P(l)=obj.objects(I).P(l)+1;
                obj.objects(I).msts = [obj.objects(I).msts k];
                obj.measurements.obj_id(k)=obj.objects(I).id;
            else
                N_obj = max([obj.objects.id]);
                object.id = N_obj+1;
                object.P = zeros(5,1);object.P(l)=1;
                object.pos = pos;
                object.msts = k;
                object.idPrev = k;
                obj.objects=[obj.objects object];
                obj.measurements.obj_id(k) = N_obj+1;
            end
        end
    end
    
    
        %=======        MHDP     ===========
    function obj = MHDPsample(obj)
%         ????? track
        obj.track(1).objects = [obj.objects];
        obj.track(1).measurements = [obj.measurements];
        obj.track(1).confidence = [];
        obj.track(1).posteri = 0;
        for k=length(obj.measurements.id1):-1:1 
        % for each measurement y_t^k
        % ????1089??????        
            length_track_prune = length(obj.track);
            for i=1:1:length_track_prune
                
                obj_idx = find([obj.track(i).objects.id]==obj.track(i).measurements.obj_id(k));  %??????κ??????????????????object???????????1098??
                l=obj.track(i).measurements.label(k);  %5???е??????
                obj.track(i).objects(obj_idx).P(l)=obj.track(i).objects(obj_idx).P(l)-1;
                if sum(obj.track(i).objects(obj_idx).P)==0
                    obj.track(i).objects(obj_idx)=[];
                else
                    obj.track(i).objects(obj_idx).msts(obj.track(i).objects(obj_idx).msts==k)=[];
                end
                p = [obj.track(i).objects.P];  % ????????????????????????? (5 x 1097)
                obj_poses = [obj.track(i).objects.pos];   % ????????????????????λ?????

                N = length(obj.track(i).objects);

                % ??k???????????????????
                t = obj.odoms.theta(obj.track(i).measurements.id1(k));
                R = [cos(t) sin(t); 
                    -sin(t) cos(t)];
                % ???????λ???????????k???????λ??
                pos = obj.odoms.pos(:,obj.track(i).measurements.id1(k))+R*obj.track(i).measurements.dpos(:,k);

                % DP(i)???????        ????????????????????
                % ????????????????1????У?????????????????????????????1????????????????????
                dp_prior = p(obj.track(i).measurements.label(k),:);

                % ????????????????????
                sq_dist = sum((obj_poses - repmat(pos,1,N)).^2,1);   %repmat(pos,1,N))?????pos??2?λ??????? N ??

                % Compute posterior p_i of being object i:
                posterior = exp(-4*sq_dist).*dp_prior;   %???????????????????????????????

                %==========    ???DP????????  ================
                prior_fenmu = sum(dp_prior) + obj.DP_alpha;  %??ζ????????
                old_prior = dp_prior ./ prior_fenmu;
                old_posterior = old_prior .* exp(-4*sq_dist);
                new_posterior = obj.DP_alpha ./ prior_fenmu;
    
                %==========   yuanshi zuidahouyan    ==============               
    %           y_t^k = argmax(posterior)  ???????????????????
    %             [prob, I]=max(posterior);   % ?????????????????????н????????У? ????? [aa, I]=min(sq_dist)??

                %==========   ????????    ==============
                Len = length(old_posterior);
                [prob, I]=sort(old_posterior);
%                 Len = length(posterior);
%                 [prob, I]=sort(posterior);
%                 new_posterior = obj.DP_alpha;
                prob_1 = prob(Len-1);    %  sex???????
                I_1 = I(Len-1);
                prob_2 = prob(Len);    %  max???????
                I_2 = I(Len);

% %             ?????????????С??  new_posterior ??????????????????????
% %             ??????????????ж?????? new_posterior ???????????????????????
                if prob_2 > new_posterior   
                    tree.prob = [];
                    tree.I = [];
%                    prior_delta = prob_2 - prob_1;   %?????????max - sec
                    for tree_i=Len-1:-1:1  % ??Len-1????????????????prob_1???????????????track??????
%                        cur_delta = prob(tree_i) - prob(tree_i-1);
%                         if (length(tree.prob)<4)   % TODO
                        if ((length(tree.prob)<4))
                            tree.prob = [tree.prob prob(tree_i)];
                            tree.I = [tree.I I(tree_i)];
                        else
                            break;
                        end
%                        prior_delta = cur_delta;
                    end

                    tree_len = length(tree.prob);
                    length_track = length(obj.track);
                    if ((tree_len~=0) && (~isempty(tree.prob)))   % ??????ж?????
                        for tree_j=1:1:tree_len
                            if(tree.prob(tree_j)>new_posterior)   % ???,?????θ?壬?·??====??θ?崦??????ж??????????????????????????????????????   % ???,?????θ?壬?·??====??θ?崦??????ж??????????????????????????????????????
                                obj.track(length_track+tree_j) = obj.track(i);
                                obj.track(length_track+tree_j).objects(tree.I(tree_j)).P(l)=obj.track(i).objects(tree.I(tree_j)).P(l)+1;
                                obj.track(length_track+tree_j).objects(tree.I(tree_j)).msts = [obj.track(i).objects(tree.I(tree_j)).msts k];
                                obj.track(length_track+tree_j).objects(tree.I(tree_j)).poses = [obj.track(i).objects(tree.I(tree_j)).poses pos];
                                obj.track(length_track+tree_j).measurements.obj_id(k)=obj.track(i).objects(tree.I(tree_j)).id;
                                obj.track(length_track+tree_j).confidence=[obj.track(i).confidence tree.prob(tree_j)];   % track ????????
                            else  
                                obj.track(length_track+tree_j) = obj.track(i);
                                N_obj = max([obj.track(i).objects.id]);
                                object.id = N_obj+1;
                                object.P = zeros(5,1);object.P(l)=1;
                                object.pos = pos;
                                object.poses = pos;
                                object.msts = k;
                                object.idPrev = k;
                                obj.track(length_track+tree_j).objects=[obj.track(i).objects object];
                                obj.track(length_track+tree_j).measurements.obj_id(k) = N_obj+1;
                                obj.track(length_track+tree_j).confidence=[obj.track(i).confidence new_posterior];
                                
                                break;  %????????????????????????????????new??track??
                                
                            end
                        end
                    end
                end
        % %     # 2 
                if(prob_2>new_posterior) % ??? or ??????????????????????
                    obj.track(i).objects(I_2).P(l)=obj.track(i).objects(I_2).P(l)+1;
                    obj.track(i).objects(I_2).msts = [obj.track(i).objects(I_2).msts k];
                    obj.track(i).objects(I_2).poses = [obj.track(i).objects(I_2).poses pos];
                    obj.track(i).measurements.obj_id(k)=obj.track(i).objects(I_2).id;
                    obj.track(i).confidence=[obj.track(i).confidence prob_2];     % track????????
                else    %?μ?
                    N_obj = max([obj.track(i).objects.id]);
                    object.id = N_obj+1;
                    object.P = zeros(5,1);object.P(l)=1;
                    object.pos = pos;
                    object.poses = pos;
                    object.msts = k;
                    object.idPrev = k;
                    obj.track(i).objects=[obj.track(i).objects object];
                    obj.track(i).measurements.obj_id(k) = N_obj+1;
                    obj.track(i).confidence=[obj.track(i).confidence new_posterior];    % ????????????μ???????????? DP_alpha
                end            
            end 
% %   ============= ??? ==============
            while (length(obj.track)>20)
                temp_sum_conf = inf;
                for r=1:1:length(obj.track)
                    sum_conf = sum(obj.track(r).confidence);
                    obj.track(r).posteri = sum_conf;     % track???????????????
                    if(sum_conf<temp_sum_conf)
                        temp_sum_conf = sum_conf;
                        conf_track_index = r;             
                    end
                end
                obj.track(conf_track_index)=[];  
            end
% %    =================================
        end 
    end
    
    
    %=======        MHDP     ===========
    function obj = MHDPsample_Chi(obj)
%         ????? track
        obj.track(1).objects = [obj.objects];
        obj.track(1).measurements = [obj.measurements];
        obj.track(1).confidence = [];
        obj.track(1).posteri = 0;
        for k=length(obj.measurements.id1):-1:1 
        % for each measurement y_t^k
        % ????1089??????        
            length_track_prune = length(obj.track);
            for i=1:1:length_track_prune
                
                obj_idx = find([obj.track(i).objects.id]==obj.track(i).measurements.obj_id(k));  %??????κ??????????????????object???????????1098??
                l=obj.track(i).measurements.label(k);  %5???е??????
                obj.track(i).objects(obj_idx).P(l)=obj.track(i).objects(obj_idx).P(l)-1;
                if sum(obj.track(i).objects(obj_idx).P)==0
                    obj.track(i).objects(obj_idx)=[];
                else
                    obj.track(i).objects(obj_idx).msts(obj.track(i).objects(obj_idx).msts==k)=[];
                end
                p = [obj.track(i).objects.P];  % ????????????????????????? (5 x 1097)
                obj_poses = [obj.track(i).objects.pos];   % ????????????????????λ?????

                N = length(obj.track(i).objects);

                % ??k???????????????????
                t = obj.odoms.theta(obj.track(i).measurements.id1(k));
                R = [cos(t) sin(t); 
                    -sin(t) cos(t)];
                % ???????λ???????????k???????λ??
                pos = obj.odoms.pos(:,obj.track(i).measurements.id1(k))+R*obj.track(i).measurements.dpos(:,k);

                dp_prior = p(obj.track(i).measurements.label(k),:);

                sq_dist = sum((obj_poses - repmat(pos,1,N)).^2,1);   %repmat(pos,1,N))

                % Compute posterior p_i of being object i:
                posterior = exp(-4*sq_dist).*dp_prior;   

                %==========    DP  ================
                prior_fenmu = sum(dp_prior) + obj.DP_alpha;  
                old_prior = dp_prior ./ prior_fenmu;
                old_posterior = old_prior .* exp(-4*sq_dist);
                new_posterior = obj.DP_alpha ./ prior_fenmu;
    
               
                Len = length(old_posterior);
                [prob, I]=sort(old_posterior);
                prob_1 = prob(Len-1);    %  sex
                I_1 = I(Len-1);
                prob_2 = prob(Len);    %  max
                I_2 = I(Len);

                
                if prob_2 > new_posterior   %% important
                    tree.prob = [];
                    tree.I = [];
                    for tree_i=Len-1:-1:1  
                        if ((length(tree.prob)<4))
                            tree.prob = [tree.prob prob(tree_i)];
                            tree.I = [tree.I I(tree_i)];
                        else
                            break;
                        end
                    end

                    tree_len = length(tree.prob);
                    length_track = length(obj.track);
                    if ((tree_len~=0) && (~isempty(tree.prob)))   
                        for tree_j=1:1:tree_len
                            if(size(obj.track(i).objects(tree.I(tree_j)).msts,2)>1) % judge the number of measurement in the table
                                poses = obj.track(i).objects(tree.I(tree_j)).poses;
                                dist = pdist([pos,poses]','mahal');
                                md = mean(dist(1:size(obj.track(i).objects(tree.I(tree_j)).msts,2)));
                                if md < 5.991
                                    obj.track(length_track+tree_j) = obj.track(i);
                                    obj.track(length_track+tree_j).objects(tree.I(tree_j)).P(l)=obj.track(i).objects(tree.I(tree_j)).P(l)+1;
                                    obj.track(length_track+tree_j).objects(tree.I(tree_j)).msts = [obj.track(i).objects(tree.I(tree_j)).msts k];
                                    obj.track(length_track+tree_j).objects(tree.I(tree_j)).poses = [obj.track(i).objects(tree.I(tree_j)).poses pos];
                                    obj.track(length_track+tree_j).measurements.obj_id(k)=obj.track(i).objects(tree.I(tree_j)).id;
                                    obj.track(length_track+tree_j).confidence=[obj.track(i).confidence tree.prob(tree_j)];
                                else
                                    obj.track(length_track+tree_j) = obj.track(i);
                                    N_obj = max([obj.track(i).objects.id]);
                                    object.id = N_obj+1;
                                    object.P = zeros(5,1);object.P(l)=1;
                                    object.pos = pos;
                                    object.poses = pos;
                                    object.msts = k;
                                    object.idPrev = k;
                                    obj.track(length_track+tree_j).objects=[obj.track(i).objects object];
                                    obj.track(length_track+tree_j).measurements.obj_id(k) = N_obj+1;
                                    obj.track(length_track+tree_j).confidence=[obj.track(i).confidence new_posterior];
                                    
                                    md_new_lm_flag = true; 
                                    
                                    break;

                                end
                            else
                                if(tree.prob(tree_j) > new_posterior)   
                                    obj.track(length_track+tree_j) = obj.track(i);
                                    obj.track(length_track+tree_j).objects(tree.I(tree_j)).P(l)=obj.track(i).objects(tree.I(tree_j)).P(l)+1;
                                    obj.track(length_track+tree_j).objects(tree.I(tree_j)).msts = [obj.track(i).objects(tree.I(tree_j)).msts k];
                                    obj.track(length_track+tree_j).objects(tree.I(tree_j)).poses = [obj.track(i).objects(tree.I(tree_j)).poses pos];
                                    obj.track(length_track+tree_j).measurements.obj_id(k)=obj.track(i).objects(tree.I(tree_j)).id;
                                    obj.track(length_track+tree_j).confidence=[obj.track(i).confidence tree.prob(tree_j)];   % track
                                else  
                                    obj.track(length_track+tree_j) = obj.track(i);
                                    N_obj = max([obj.track(i).objects.id]);
                                    object.id = N_obj+1;
                                    object.P = zeros(5,1);object.P(l)=1;
                                    object.pos = pos;
                                    object.poses = pos;
                                    object.msts = k;
                                    object.idPrev = k;
                                    obj.track(length_track+tree_j).objects=[obj.track(i).objects object];
                                    obj.track(length_track+tree_j).measurements.obj_id(k) = N_obj+1;
                                    obj.track(length_track+tree_j).confidence=[obj.track(i).confidence new_posterior];
                                    
                                    break;  

                                end
                            end
                        end
                    end
                end
        %      # 2 
                if(size(obj.track(i).objects(I_2).msts,2)>1)
                    poses = obj.track(i).objects(I_2).poses;
                    dist = pdist([pos,poses]','mahal');
                    md = mean(dist(1:size(obj.track(i).objects(I_2).msts,2)));
                    if md < 5.991
                        obj.track(i).objects(I_2).P(l)=obj.track(i).objects(I_2).P(l)+1;
                        obj.track(i).objects(I_2).msts = [obj.track(i).objects(I_2).msts k];
                        obj.track(i).objects(I_2).poses = [obj.track(i).objects(I_2).poses pos];
                        obj.track(i).measurements.obj_id(k)=obj.track(i).objects(I_2).id;
                        obj.track(i).confidence=[obj.track(i).confidence prob_2];     % track
                    else    
                        if md_new_lm_flag == false
                            N_obj = max([obj.track(i).objects.id]);
                            object.id = N_obj+1;
                            object.P = zeros(5,1);object.P(l)=1;
                            object.pos = pos;
                            object.poses = pos;
                            object.msts = k;
                            object.idPrev = k;
                            obj.track(i).objects=[obj.track(i).objects object];
                            obj.track(i).measurements.obj_id(k) = N_obj+1;
                            obj.track(i).confidence=[obj.track(i).confidence new_posterior];    % DP_alpha
                        end   
                    end
                else
                    if(prob_2 > new_posterior) 
                        obj.track(i).objects(I_2).P(l)=obj.track(i).objects(I_2).P(l)+1;
                        obj.track(i).objects(I_2).msts = [obj.track(i).objects(I_2).msts k];
                        obj.track(i).objects(I_2).poses = [obj.track(i).objects(I_2).poses pos];
                        obj.track(i).measurements.obj_id(k)=obj.track(i).objects(I_2).id;
                        obj.track(i).confidence=[obj.track(i).confidence prob_2];     % track
                    else
                        N_obj = max([obj.track(i).objects.id]);
                        object.id = N_obj+1;
                        object.P = zeros(5,1);object.P(l)=1;
                        object.pos = pos;
                        object.poses = pos;
                        object.msts = k;
                        object.idPrev = k;
                        obj.track(i).objects=[obj.track(i).objects object];
                        obj.track(i).measurements.obj_id(k) = N_obj+1;
                        obj.track(i).confidence=[obj.track(i).confidence new_posterior];    % DP_alpha
                    end 
                end
                md_new_lm_flag = false;
            end
% %   ============= prune ==============
            while (length(obj.track)>20)
                temp_sum_conf = inf;
                for r=1:1:length(obj.track)
                    sum_conf = sum(obj.track(r).confidence);
                    obj.track(r).posteri = sum_conf;     % track
                    if(sum_conf<temp_sum_conf)
                        temp_sum_conf = sum_conf;
                        conf_track_index = r;             
                    end
                end
                obj.track(conf_track_index)=[];  
            end
% %    =================================
        end 
    end
    
    function [obj]= optimizeDP(obj,maxiteration)
        for i=1:maxiteration
            obj.plot();
            title(sprintf('DP Iter %d, %d objects', i-1, length(obj.objects)))
            obj=obj.DPsample();
            obj.write_isam('isam_input.txt');
            !/usr/local/bin/isam isam_input.txt -W isam_output.txt -B
            [obj] = obj.read_isam('isam_output.txt');
        end

    end
    
    function [obj]= optimizeHDP(obj,maxiteration)
        for i=1:maxiteration
            obj.plot();
            title(sprintf('HDP Iter %d, %d objects', i-1, length(obj.objects)))
            obj=obj.HDPsample();
            obj.write_isam('isam_input.txt');
            !/usr/local/bin/isam isam_input.txt -W isam_output.txt -B
            [obj] = obj.read_isam('isam_output.txt');
        end

    end
    
    function [obj]= optimizeMHDP(obj,maxiteration)
    for i=1:maxiteration
        obj.plot();
        title(sprintf('MHDP Iter %d, %d objects', i-1, length(obj.objects)))
        obj=obj.MHDPsample();
        temp_best_conf=0;
        for rr=1:1:length(obj.track)           %  TODO  ??????????               
            best_conf =  sum(obj.track(rr).confidence);                
            if(best_conf>temp_best_conf)
                temp_best_conf = best_conf;
                conf_best_track_index = rr;             
            end
        end
        obj.objects = [obj.track(conf_best_track_index).objects];
        obj.measurements = [obj.track(conf_best_track_index).measurements];
        obj.track = [];   % ???
        obj.write_isam('isam_input.txt');
        !/usr/local/bin/isam isam_input.txt -W isam_output.txt -B
        [obj] = obj.read_isam('isam_output.txt');
    end
end
    
    function [obj]= optimizeMHDP_Chi(obj,maxiteration)
        for i=1:maxiteration
            obj.plot();
            title(sprintf('MHDP Chi Iter %d, %d objects', i-1, length(obj.objects)))
            obj=obj.MHDPsample_Chi();
            temp_best_conf=0;
            for rr=1:1:length(obj.track)           %  TODO  ??????????               
                best_conf =  sum(obj.track(rr).confidence);                
                if(best_conf>temp_best_conf)
                    temp_best_conf = best_conf;
                    conf_best_track_index = rr;             
                end
            end
            obj.objects = [obj.track(conf_best_track_index).objects];
            obj.measurements = [obj.track(conf_best_track_index).measurements];
            obj.track = [];   % ???
            obj.write_isam('isam_input.txt');
            !/usr/local/bin/isam isam_input.txt -W isam_output.txt -B
            [obj] = obj.read_isam('isam_output.txt');
        end
    end
    
    
    function [Iodom, Iobj] = computeEntropy(obj)
        factorgraph = gtsam.NonlinearFactorGraph;
        estimates = gtsam.Values;
        odom_cov = gtsam.noiseModel.Diagonal.Sigmas(obj.odom_noise);
        ms_cov = gtsam.noiseModel.Diagonal.Sigmas(obj.lm_noise);
        
        factorgraph.add(gtsam.NonlinearEqualityPose2(0,gtsam.Pose2));
        estimates.insert(0, gtsam.Pose2);
        
        odoms = [obj.odoms.pos; obj.odoms.theta];
        dodom = [obj.odoms.dpos; obj.odoms.dtheta];
        for k=1:size(dodom,2)
            estimates.insert(k,gtsam.Pose2(odoms(:,k)));
            factorgraph.add(gtsam.BetweenFactorPose2(k-1, k , gtsam.Pose2(dodom(:,k)), odom_cov));
        end
        
        objpos = [obj.objects.pos];
        for k=1:length(obj.objects)
            estimates.insert(obj.objects(k).id+1000, gtsam.Pose2([objpos(:,k);0]));
        end
        id1=obj.measurements.id1;
        id2=obj.measurements.obj_id;
        for k=1:length(id1)
            factorgraph.add(gtsam.BetweenFactorPose2(...
                id1(k), id2(k)+1000, gtsam.Pose2([obj.measurements.dpos(:,1);0]), ms_cov));
        end
        
        marginals = gtsam.Marginals(factorgraph,estimates);
        Iodom=zeros(size(dodom,2),1);
        for k=1:size(dodom,2)
            Iodom(k) = log(det(marginals.marginalCovariance(k)));
        end
        Iobj=zeros(length(obj.objects),1);
        for k=1:length(obj.objects)
            covar = marginals.marginalCovariance(obj.objects(k).id+1000);
            Iobj(k) = log(det(covar(1:2,1:2)));
        end 
    end
    
    function [err_odom, err_obj]=computeError(obj, true_traj,true_obj)
        est_traj = [obj.odoms.pos];
        Nodom = min([length(true_traj) length(obj.odoms.pos)]);
        true_traj(3,:)=0; true_traj=true_traj(:,1:Nodom);
        est_traj(3,:)=0; est_traj=est_traj(:,1:Nodom);
        [T] = estimateRigidTransform(true_traj,est_traj);
        est_traj(4,:)=1; est_traj = T*est_traj;
        err_odom = sum( (true_traj(1:2,:)-est_traj(1:2,:)).^2,1);
        
        idx = obj.measurements.obj_id>0;
        tobj  = true_obj(:,obj.measurements.id2(idx));
        tobj(3,:)=0;
        
        objpos = [obj.objects.pos];
        for i=1:length(obj.objects)
            id2idx(obj.objects(i).id)=i;
        end
        eobj = objpos(:,id2idx(obj.measurements.obj_id(idx)));
        eobj(3,:)=0; eobj(4,:)=1;
        eobj = T*eobj;
        err_obj = sum( (tobj(1:2,:)-eobj(1:2,:)).^2,1);
    end
    
end    
end