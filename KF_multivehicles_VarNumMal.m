% Multi-vehicles Attack-resistant Cooperative Tracking
function [RMSE]=KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,filter_mode,buffer_size,space_attack_mode,time_attack_mode,randAver_times,test_index,collu_design_mal_devi_coef,collu_rand_mal_devi_coef)
%% Set parameters

dt=0.1; %Measurement interval of self and other observations
t=0:dt:20; %Total simulation time

% var_self=4; %Variance of self-observation
% var_mea=9;  %Variance of observations from other vehicles
% mal_var_coef=0.7872; %mal_var/var_mea

% num_vehicle = 20; % Number of maximun other vehicles observing Vehicle 1
% num_malicious = 0; % Number of malicous vehicles
% num_minvehi=20; % Number of minimum other vehicles in sweeping simulation

num_filter=7; %Filter number in comparative simulation of all filter and groundtruth
% buffer_size=20; %Buffer size of sequence test
% attack_mode = 'collu_design'; % Attacks mode of malicious vehicles, it can be "independent rand attack, colluding rand attack or colluding design attack".
mal_index_mode = 'random'; %Index method of malicious vehicles

% collu_design_mal_devi_coef=1; %malicious deviation/sigma_mea in collu_design attack
% collu_rand_mal_devi_coef=10; %malicious deviation/sigma_mea in collu_rand attack
collu_design_mal_deviation=collu_design_mal_devi_coef*sqrt(var_mea);
collu_rand_mal_deviation=collu_rand_mal_devi_coef*sqrt(var_mea);

% randAver_times = 20; %Interation number of whole route simulation

size=length(t);
F=[1 dt 0 0;
    0 1 0 0;
    0 0 1 dt;
    0 0 0 1]; %State evolute matrix
H1=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];%Self-observing matrix (Vehicle 1)
H2=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];%Observing matrix of other vehicles
C=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
B=[dt*dt/2 0;
    dt 0;
    0 dt*dt/2;
    0 dt];  

alpha = 0.1; % standard deviation of control noise of Vehicle 1
u1=zeros(2,size);% Control sequences of Vehicle 1
u1(1,1:10)=25;
u1(1,100:105)=10;
u1(1,190:200)=-20;  % Accelerate and deaccelerate along the X direction
u1(2,46:55)=20;
u1(2,56:60)=-40; % Change lane in Y direction
ori_u1=u1;
u1 = u1+alpha*randn(2,size);

%Other noises
R11=[var_self 0 0 0;
    0 var_self 0 0;
    0 0 var_self 0;
    0 0 0 var_self]; %Noise matrix of self-observation
R21=[var_mea 0 0 0;
    0 var_mea 0 0;
    0 0 var_mea 0;
    0 0 0 var_mea]; %Noise matrix of other-observations
R_mal=[mal_var_coef*var_mea 0 0 0;
    0 mal_var_coef*var_mea 0 0;
    0 0 mal_var_coef*var_mea 0;
    0 0 0 mal_var_coef*var_mea]; %Pre-defined 'noise' matrix of malicious data
Q1=[0.1 0 0 0;
   0 0.15 0 0;
   0 0 0.2 0;
   0 0 0 0.2];%Noise matrix of movement of Vehicle 1

%% Kalman Filtering
msex11_=0;
msev11_=0;
msexY11_=0;
msevY11_=0;
msex_=zeros(1,num_vehicle);
msev_=zeros(1,num_vehicle);

%Results container when all filters are compared
All_msex_=cell(1,num_filter+2);
All_msev_=cell(1,num_filter+2);
for i=1:num_filter+2
    All_msex_{i}=zeros(1,num_vehicle);
    All_msev_{i}=zeros(1,num_vehicle);
end
%Initiate the FP and FN counters
    false_pos_count=zeros(1,num_vehicle);
    false_neg_count=zeros(1,num_vehicle);

%Beginning of each simulation
for time = 1:randAver_times
    %Random parameters 
    u1 = u1+alpha*randn(2,size);

    w1=sqrt(Q1)*randn(4,size);
    v11=sqrt(R11)*randn(4,size);
    v21=cell(1,num_vehicle);
    for i = 1:num_vehicle
        v21{i}=sqrt(R21)*randn(4,size);
    end

    %Observation results and groundtruth
    x1=zeros(4,size);% Vehicle 1 groundtruth, four rows are "x position, x speed, y position, y speed".
    x1(:,1)=[1;0;5;0];% The initial start point of vehicle 1 is (1,5)
    for i=1:(size-1)
        x1(:,i+1)=F*x1(:,i)+B*u1(:,i);
    end
    x1=x1+C*w1;
    Y11=H1*x1+v11;% Self-observation of vehicle 1

    Y21=cell(1,num_vehicle);
    for i = 1:num_vehicle
        Y21{i} = H2*x1+v21{i};
    end %Observations from other vehicles

    %Indexing mode of indexes of malicious vehicles
    if(strcmp(mal_index_mode,'random'))
        temp=randperm(num_minvehi);
        malicious_index=temp(1:num_malicious);
    elseif(strcmp(mal_index_mode,'sequential'))
        malicious_index=1:num_malicious;
    elseif(strcmp(mal_index_mode,'custom'))
        malicious_index=[2,4,7,9,11,12,13];
    end

    X11=zeros(4,size);% Self-estimation of state of v1
    X11(:,1)=x1(:,1);
    X21=cell(1,num_vehicle);
    for i=1:num_vehicle
        X21{i}=zeros(4,size);
        X21{i}(:,1)=x1(:,1);
    end

    P11=20*eye(4);
    P21=cell(1,num_vehicle);
    for i=1:num_vehicle
        P21{i} = 30*eye(4);
    end %Self-estimation and estimations from other vehicles of the covariance of the state of v1

    % Global estimation of state of v1  
    X_1=cell(1,num_vehicle);
    for i = 1:num_vehicle
        X_1{i}=zeros(4,size);
        X_1{i}(:,1)=x1(:,1);
    end

    %Results container for comparative simulation--for filter mode "All"
    All_X1=cell(num_filter+2,num_vehicle); 
    for i=2:num_filter+2
        for j=1:num_vehicle
            All_X1{i,j}=zeros(4,size);
            All_X1{1,j}=x1;
        end
    end
   
    %Buffer for sequential detectors
    DataSeq_buffer=cell(1,num_vehicle);
    for i=1:num_vehicle
        DataSeq_buffer{i}=zeros(4,buffer_size);
    end
    
    %Create the groudtruth of trust table, 1 means that vehicle with this index is honest
    gt_trust=cell(1,num_vehicle);
    for i=num_minvehi:num_vehicle
        gt_trust{i}=ones(1,i);
        for j=1:num_malicious
            gt_trust{i}(malicious_index(j))=0;
        end
    end
    %Initiate the previous trust table which is updated in sequential detetors
    % prev_trust_table=cell(1,num_vehicle);
    % for j=num_minvehi:num_vehicle
    %     prev_trust_table{j}=ones(1,j);
    % end
    %Initialize the total trust values for all other vehicles before each simulation
    total_trust_val=cell(1,num_vehicle);
    for j=num_minvehi:num_vehicle
        total_trust_val{j}=zeros(1,j);
    end

    if(strcmp(time_attack_mode,'long_design'))
        rand_degree=rand(num_malicious,1)*2*pi; %Each time the rand_degree in polar coordinates is the same for one vehicle
    end



    for i=1:(size-1)
        % 1-1
        X11_ = F*X11(:,i)+B*u1(:,i);
        P11_ = F*P11*F'+C*Q1*C';
        K11 = P11_*H1'/(H1*P11_*H1'+R11);
        X11(:,i+1) = X11_+K11*(Y11(:,i+1)-H1*X11_);
        P11 = P11_-K11*H1*P11_;
        % 2-1
        for j=1:num_vehicle
            X21_ = F*X21{j}(:,i)+B*u1(:,i);
            P21_ = F*P21{j}*F'+C*Q1*C';
            K21 = P21_*H2'/(H2*P21_*H2'+R21);
            X21{j}(:,i+1) = X21_+K21*(Y21{j}(:,i+1)-H2*X21_);
            P21{j} = P21_-K21*H2*P21_;
        end

        for j=num_minvehi:num_vehicle
            Pg=eye(4)/P11;
            for k=1:j
                Pg=Pg+eye(4)/P21{k};
            end
            A11_=(eye(4)/P11)/Pg;
            A21_=cell(1,j);
            X_1{j}(:,i+1)=A11_*X11(:,i+1);
            for k=1:j
                A21_{k}=(eye(4)/P21{k})/Pg;
                X_1{j}(:,i+1)=X_1{j}(:,i+1)+A21_{k}*X21{k}(:,i+1);
            end
            All_X1{9,j}(:,i+1)=X_1{j}(:,i+1);
        end
    end





    %Kalman filter and malicious detecting & filtering  
    for i=1:(size-1)
        if(i<buffer_size)
            Mal_time_count=0; %Count how many times malicious attack occur after buffer_size if reached
        end
        
        % Attack methods in different attack mode
        if(strcmp(space_attack_mode,'indep'))
            % if(strcmp(time_attack_mode,'long_design')) 
            %     for j=1:num_malicious
            %         rand_radius=(1+0.3*rand(1,1))*collu_design_mal_deviation;
            %         Y21{malicious_index(j)}(1,i+1)=x1(1,i+1)+rand_radius*cos(rand_degree(j));
            %         Y21{malicious_index(j)}(3,i+1)=x1(3,i+1)+rand_radius*sin(rand_degree(j));
            %     end
            %     Mal_time_count=Mal_time_count+1;
            % elseif(strcmp(time_attack_mode,'long_rand'))
            %     rand_degree=rand(num_malicious,1)*2*pi; %Each time rand_degree are different
            %     for j=1:num_malicious
            %         rand_radius=(1+0.3*rand(1,1))*collu_rand_mal_deviation;
            %         Y21{malicious_index(j)}(1,i+1)=x1(1,i+1)+rand_radius*cos(rand_degree(j));
            %         Y21{malicious_index(j)}(3,i+1)=x1(3,i+1)+rand_radius*sin(rand_degree(j));
            %     end
            %     Mal_time_count=Mal_time_count+1;
            % elseif(strcmp(time_attack_mode,'short_rand'))
            %     if(mod(i,50)==30)
            %         rand_degree=rand(num_malicious,1)*2*pi; 
            %         for j=1:num_malicious
            %             rand_radius=(1+0.3*rand(1,1))*collu_rand_mal_deviation;
            %             Y21{malicious_index(j)}(1,i+1)=x1(1,i+1)+rand_radius*cos(rand_degree(j));
            %             Y21{malicious_index(j)}(3,i+1)=x1(3,i+1)+rand_radius*sin(rand_degree(j));
            %         end
            %     end
            %     for j=num_minvehi:num_vehicle
            %         for k=1:num_malicious
            %             if(mod(i,50)==30)
            %                 gt_trust{j}(malicious_index(k))=0; %Only mark the malicious vehicles in gt when attacks occur
            %                 Mal_time_count=Mal_time_count+1; %Also only increase the malicious count when attacks occur
            %             else
            %                 gt_trust{j}(malicious_index(k))=1; 
            %             end
            %         end
            %     end
            % end
        elseif(strcmp(space_attack_mode,'collu'))
            if(strcmp(time_attack_mode,'long_design')) 
                for j=1:num_malicious
                    Y21{malicious_index(j)}(:,i+1)=x1(:,i+1)+[0;0;collu_design_mal_deviation;0]+sqrt(R_mal)*randn(4,1);
                end
                Mal_time_count=Mal_time_count+1;
            elseif(strcmp(time_attack_mode,'long_rand'))
                positive_devi=(mod(i,8)<4);
                for j=1:num_malicious
                    Y21{malicious_index(j)}(3,i+1)=x1(3,i+1)-0.5*collu_rand_mal_deviation+positive_devi*collu_rand_mal_deviation;
                end
                Mal_time_count=Mal_time_count+1;
            elseif(strcmp(time_attack_mode,'short_rand'))
                %In short_rand attack, it's difficult to define TNR, thus it's a better to achieve a small FNR
                pulse_occur=(mod(i,40)==0);
                pulse_mal_time_range=(mod(i,40)<buffer_size)&&(floor(i/40)>0); 
                for j=1:num_malicious
                    Y21{malicious_index(j)}(3,i+1)=x1(3,i+1)+(pulse_occur)*collu_rand_mal_deviation;
                end
                for j=num_minvehi:num_vehicle
                    for k=1:num_malicious
                        if(pulse_mal_time_range) %If only pursue small FNR, pulse_occur definition is better
                            gt_trust{j}(malicious_index(k))=0;
                        else
                            gt_trust{j}(malicious_index(k))=1;
                        end
                    end
                end
                if(pulse_mal_time_range)
                    Mal_time_count=Mal_time_count+1;
                end
            end
        end

        % Kalman filtering of 1->1
        X11_ = F*X11(:,i)+B*u1(:,i);
        P11_ = F*P11*F'+C*Q1*C';
        K11 = P11_*H1'/(H1*P11_*H1'+R11);
        X11(:,i+1) = X11_+K11*(Y11(:,i+1)-H1*X11_);
        P11 = P11_-K11*H1*P11_;

        % Kalman filtering of others veicles->1
        X21_=zeros(4,num_vehicle);
        P21_=cell(1,num_vehicle);
        K21=cell(1,num_vehicle);
        cur_u1=u1(:,i);
        for j=1:num_vehicle
            X21_(:,j) = F*X21{j}(:,i)+B*cur_u1;
            P21_{j} = F*P21{j}*F'+C*Q1*C';
            K21{j} = P21_{j}*H2'/(H2*P21_{j}*H2'+R21);
            X21{j}(:,i+1) = X21_(:,j)+K21{j}*(Y21{j}(:,i+1)-H2*X21_(:,j));
            P21{j} = P21_{j}-K21{j}*H2*P21_{j};
        end

        %Buffer data sequence
        if(i==buffer_size) %When the time firstly reach the buffer size, put previous buffer_size data into buffer
            for k=1:num_vehicle
                DataSeq_buffer{k}=Y21{k}(:,2:buffer_size+1); %The data in the first clock are all the same, so abandon it
            end
        elseif(i>buffer_size) %Later we just need to remove the oldest data and add a new one into buffer, it works like a FIFO
            for k=1:num_vehicle
                temp_buffer=DataSeq_buffer{k}(:,2:buffer_size);
                DataSeq_buffer{k}=[temp_buffer, Y21{k}(:,i+1)];
            end  
        end
        
        %Check the data after adding malicious information
        xtemp_scatter=zeros(2,num_vehicle);
        vtemp_scatter=zeros(2,num_vehicle);
        for j=1:num_vehicle
        	xtemp_scatter(:,j)=Y21{j}([1,3],i+1);
        	vtemp_scatter(:,j)=Y21{j}([2,4],i+1);
        end
        
        
%         figure;
%         scatter(xtemp_scatter(1,:),xtemp_scatter(2,:),'b');
%         hold on;
%         scatter(xtemp_scatter(1,malicious_index(1:num_malicious)),xtemp_scatter(2,malicious_index(1:num_malicious)));
        % figure;
        % scatter(vtemp_scatter(1,:),vtemp_scatter(2,:),'b');

        %Getting the global estimation without or with filtering
        for j=num_minvehi:num_vehicle
            if(strcmp(filter_mode,'None')||strcmp(filter_mode,'Al')) %Without filtering
                Pg=eye(4)/P11;
                for k=1:j
                    Pg=Pg+eye(4)/P21{k};
                end
                A11_=(eye(4)/P11)/Pg;
                A21_=cell(1,j);
                X_1{j}(:,i+1)=A11_*X11(:,i+1);
                for k=1:j
                    A21_{k}=(eye(4)/P21{k})/Pg;
                    X_1{j}(:,i+1)=X_1{j}(:,i+1)+A21_{k}*X21{k}(:,i+1);
                end
                All_X1{2,j}(:,i+1)=X_1{j}(:,i+1); %Move the estimation to the container for later comparision    
            end
            if(strcmp(filter_mode,'RML')||strcmp(filter_mode,'Al'))
                [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21); %Combine self-estimation with others' estimation
                [pos_ML, vel_ML]=Robust_ML(X_est,P_est);
                X_1{j}(:,i+1)=[pos_ML(1);vel_ML(1);pos_ML(2);vel_ML(2)];
                All_X1{3,j}(:,i+1)=X_1{j}(:,i+1);    
            end
            if(strcmp(filter_mode,'LMS')||strcmp(filter_mode,'All'))
                [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21);
                [pos_LMS, vel_LMS]=LMS(X_est,P_est);
                X_1{j}(:,i+1)=[pos_LMS(1);vel_LMS(1);pos_LMS(2);vel_LMS(2)];
                All_X1{4,j}(:,i+1)=X_1{j}(:,i+1);    
            end
            if(strcmp(filter_mode,'MAE')||strcmp(filter_mode,'All'))
                [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21);
                [pos_MAE, vel_MAE]=MAE(X_est,P_est);
                X_1{j}(:,i+1)=[pos_MAE(1);vel_MAE(1);pos_MAE(2);vel_MAE(2)];
                All_X1{5,j}(:,i+1)=X_1{j}(:,i+1);    
            end
            if(strcmp(filter_mode,'SeqMMSE')||strcmp(filter_mode,'Al'))
                [X_1{j}(:,i+1),false_pos_count(j),false_neg_count(j),total_trust_val{j}]=SeqDetector("SeqMMSE",x1,i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,var_mea,var_self,F,B,ori_u1,total_trust_val{j},false_pos_count(j),false_neg_count(j),gt_trust,test_index);
                All_X1{6,j}(:,i+1)=X_1{j}(:,i+1); 
            end
            if(strcmp(filter_mode,'Dead_Reckon')||strcmp(filter_mode,'Al'))
                [X_1{j}(:,i+1),false_pos_count(j),false_neg_count(j),total_trust_val{j}]=SeqDetector("Dead_Reckon",x1,i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,var_mea,var_self,F,B,ori_u1,total_trust_val{j},false_pos_count(j),false_neg_count(j),gt_trust,test_index);
                All_X1{7,j}(:,i+1)=X_1{j}(:,i+1); 
            end
            if(strcmp(filter_mode,'SeqResE')||strcmp(filter_mode,'All'))
                [X_1{j}(:,i+1),false_pos_count(j),false_neg_count(j),total_trust_val{j}]=SeqDetector("SeqResE",x1,i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,var_mea,var_self,F,B,ori_u1,total_trust_val{j},false_pos_count(j),false_neg_count(j),gt_trust,test_index);
                All_X1{8,j}(:,i+1)=X_1{j}(:,i+1); 
            end
            if(strcmp(filter_mode,'All_benign')||strcmp(filter_mode,'Al'))
 
            end
        end
        
    end

    %Caculate TPR,FPR,TNR,FNR for detection methods
    false_pos_ratio=false_pos_count/(time*((size-buffer_size+1)*num_vehicle-Mal_time_count*num_malicious));
    false_neg_ratio=false_neg_count/(time*Mal_time_count*num_malicious);
    TPR=1-false_neg_ratio(num_vehicle)
    FPR=false_pos_ratio(num_vehicle)
    TNR=1-false_pos_ratio(num_vehicle)
    FNR=false_neg_ratio(num_vehicle)

    %Caculate the RMSE for each methods
    t_=10:size;
    if(strcmp(filter_mode,'All'))
        All_msex=cell(1,num_filter+2);
        All_msev=cell(1,num_filter+2);
        for i = 2:num_filter+2
            All_msex{i}=zeros(1,num_vehicle);
            All_msev{i}=zeros(1,num_vehicle);
            for j=num_minvehi:num_vehicle
                [All_msex{i}(j),All_msev{i}(j)]=MSE(All_X1{i,j}(:,t_),x1(:,t_));
            end
            All_msex_{i} = All_msex_{i} + All_msex{i};
            All_msev_{i} = All_msev_{i} + All_msev{i};
        end 
    else
        msex=zeros(1,num_vehicle);
        msev=zeros(1,num_vehicle);
        for i = 1:num_vehicle
            [msex(i),msev(i)]=MSE(X_1{i}(:,t_),x1(:,t_));
        end
        msex_ = msex_ + msex;
        msev_ = msev_ + msev; 
    end
    
    %Plot the estimated trajectories of each methods
    % figure;
    % hold on;
    % grid on;
    % box on;
    % % for i=1:num_filter+2
    % %     plot(All_X1{i,num_vehicle}(1,:),All_X1{i,num_vehicle}(3,:));
    % % end
    % for i=[1 4 5 7 8]
    %     if(i==4||i==5)
    %         lineshape='--';
    %         linewidth=1;
    %     elseif(i==7||i==8)
    %         lineshape='-.';
    %         linewidth=1.5;
    %     elseif(i==1)
    %         lineshape='-';
    %         linewidth=1.5;
    %     end
    %     plot(All_X1{i,num_vehicle}(1,20:size),All_X1{i,num_vehicle}(3,20:size),lineshape,'Linewidth',linewidth);
    % end
    % % plot(X11(1,:),X11(3,:),'k');
    % title(['Attack\_mode: ' space_attack_mode time_attack_mode  ', Total:' num2str(num_vehicle) ', NumMal:' num2str(num_malicious) ', VarMea:' num2str(var_mea) ', VarMal:' num2str(mal_var_coef*var_mea) ', Devi1:' num2str(collu_design_mal_devi_coef) ', Devi2:' num2str(collu_rand_mal_devi_coef) ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
    % xlabel('X direction (m^2)');
    % ylabel('Y direction (m^2)');
    % legend('Groundtruth','LMS','MAE','DMMSD(Proposed)','MRED(Proposed)');
    % set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',10);

    %Plot velocity
%     figure;
%     hold on;
%     for i=1:num_filter+2
%         plot(All_X1{i,num_vehicle}(2,:),All_X1{i,num_vehicle}(4,:));
%     end
%     plot(X11(1,:),X11(3,:),'k');
%     legend('Groundtruth','Without filtering','RML','LMS','MAE','SeqMMSE','Dead Reckon','SeqResE','Self-KF');

end
%% output
if(strcmp(filter_mode,'All'))
    for i = 2:num_filter+2
        All_msex_{i} = All_msex_{i}./randAver_times;
        All_msev_{i} = All_msev_{i}./randAver_times;
    end
else
    msex_ = msex_ ./ randAver_times;
    msev_ = msev_ ./ randAver_times;
end

RMSE(1)=All_msex_{9}(num_vehicle);
RMSE(2)=All_msex_{4}(num_vehicle);
RMSE(3)=All_msex_{5}(num_vehicle);
RMSE(4)=All_msex_{7}(num_vehicle);
RMSE(5)=All_msex_{8}(num_vehicle);

% figure
%     title(['Attack\_mode: ' space_attack_mode time_attack_mode  ', NumMal:' num2str(num_malicious) ', VarMea:' num2str(var_mea) ', VarMal:' num2str(mal_var_coef*var_mea) ', Devi1:' num2str(collu_design_mal_devi_coef) ', Devi2:' num2str(collu_rand_mal_devi_coef) ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
% hold on
% box on
% grid on
% if(strcmp(filter_mode,'All'))
%     for i=[9 4 5 7 8]
% %     for i=2:num_filter+2
%         plot(num_minvehi+1:(num_vehicle+1),All_msex_{i}(num_minvehi:num_vehicle),'Linewidth',2)
%     end
%     legend('RMSE of position--All Benign','RMSE of position--LMS','RMSE of position--MAE','RMSE of position--DMMSD(Proposed)','RMSE of position--MRED(Proposed)');
%     % legend('RMSE of position--Without filtering','RMSE of position--RML','RMSE of position--LMS','RMSE of position--MAE','RMSE of position--SeqMMSE','RMSE of position--Dead Reckon','RMSE of position--SeqResE','RMSE of position--All Benign');
%     xlabel('Number of Vehicles');
%     ylabel('RMSE');
%     ylim([0 2]);
%     set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',10);

% %     figure
% %     hold on
% %     grid on
% %     for i=2:num_filter+2
% %         plot(num_minvehi+1:(num_vehicle+1),All_msev_{i}(num_minvehi:num_vehicle))
% %     end
% %     legend('RMSE of speed--Without filtering','RMSE of speed--RML','RMSE of speed--LMS','RMSE of speed--RML+SeqMMSE');
% %     xlabel('Number of Vehicles')
% %     ylabel('RMSE')
% else
%     plot(num_minvehi+1:(num_vehicle+1),msex_(num_minvehi:num_vehicle),'b-')
%     plot(num_minvehi+1:(num_vehicle+1),msev_(num_minvehi:num_vehicle),'r-')
%     legend('RMSE of position','RMSE of speed');
%     xlabel('Number of Vehicles')
%     ylabel('RMSE')
% end

end


function [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21)
    X_est=zeros(4,j);
    for k=1:j
        X_est(:,k)=X21{k}(:,i+1);
    end
    X_est=[X_est, X11(:,i+1)];
    P_est=cat(2,P21,{P11});
end
