% Multi-vehicles Attack-resistant Cooperative Tracking
function [RMSE,TPR,FPR,TNR,FNR]=CoopTracking_MalDetection(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,filter_mode,buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,collu_rand_mal_devi_coef,test_mode)
%% Set parameters

dt=0.1; %Measurement interval of self and other observations
t=0:dt:20; %Total simulation time
num_filter=7; %Filter number in comparative simulation of all filter and groundtruth

mal_index_mode = 'random'; %Index method of malicious vehicles, it can be 'sequential', 'random' or 'custom' (e.g. if total vehicle number is 20, malicious vehicle number is 5, then indeice of malicious vehicle are '1,2,3,4,5' if mode is 'sequential', 5 random number in 1-30 if mode is 'random' or 5 mannually chosen numbers if mode is 'custom')

collu_design_mal_deviation=collu_design_mal_devi_coef*sqrt(var_mea);
collu_rand_mal_deviation=collu_rand_mal_devi_coef*sqrt(var_mea);

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
    u1 = u1+alpha*randn(2,size); %Add noise to control sequence

    w1=sqrt(Q1)*randn(4,size); %State noise
    v11=sqrt(R11)*randn(4,size); %Self observation noise
    v21=cell(1,num_vehicle); %Others' observation noise
    for i = 1:num_vehicle
        v21{i}=sqrt(R21)*randn(4,size);
    end

    %Observation results and groundtruth
    x1=zeros(4,size);% Vehicle 1 groundtruth, four rows are "x position, x speed, y position, y speed".
    x1(:,1)=[1;0;5;0];% The initial start point of vehicle 1 is (1,5)

    %Groundtruth (x1) of movement of target vehicle
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

    %Before adding some malicious deviation to assumed malicious data, compute the estimation when all vehicles are benign to use in the performance comparison.
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


    %Add malicious deviation and do kalman filter and malicious user filtering  
    for i=1:(size-1)
        if(i<buffer_size)
            Mal_time_count=0; %Count how many times malicious attack occur after buffer_size if reached
        end
        
        % Attack methods in different attack mode
        if(strcmp(space_attack_mode,'indep'))
            % Indepedent attack mode is removed since our concentration is the colluding (coordinated) attack, if needed please find them in the initial commit.
        elseif(strcmp(space_attack_mode,'collu'))
            if(strcmp(time_attack_mode,'long_design')) 
                for j=1:num_malicious
                    Y21{malicious_index(j)}(:,i+1)=x1(:,i+1)+[0;0;collu_design_mal_deviation;0]+sqrt(R_mal)*randn(4,1);
                end
                Mal_time_count=Mal_time_count+1;
            elseif(strcmp(time_attack_mode,'long_rand'))
                positive_devi=(mod(i,4)<2);
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

        %Get the global estimation without or with filtering
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
                [X_1{j}(:,i+1),false_pos_count(j),false_neg_count(j)]=SeqDetector("SeqMMSE",i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,var_mea,F,B,ori_u1,false_pos_count(j),false_neg_count(j),gt_trust);
                All_X1{6,j}(:,i+1)=X_1{j}(:,i+1);
            end
            if(strcmp(filter_mode,'DMMSD')||strcmp(filter_mode,'All'))
                [X_1{j}(:,i+1),false_pos_count(j),false_neg_count(j)]=SeqDetector("DMMSD",i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,var_mea,F,B,ori_u1,false_pos_count(j),false_neg_count(j),gt_trust);
                All_X1{7,j}(:,i+1)=X_1{j}(:,i+1); 
            end
            if(strcmp(filter_mode,'MRED')||strcmp(filter_mode,'Al'))
                [X_1{j}(:,i+1),false_pos_count(j),false_neg_count(j)]=SeqDetector("MRED",i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,var_mea,F,B,ori_u1,false_pos_count(j),false_neg_count(j),gt_trust);
                All_X1{8,j}(:,i+1)=X_1{j}(:,i+1); 
            end
            if(strcmp(filter_mode,'All_benign')||strcmp(filter_mode,'Al'))
                %Doesn't need to do anything, 'All_benign' result has been saved in All_X1{9,}
            end
        end
        
    end

    %Caculate TPR,FPR,TNR,FNR for detection methods
    false_pos_ratio=false_pos_count/(time*((size-buffer_size+1)*num_vehicle-Mal_time_count*num_malicious)); % FPR=FP/N
    false_neg_ratio=false_neg_count/(time*Mal_time_count*num_malicious); %FNR=FN/P
    TPR=1-false_neg_ratio(num_vehicle); % TPR=1-FNR
    FPR=false_pos_ratio(num_vehicle); 
    TNR=1-false_pos_ratio(num_vehicle); %TNR=1-FPR
    FNR=false_neg_ratio(num_vehicle);
    disp("TPR is: "+num2str(TPR)+", FPR is: "+num2str(FPR)+", TNR is: "+num2str(TNR)+", FNR is: "+num2str(FNR));

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
            All_msev_{i} = All_msev_{i} + All_msev{i}; %All_mse_ is the container to accumulate RMSE over different iterations, while All_mse is just the tmp result in each independent simulation.
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
    
    if strcmp(test_mode,'oneshot')
        % Plot the estimated trajectories of each methods
        figure;
        hold on;
        grid on;
        box on;
        marker_indices=1:5:size-20;
        plot(All_X1{1,num_vehicle}(1,20:size),All_X1{1,num_vehicle}(3,20:size),'Linewidth',1.5);
        plot(All_X1{4,num_vehicle}(1,20:size),All_X1{4,num_vehicle}(3,20:size),'--o','Linewidth',1,'Markersize',8,'MarkerIndices',marker_indices);
        plot(All_X1{5,num_vehicle}(1,20:size),All_X1{5,num_vehicle}(3,20:size),'--*','Linewidth',1,'Markersize',12,'MarkerIndices',marker_indices);
        plot(All_X1{7,num_vehicle}(1,20:size),All_X1{7,num_vehicle}(3,20:size),'-.s','Linewidth',1.5,'Markersize',8,'MarkerIndices',marker_indices);
        plot(All_X1{8,num_vehicle}(1,20:size),All_X1{8,num_vehicle}(3,20:size),'-.x','Linewidth',1.5,'Markersize',12,'MarkerIndices',marker_indices);
        
        xlim([20 550]);
        ylim([4 25]);
        title(['Attack\_mode: ' space_attack_mode time_attack_mode  ', Total:' num2str(num_vehicle) ', NumMal:' num2str(num_malicious) ', VarMea:' num2str(var_mea) ', VarMal:' num2str(mal_var_coef*var_mea) ', Devi1:' num2str(collu_design_mal_devi_coef) ', Devi2:' num2str(collu_rand_mal_devi_coef) ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
        xlabel('X direction (m)','Fontsize',24);
        ylabel('Y direction (m)','Fontsize',24);
        legend('Trajectory of Ground Truth','Trajectory of LMS','Trajectory of MAE','Trajectory of DMMSD(Proposed)');
%         legend('Trajectory of Ground Truth','Trajectory of LMS','Trajectory of MAE','Trajectory of DMMSD(Proposed)','Trajectory of MRED(Proposed)');
        set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',18);
        set(gca,'LooseInset',get(gca,'TightInset'));
        set(gca,'looseInset',[0 0 0 0]);
    end
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

%In terms of RMSE, we only compare the performance of 'All Benign', 'LMS', 'MAE', 'DMMSD' and 'MRED', so only part of RMSE are recorded
RMSE(1)=All_msex_{9}(num_vehicle);
RMSE(2)=All_msex_{4}(num_vehicle);
RMSE(3)=All_msex_{5}(num_vehicle);
RMSE(4)=All_msex_{7}(num_vehicle);
RMSE(5)=All_msex_{8}(num_vehicle);

if strcmp(test_mode,'varying_total')
    figure
        title(['Attack\_mode: ' space_attack_mode time_attack_mode  ', NumMal:' num2str(num_malicious) ', VarMea:' num2str(var_mea) ', VarMal:' num2str(mal_var_coef*var_mea) ', Devi1:' num2str(collu_design_mal_devi_coef) ', Devi2:' num2str(collu_rand_mal_devi_coef) ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
    hold on
    box on
    grid on
    if(strcmp(filter_mode,'All'))
        for i=[9 4 5 7 8]
            plot(num_minvehi+1:(num_vehicle+1),All_msex_{i}(num_minvehi:num_vehicle),'Linewidth',2)
        end
        legend('RMSE of position--All Benign','RMSE of position--LMS','RMSE of position--MAE','RMSE of position--DMMSD(Proposed)','RMSE of position--MRED(Proposed)');
        xlabel('Number of Vehicles');
        ylabel('RMSE');
        ylim([0 2]);
        set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',10);
    
    else
        plot(num_minvehi+1:(num_vehicle+1),msex_(num_minvehi:num_vehicle),'b-')
        plot(num_minvehi+1:(num_vehicle+1),msev_(num_minvehi:num_vehicle),'r-')
        legend('RMSE of position','RMSE of speed');
        xlabel('Number of Vehicles')
        ylabel('RMSE')
    end
end

end


function [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21)
    X_est=zeros(4,j);
    for k=1:j
        X_est(:,k)=X21{k}(:,i+1);
    end
    X_est=[X_est, X11(:,i+1)];
    P_est=cat(2,P21,{P11});
end
