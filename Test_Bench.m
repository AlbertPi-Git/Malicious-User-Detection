clear;

var_self=16; %Variance of self-observation
var_mea=16;  %Variance of observations from other vehicles
mal_var_coef=0.8; %variance of malicious observaion/variance of normal observation
num_vehicle=20; % Number of maximun other vehicles observing target vehicle
num_minvehi=20; % Number of minimum other vehicles in varying vehicle number simulation
num_malicious = 8; % Number of malicous vehicles
% filter_mode='Dead_Reckon';
% filter_mode='SeqMMSE';
% filter_mode='SeqResE';
filter_mode='All';
space_attack_mode='collu'; %Space attack mode, it can be 'indep' (indepedent or 'uncoordinated' as refered in paper) or 'collu' (colluding or 'coordinated' as refered in paper)
time_attack_mode = 'long_design'; %Temporal attack mode, it can be 'long design' (trajectory attack), 'long rand' (continuous random) or 'short rand' (impulsing random)
collu_rand_mal_devi_coef=10;
collu_design_mal_devi_coef=1.5; %deviation in cooradinated trajectory attack=collu_design_mal_devi_coef*sqrt(var_mea)
buffer_size=16; %Buffer size for sequential detection
randAver_times = 1; %How many times will the whole simulation be execeuted to get average performance results

test_mode='oneshot'; %it can be 'oneshot', 'varying_mali', 'varying_total', 'collu_rand_devi_sweep', 'collu_design_devi_sweep' and 'collu_design_var_sweep'

%Simple oneshot test, no parameter sweeping, use it to get trajectory estimations of all algorithms
if strcmp(test_mode,'oneshot')
    num_minvehil=num_vehicle;
    KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,filter_mode,buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,collu_rand_mal_devi_coef,test_mode);
end

%Num_malicious Sweep
if strcmp(test_mode,'varying_mali') % Need to set num_minvehicle and num_vehicle as the same
    num_minvehi=num_vehicle;
    num_malicious=0:floor((num_minvehi-1)/2);
    size_num_mal=size(num_malicious,2);
    RMSE=zeros(5,size_num_mal);
    for i=1:size_num_mal
        [RMSE(:,i),~,~,~,~]=KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious(i),filter_mode,buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,collu_rand_mal_devi_coef,test_mode);
    end
    figure;
    title(['Attack\_mode: ' space_attack_mode time_attack_mode  ', NumTotal:' num2str(num_vehicle) ', VarMea:' num2str(var_mea) ', VarMal:' num2str(mal_var_coef*var_mea) ', Devi1:' num2str(collu_design_mal_devi_coef) ', Devi2:' num2str(collu_rand_mal_devi_coef) ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
    hold on
    box on
    grid on
    
    plot(num_malicious,RMSE(1,:),'Linewidth',2.5);
    plot(num_malicious,RMSE(2,:),'--o','Linewidth',2.5,'Markersize',8);
    plot(num_malicious,RMSE(3,:),'--*','Linewidth',2.5,'Markersize',12);
    plot(num_malicious,RMSE(4,:),'-s','Linewidth',2.5,'Markersize',8);
    plot(num_malicious,RMSE(5,:),'-x','Linewidth',2.5,'Markersize',12);
    
    legend('RMSE curve of Ground Truth','RMSE curve of LMS','RMSE curve of MMAE','RMSE curve of DMMSD(Proposed)','RMSE curve of MRED(Proposed)');
    xlabel('Number of malicious vehicles','fontsize',20);
    ylabel('RMSE (m)','fontsize',20);
    ylim([0.4 2.2]);
    set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',14);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca,'looseInset',[0 0 0 0]);
    set(gcf, 'PaperPosition', [-0.75 0.2 26.5 26]);
    set(gcf, 'PaperSize', [30 30]);
end

%Num_vehicle sweep
if strcmp(test_mode,'varying_total') % Need to set num_minvehicle and num_vehicle differently
    KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,filter_mode,buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,collu_rand_mal_devi_coef,test_mode); 
end

%Collu_rand devi sweep test
if strcmp(test_mode,'collu_rand_devi_sweep')
    attack_mode = 'collu_rand';
    collu_rand_mal_devi_coef=5:1:15;
    size_rand_devi=size(collu_rand_mal_devi_coef,2);
    for i=1:size_rand_devi
        [TPR(i),FPR(i),TNR(i),FNR(i)]=KF_multivehicles(var_self,var_mea,num_vehicle,num_minvehi,num_malicious,filter_mode,buffer_size,attack_mode,randAver_times,1,collu_rand_mal_devi_coef(i));
    end

    figure;
    hold on;
    grid on;
    plot(collu_rand_mal_devi_coef,TPR);
    plot(collu_rand_mal_devi_coef,FPR);
    xlabel('collu\_rand\_mal\_devi\_coef /Sigma_{mea}');
    legend('TPR','FPR');
end

%Collu_design devi sweep test
if strcmp(test_mode,'collu_design_devi_sweep')
    space_attack_mode='collu';
    time_attack_mode = 'long_design';
    collu_design_mal_devi_coef=0.8:0.2:2.2;     %Change malicious deviation coefficient
    mal_var_coef=1;     %Fix malicious variance coefficient
    size_design_devi=size(collu_design_mal_devi_coef,2);
    TPR_Devi=zeros(3,size_design_devi);
    FPR_Devi=zeros(3,size_design_devi);
    TNR_Devi=zeros(3,size_design_devi);
    FNR_Devi=zeros(3,size_design_devi);
    
    for i=1:size_design_devi
        [~,TPR_Devi(1,i),FPR_Devi(1,i),TNR_Devi(1,i),FNR_Devi(1,i)]=KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,'SeqMMSE',buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef(i),1,test_mode);
        [~,TPR_Devi(2,i),FPR_Devi(2,i),TNR_Devi(2,i),FNR_Devi(2,i)]=KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,'DMMSD',buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef(i),1,test_mode);
        [~,TPR_Devi(3,i),FPR_Devi(3,i),TNR_Devi(3,i),FNR_Devi(3,i)]=KF_multivehicles(var_self,var_mea,mal_var_coef,num_vehicle,num_minvehi,num_malicious,'MRED',buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef(i),1,test_mode);
    end

    figure;
    hold on;
    grid on;
    box on;
    plot(collu_design_mal_devi_coef.*sqrt(var_mea),TPR_Devi(1,:),'--o','Linewidth',2.5,'Markersize',8);
    plot(collu_design_mal_devi_coef.*sqrt(var_mea),TPR_Devi(2,:),'-.*','Linewidth',2.5,'Markersize',12);
    plot(collu_design_mal_devi_coef.*sqrt(var_mea),TPR_Devi(3,:),'-x','Linewidth',2.5,'Markersize',12);
    plot(collu_design_mal_devi_coef.*sqrt(var_mea),FPR_Devi(1,:),'--s','Linewidth',2.5,'Markersize',9);
    plot(collu_design_mal_devi_coef.*sqrt(var_mea),FPR_Devi(2,:),'-.d','Linewidth',2.5,'Markersize',9);
    plot(collu_design_mal_devi_coef.*sqrt(var_mea),FPR_Devi(3,:),'-^','Linewidth',2.5,'Markersize',9);
    xlabel('Malicious deviation in Y direction (m)','Fontsize',20);
    ylabel('TPR & FPR','Fontsize',20);
    legend('TPR of SeqMMSE','TPR of DMMSD(Proposed)','TPR of MRED(Proposed)','FPR of SeqMMSE','FPR of DMMSD(Proposed)','FPR of MRED(Proposed)');
    title(['Attack\_mode: ' time_attack_mode  ', Total:' num2str(num_vehicle) ', NumMal:' num2str(num_malicious) ', VarMea:' num2str(var_mea) ', VarMal:' num2str(mal_var_coef*var_mea)  ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
    set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',14);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca,'looseInset',[0 0 0 0]);
    set(gcf, 'PaperPosition', [-0.75 0.2 26.5 26]);
    set(gcf, 'PaperSize', [30 30]);
end

%Collu_design Var sweep test
if strcmp(test_mode,'collu_design_var_sweep')
    space_attack_mode='collu';
    time_attack_mode = 'long_design';
    collu_design_mal_devi_coef=1.5;     %Fix malicious deviation coefficient
    mal_var_coef=0.5:0.1:1.5;       %Change malicious variance coefficient
    size_design_var=size(mal_var_coef,2);
    TPR_Var=zeros(3,size_design_var);
    FPR_Var=zeros(3,size_design_var);
    TNR_Var=zeros(3,size_design_var);
    FNR_Var=zeros(3,size_design_var);

    for i=1:size_design_var
        [~,TPR_Var(1,i),FPR_Var(1,i),TNR_Var(1,i),FNR_Var(1,i)]=KF_multivehicles(var_self,var_mea,mal_var_coef(i),num_vehicle,num_minvehi,num_malicious,'SeqMMSE',buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,1,test_mode);
        [~,TPR_Var(2,i),FPR_Var(2,i),TNR_Var(2,i),FNR_Var(2,i)]=KF_multivehicles(var_self,var_mea,mal_var_coef(i),num_vehicle,num_minvehi,num_malicious,'DMMSD',buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,1,test_mode);
        [~,TPR_Var(3,i),FPR_Var(3,i),TNR_Var(3,i),FNR_Var(3,i)]=KF_multivehicles(var_self,var_mea,mal_var_coef(i),num_vehicle,num_minvehi,num_malicious,'MRED',buffer_size,space_attack_mode,time_attack_mode,randAver_times,collu_design_mal_devi_coef,1,test_mode);
    end

    figure;
    hold on;
    grid on;
    box on;
    plot(mal_var_coef,TPR_Var(1,:),'--o','Linewidth',2.5,'Markersize',8);
    plot(mal_var_coef,TPR_Var(2,:),'-.*','Linewidth',2.5,'Markersize',12);
    plot(mal_var_coef,TPR_Var(3,:),'-x','Linewidth',2.5,'Markersize',12);
    plot(mal_var_coef,FPR_Var(1,:),'--s','Linewidth',2.5,'Markersize',9);
    plot(mal_var_coef,FPR_Var(2,:),'-.d','Linewidth',2.5,'Markersize',9);
    plot(mal_var_coef,FPR_Var(3,:),'-^','Linewidth',2.5,'Markersize',9);
    xlabel('Variance of malicious observations','Fontsize',20);
    ylabel('TPR & FPR','Fontsize',20);
    legend('TPR of SeqMMSE','TPR of DMMSD(Proposed)','TPR of MRED(Proposed)','FPR of SeqMMSE','FPR of DMMSD(Proposed)','FPR of MRED(Proposed)');
    title(['Attack\_mode: ' time_attack_mode  ', Total:' num2str(num_vehicle) ', NumMal:' num2str(num_malicious) ', VarMea:' num2str(var_mea) ', Mal\_Devi:' num2str(collu_design_mal_devi_coef)  ', Aver:' num2str(randAver_times) ', buffer size:' num2str(buffer_size)]);
    set(gca,'Linewidth',1.4,'GridLineStyle','--','Fontsize',14);
    set(gca,'LooseInset',get(gca,'TightInset'));
    set(gca,'looseInset',[0 0 0 0]);
    set(gcf, 'PaperPosition', [-0.75 0.2 26.5 26]);
    set(gcf, 'PaperSize', [30 30]);
end