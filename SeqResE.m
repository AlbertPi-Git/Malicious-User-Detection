%Sequential Residual Error Average algorithm
% function  [trust_table,If_Urgent]=SeqResE(x1,Buffer,Var_mea,Var_self,gt_trust,test_index)

% 	buffer_size=size(Buffer{1},2); %Buffer size of buffer data
% 	total_vehicle=size(Buffer,2); %Include the target vehicle itself

%     %RSE test
%     % Res_index=3;
%     Res_index=test_index;
%     ResE=zeros(4,total_vehicle-1);
%     Expec_ResE=[0,Var_mea+Var_self];
%     Var_ResE=[(Var_mea+Var_self)/buffer_size,2*(Var_mea+Var_self)^2/buffer_size];
%     Confi_level=0.95;
%     ResSE_Lb_Ub=(Expec_ResE(2)/buffer_size).*[chi2inv((1-Confi_level)/2,buffer_size) chi2inv((1+Confi_level)/2,buffer_size)];
%     Res_Test=[false,false];

%     for i=1:total_vehicle-1
%         temp=Buffer{i}([1,3],:)-Buffer{total_vehicle}([1,3],:);
%         ResE([1,2],i)=sum(temp,2)/buffer_size;
%         ResE([3,4],i)=sum(temp.^2,2)/buffer_size;
%         if(abs(ResE(1,i)-Expec_ResE(1))>Res_index*sqrt(Var_ResE(1))||abs(ResE(2,i)-Expec_ResE(1))>Res_index*sqrt(Var_ResE(1)))
%             Res_Test(1)=true;
%         end
%         if(abs(ResE(3,i)-Expec_ResE(2))>Res_index*sqrt(Var_ResE(2))||abs(ResE(4,i)-Expec_ResE(2))>Res_index*sqrt(Var_ResE(2)))
%             Res_Test(2)=true;
%         end
%         % if(ResE(3,i)<ResSE_Lb_Ub(1)||ResE(4,i)<ResSE_Lb_Ub(1)||ResE(3,i)>ResSE_Lb_Ub(2)||ResE(4,i)>ResSE_Lb_Ub(2))
%         %     Res_Test(2)=true;
%         % end
%     end
%     [~,min_index(1)]=min(sqrt((ResE(1,:)-Expec_ResE(1)).^2+(ResE(2,:)-Expec_ResE(1)).^2));
%     [~,min_index(2)]=min((ResE(3,:)-Expec_ResE(2)).^2+(ResE(4,:)-Expec_ResE(2)).^2);
%     Assumed_honest_index=min_index;
    
%     ResE_trust_table=ones(2,total_vehicle-1);
%     for i=1:2
%         if(Res_Test(i))
%             cluster_id=kmeans(ResE([2*i-1,2*i],:)',2);
%             for j=1:total_vehicle-1
%                 if(cluster_id(j)~=cluster_id(Assumed_honest_index(i))) %Self-estimation is always trustworthy, thus vehicles in the same cluster with target is honest
%                     ResE_trust_table(i,j)=0;
%                 end
%             end
%         else
%             ResE_trust_table(i,:)=ones(1,total_vehicle-1);
%         end
%     end
    
%     %Generate the trust table of other vehicle from clustering results
%     If_Urgent=false;
%     trust_table=ResE_trust_table(1,:)&ResE_trust_table(2,:);
%     if(sum(trust_table)==0)
%         disp("Trust table is all zeros");
%     end
    
% end

function  [trust_table,If_Urgent]=SeqResE(x1,Buffer,Var_mea,Var_self,gt_trust,test_index)

    buffer_size=size(Buffer{1},2); %Buffer size of buffer data
    total_vehicle=size(Buffer,2); %Include the target vehicle itself

    %RSE test
    Res_index=test_index;
    ResE=zeros(4,total_vehicle-1);
    Expec_ResE=[0,2*Var_mea];
    Var_ResE=[(2*Var_mea)/buffer_size,2*(2*Var_mea)^2/buffer_size];
    Confi_level=0.9999;
    ResSE_Lb_Ub=(Expec_ResE(2)/buffer_size).*[chi2inv((1-Confi_level)/2,buffer_size) chi2inv((1+Confi_level)/2,buffer_size)];
    Res_Test=[false,false];
    reference_id=1;

    for i=1:total_vehicle-1
        temp=Buffer{i}([1,3],:)-Buffer{reference_id}([1,3],:);
        ResE([1,2],i)=sum(temp,2)/buffer_size;
        ResE([3,4],i)=sum(temp.^2,2)/buffer_size;
    end
    ResE(:,reference_id)=[];

    for i=1:total_vehicle-2
        if(abs(ResE(1,i)-Expec_ResE(1))>Res_index*sqrt(Var_ResE(1))||abs(ResE(2,i)-Expec_ResE(1))>Res_index*sqrt(Var_ResE(1)))
            Res_Test(1)=true;
        end
        if(ResE(3,i)<ResSE_Lb_Ub(1)||ResE(4,i)<ResSE_Lb_Ub(1)||ResE(3,i)>ResSE_Lb_Ub(2)||ResE(4,i)>ResSE_Lb_Ub(2))
            Res_Test(2)=true;
        end
    end
    
    ResE_trust_table=ones(2,total_vehicle-1);
    for i=1:2
        if(Res_Test(i))
            cluster_id=kmeans(ResE([2*i-1,2*i],:)',2);
            honest_id=mode(cluster_id);
            for j=1:total_vehicle-2
                if(cluster_id(j)~=honest_id) %Self-estimation is always trustworthy, thus vehicles in the same cluster with target is honest
                    ResE_trust_table(i,j+(j>=reference_id))=0;
                end
            end
            [~,min_Res_index]=min(sum(ResE([2*i-1,2*i],:).^2));
            if(cluster_id(min_Res_index)~=honest_id)
                ResE_trust_table(i,reference_id)=0;
            end
        else
            ResE_trust_table(i,:)=ones(1,total_vehicle-1);
        end
    end
    
    %Generate the trust table of other vehicle from clustering results
    If_Urgent=false;
    trust_table=ResE_trust_table(1,:)&ResE_trust_table(2,:);
    if(sum(trust_table)==0)
        disp("Trust table is all zeros");
    end
    
end


function flag=MMSE_check(pos,cdf_index,variance)

    total_vehicle=size(pos,2);
    c_square=(cdf_index*2)/sqrt(total_vehicle)+2;
    pos_threshold=c_square*variance;

    [~,MMSE]=MMSE_compute(pos);
    if(MMSE>pos_threshold)
        flag=true;
    else
        flag=false;
    end
end

function [pos_min,pos_MMSE]=MMSE_compute(pos)
    total_vehicle=size(pos,2);
    pos_min=sum(pos,2)/total_vehicle;
    pos_MMSE=sum(sum((pos-pos_min).^2))/total_vehicle;
end