%Sequential malicious data detector
function [State_est,false_pos_count,false_neg_count,total_trust_val] = SeqDetector(mode,x1,i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,Var_mea,Var_self,F,B,ori_u1,total_trust_val,false_pos_count,false_neg_count,gt_trust,test_index)
	
    if(i>=buffer_size)
        % If_Urgent=false;
        % if(strcmp(mode,"Dead_Reckon"))
        %     [update_trust_table,If_Urgent]=Dead_Reckon(x1,[DataSeq_buffer(1:j) Y11(:,i-buffer_size+2:i+1)],i,dt,F,B,ori_u1,Var_mea,Var_self,gt_trust{j}); %Use Multi_Seq detection algorithm
        % %If_Urgent: when it's 'true', use the current update_trust_table to pick vehicles or use accumulate total trust values
        % elseif(strcmp(mode,"SeqMMSE"))
        %     update_trust_table=ARMMSE(DataSeq_buffer,Var_mea,j,0.4); %Use SeqMMSE detection algorithm to update trust table
        % elseif(strcmp(mode,"SeqResE"))
        %     update_trust_table=SeqResE(x1,[DataSeq_buffer(1:j) Y11(:,i-buffer_size+2:i+1)],Var_mea,Var_self,gt_trust{j},2.4);
        % end
        % total_trust_val=total_trust_val+update_trust_table; %Accumulate the trust value for each vehicles
        % false_pos_count=false_pos_count+sum((xor(update_trust_table,gt_trust{j}))&gt_trust{j});  %regard right as wrong
        % false_neg_count=false_neg_count+sum((xor(update_trust_table,gt_trust{j}))&update_trust_table); %regard wrong as right
    end
    if(i<buffer_size) %Use a robust filtering algorithm before buffer is ready
        [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21);
        [pos_ML,vel_ML]=Robust_ML(X_est,P_est);
        State_est=[pos_ML(1);vel_ML(1);pos_ML(2);vel_ML(2)];
    else %Use the optimal combination when buffer is full and detection is done
        If_Urgent=false;
        if(strcmp(mode,"Dead_Reckon"))
            [update_trust_table,If_Urgent]=Dead_Reckon(x1,[DataSeq_buffer(1:j) Y11(:,i-buffer_size+2:i+1)],i,dt,F,B,ori_u1,Var_mea,Var_self,gt_trust{j}); %Use Multi_Seq detection algorithm
        %If_Urgent: when it's 'true', use the current update_trust_table to pick vehicles or use accumulate total trust values
        elseif(strcmp(mode,"SeqMMSE"))
            update_trust_table=ARMMSE(DataSeq_buffer,Var_mea,j,0.4); %Use SeqMMSE detection algorithm to update trust table
        elseif(strcmp(mode,"SeqResE"))
            update_trust_table=SeqResE(x1,[DataSeq_buffer(1:j) Y11(:,i-buffer_size+2:i+1)],Var_mea,Var_self,gt_trust{j},2.4);
        end
        total_trust_val=total_trust_val+update_trust_table; %Accumulate the trust value for each vehicles
        false_pos_count=false_pos_count+sum((xor(update_trust_table,gt_trust{j}))&gt_trust{j});  %regard right as wrong
        false_neg_count=false_neg_count+sum((xor(update_trust_table,gt_trust{j}))&update_trust_table); %regard wrong as right
        trust_table=update_trust_table;
        if(If_Urgent)
            instant_trust_val=total_trust_val.*trust_table;
            Weights=instant_trust_val/sum(instant_trust_val);
        else
            if(sum(trust_table)~=0)
                total_trust_val=trust_table;
                Weights=total_trust_val/sum(total_trust_val); %Use the accumulate trust value to generate weights for each vehicle
            else
                Weights=trust_table;
            end
        end
        Pg=eye(4)/P11;
        for k=1:j
            if(Weights(k)~=0) %Just include data from honest nodes
                Pg=Pg+eye(4)/P21{k};
            end
        end
        A11_=(eye(4)/P11)/Pg;
        A21_=cell(1,j);
        State_est=A11_*X11(:,i+1); %State_est: state estimation of next time frame
        for k=1:j
            if(Weights(k)~=0)
                A21_{k}=(eye(4)/P21{k})/Pg;
            end
        end
        B21_=cell(1,j); %A21_ are computed when all other vehicles are assumed as equal-weighted
        sum_temp1=zeros(4,4); %B21_ are adjusted normalized coefficients using Weights above
        sum_temp2=zeros(4,4);
        %Normalizing 
        if(sum(Weights)~=0)
            for k=1:j
                if(Weights(k)~=0)
                    sum_temp1=sum_temp1+A21_{k};
                    sum_temp2=sum_temp2+A21_{k}.*Weights(k);
                end
            end
            coeff_b=sum_temp1/sum_temp2;
        else
            coeff_b=Weights;
        end

        for k=1:j
            if(Weights(k)~=0)
                B21_{k}=coeff_b*A21_{k}.*Weights(k);
            else
                B21_{k}=zeros(4,4);
            end
            State_est=State_est+B21_{k}*X21{k}(:,i+1);
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

