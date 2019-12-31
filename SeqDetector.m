%Sequential malicious data detector
function [State_est,false_pos_count,false_neg_count] = SeqDetector(mode,i,j,dt,buffer_size,DataSeq_buffer,X11,Y11,P11,X21,P21,Var_mea,F,B,ori_u1,false_pos_count,false_neg_count,gt_trust)
	
    if(i<buffer_size) %Use a robust filtering algorithm before buffer is ready
        [X_est, P_est]=Array_combination(i,j,X11,P11,X21,P21);
        [pos_ML,vel_ML]=Robust_ML(X_est,P_est);
        State_est=[pos_ML(1);vel_ML(1);pos_ML(2);vel_ML(2)];
    else %Use the optimal combination when buffer is full and detection is done
        if(strcmp(mode,"DMMSD"))
            trust_table=DMMSD([DataSeq_buffer(1:j) Y11(:,i-buffer_size+2:i+1)],i,dt,F,B,ori_u1,Var_mea); 
        elseif(strcmp(mode,"SeqMMSE"))
            trust_table=SeqMMSE(DataSeq_buffer,Var_mea,j,0.8); 
        elseif(strcmp(mode,"MRED"))
            trust_table=MRED([DataSeq_buffer(1:j) Y11(:,i-buffer_size+2:i+1)],Var_mea,2.4);
        end
        false_pos_count=false_pos_count+sum((xor(trust_table,gt_trust{j}))&gt_trust{j});  %FP=number of regarding benign as malicious, predicted malicious vehicle is 0 in update trust table, beign vehicle is 1
        %while positive means prediction is 0 in update trust table, so be careful when compute FP and FN
        false_neg_count=false_neg_count+sum((xor(trust_table,gt_trust{j}))&trust_table); %regard wrong as right
        Pg=eye(4)/P11;
        for k=1:j
            if(trust_table(k)) %Just include data from honest nodes
                Pg=Pg+eye(4)/P21{k};
            end
        end
        A11_=(eye(4)/P11)/Pg;
        State_est=A11_*X11(:,i+1); %State_est: state estimation of next time frame
        A21_=cell(1,j);
        for k=1:j
            if(trust_table(k))
                A21_{k}=(eye(4)/P21{k})/Pg;
                State_est=State_est+A21_{k}*X21{k}(:,i+1);
            end
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

