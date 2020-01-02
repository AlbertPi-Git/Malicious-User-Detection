%Multi-length sequence detectors
function  [trust_table]=DMMSD(Buffer,cur_clk,dt,F,B,ori_u1,Var_mea)

	buffer_size=size(Buffer{1},2); %Buffer size of buffer data
	total_vehicle=size(Buffer,2); %Include the target vehicle itself
	seq_ini_clk=cur_clk-buffer_size+1; %The time when the first column of buffered data were recorded

    %Using the dynamic model to get the predictions at the current time of all previous states of all vehicles
	Cur_states=Buffer;
	for i=1:total_vehicle
		for j=1:buffer_size
			for k=1:buffer_size-j %Predict the states of all past states at current time by using dynamic model
				Cur_states{i}(:,j)=F*Cur_states{i}(:,j)+B*ori_u1(:,seq_ini_clk+k+j-1); 
			end
		end
	end

    %Long, medium, short sequence test
    Seq_len=[buffer_size];
    num_len=size(Seq_len,2);

    Aver_state=cell(1,num_len); %Average states of predictions and current states
    Aver_pos=cell(1,num_len);
    cluster_id=cell(1,num_len); %Cluster results
    Seq_trust_table=cell(1,num_len); 
    for i=1:num_len
        Aver_state{i}=zeros(4,total_vehicle);
        for j=1:total_vehicle
            Aver_state{i}(:,j)=sum(Cur_states{j}(:,buffer_size-Seq_len(i)+1:buffer_size),2)/Seq_len(i); %Get variance and average state of all vehicle according to predictions above
        end
        Aver_pos{i}=(Aver_state{i}([1,3],:))';
    end
    
    for i=1:num_len
        len=Seq_len(i);
        New_Variance=Var_mea*(6+(len-1)*(2*len-1)*dt^2)/(6*len);
        cdf_index=0.3;
        flag=MMSE_check(Aver_state{i}([1,3],1:total_vehicle-1),cdf_index,New_Variance); %Self-estimation is not included in this check
        if(flag)
            cluster_id{i}=zeros(total_vehicle,1);
            cluster_id{i}=cluster_id{i}+(kmeans(Aver_pos{i}(:,2),2,'Replicates',5)-1);


            seq_honest_id=mode(cluster_id{i});  %Cluster with more vehicles are assumed to be honest
            Seq_trust_table{i}=ones(1,total_vehicle-1);
            for j=1:total_vehicle-1
                if(cluster_id{i}(j)~=seq_honest_id) 
                    Seq_trust_table{i}(j)=0;
                end
            end
        else %Or just use the ARMMSE detection result as the final result
            Seq_trust_table{i}=ones(1,total_vehicle-1);
        end
    end
    
    %Generate the trust table of other vehicle from clustering results
    trust_table=Seq_trust_table{1};
    
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

