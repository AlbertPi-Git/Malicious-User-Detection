%Recursive search function
%This is not the original version, it's added with self-estimation 
function trust_table = ARMMSE_search(pos,var_mea,trust_table,cdf_index)

	total_vehicle=size(pos{1},2);
	other_vehicle=total_vehicle-1;
	buffer_size=size(pos,2);

%     cdf_index=0.2; %cumulative probability index, 2*normcdf(cdf_index)-1 equals to the theoratical probability that MSE of honest subset is within the threshold 
	c_square=(cdf_index*2)/sqrt(total_vehicle-1)+2; %total_vehicle-1: exclude an arbitary estimation to form the subset
    pos_threshold=c_square*var_mea; %MMSE threshold given by observation variance and vehicle number of this subset
    prob_threshold=0.15; %Probability threshold 

	count=zeros(1,other_vehicle);
	total_MMSE=zeros(1,other_vehicle);
    
    %MMSE check and count of all subsets over all dims of buffer size
	for i=1:buffer_size
		pos_subset=cell(1,other_vehicle);
        for j=1:other_vehicle
            pos_subset{j}=pos{i}([1,2],1:other_vehicle); %Exclude the index recording row: 3rd row
			pos_subset{j}(:,j)=[]; %Subset j is the subset of original pos without jth column(that is without jth vehicle)
			pos_subset{j}=[pos_subset{j} pos{i}([1,2],total_vehicle)]; %Add the self-estimation to the subset
        end
        for j=1:other_vehicle
	  		[pos_min, pos_MMSE]=MMSE_compute(pos_subset{j});
	        if(pos_MMSE>pos_threshold) %If MMSE of subset without jth vehicle exceeds the threshold
	        	count(j)=count(j)+1;
	        end
	        total_MMSE(j)=total_MMSE(j)+pos_MMSE;
        end
	end

	malicious_prob=count./buffer_size; %Probability of containing malicious vehicles of all subsets

	[min_prob, min_index]=min(malicious_prob);
	[min_total_MMSE,min_total_index]=min(total_MMSE);
	%If the subset with minimum probability of containing malicious vehicles exceed prob threshold,
	%then get into the recursive function again to keep finding the honest subset.
	if(min_prob>prob_threshold) 
		pos_next=cell(1,buffer_size);
		for i=1:buffer_size
			pos_next{i}=pos{i};
			pos_next{i}(:,min_total_index)=[];
		end	%Choose the subset with min prob as the next input
		trust_table=ARMMSE_search(pos_next,var_mea,trust_table,cdf_index);
	else %Didn't exceed prob threshold, then we find a honest subset
		for i=1:other_vehicle
			if(i~=min_index)
				trust_table(pos{1}(3,i))=1; %Mark all vehicles in this subset as honest in trust table
			end								%3rd row is the original index of each vehicle
		end
	end

	%After finding honest subset, check the vehicle that was excluded in this depth is honest or not. 
	recheck_pos_subset=cell(1,buffer_size);
	for i=1:buffer_size
		recheck_pos_subset{i}=pos{i}(:,[min_index,total_vehicle]); %Add the exluded one and self-estimation to recheck subset
	end
	for i=1:buffer_size
		for j=1:other_vehicle
			if(trust_table(pos{1}(3,j))==1) %Add those honest ones to recheck subset
				recheck_pos_subset{i}=cat(2,recheck_pos_subset{i},pos{i}(:,j));
			end
		end
	end

	recheck_count=0;
    num_recheck=size(recheck_pos_subset{i},2);
	recheck_c_square=(cdf_index*2)/sqrt(num_recheck)+2;
	recheck_threshold=recheck_c_square*var_mea; %The number changed, so a new MMSE threshold is needed
	%MMSE recheck but no need to divide into subsets again
	for i=1:buffer_size
  		[pos_min, pos_MMSE]=MMSE_compute(recheck_pos_subset{i});
        if(pos_MMSE>recheck_threshold) 
        	recheck_count=recheck_count+1;
        end
    end
    recheck_prob=recheck_count/buffer_size;
    if(recheck_prob<prob_threshold) %Decide the excluded one is honest or not
    	trust_table(min_index)=1;
    end

end

function [pos_min,pos_MMSE]=MMSE_compute(pos)
	total_vehicle=size(pos,2);
	pos_min=sum(pos,2)/total_vehicle;
	pos_MMSE=sum(sum((pos-pos_min).^2))/total_vehicle;
end

