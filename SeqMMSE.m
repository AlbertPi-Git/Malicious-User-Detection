%Sequential MMSE detector
%This is not the original version, it's added with self-estimation 
function pos_trust_table = SeqMMSE(Buffer,var_mea,total_vehicle,cdf_index,prob_threshold) 
%Input: Buffer of sequential ARMMSE detector, variance of observation of other results
%Output: Trust table of all vehicles except vehicle 1 itself

	buffer_size=size(Buffer{1},2);
	pos=cell(1,buffer_size); %position
% 	vel=cell(1,buffer_size); %velocity, but in the paper we decide only use position, so all vel related are commented
	for i=1:buffer_size
		pos{i}=zeros(3,total_vehicle);
% 		vel{i}=zeros(3,total_vehicle);
		for j=1:total_vehicle
			pos{i}([1,2],j)=Buffer{j}([1,3],i);
% 			vel{i}([1,2],j)=Buffer{j}([2,4],i);
			pos{i}(3,j)=j; %Record each index of vehicle before entering the search algorithm
% 			vel{i}(3,j)=j; 
		end
	end %Convert the buffered data from vehicle index based cell to buffer dim based cell
	pos_trust_table=zeros(1,total_vehicle);
	pos_trust_table=SeqMMSE_search(pos,var_mea,pos_trust_table,cdf_index,prob_threshold); %Use the recursive function to find the trust value of each vehicle
    
end