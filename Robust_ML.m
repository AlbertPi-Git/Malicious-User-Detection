function [pos_ML, vel_ML] = Robust_ML(X_est,P_est)
	
	total_vehicle=size(X_est,2); %Number of vehicles used in RML estimation, the target vehicle is also involved
	pos=X_est([1,3],:); %Get the positions and velocities from all vehicle
	vel=X_est([2,4],:);
	P_pos=cell(1,total_vehicle);
	P_vel=cell(1,total_vehicle);
	for i=1:total_vehicle %Get the variance of positions and velocities of all vehicle
		P_pos{i}=eye(2);
		P_pos{i}(1,1)=P_est{i}(1,1);
		P_pos{i}(2,2)=P_est{i}(3,3);
		P_vel{i}=eye(2);
		P_vel{i}(1,1)=P_est{i}(2,2);
		P_vel{i}(2,2)=P_est{i}(4,4);
    end
    
    options=optimset('MaxFunEvals',200,'MaxIter',200);
	F_posmin=@(x) -Func_ML(pos,P_pos,x);
	F_velmin=@(x) -Func_ML(vel,P_vel,x);
	pos_ML=fminsearch(F_posmin,pos(:,total_vehicle),options); %'pos(:,total_vehicle)' is the initial search point use self-estimation
	vel_ML=fminsearch(F_velmin,vel(:,total_vehicle),options); %It depends on the position of X11 in X_est in Array_combination function at KF_multivehicles 

end

function f = Func_ML(pos,P,x)

	total_vehicle=size(pos,2);

	f=0;
	for i=1:total_vehicle
		f=f+(exp(-((x(1)-pos(1,i))^2/P{i}(1,1)+(x(2)-pos(2,i))^2/P{i}(2,2))/2))/(2*pi*sqrt(P{i}(1,1)*P{i}(2,2)));
	end
end

