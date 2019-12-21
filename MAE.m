function [pos_MAE, vel_MAE] = MAE(X_est,P_est)

	total_vehicle=size(X_est,2); %Number of vehicles used in MAE estimation, the target vehicle is also involved
	pos=X_est([1,3],:); %Get the positions and velocities from all vehicle
	vel=X_est([2,4],:);

    options=optimset('MaxFunEvals',400,'MaxIter',400);
    F_pos=@(x) Func_MAE(pos,x);
    F_vel=@(x) Func_MAE(vel,x);
    pos_MAE=fminsearch(F_pos,pos(:,total_vehicle),options); %'pos(:,total_vehicle)' is the initial search point use self-estimation
	vel_MAE=fminsearch(F_vel,vel(:,total_vehicle),options); %It depends on the position of X11 in X_est in Array_combination function at KF_multivehicles 

end

function f = Func_MAE(pos,x)

	total_vehicle=size(pos,2);
	L1_norm=0;
	for i=1:total_vehicle
		L1_norm=L1_norm+sqrt((pos(1,i)-x(1))^2+(pos(2,i)-x(2))^2);
	end
	f=L1_norm;
end

