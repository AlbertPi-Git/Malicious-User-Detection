function [pos_LMS, vel_LMS] = LMS(X_est,P_est)

	total_vehicle=size(X_est,2); %Number of vehicles used in LMS estimation, the target vehicle is also involved
	pos=X_est([1,3],:); %Get the positions and velocities from all vehicle
	vel=X_est([2,4],:);

    options=optimset('MaxFunEvals',500,'MaxIter',500);
    F_posmed=@(x) Func_LMS(pos,x);
    F_velmed=@(x) Func_LMS(vel,x);
    pos_LMS=fminsearch(F_posmed,pos(:,total_vehicle),options); %'pos(:,total_vehicle)' is the initial search point use self-estimation
	vel_LMS=fminsearch(F_velmed,vel(:,total_vehicle),options); %It depends on the position of X11 in X_est in Array_combination function at KF_multivehicles 

end

function f = Func_LMS(pos,x)

	total_vehicle=size(pos,2);
	LS=zeros(1,total_vehicle);
	for i=1:total_vehicle
		LS(i)=(pos(1,i)-x(1))^2+(pos(2,i)-x(2))^2;
	end
	f=median(LS);
end

