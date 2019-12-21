function [mseX,mseV] =MSE(X,Y)

if size(X)~=size(Y)
    disp('size not equ')
    return
end
d = X - Y;
% %original wrong RMSE computation
mseX = sum(sqrt(d(1,:).^2 + d(3,:).^2))/size(X,2);
mseV = sum(sqrt(d(2,:).^2 + d(4,:).^2))/size(X,2);

%right definition computation
% mseX=sqrt(sum(d(1,:).^2+ d(3,:).^2)/size(X,2));
% mseV=sqrt(sum(d(2,:).^2+ d(4,:).^2)/size(X,2));
end

