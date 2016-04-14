clearvars
clc
close all
L=load('Problem');
X_hl=L.Problem.X_hl;
X_hl(1,4:6)=X_hl(1,4:6)*pi/180;
%%
T_cl=zeros(5,3);
R_cl=cell(5,1);
X_cl=zeros(6,5);
tform=cell(5,1);
K=cell(5,1);
for i=1:5
    k=L.Problem.K(i);
    X_hc = L.Problem.X_hc(i);
    X = X_hc.X_hc;
    X(1,4:6) = X(1,4:6)*pi/180;
    Y=k.K;
    K{i}=Y;
    X_cl=ssc_tail2tail(X',X_hl');
    T_cl(i,:)=X_cl(1:3,1);
    R_cl{i,1}=rotxyz(X_cl(4:6,1));
    A=[R_cl{i,1}',[0; 0; 0];[T_cl(i,:) 1]];
    tf=affine3d(A);
    tform{i}=tf;
end
%%
save('laser2camera.mat', 'T_cl', 'R_cl','tform','K');