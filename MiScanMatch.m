clc
clear
close all
X_bl=[2.4 -0.01 -2.3,pi 0 pi/2 ];
eul=[pi/2 0 pi];
rotm = eul2rotm(eul);
T=[2.4 -0.01 -2.3  1];
A=[rotm,[0 0 0]';T];
tf=affine3d(A);
%%
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',200);
M1=load(scanName1);
pts1 = pointCloud(M1.SCAN.XYZ');
pts1.Normal=pcnormals(pts1);
pts1=pctransform(pts1,tf);
[ptc1,nptc1,plane1,p1]=groundPlane(pts1,4);
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',205);
M2=load(scanName2);
pts2 = pointCloud(M2.SCAN.XYZ');
pts2.Normal=pcnormals(pts2);
pts2=pctransform(pts2,tf);
[ptc2,nptc2,plane2,p2]=groundPlane(pts2,4);
xlimit2=nptc2.XLimits(1,2)-nptc2.XLimits(1,1);
ylimit2=nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit=max(xlimit1,xlimit2);
ylimit=max(ylimit1,ylimit2);
[SI1,SB1]=grid2D(nptc1.Location,xlimit,ylimit);
[SI2,SB2]=grid2D(nptc2.Location,xlimit,ylimit);
pose1=ssc_tail2tail(M1.SCAN.X_wv,M2.SCAN.X_wv);
display('Part 1 done');
%%
% [R12,eul]=rot(plane2,plane1);
% t=-R12*p2'+p1';
% pose2=[t',eul(1,3),eul(1,2),eul(1,1)]';
% A=[R12,[0 0 0]';[t' 1]];
% tf12=affine3d(A);
% pts2=pctransform(pts2,tf12);
% nptc2r=pctransform(nptc2,tf12);
%%
tf1 = pcregrigid(pts2,pts1);
eul2 = affine3dtoeul(tf1);
t1=[tf1.T(4,1);tf1.T(4,2);tf1.T(4,3)];
pose2=[t1;eul2(1,3);eul2(1,2);eul2(1,1)];
display('Part 2 done');
%%
step=0.01;
iter=3/step;
% ty=-1.01;
% Ty=zeros(iter,1);
tx=-step;
Tx=zeros(iter,1);
m=zeros(iter,1);
for i=1:iter
tx=tx+step;
Tx(i,1)=tx;
% ty=ty+step;
% Ty(i,1)=ty;
t2=[tx pose1(2,1) pose1(3,1)];
eul1=[pose1(6,1) pose1(5,1) pose1(4,1)];
rotm1 = eul2rotm(eul1);
B=[rotm1,[0 0 0]';[t2 1]];
tf23=affine3d(B);
nptc22=pctransform(nptc2,tf23);
[SI2,SB2]=grid2D(nptc22.Location,xlimit,ylimit);
m(i,1)=mutual_information(SB1,SB2,128);
% t2=[tf1.T(4,1);ty;tf1.T(4,3)];
% B=[rotm,[0 0 0]';[t2' 1]];
% tf2=affine3d(B);
% pts2r=pctransform(pts2,tf23);
% figure
% pcshowpair(pts1,pts2r)
end
%%
display('Part 3 done');
index=find(m==max(m));
TX=Tx(index);
%TY=Ty(index);
figure
plot(Tx,m);
% figure
% plot(Ty,m);
figure
imagesc(SB1)
figure
imagesc(SB2)
%%
