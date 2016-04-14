clc;
clearvars
close all
file_path1=('F:\HP\experiments\results\result_');
c=100;
l=200;
k_max=1;
j_max=1;
TX=zeros(c,1);
TY=zeros(c,1);
TZ=zeros(c,1);
RX=zeros(c,1);
RY=zeros(c,1);
RZ=zeros(c,1);
tx=zeros(c,1);
ty=zeros(c,1);
tz=zeros(c,1);
rx=zeros(c,1);
ry=zeros(c,1);
rz=zeros(c,1);
%%
for j=1:j_max

scanName1 = sprintf('%s/SCANS/Scan%04d.mat','F:\',l+25*(j-1));
M1=load(scanName1);
pts1 = pointCloud(M1.SCAN.XYZ');
pts1.Normal=pcnormals(pts1);
%pts1=pcdenoise(pts1);
% pts1o = pcdownsample(pts1,'random',0.25);
[ptc1,nptc1,plane1,p1]=gp(pts1);
gridStep = 0.5;
nptc1 = pcdownsample(nptc1,'gridAverage',gridStep);
%plane1=plane1.Normal';
xlimit1=nptc1.XLimits(1,2)-nptc1.XLimits(1,1);
ylimit1=nptc1.YLimits(1,2)-nptc1.YLimits(1,1);
%%
for k=1:k_max
for i=1:c 
ax=(pi/9)*(1-2*rand);
Tx=2-4*rand;
ay=(pi/9)*(1-2*rand);
Ty=2-4*rand;
az=(pi/12)*(1-2*rand);
Tz=2-4*rand;
Rx=[1 0 0;0 cos(ax) sin(ax);0 -sin(ax) cos(ax)];
Ry=[cos(ay) 0 -sin(ay);0 1 0;sin(ay) 0 cos(ay)];
Rz=[cos(az) sin(az) 0;-sin(az) cos(az) 0;0 0 1];
R=Rz*Ry*Rx;
T=[Tx Ty Tz 1];
A=[R,[0 0 0]';T];
tfr=affine3d(A);
%%
tic;
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','F:\',l+25*(j-1)+k);
M2=load(scanName2);
pts2 = pointCloud(M2.SCAN.XYZ');
%pts2=pcdenoise(pts2);
pts2.Normal=pcnormals(pts2);
pts2=pctransform(pts2,tfr);
%pts2o = pcdownsample(pts2,'random',0.25);
[ptc2,nptc2,plane2,p2]=groundPlane(pts2,4);
nptc2 = pcdownsample(nptc2,'gridAverage',gridStep);
%plane2=plane2.Normal';
xlimit2=nptc2.XLimits(1,2)-nptc2.XLimits(1,1);
ylimit2=nptc2.YLimits(1,2)-nptc2.YLimits(1,1);
xlimit=max(xlimit1,xlimit2);
ylimit=max(ylimit1,ylimit2);
[SI1,SB1]=grid2D(nptc1.Location,xlimit,ylimit);
[SI1R,SB1R]=grid2DR(nptc1.Location,xlimit,ylimit);
%%
[R12,eul1]=rot(plane1,plane2);
 %t=-R12*p2'+p1';
 %t=R12*p1'-p2';
 t=R12*mean(pts1.Location)'-mean(pts2.Location)';
%  x1=[t(1,1); t(2,1)];
 %pose1=ssc_tail2tail(M1.SCAN.X_wv,M2.SCAN.X_wv);
%%
%tf0=[0;0];
ObjectiveFunction1=@(tf)mi2(tf,SB1R,nptc2,xlimit,ylimit);
[tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,0);
%%
x0=[0;0];
objectiveFunction2=@(x)mi(x,tf,SB1,nptc2,xlimit,ylimit);
[x,fval2,exitFlag2,output2] = patternsearch(objectiveFunction2,x0);
%%
TR=[eul1(1,3); eul1(1,2); tf];
eul=[tf eul1(1,2) eul1(1,3)];
rt=eul2rotm(eul);
%t=-rt*p2'+p1';
TT=[x' t(3,1)];
B=[rt,[0 0 0]';[TT 1]];
tform=affine3d(B);
pts22=pctransform(pts2,tform);
%%
TX(i,1)=Tx;
TY(i,1)=Ty;
TZ(i,1)=Tz;
RX(i,1)=ax;
RY(i,1)=ay;
RZ(i,1)=az;
tx(i,1)=TT(1,1);
ty(i,1)=TT(1,2);
tz(i,1)=TT(1,3);
rx(i,1)=TR(1,1);
ry(i,1)=TR(2,1);
rz(i,1)=TR(3,1);
toc;
end
% file_name1=[file_path1 num2str(l+25*(j-1)) '-' num2str(l+25*(j-1)+k) '.mat'];
% save(file_name1,'tx','ty','tz','rx','ry','rz','TX','TY','TZ','RX','RY','RZ','TT','TR');
% %
% hold all
% plot(tx+TX,'*');
% plot(TY,'o');
 end
 end