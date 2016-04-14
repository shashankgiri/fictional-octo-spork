clear M1 M2 pose1 pose2 plane1 plane2 pts1r pts2r ptc1 ptc2 pts1 pts1r pts2 pts22r pts2r pts2rt tf tf1 tf12 tf23 R12 rotm rotm1 
X_bl=[2.4 -0.01 -2.3,pi 0 pi/2 ];
eul=[pi/2 0 pi];
rotm = eul2rotm(eul);
T=[2.4 -0.01 -2.3 1];
A=[rotm,[0 0 0]';T];
tf=affine3d(A);
%%
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',200);
M1=load(scanName1);
pts1 = pointCloud(M1.SCAN.XYZ');
pts1.Normal=pcnormals(pts1);
p1=M1.SCAN.X_wv;
%pts1r=pctransform(pts1,tf);
pts1r=pts1;
[ptc1,nptc1,plane1]=groundPlane(pts1r,4);
%%
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',205);
M2=load(scanName2);
pts2 = pointCloud(M2.SCAN.XYZ');
pts2.Normal=pcnormals(pts2);
p2=M2.SCAN.X_wv;
%pts2r=pctransform(pts2,tf);
pts2r=pts2;
[ptc2,nptc2,plane2]=groundPlane(pts2r,4);
%%
pose1=ssc_tail2tail(p1,p2);
t1=[pose1(1,1) pose1(2,1) pose1(3,1)];
eul1=[pose1(6,1) pose1(5,1) pose1(4,1)];
rotm1 = eul2rotm(eul1);
B=[rotm1,[0 0 0]';[t1 1]];
tf23=affine3d(B);
pts2rt=pctransform(pts2r,tf23);
%%
[R12,eul]=rot(plane1,plane2);
t=-R12*mean(pts2r.Location)'+mean(pts1r.Location)';
C=[R12,[0 0 0]';[t' 1]];
tf12=affine3d(C);
pts2rr=pctransform(pts2r,tf12);
nptc2r=pctransform(nptc2,tf12);
%%
tf1 = pcregrigid(nptc2r,nptc1);
pts22r = pctransform(pts2rr,tf1);
%%
eul2 = affine3dtoeul(tf1);
t2=[tf1.T(4,1);tf1.T(4,2);tf1.T(4,3)];
pose2=[t2;eul2(1,3);eul2(1,2);eul2(1,1)];
%%
figure(1)
pcshowpair(pts1,pts2);
set(gca,'color','black');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on
figure(2)
pcshowpair(pts1r,pts2rt);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(3)
pcshowpair(pts1r,pts22r);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');