clear 
close all
clc
file_path1=('I:\experiments\ScanResults\data_');
l=200;
c=1;
k_max=1;
j_max=5;
mt1=zeros(3,j_max);
mr1=zeros(3,j_max);
mt2=zeros(3,j_max);
mr2=zeros(3,j_max);
mtx1=zeros(k_max,1);
mtx2=zeros(k_max,1);
mty1=zeros(k_max,1);
mty2=zeros(k_max,1);
mtz1=zeros(k_max,1);
mtz2=zeros(k_max,1);
mrx1=zeros(k_max,1);
mrx2=zeros(k_max,1);
mry1=zeros(k_max,1);
mry2=zeros(k_max,1);
mrz1=zeros(k_max,1);
mrz2=zeros(k_max,1);
for k=1:k_max
scanName1 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',l+5*(k-1));
M1=load(scanName1);
pts1 = pointCloud(M1.SCAN.XYZ');
pts1.Normal=pcnormals(pts1);
[ptc1,nptc1,plane1,p1]=groundPlane(pts1,1);
[SI1,SB1]=grid2D(nptc1.Location);
for j=1:j_max
scanName2 = sprintf('%s/SCANS/Scan%04d.mat','I:\thesis\mat-files\',l+5*(k-1)+j);
M2=load(scanName2);
pts2 = pointCloud(M2.SCAN.XYZ');
pts2.Normal=pcnormals(pts2);
[ptc2,nptc2,plane2,p2]=groundPlane(pts2,1);
[SI2,SB2]=grid2D(nptc2.Location);
save('im1','SB1','plane1','p1');
save('im2','nptc2','plane2','p2');

%%
[R12,eul1]=rot(plane1.Normal',plane2.Normal');
 t=-R12*p2'+p1';
 x1=[t(1,1); t(2,1)];
%%
options = psoptimset('SearchMethod', @mi2);
ObjectiveFunction1=@mi2;
[tf,fval1,exitFlag1,output1] = patternsearch(ObjectiveFunction1,0);
save('rz','tf')
%%
x0=[0;0];
options = psoptimset('SearchMethod', @mi);
objectiveFunction2=@mi;
[x,fval2,exitFlag2,output2] = patternsearch(objectiveFunction2,x0);
%%
 R1=[eul1(1,3); eul1(1,2); tf];
 T1=[x; t(3,1)];
 pose1=ssc_tail2tail(M1.SCAN.X_wv,M2.SCAN.X_wv);
 [mt1(:,j),mr1(:,j)]=err(pose1,T1,R1);
tf1=pcregrigid(pts2,pts1);
eul2 = affine3dtoeul(tf1);
R2=[eul2(1,3) eul2(1,2) eul2(1,1)];
T2=tf1.T(4,1:3);
pose2=[T2 R2]';
[mt2(:,j),mr2(:,j)]=err(pose2,T1,R1);
end
mtx1(k,1)=sqrt(mean(mt1(1,:)));
mtx2(k,1)=sqrt(mean(mt2(1,:)));
mty1(k,1)=sqrt(mean(mt1(2,:)));
mty2(k,1)=sqrt(mean(mt2(2,:)));
mtz1(k,1)=sqrt(mean(mt1(3,:)));
mtz2(k,1)=sqrt(mean(mt2(3,:)));
mrx1(k,1)=sqrt(mean(mr1(1,:)));
mrx2(k,1)=sqrt(mean(mr2(1,:)));
mry1(k,1)=sqrt(mean(mr1(2,:)));
mry2(k,1)=sqrt(mean(mr2(2,:)));
mrz1(k,1)=sqrt(mean(mr1(3,:)));
mrz2(k,1)=sqrt(mean(mr2(3,:)));
end
file_name1=[file_path1 num2str(l+5*(k-1)) '-' num2str(l+5*(k-1)+j) '.mat'];
 save(file_name1,'mtx1','mty1','mtz1','mrx1','mry1','mrz1','mtx2','mty2','mtz2','mrx2','mry2','mrz2');