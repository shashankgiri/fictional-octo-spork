ptCloudA=pointCloud(XYZ1);
ptCloudA.Normal=norm1';
ptCA=pcdenoise(ptCloudA);
ptCloudB=pointCloud(XYZt);
ptCloudB.Normal=normt';
ptCloudOutA=pcdownsample(ptCloudA,'random',0.95);
ptCloudOutB=pcdownsample(ptCloudB,'random',0.99664);
[tform,ptCloudOutBt] = pcregrigid(ptCloudOutB,ptCloudA);
%%
figure(1)
pcshowpair(ptCloudA,ptCloudB);
set(gca,'color','black');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on
figure(2)
pcshowpair(ptCloudA,ptCloudOutBt);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(3)
pcshow(ptCA)
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
%%
ptca=ptCloudA;
pA=M1.SCAN.X_wv;
eulA=[pA(6,1) pA(5,1) pA(4,1)];
rotmA = eul2rotm(eulA);
trA=[pA(1,1) pA(2,1) pA(3,1)];
ptcA=ptca.Location;
ptcA=ptcA*rotmA+repmat(trA,ptca.Count,1);
ptcAA=pointCloud(ptcA);
ptcb=ptCloudOutBt;
pB=M2.SCAN.X_wv;
eulB=[pB(6,1) pB(5,1) pB(4,1)];
rotmB = eul2rotm(eulB);
trB=[pB(1,1) pB(2,1) pB(3,1)];
ptcB=ptcb.Location;
ptcB=ptcB*rotmB+repmat(trB,ptcb.Count,1);
ptcBB=pointCloud(ptcB);
figure
pcshowpair(ptcAA,ptcBB);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
%%
basisz=[0 0 1];
[xyzz1,nxyzz1,zg1,nzg1,pz1,npz1,Nz1,nNz1,Vz1,nVz1]=plane_fit(norm1,basisz,n1,XYZ1,0.9659);
[xyzz2,nxyzz2,zg2,nzg2,pz2,npz2,Nz2,nNz2,Vz2,nVz2]=plane_fit(ptCloudOutBt.Normal',basisz,ptCloudOutBt.Count,ptCloudOutBt.Location,0.9659);
%[xyzz2,nxyzz2,zg2,nzg2,pz2,npz2,Nz2,nNz2,Vz2,nVz2]=plane_fit(normt,basisz,nr,XYZt,0.9659);
% [xyz1,nxyz1,gr1,ngr1,pr1,Nr1,Vr1]=ransac_plane(XYZ1,norm1,n1,50);
% [xyzt,nxyzt,grt,ngrt,prt,Nrt,Vrt]=ransac_plane(XYZt,normt,nr,50);
ptA=pointCloud(nxyzz1);
ptB=pointCloud(nxyzz2);
ptA.Normal=nzg1;
ptB.Normal=nzg2;
[tf,ptBt] = pcregrigid(ptB,ptA);
ptBTT = pctransform(ptCloudOutBt,tf);
%%
figure(1)
pcshowpair(ptCloudA,ptCloudB);
set(gca,'color','black');
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on
figure(2)
pcshowpair(ptCloudA,ptCloudOutBt);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');
figure(3)
pcshowpair(ptCloudA,ptBTT);
set(gca,'color','black');
hold off
grid on;
alpha(0.3);
xlabel('X');
ylabel('Y');
zlabel('Z');