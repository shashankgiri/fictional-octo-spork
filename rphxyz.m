M1=load('I:\thesis\mat-files\SCANS\Scan0366.mat');
M2=load('I:\thesis\mat-files\SCANS\Scan0365.mat');
ptXYZ1=pointCloud(M1.SCAN.XYZ');
ptXYZ2=pointCloud(M2.SCAN.XYZ');
% ptXYZ1.Normal=normal_estimation(ptXYZ1.Location,ptXYZ1.Count);
% ptXYZ2.Normal=normal_estimation(ptXYZ2.Location,ptXYZ2.Count);
ptXYZ1.Normal=pcnormals(ptXYZ1);
ptXYZ2.Normal=pcnormals(ptXYZ2);
%%
% [xyz11,nxyz11,gr11,ngr11,pr11,plane11,Vr11]=ransac_plane(ptXYZ1.Location,ptXYZ1.Normal,ptXYZ1.Count,10);
% ptXYZ11ot=pointCloud(nxyz11,'Normal',ngr11);
% [xyz22,nxyz22,gr22,ngr22,pr22,plane22,Vr22]=ransac_plane(ptXYZ2.Location,ptXYZ2.Normal,ptXYZ2.Count,10);
% ptXYZ22ot=pointCloud(nxyz22,'Normal',ngr22);
% %%
% [Rt1,el1]=rot(plane11,plane22);
[tform1,ptXYZ2r] = pcregrigid(ptXYZ2,ptXYZ1);
el1=affine3dtoeul(tform1);
x=tform1.T(4,1);
y=tform1.T(4,2);
z=tform1.T(4,3);
qx=el1(1,3);
qy=el1(1,2);
qz=el1(1,1);
%%
% pA=M1.SCAN.X_wv;
% eulA=[pA(6,1) pA(5,1) pA(4,1)];
% rotOA = eul2rotm(eulA);
% trA=[pA(1,1) pA(2,1) pA(3,1)];
% pB=M2.SCAN.X_wv;
% eulB=[pB(6,1) pB(5,1) pB(4,1)];
% rotOB = eul2rotm(eulB);
% trB=[pB(1,1) pB(2,1) pB(3,1)];
% rotBA=rotOB'*rotOA;
% d=trA-trB;
% ro=rotm2eul(rotBA);