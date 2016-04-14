[XYZ1,norm1,n1]=normal_estimation('I:\thesis\mat-files',365);
%[XYZ2,norm2,n2]=normal_estimation('I:\thesis\mat-files',366);
%%
% basisx=[1 0 0];
% [xyzx1,nxyzx1,xg1,nxg1,px1,npx1,Nx1,nNx1,Vx1,nVx1]=plane_fit(norm1,basisx,n1,XYZ1,0.95);
%[xyzx2,nxyzx2,xg2,nxg2,px2,npx2,Nx2,nNx2,Vx2,nVx2]=plane_fit(norm2,basisx,n2,XYZ2,0.95);
%basisy=[0 1 0];
%[xyzy1,nxyzy1,yg1,nyg1,py1,npy1,Ny1,nNy1,Vy1,nVy1]=plane_fit(norm1,basisy,n1,XYZ1,0.95);
%[xyzy2,nxyzy2,yg2,nyg2,py2,npy2,Ny2,nNy2,Vy2,nVy2]=plane_fit(norm2,basisy,n2,XYZ2,0.95);
basisz=[0 0 1];
[xyzz1,nxyzz1,zg1,nzg1,pz1,npz1,Nz1,nNz1,Vz1,nVz1]=plane_fit(norm1,basisz,n1,XYZ1,0.9396);
[xyzz2,nxyzz2,zg2,nzg2,pz2,npz2,Nz2,nNz2,Vz2,nVz2]=plane_fit(ptCloudOutBt.Normal',basisz,ptCloudOutBt.Count,ptCloudOutBt.Location,0.9659);
%%
%plot_normal(xyzx1,xg1,1);
%plot_normal(xyzz1,zg1,1);
plot_normal(nxyzz1,nzg1,1);
plot_normal(nxyzz2,nzg2,2);
% plot_plane_fit(XYZ1,xyzz1,Nz1,pz1,zg1,3);
% plot_plane_fit(XYZ1,nxyzz1,nNz1,npz1,nzg1,4);
%%
[opz1,abcz1,Npz1,ovz1,ozg1]=optimize_plane(xyzz1,Nz1,pz1);
plot_plane_fit(XYZ1,abcz1,Npz1,opz1,ozg1,1);
%%
indices=zeros(n1,1);
dists=zeros(n1,1);
for i=1:n1
[indices(i),dists(i)] = findNearestNeighbors(ptCloudA,ptCloudOutB.Location(i,:),1);
end
%%
ptng1=pointCloud(nxyzz1);
ptng2=pcdownsample(pointCloud(nxyzz2),'random',0.9682);
%%
%[IDX,D] = knnsearch(XYZ1,ptCloudOutB.Location);
[IDX1,D1] = knnsearch(ptng1.Location,ptng2.Location);