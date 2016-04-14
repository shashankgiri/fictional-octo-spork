function [xyz,nxyz,g,ng,p,np,Nn,nNn,Vb,nVb]=plane_fit(norm,basis,n,XYZ,Err)
clear pp RR VV nn N xyz;
valid_points=zeros(n,1);
    G=basis*norm;
   for m=1:n;
   if  abs(G(1,m))>Err
       valid_points(m,1)=valid_points(m,1)+1;
   end   
   end
   non_valid_points=ones(n,1)-valid_points;
   N1=find(valid_points);
   N2=find(non_valid_points);
   a=size(N1,1);
   na=size(N2,1);
   xyz=zeros(a,3);
   nxyz=zeros(na,3);
   g=zeros(a,3);
   ng=zeros(na,3);
   NN=norm';
   for s=1:a
   xyz(s,:)=XYZ(N1(s,1),:);
   g(s,:)=NN(N1(s,1),:);
   end  
   for ns=1:na
   nxyz(ns,:)=XYZ(N2(ns,1),:);
   ng(ns,:)=NN(N2(ns,1),:);
   end  
p = mean(xyz,1);
R = bsxfun(@minus,xyz,p);
[Vb,~] = eig(R'*R);
Nn = Vb(:,1);
Vb = Vb(:,2:end);
np = mean(nxyz,1);
nR = bsxfun(@minus,nxyz,np);
[nVb,~] = eig(nR'*nR);
nNn = nVb(:,1);
nVb = nVb(:,2:end);
end