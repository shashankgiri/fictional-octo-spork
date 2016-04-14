function [p,abc,Np,V,points]=optimize_plane(xyz,Nn,pp)
abc=xyz;
aa=size(xyz,1);
Np=Nn;
err=0.5;
while err>0.01
valid_dist=zeros(aa,1);
dd=pp*Np;
EE=abs((abc*Np-repmat(dd,aa,1))/sqrt(sum(Np.^2)));
for ii=1:aa
if EE(ii,1)<err
    valid_dist(ii,1)=valid_dist(ii,1)+1;
end
end
KK=find(valid_dist);
oo=size(KK,1);
points=zeros(oo,3);
for l=1:oo
    points(l,:)=abc(KK(l,1),:);  
end
p = mean(points,1);
R = bsxfun(@minus,abc,p);
[V,~] = eig(R'*R);
nn = V(:,1);
V = V(:,2:end);
abc=points;
Np=nn;
aa=size(abc,1);
err=err-0.01;
end
