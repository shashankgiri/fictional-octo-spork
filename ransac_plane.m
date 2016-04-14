function [xyz,nxyz,gr,ngr,pr,Nr,Vr]=ransac_plane(XYZ,norm,n,ra)
score=zeros(1,ra);
Z=zeros(ra,n);
valid=zeros(ra,n);
in=randi(n,ra,1);
for l=1:ra
for k=1:n
Z(l,k)=norm(in(l,1),:)*norm(k,:)';
if Z(l,k)>0.9848
score(l)=score(l)+1;
valid(l,k)=valid(l,k)+1;
end
end
end
max_num=max(score);
x=find(score==max_num);
qr=find(valid(x,:));
lr=find(valid(x,:)==0);
nqr=find(lr);
t=size(qr,2);
xyz=zeros(t,3);
gr=zeros(t,3);
for i=1:t
    xyz(i,:)=XYZ(qr(1,i),:);
    gr(i,:)=norm(qr(1,i),:);
end
e=size(nqr,2);
nxyz=zeros(e,3);
ngr=zeros(e,3);
for j=1:e
  nxyz(j,:)=XYZ(nqr(1,j),:);
  ngr(j,:)=norm(nqr(1,j),:);
end
pr = mean(xyz,1);
covr= bsxfun(@minus,xyz,pr);
[Vr,~] = eig(covr'*covr);
Nr = Vr(:,1);
Vr = Vr(:,2:end);