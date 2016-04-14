function I = mi(x,tf,SB1,nptc2,xlimit,ylimit)
tx = x(1,1);
ty = x(2,1);
r =  tf;
tz=0;%t(3,1);
eul=[r,0,0];
%eul=[r,eul1(1,2),eul1(1,3)];
R=eul2rotm(eul);
B=[R,[0 0 0]';[tx ty tz 1]];
tform=affine3d(B);
 nptc22=pctransform(nptc2,tform);
[~,SB2]=grid2D(nptc22.Location,xlimit,ylimit);
SB1 = histeq(SB1);
SB2 = histeq(SB2);
H = histcounts2(SB1,SB2,256,'BinMethod','fd');%,'Normalization','probability');
H = imgaussfilt(H,3);
% joint_hist=hist2(SB1,SB2,256);
H=H/sum(sum(H));
[a, b]=size(H);

% H=joint_hist/sum(sum(joint_hist));
 
y_marg=sum(H,1); 
x_marg=sum(H,2);

Hx=0;
Hy=0;
for i=1:b
    if y_marg(i)~=0
       Hy = Hy -(y_marg(i)*(log2(y_marg(i))));
    end
end
for j=1:a
    if x_marg(j)~=0
       Hx = Hx -(x_marg(j)*(log2(x_marg(j))));
    end
end

H_xy = -sum(sum(H.*(log2(H+(H==0)))));
I =-(Hx + Hy)/H_xy;
end