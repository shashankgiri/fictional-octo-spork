function I = MI_2(x,y,tf,SB1,nptc2,xlimit,ylimit)

tx = x;
ty = y;
r =  tf;
tz=0;%t(3,1);
%eul=[r,eul1(1,2),eul1(1,3)];
eul=[r,0,0];
R=eul2rotm(eul);
B=[R,[0 0 0]';[tx ty tz 1]];
tform=affine3d(B);
 nptc22=pctransform(nptc2,tform);
[~,SB2]=grid2D(nptc22.Location,xlimit,ylimit);

joint_hist=hist2(SB1,SB2,256);

[a, b]=size(joint_hist);

H=joint_hist/sum(sum(joint_hist));
 
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
I = (Hx + Hy)/H_xy;
I = sqrt((I/Hx)*(I/Hy));
end