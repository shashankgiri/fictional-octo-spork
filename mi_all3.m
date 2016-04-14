function I = mi_all3(x,y,tf,SB1,SRB1,Sib1,SCB1,nptc2,Ref2,int2,rgb2,xlimit,ylimit)
tx = x;
ty = y;
r =  tf;
tz=0;%t(3,1);
eul=[r,0,0];
%eul=[r,eul1(1,2),eul1(1,3)];
R=eul2rotm(eul);
B=[R,[0 0 0]';[tx ty tz 1]];
tform=affine3d(B);
 nptc22=pctransform(nptc2,tform);
[~,SB2]=grid2D(nptc22.Location,xlimit,ylimit);
[~,SRB2] = grid_reflectivity(nptc2.Location,Ref2,xlimit,ylimit);
[~,Sib2] = grid_intensity(nptc2.Location,int2,xlimit,ylimit);
[~,SCB2] = grid_rgb(nptc2.Location,rgb2,xlimit,ylimit);
SB1 = histeq(SB1);
SB2 = histeq(SB2);
SRB1 = histeq(SRB1);
SRB2 = histeq(SRB2);
Sib1 = histeq(Sib1);
Sib2 = histeq(Sib2);
SCB1 = histeq(SCB1);
SCB2 = histeq(SCB2);
Hz = histcounts2(SB1,SB2,128,'BinMethod','fd');%,'Normalization','probability');
Hz = imgaussfilt(Hz,3);
% joint_hist=hist2(SB1,SB2,256);
Hz = Hz/sum(sum(Hz));
[az, bz]=size(Hz);
Hr = histcounts2(SRB1,SRB2,128,'BinMethod','fd');%,'Normalization','probability');
Hr = imgaussfilt(Hr,3);
Hr = Hr/sum(sum(Hr));
[ar, br]=size(Hr);
Hi = histcounts2(Sib1,Sib2,128,'BinMethod','fd');%,'Normalization','probability');
Hi = imgaussfilt(Hi,3);
Hi = Hi/sum(sum(Hi));
[ai, bi]=size(Hi);
Hc = histcounts2(SCB1,SCB2,128,'BinMethod','fd');%,'Normalization','probability');
Hc = imgaussfilt(Hc,3);
Hc = Hc/sum(sum(Hc));
[ac, bc]=size(Hc);
% H=joint_hist/sum(sum(joint_hist));
 
y_margz=sum(Hz,1); 
x_margz=sum(Hz,2);
Hxz=0;
Hyz=0;
y_margr=sum(Hr,1); 
x_margr=sum(Hr,2);
Hxr=0;
Hyr=0;
y_margi=sum(Hi,1); 
x_margi=sum(Hi,2);
Hxi=0;
Hyi=0;
y_margc=sum(Hc,1); 
x_margc=sum(Hc,2);
Hxc=0;
Hyc=0;
for i=1:bz
    if y_margz(i)~=0
       Hyz = Hyz -(y_margz(i)*(log2(y_margz(i))));
    end
end
for i=1:br
    if y_margr(i)~=0
       Hyr = Hyr -(y_margr(i)*(log2(y_margr(i))));
    end
end
for i=1:bi
    if y_margi(i)~=0
       Hyi = Hyi -(y_margi(i)*(log2(y_margi(i))));
    end
end
for i=1:bc
    if y_margc(i)~=0
       Hyc = Hyc -(y_margc(i)*(log2(y_margc(i))));
    end
end
for j=1:az
    if x_margz(j)~=0
       Hxz = Hxz -(x_margz(j)*(log2(x_margz(j))));
    end
end
for j=1:ar
    if x_margr(j)~=0
       Hxr = Hxr -(x_margr(j)*(log2(x_margr(j))));
    end
end
for j=1:ai
    if x_margi(j)~=0
       Hxi = Hxi -(x_margi(j)*(log2(x_margi(j))));
    end
end
for j=1:ac
    if x_margc(j)~=0
       Hxc = Hxc -(x_margc(j)*(log2(x_margc(j))));
    end    
end

% H_xyz = -sum(sum(Hz.*(log2(Hz+(Hz==0)))));
% H_xyr = -sum(sum(Hr.*(log2(Hr+(Hr==0)))));
% H_xyi = -sum(sum(Hi.*(log2(Hi+(Hi==0)))));
% H_xyc = -sum(sum(Hc.*(log2(Hc+(Hc==0)))));
H_xy = -sum(sum(Hz.*(log2(Hz+(Hz==0)))))-sum(sum(Hr.*(log2(Hr+(Hr==0)))))-sum(sum(Hi.*(log2(Hi+(Hi==0)))))-sum(sum(Hc.*(log2(Hc+(Hc==0)))));
% I =-((Hxz + Hyz)/H_xyz) - ((Hxr + Hyr)/H_xyr) - ((Hxi + Hyi)/H_xyi) - ((Hxc + Hyc)/H_xyc);
I = -(Hxz + Hyz + Hxr + Hyr + Hxi + Hyi + Hxc + Hyc)/H_xy;
end