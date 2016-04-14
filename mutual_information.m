function mi=mutual_information(im1,im2,bin)
joint_hist=hist2(im1,im2,bin);

[a, b]=size(joint_hist);
%joint_hist = smoothts(joint_hist,'g');
%joint_hist=imgaussfilt(joint_hist,5);
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
mi = (Hx + Hy)/H_xy;
%mi = imgaussfilt(mi,5);
end
