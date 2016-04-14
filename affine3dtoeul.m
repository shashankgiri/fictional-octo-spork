function eul=affine3dtoeul(tform)
rotm=[tform.T(1,1) tform.T(1,2) tform.T(1,3); tform.T(2,1) tform.T(2,2) tform.T(2,3); tform.T(3,1) tform.T(3,2) tform.T(3,3)];
eul = rotm2eul(rotm);
end