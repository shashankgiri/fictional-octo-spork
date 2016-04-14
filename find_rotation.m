function [rotm,eul]=find_rotation(V1,V2)
r = vrrotvec(V1,V2);
rotm = vrrotvec2mat(r);
eul = rotm2eul(rotm);
end