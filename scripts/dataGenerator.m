clear;
dataRecord = [];
iList=0.03:-0.002:-0.03;
% dataRecord = 
parfor i=1:size(iList,2)
    i1 = iList(i);
    res = comp(0.0850-i1, -0.0750, -0.085-i1, 0.0750, -0.085-i1, -0.0750);
    dataRecord = [dataRecord; [i1, res(2,3), res(3,3), res(4,3)]];
end
dataRecord = double(dataRecord);
% Pfca =  [  0.0850    0.0750   -0.0220
%                 0.0850   -0.0750   -0.0220
%                -0.0850    0.0750   -0.0220
%                -0.0850   -0.0750   -0.0220];
save("comp.mat",'dataRecord');