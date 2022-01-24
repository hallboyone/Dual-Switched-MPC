% Title   - preSetPreviewedDisturbance.m
% Author  - Richard A. Hall (hallboyone@icloud.com)
% Date    - Jan 20, 2022
% Purpose - To find the pre-set of a Polyhedron under linear dynamics with
% a additive disturbance known to the system beforehand. 

function preS = preSetPreviewedDisturbance(S, A, B, X, U, W)
% PRESETPREVIEWEDDISTURBANCE Find the linear pre-set with a known additive
% disturbance
%     System dynamics of the form x(t+1) = Ax(t)+Bu(t)+w(t)

% Pre-set operations without disturbance preview: (S-W)+(-(B*U))*A
% Pre-set operations with disturbance preview:    (S+(-(B*U))-W)*A
preS = S+(-B*U);
preS.minHRep;
preS = preS - W;
preS.minHRep;
preS = preS*A;
preS.minHRep;
preS = preS&X;
preS.minHRep;
end