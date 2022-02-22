function preS = PreviewedPreSet(S, M, W)
% PREVIEWEDPRESET Find the linear pre-set with a known additive
% disturbance
%     System dynamics of the form x(t+1) = Ax(t)+Bu(t)+w(t)

% Pre-set operations without disturbance preview: (S-W)+(-(B*U))*A
% Pre-set operations with disturbance preview:    (S+(-(B*U))-W)*A
preS = S+(-M.B*M.U);
preS.minHRep;
preS = preS - (W);
preS.minHRep;
preS = preS * M.A;
preS.minHRep;
preS = preS & M.X;
preS.minHRep;
end