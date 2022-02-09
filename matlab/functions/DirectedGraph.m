function G = DirectedGraph(E, L)
G.L = L;
G.E = E;
G.nummodes = VerifyLabels(L);
G.numnodes = numel(L);
VerifyDims(E,L);
end

function maxidx = VerifyLabels(L)
maxidx = max(L);
num_found = 0;
for i=1:maxidx
    idx = find(i==L);
    if isempty(idx)
        error("Labels must only contain sequential integers starting at 1")
    end
    num_found = num_found + numel(idx);
end
if num_found ~= numel(L)
    error("Labels must only contain sequential integers starting at 1")
end
end

function VerifyDims(E, L)
if size(E,1) ~= size(E,2)
    error("Edge matrix must be square")
end
if size(E,1) ~= numel(L)
    error("Edge matrix must have the same diminsion as the Labels list")
end
end