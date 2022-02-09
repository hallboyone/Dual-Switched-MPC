function A = Agent(G, M)
A.M = M;
A.G = G;

A.nummodes = numel(A.M);
A.nx = M(1).nx;
A.nu = M(1).nu;

if numel(A.M) ~= A.G.nummodes
    error("Labels of graph must match number of modes")
end
end