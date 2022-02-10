function S = convexHullOfUnion(sets)
    num_verticies = 0;
    for i=1:numel(sets)
        num_verticies = num_verticies + size(sets(i).V, 1);
    end
    V = zeros(num_verticies, sets(1).Dim);
    idx = 1;
    for i=1:numel(sets)
        V(idx:idx+size(sets(i).V, 1)-1, :) = sets(i).V;
        idx = idx+size(sets(i).V, 1);
    end
    S = Polyhedron(V);
    S.minVRep();
end