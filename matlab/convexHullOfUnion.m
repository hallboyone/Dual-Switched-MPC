function S = convexHullOfUnion(sets)
    num_verticies = 0;
    for i=1:numel(sets)
        for j = 1:numel(sets{i})
            num_verticies = num_verticies + size(sets{i}{j}.V, 1);
        end
    end
    V = zeros(num_verticies, sets{1}{1}.Dim);
    idx = 1;
    for i=1:numel(sets)
        for j = 1:numel(sets{i})
            V(idx:idx+size(sets{i}{j}.V, 1)-1, :) = sets{i}{j}.V;
            idx = idx+size(sets{i}{j}.V, 1);
        end
    end
    S = Polyhedron(V);
    S.minVRep();
end