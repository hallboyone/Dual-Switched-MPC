function S = convexHullOfUnion(agent, sets)
    S = repmat(Polyhedron.emptySet(sets(1).Dim), 1, agent.nummodes);
    for l = 1:numel(S)
        num_verticies = 0;
        for n=1:agent.graph.numnodes
            if agent.graph.node(n).label == l
                num_verticies = num_verticies + size(sets(n).V, 1);
            end
        end
        V = zeros(num_verticies, sets(1).Dim);
        idx = 1;
        for n=1:agent.graph.numnodes
            if agent.graph.node(n).label == l
                V(idx:idx+size(sets(n).V, 1)-1, :) = sets(n).V;
                idx = idx+size(sets(n).V, 1);
            end
        end
        S(l) = Polyhedron(V);
        S(l).minVRep();
    end
end