function x = InsertInto(x, field, z)
if field=="safe_sets"
    for n=1:x.graph.numnodes
        x.graph.node{n}.S = z(n);
    end
end
end