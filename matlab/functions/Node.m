function N = Node(label, edges, nx)
N.label = label;
N.edges = edges;
N.S = Polyhedron.emptySet(nx);
end