% Author  - Richard A. Hall (hallboyone@icloud.com)
% Date    - Feb 21, 2022
% Purpose - To compute the safe-set collections of an externally switched
% linear system with two seperate switching signals.

% System dynamics are of the form
%    [x1(t+1)] = A11(s1(t))*x1(t) + A12(s1(t))*x2(t) + B1(s1(t))*u1(t)
%    [x2(t+1)] = A22(s2(t))*x2(t) + A21(s2(t))*x1(t) + B2(s2(t))*u2(t)
% where s1(t) (s2(t)) is a switching signal that map the current time to an
% index between 1 and M1 (M2). 
%
% Our objective is to find two safe-set collections for the x1 states and
% x2 states that are indexed by the respective switching signal.
                    
addpath ./functions/

A11 = {[1 1; 0 1], [1 1; 0 1], [1 1; 0 1]};
A12 = {0.08*eye(2), 0.08*eye(2), zeros(2)   };
A13 = {zeros(2)   , 0.04*eye(2), zeros(2)   };
A14 = {zeros(2)   , zeros(2)   , 0.06*eye(2)};

A22 = {[2 -0.96; 1 0], [2 -0.96; 1 0], [2 -0.96; 1 0]};
A21 = {0.05*eye(2), zeros(2)   , zeros(2)   };
A23 = {zeros(2)   , 0.05*eye(2), zeros(2)   };
A24 = {zeros(2)   , 0.07*eye(2), 0.07*eye(2)};

A33 = {[1 0; 0 1], [1 0; 0 1], [1 0; 0 1], [1 0; 0 1]};
A31 = {zeros(2)   , 0.04*eye(2), zeros(2)   };
A32 = {zeros(2)   , 0.04*eye(2), 0.04*eye(2)};
A34 = {0.1*eye(2) , zeros(2)   , zeros(2)   };

A44 = {[1.1 2; 0 0.95], [1.1 2; 0 0.95], [1.1 2; 0 0.95]};
A41 = {zeros(2)   , 0.02*eye(2), 0.02*eye(2)};
A42 = {zeros(2)   , zeros(2)   , zeros(2)   };
A43 = {0.025*eye(2), 0.025*eye(2), zeros(2) };

% A{affected_agent, affecting agent}{mode_of_affected}
A = {A11, A12, A13, A14;
     A21, A22, A23, A24;
     A31, A12, A33, A34;
     A41, A12, A43, A44};


B1 = {[0.5;1], [0.5;1], [0.5;1]};
B2 = {[1;0], [1;0], [1;0]};
B3 = {[0.5;1], [0.5;1], [0.5;1]};
B4 = {[0; 0.7787], [0; 0.7787], [0; 0.7787]};
B = {B1, B2, B3, B4};

X1 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X2 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X3 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X4 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X = {X1, X2, X3, X4};

U1 = {Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]')};
U2 = {Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]')};
U3 = {Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]')};
U4 = {Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]')};
U = {U1, U2, U3, U4};

G1 = DirectedGraphWith({Node(1, 2),Node(1, 3),Node(1, [3,4,7]),...
                        Node(2, 5),Node(2, 6),Node(2, [6,7,1]),...
                        Node(3, 8),Node(3, 9),Node(3, [9,1,4])});

G2 = DirectedGraphWith({Node(1, 2),Node(1, 3),Node(1, [3,4,7]),...
                        Node(2, 5),Node(2, 6),Node(2, [6,7,1]),...
                        Node(3, 8),Node(3, 9),Node(3, [9,1,4])});

G3 = DirectedGraphWith({Node(1, 2),Node(1, 3),Node(1, [3,4,7]),...
                        Node(2, 5),Node(2, 6),Node(2, [6,7,1]),...
                        Node(3, 8),Node(3, 9),Node(3, [9,1,4])});
                    
G4 = DirectedGraphWith({Node(1, 2),Node(1, 3),Node(1, [3,4,7]),...
                        Node(2, 5),Node(2, 6),Node(2, [6,7,1]),...
                        Node(3, 8),Node(3, 9),Node(3, [9,1,4])});
G = {G1, G2, G3, G4};  

% Create the array of two agents. Each has a switching graph and two modes.
system = BuildSystem(A, B, X, U, G); 

W = {repmat(Polyhedron.emptySet(system{1}.nx), system{1}.nummodes, 1),...
     repmat(Polyhedron.emptySet(system{2}.nx), system{2}.nummodes, 1),...
     repmat(Polyhedron.emptySet(system{3}.nx), system{3}.nummodes, 1),...
     repmat(Polyhedron.emptySet(system{4}.nx), system{4}.nummodes, 1)};
S = cell(1,numel(system));
k = 0;
while true
    fprintf("Outer Iteration %d\n", k);
    k = k+1;
    old_S = S;
    for a=1:numel(system)
        S_new = ParNodalSafeSets(system{a}, W{a}, false);
        W{a} = convexHullOfUnion(system{a}, S_new);
    end
    for a=1:numel(system)
        S_new = ParNodalSafeSets(system{a}, W{a}, false);
        W{a} = convexHullOfUnion(system{a}, S_new);
        S{a} = S_new;
    end
    if areEqual(S, old_S, true)
        break
    end
end

function is_equal = areEqual(safe_sets, old_sets, plot_tf)
row_len = 0;
for m=1:numel(safe_sets)
    row_len = max(row_len, numel(safe_sets{m}));
end
is_equal = true;
    for m=1:2
        for i=1:numel(safe_sets{m})
            if plot_tf
                subplot(2,row_len,(m-1)*row_len+i)
                plot(safe_sets{m}(i), 'alpha', 0.2)
                xlim([-2.5, 2.5])
                ylim([-2.5, 2.5])
                xlabel("$c_c$",'Interpreter','latex','fontsize',15);
                ylabel("$c_k$",'Interpreter','latex','fontsize',15);
                title(['$\mathcal{S}_{(', num2str(m),',',num2str(i),')}$'],'Interpreter','latex','fontsize',15)
                hold on
            end
            if isempty(old_sets{1}) || (old_sets{m}(i) ~= safe_sets{m}(i))
                is_equal = false;
            end
        end
    end
if plot_tf
    drawnow
end
end