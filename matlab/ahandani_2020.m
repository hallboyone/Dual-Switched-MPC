% Author  - Richard A. Hall (hallboyone@icloud.com)
% Date    - Feb 21, 2022
% Purpose - To compute the safe-set collections of an externally switched
% linear system with seperate switching signals. System is modeled after 
% Ahandani's 2020 paper.
                    
addpath ./functions/

% Define the dynamics of each agent. A1{mode}{neighbor}
A1 = {{[1 1; 0 1], 0.08*eye(2), zeros(2),    zeros(2)},...
      {[1 1; 0 1], 0.08*eye(2), 0.04*eye(2), zeros(2)},...
      {[1 1; 0 1], zeros(2),    zeros(2),    0.06*eye(2)}};

A2 = {{0.05*eye(2), [2 -0.96; 1 0], zeros(2),    zeros(2)},...
      {zeros(2),    [2 -0.96; 1 0], 0.05*eye(2), 0.07*eye(2)},...
      {zeros(2),    [2 -0.96; 1 0], zeros(2),    0.07*eye(2)}};

A3 = {{zeros(2),    zeros(2),    [1 -0.2; 0 1], 0.1*eye(2)},...
      {0.04*eye(2), 0.04*eye(2), [1 -0.2; 0 1], zeros(2)},...
      {zeros(2),    0.04*eye(2), [1 -0.2; 0 1], zeros(2)}};

A4 = {{zeros(2),    zeros(2), 0.025*eye(2), [1.1 2; 0 0.95]},...
      {0.02*eye(2), zeros(2), 0.025*eye(2), [1.1 2; 0 0.95]},...
      {0.02*eye(2), zeros(2), zeros(2),     [1.1 2; 0 0.95]}};

% A{agent}{mode}{neighbor}
A = {A1, A2, A3, A4};

B1 = {[0.5;1], [0.5;1], [0.5;1]};
B2 = {[1;0], [1;0], [1;0]};
B3 = {[0.5;1], [0.5;1], [0.5;1]};
B4 = {[0; 0.7787], [0; 0.7787], [0; 0.7787]};
% B{agent}{mode}
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

G1 = DirectedGraphWith({Node(1, 2, 2),Node(1, 3, 2),Node(1, [3,4,7], 2),...
                        Node(2, 5, 2),Node(2, 6, 2),Node(2, [6,7,1], 2),...
                        Node(3, 8, 2),Node(3, 9, 2),Node(3, [9,1,4], 2)});

G2 = DirectedGraphWith({Node(1, 2, 2),Node(1, 3, 2),Node(1, [3,4,7], 2),...
                        Node(2, 5, 2),Node(2, 6, 2),Node(2, [6,7,1], 2),...
                        Node(3, 8, 2),Node(3, 9, 2),Node(3, [9,1,4], 2)});

G3 = DirectedGraphWith({Node(1, 2, 2),Node(1, 3, 2),Node(1, [3,4,7], 2),...
                        Node(2, 5, 2),Node(2, 6, 2),Node(2, [6,7,1], 2),...
                        Node(3, 8, 2),Node(3, 9, 2),Node(3, [9,1,4], 2)});
                    
G4 = DirectedGraphWith({Node(1, 2, 2),Node(1, 3, 2),Node(1, [3,4,7], 2),...
                        Node(2, 5, 2),Node(2, 6, 2),Node(2, [6,7,1], 2),...
                        Node(3, 8, 2),Node(3, 9, 2),Node(3, [9,1,4], 2)});
G = {G1, G2, G3, G4};  

% Create the array of agents. Each has a switching graph and three modes.
system = BuildSystem(A, B, X, U, G); 

k = 0;
while true
    fprintf("Outer Iteration %d\n", k);
    k = k+1;

    % Save the old safe-sets to compare later
    old_S = ExtractFrom(system, "safe_sets");

    % Run this twice to return to a valid state (even steps are valid, odd are
    % invalid)
    for i=1:2
        % Extract the union of each agent's safe-sets into a cell array
        U = ExtractFrom(system, "safe_set_union");
        % For each agent, compute its safe-sets using the current safe-set unions
        for a=1:numel(system)
            system{a} = ParNodalSafeSets(system{a}, U, false);
        end
    end

    % Get the newly computed safe-sets and compare with the old safe-sets
    S = ExtractFrom(system, "safe_sets");
    if areEqual(S, old_S, false)
        break
    end
    old_S = S;
end

function is_equal = areEqual(safe_sets, old_sets, plot_tf)
row_len = 0;
for m=1:numel(safe_sets)
    row_len = max(row_len, numel(safe_sets{m}));
end
is_equal = true;
    for m=1:numel(safe_sets)
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
            if isempty(old_sets{1}) || (old_sets{m}{i} ~= safe_sets{m}{i})
                is_equal = false;
            end
        end
    end
if plot_tf
    drawnow
end
end