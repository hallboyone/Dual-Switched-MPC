addpath ./functions/

Ts = 0.3;

num_modes = 3;

% Number of rows and columns in array. Agents are numbered across the rows.
% [1]    [2]    [3]    ... [nc]
% [nc+1] [nc+2] [nc+3] ... [2*nc]
%  :      :      :     `.   :
num_row = 3;
num_col = 3;
num_elm = num_row*num_col;

% Create A matrix collection
A = cell(num_modes, 1);
B = cell(num_modes, 1);
Mk = zeros(num_elm);
Mc = zeros(num_elm);

% Create four random springs and dampers for each agent. Note some some
% wont be used if they are at an edge.
K = 2.5*rand(num_elm, 4);
C = 0.5*rand(num_elm, 4);

for m_idx = 1:num_modes
    % Masses of each agent for mode m_idx
    m = 1*(1+rand(num_elm, 1))*sqrt(2);
    
    % B matrix for mode m_idx. Init to zeros.
    B{m_idx} = zeros(2*num_elm, num_elm);
    
    % For each agent
    for n = 1:num_elm
        % If there is an agent below (index n+num_col)
        if n+num_col <= num_elm
            Mk(n,n+num_col) = K(n,1)/m(n);   
            Mk(n,n) = Mk(n,n) - K(n,1)/m(n);
            Mc(n,n+num_col) = C(n,1)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,1)/m(n);
        end
        % If there is an agent above (index n-num_col)
        if n-num_col > 0
            Mk(n,n-num_col) = K(n,2)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,2)/m(n);
            Mc(n,n-num_col) = C(n,2)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,2)/m(n);
        end
        % If there is an agent to the right (index n+1)
        if mod(n, num_col) ~= 0
            Mk(n,n+1) = K(n,3)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,3)/m(n);
            Mc(n,n+1) = -C(n,3)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,3)/m(n);
        end
        % If there is an agent to the left (index n-1)
        if mod(n-1, num_col) ~= 0
            Mk(n,n-1) = K(n, 4)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,4)/m(n);
            Mc(n,n-1) = -C(n,4)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,4)/m(n);
        end
        B{m_idx}(2*n, n) = 1/m(n);
    end
    % Dynamics are currently split into Mk and Mc which contain the effects
    % of the springs and dampeners respectivly. Create the resulting
    % dynamics for the states [x1; v1; x2; v2; ...]
    A{m_idx} = remapDynamics([zeros(num_elm), eye(num_elm); Mk, Mc]);
    
    % Convert to discrete time dynamics and save.
    sys_d = c2d(ss(A{m_idx}, B{m_idx}, eye(num_elm*2), []), Ts);
    B{m_idx} = sys_d.B;
    A{m_idx} = sys_d.A;
end

% Dynamics are currently cell arrays where the cells contain the
% centralized dynamics of the corrisponding mode. This function reformats
% the dynamics into nested cell arrays in the form A{mode}{source}{dest}
[A, B] = formatDynamics(A, B);

% Create state constraint collection
X = cell(num_elm, 1);
for a_idx = 1:num_elm
    X{a_idx} = cell(num_modes, 1);
    for m_idx = 1:num_modes
        X{a_idx}{m_idx} = Polyhedron([eye(2);-eye(2)], 2.5*(1+m_idx/(num_modes))*[1;1;1;1]);
    end
end

% Create input constraint collection
U = cell(num_elm, 1);
for a_idx = 1:num_elm
    U{a_idx} = cell(num_modes, 1);
    for m_idx = 1:num_modes
        U{a_idx}{m_idx} = Polyhedron([1;-1], 25*[1, 1]');
    end
end

% Define a graph with 15 nodes for each agent. The Nodes are defined as 
%   Node(node_label,linked_nodes,dimeninsion)
G = cell(num_elm, 1);
for a_idx = 1:num_elm
    G{a_idx} = DirectedGraphWith({Node(1, 2, 2),...
                                  Node(1, 3, 2),...
                                  Node(1, [4,6], 2),...
                                  Node(1, [5, 12], 2),...
                                  Node(1, [6, 12, 5], 2),...
                                  Node(2, 7, 2),...
                                  Node(2, 8, 2),...
                                  Node(2, [9, 11], 2),...
                                  Node(2, [1, 10], 2),...
                                  Node(2, [1, 11], 2),...
                                  Node(3, 12, 2),...
                                  Node(3, 13, 2),...
                                  Node(3, [1,14], 2),...
                                  Node(3, [7,15], 2),...
                                  Node(3, [1,6], 2)});
end

% Create full system
system = BuildSystem(A, B, X, U, G);
for a_idx = 1:num_elm
    for n_idx = 1:G{1}.numnodes
        system{a_idx}.graph.node{n_idx}.save_fig_to = ['sim_fig/fig_', int2str(a_idx), '_',  int2str(n_idx), '_'];
    end
end

plot_inner = false;
plot_outer = true;
par_inner  = false;
system = ComputeSafeSets(system, plot_outer, plot_inner, par_inner);

function M = remapDynamics(M)
num_elm = size(M,1)/2;
idx_map = floor(1:0.5:num_elm+0.5)+repmat([0, num_elm], 1, num_elm);
M = M(idx_map, :);
M = M(:, idx_map);
end

function [formatedA, formatedB] = formatDynamics(A, B)
num_elm = size(A{1},1)/2;
num_mode = numel(A);

formatedA = cell(1, num_elm);
formatedB = cell(1, num_elm);
for i=1:num_elm
    formatedA{i} = cell(1, num_mode);
    formatedB{i} = cell(1, num_mode);
    for m=1:num_mode
        formatedA{i}{m} = cell(1, num_elm);
        for n=1:num_elm
            formatedA{i}{m}{n} = A{m}(1+2*(i-1):2*i, 1+2*(n-1):2*n);
        end
        
        formatedB{i}{m} = B{m}(1+2*(i-1):2*i, i);
    end
end
end
