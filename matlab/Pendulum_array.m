addpath ./functions/

Ts = 0.1;

num_modes = 3;

num_row = 3;
num_col = 3;
num_elm = num_row*num_col;

% Create A matrix collection
A = cell(num_modes, 1);
B = cell(num_modes, 1);
Mk = zeros(num_elm);
Mc = zeros(num_elm);
K = 1*rand(num_elm, 4);
C = 0.5*rand(num_elm, 4);
for m_idx = 1:num_modes
    m = 1.5*(1+rand(num_elm, 1))*sqrt(2);
    B{m_idx} = zeros(2*num_elm, num_elm);
    for n = 1:num_elm
        if n+num_col <= num_elm
            Mk(n,n+num_col) = K(n,1)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,1)/m(n);
            Mc(n,n+num_col) = C(n,1)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,1)/m(n);
        end
        if n-num_col > 0
            Mk(n,n-num_col) = K(n,2)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,2)/m(n);
            Mc(n,n-num_col) = C(n,2)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,2)/m(n);
        end
        if mod(n, num_col) ~= 0
            Mk(n,n+1) = K(n,3)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,3)/m(n);
            Mc(n,n+1) = -C(n,3)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,3)/m(n);
        end
        if mod(n-1, num_col) ~= 0
            Mk(n,n-1) = K(n, 4)/m(n);
            Mk(n,n) = Mk(n,n) - K(n,4)/m(n);
            Mc(n,n-1) = -C(n,4)/m(n);
            Mc(n,n) = Mc(n,n) - C(n,4)/m(n);
        end
        B{m_idx}(2*n, n) = 1/m(n);
    end
    A{m_idx} = remapDynamics([zeros(num_elm), eye(num_elm); Mk, Mc]);
    sys_d = c2d(ss(A{m_idx}, B{m_idx}, eye(num_elm*2), []), Ts);
    B{m_idx} = sys_d.B;
    A{m_idx} = sys_d.A;
end
[A, B] = formatDynamics(A, B);

% % Create B matrix collection
% B = cell(num_elm, 1);
% for a_idx = 1:num_elm
%     B{a_idx} = cell(num_modes, 1);
%     for m_idx = 1:num_modes
%         B{a_idx}{m_idx} = [0;1];
%     end
% end

% Create state constraint collection
X = cell(num_elm, 1);
for a_idx = 1:num_elm
    X{a_idx} = cell(num_modes, 1);
    for m_idx = 1:num_modes
        X{a_idx}{m_idx} = Polyhedron([eye(2);-eye(2)], 2*[1;1;1;1]);
    end
end

% Create input constraint collection
U = cell(num_elm, 1);
for a_idx = 1:num_elm
    U{a_idx} = cell(num_modes, 1);
    for m_idx = 1:num_modes
        U{a_idx}{m_idx} = Polyhedron([1;-1], 20*[1, 1]');
    end
end

% Min dwell time of 4. Complete graph.
G = cell(num_elm, 1);
for a_idx = 1:num_elm
    G{a_idx} = DirectedGraphWith({Node(1, 2, 2),...
                                  Node(1, 3, 2),...
                                  Node(1, 4, 2),...
                                  Node(1, [5,7], 2),...
                                  Node(1, [6, 13], 2),...
                                  Node(1, [7, 13], 2),...
                                  Node(2, 8, 2),...
                                  Node(2, 9, 2),...
                                  Node(2, 10, 2),...
                                  Node(2, [11, 13], 2),...
                                  Node(2, [1, 12], 2),...
                                  Node(2, [1, 13], 2),...
                                  Node(3, 14, 2),...
                                  Node(3, 15, 2),...
                                  Node(3, 16, 2),...
                                  Node(3, [1,17], 2),...
                                  Node(3, [7,18], 2),...
                                  Node(3, [1,7], 2)});
end

system = BuildSystem(A, B, X, U, G); 
system{1}.graph.node{6}.save_fig_to = 'fig_1_6_';

plot_inner = false;
plot_outer = true;
par_inner = false;
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
