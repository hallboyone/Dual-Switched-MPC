% Title   - dual_switched_safe_sets.m
% Author  - Richard A. Hall (hallboyone@icloud.com)
% Date    - Jan 20, 2022
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

load_saved_vars = true;
if load_saved_vars
    saved_vars = load('rand_dyn.mat');
    A = saved_vars.A;
    Ac = saved_vars.Ac;
    B = saved_vars.B;
else
    A11 = {0.95*eye(2) + 0.25*(rand(2)-0.5) + [0 1; 0 0], 0.95*eye(2) + 0.25*(rand(2)-0.5) + [0 -1; 0 0]};
    A22 = {0.85*eye(2) + 0.25*(rand(2)-0.5) + [0 -1; 0 0], 0.85*eye(2) + 0.25*(rand(2)-0.5) + [0 1; 0 0]};
    A12 = {0.125*(rand(2)-0.5), 0.125*(rand(2)-0.5)};
    A21 = {0.125*(rand(2)-0.5), 0.125*(rand(2)-0.5)};
    A = {A11, A22};
    Ac = {A12, A21};

    B1 = {[0; 0.5] + 0.1*(rand(2,1)-0.5), [0; 0.75] + 0.1*(rand(2,1)-0.5)};
    B2 = {[0; 0.5] + 0.1*(rand(2,1)-0.5), [0; 0.5] + 0.1*(rand(2,1)-0.5)};
    B = {B1, B2};
end
X = [Polyhedron([eye(2); -eye(2)], [2; 2; 2; 2]), Polyhedron([eye(2); -eye(2)], [2; 2; 2; 2])];
U = [Polyhedron([1; -1], 4*[0.35; 0.5]),Polyhedron([1;-1], 4*[0.5; 0.35])];

% Create the two graphs that the switching signals respects. Each graph is
% made up by nodes that have a label, indicating the corresponding
% dynamics, and an array of successor nodes.
G1 = DirectedGraphWith([Node(1, 2),Node(1, 3),Node(1, [3,4]),...
                        Node(2, 5),Node(2, 6),Node(2, 7),Node(2, [7,1])]);

G2 = DirectedGraphWith([Node(1, 2),Node(1, 3),Node(1, 4),Node(1, [4,5]),...
                        Node(2, 6),Node(2, 7),Node(2, [7,1])]);
                    
% Create the array of two agents. Each has a switching graph and two modes.
system = [Agent(G1, [Mode(A{1}{1}, B{1}{1}, Ac{1}{1}, X(1), U(1)), Mode(A{1}{2}, B{1}{2}, Ac{1}{2}, X(1), U(1))]),...
          Agent(G2, [Mode(A{2}{1}, B{2}{1}, Ac{2}{1}, X(2), U(2)), Mode(A{2}{2}, B{2}{2}, Ac{2}{2}, X(2), U(2))])];

W = {repmat(Polyhedron.emptySet(system(1).nx), system(1).nummodes, 1),...
     repmat(Polyhedron.emptySet(system(2).nx), system(2).nummodes, 1)};
S = cell(1,2);
k = 0;
while true
    fprintf("Outer Iteration %d\n", k);
    k = k+1;
    old_S = S;
    for a=1:2
        S_new = ParNodalSafeSets(system(a), W{a}, false);
        W{a} = convexHullOfUnion(system(a), S_new);
    end
    for a=1:2
        S_new = ParNodalSafeSets(system(a), W{a}, false);
        W{a} = convexHullOfUnion(system(a), S_new);
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