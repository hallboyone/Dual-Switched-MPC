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

A11 = {eye(2) + (0.2*rand(2)-0.1), eye(2) + (0.2*rand(2)-0.1)};
A22 = {eye(2) + (0.2*rand(2)-0.1), eye(2) + (0.2*rand(2)-0.1)};
A12 = {0.1*(0.2*rand(2)-0.1), 0.1*(0.2*rand(2)-0.1)};
A21 = {0.1*(0.2*rand(2)-0.1), 0.1*(0.2*rand(2)-0.1)};

B1 = {[0.1; 0.1] + (0.2*rand(2,1)-0.1), [0.1; 0] + (0.2*rand(2,1)-0.1)};
B2 = {[0.1; 0.1] + (0.2*rand(2,1)-0.1), [0.1; 0] + (0.2*rand(2,1)-0.1)};

X1 = Polyhedron([eye(2); -eye(2)], [1; 2; 1; 2]);
X2 = Polyhedron([eye(2); -eye(2)], [2; 1; 2; 1]);
U1 = Polyhedron([1; -1], 10*[0.5; 0.25]);
U2 = Polyhedron([1;-1], 10*[0.25; 0.5]);

mindt_1 = [3, 4];
mindt_2 = [4, 3];

safe_sets_1 = cell(numel(mindt_1), 1);
for i=1:numel(mindt_1)
    safe_sets_1{i} = cell(mindt_1(i), 1);
    for k=1:mindt_1(i)
        safe_sets_1{i}{k} = X1;
    end
end
safe_sets_2 = cell(numel(mindt_2), 1);
for i=1:numel(mindt_2)
    safe_sets_2{i} = cell(mindt_2(i), 1);
    for k=1:mindt_2(i)
        safe_sets_2{i}{k} = X2;
    end
end
safe_sets = {safe_sets_1, safe_sets_2};

figure
first_run = true;
while true
    for node=1:2
        if node==1
            Ac = A12;
            A = A11;
            B = B1;
            U = U1;
            X = X1;
            mindt = mindt_1;
        else
            Ac = A21;
            A = A22;
            B = B2;
            U = U2;
            X = X2;
            mindt = mindt_2;
        end
        if first_run
            W = {Polyhedron.emptySet(2), Polyhedron.emptySet(2)};
        else
            S = convexHullOfUnion(safe_sets{3-node});
            W = {Ac{1}*S, Ac{2}*S};
            W{1}.minHRep();
            W{2}.minHRep();
        end
        
        for mode=1:2
            safe_sets{node}{i} = cell(mindt(mode), 1);
            for step=1:mindt(mode)
                safe_sets{node}{mode}{step} = X;
            end
        end
        
        while true
            old_safe_sets = safe_sets{node};
            for mode = 1:2
                mode_mindt = numel(safe_sets{node}{mode});
                for step = 1:mode_mindt
                    safe_sets{node}{mode}{step} = preSetPreviewedDisturbance(...
                        old_safe_sets{mode}{min(step+1, mode_mindt)},...
                        A{mode}, B{mode}, safe_sets{node}{mode}{step},...
                        U, W{mode});
                    if step == mode_mindt
                        safe_sets{node}{mode}{step} = preSetPreviewedDisturbance(...
                            old_safe_sets{3-mode}{1},...
                            A{3-mode}, B{3-mode}, safe_sets{node}{mode}{step},...
                            U, W{mode});
                    end
                end
            end
            if(areEqual(safe_sets{node},old_safe_sets, 1))
                break
            end
        end
    end
    first_run = false;
end
close(gcf)

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
                plot(safe_sets{m}{i})
                xlim([-2.5, 2.5])
                ylim([-2.5, 2.5])
                xlabel("$c_c$",'Interpreter','latex','fontsize',15);
                ylabel("$c_k$",'Interpreter','latex','fontsize',15);
                title(['$\mathcal{S}_{(', num2str(m),',',num2str(i),')}$'],'Interpreter','latex','fontsize',15)
                hold on
            end
            if (old_sets{m}{i} ~= safe_sets{m}{i})
                is_equal = false;
            end
        end
    end
if plot_tf
    drawnow
end
end