% Author  - Richard A. Hall (hallboyone@icloud.com)
% Date    - Feb 25, 2022
% Purpose - To compute the safe-set collections of an externally switched
% linear system with seperate switching signals. System is modeled after 
% Li's 2021 paper.
                    
addpath ./functions/

%% ==================== Setup distributed system =====================
Ts = 0.1;
N = 5;
m = 1500 + 400*[1:5];
r = 0.25 + 0.06*[1:5];
eta = 0.8 + 0.08*[1:5];
tau = 0.4 + 0.02*[1:5];
Ca = 0.4 + 0.1*[1:5];
f = 0.015 + 0.006*[1:5];
g = 9.8;
U = Polyhedron([1;-1],[3;3]);
R = 0.1;
G = diag([5, 2.5, 1]);

d_ref = 20;

% State Dynamics
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

% Input Dynamics
B1 = {[0.5;1], [0.5;1], [0.5;1]};
B2 = {[1;0], [1;0], [1;0]};
B3 = {[0.5;1], [0.5;1], [0.5;1]};
B4 = {[0; 0.7787], [0; 0.7787], [0; 0.7787]};
% B{agent}{mode}
B = {B1, B2, B3, B4};

% State constraints
X1 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X2 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X3 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
X4 = {Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1]), Polyhedron([eye(2);-eye(2)], [1;1;1;1])};
% X{agent}{mode}
X = {X1, X2, X3, X4};

% Input constraints
U1 = {Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]')};
U2 = {Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]'), Polyhedron([1;-1], [0.5, 0.5]')};
U3 = {Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]')};
U4 = {Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]'), Polyhedron([1;-1], [1, 1]')};
% U{agent}{mode}
U = {U1, U2, U3, U4};

% Switching graphs
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
%G{agent}
G = {G1, G2, G3, G4};  

% Create the array of agent structures. Each has a switching graph and three modes.
% system{agent}
system = BuildSystem(A, B, X, U, G); 

system = ComputeSafeSets(system, true, false, false);