function M = Mode(A, B, E, X, U)
M.nx = X.Dim;
M.nu = U.Dim;
M.A = A; % Local dynamics
M.B = B; 
M.X = X; % State constraints
M.U = U; % Input constraints
M.E = E; % Cross dynamics (cell array)
end