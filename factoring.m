function [f_poly, h_poly] = factoring(sys, deg_f, deg_h)
    % FACTORING - Factorizes a polynomial into two polynomials of specified degrees.
    %
    % Syntax:
    %   [f_poly, h_poly] = factoring(sys, deg_f, deg_h)
    %
    % Inputs:
    %   sys    - Coefficients of the input polynomial (highest degree first).
    %   deg_f  - Desired degree of the first factor polynomial (f_poly).
    %   deg_h  - Desired degree of the second factor polynomial (h_poly).
    %
    % Outputs:
    %   f_poly - Coefficients of the first factor polynomial.
    %   h_poly - Coefficients of the second factor polynomial.
    %
    % Example:
    %   sys = [1, 1, 6, 4]; % s^3 + s^2 + 6s + 4
    %   [f_poly, h_poly] = factoring(sys, 2, 1);
    %   % f_poly = [1, 2, 2], h_poly = [1, 2]

    % Validate input degrees
    if length(sys) - 1 ~= deg_f + deg_h
        error('The sum of the specified degrees must equal the degree of the polynomial.');
    end

    % Find all roots of the polynomial
    roots_all = roots(sys);

    % Generate all combinations of roots for the first polynomial
    n = length(roots_all);
    combinations = nchoosek(1:n, deg_f);

    % Initialize variables to track the best factorization
    best_error = inf;
    f_poly = [];
    h_poly = [];

    % Iterate over all combinations of roots
    for i = 1:size(combinations, 1)
        % Select roots for f_poly and h_poly
        f_roots = roots_all(combinations(i, :));
        h_roots = setdiff(roots_all, f_roots, 'stable');

        % Construct polynomials from roots
        f_test = poly(f_roots);
        h_test = poly(h_roots);

        % Ensure the degrees match the specified ones
        if length(f_test) - 1 == deg_f && length(h_test) - 1 == deg_h
            % Calculate reconstruction error
            reconstructed = conv(f_test, h_test);
            reconstruction_error = norm(reconstructed - sys) / norm(sys);

            % Update the best factorization if the error is smaller
            if reconstruction_error < best_error
                best_error = reconstruction_error;
                f_poly = f_test;
                h_poly = h_test;
            end
        end
    end

    % Throw an error if no valid factorization is found
    if isempty(f_poly) || isempty(h_poly)
        error('No valid factorization found. Check the input polynomial and degrees.');
    end
end