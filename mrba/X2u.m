function u = X2u(X, A, K, dist, cflag)
if (ischar(cflag))
    % Fisheye models %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if (size(X, 1) == 3)
        [R, t] = hom2rt(A);
        X = bsxfun(@plus, R * X, t);
    else
        X = A * X;
    end
    
    X = normalize(X(1:3, :));
    
    rs = sqrt(sum(X(1:2, :).^2));
    as = acos(X(3, :));
    
    if (strcmp(cflag, 'fisheye_r2p'))
        if (dist(2) == 0)
            mul1 = 1;
        else
            mul1 = 2 * dist(2) * as;
        end
        
        mul2 = (dist(1) - sqrt(dist(1) * dist(1) - 2 * mul1 .* as)) ./ (mul1 .* rs);        
    elseif (strcmp(cflag, 'fisheye_s2p'))
        mul2 = dist(1)/dist(2) * sin(dist(2) * as) ./ rs;
    elseif (strcmp(cflag, 'fisheye_d3p'))
        if (numel(dist) > 3)
            dist(3) = dist(5);
        end
        theta = dist(2) * as;
        mul2 = dist(1) * incsin(theta) ./ rs;
    elseif (strcmp(cflag, 'fisheye_p8p'))
        theta = dist(2) * as;
        mul2 = dist(1) * incsin(theta) ./ rs;
    elseif (strcmp(cflag, 'fisheye_e7p'))
        mul2 = (dist(1) * as) ./ rs;    
    end
    
    Y = bsxfun(@times, X(1:2,:), mul2);
    
    if (strcmp(cflag, 'fisheye_d3p'))
        r = Y(2,:).^2 + Y(1,:).^2;
        n = 2 * dist(3) * r;
        m = (1 - sqrt(1 - 2 * n)) ./ n;
        m(isnan(m)) = 1;
        Y = bsxfun(@times, Y, m);
    elseif (strcmp(cflag, 'fisheye_p8p') || strcmp(cflag, 'fisheye_e7p'))
        r = (Y(2,:).^2 + Y(1,:).^2);
        
        d1 = 1 + dist(5) .* r + dist(6) .* r .* r;
        d2 = 1 + dist(7) .* r + dist(8) .* r .* r;
        
        Y(1,:) = d1./d2 .* Y(1,:) + 2 * dist(3) * Y(1,:) .* Y(2,:) + dist(4) .* (r + 2 * Y(1,:) .* Y(1,:));
        Y(2,:) = d1./d2 .* Y(2,:) + 2 * dist(4) * Y(1,:) .* Y(2,:) + dist(3) .* (r + 2 * Y(2,:) .* Y(2,:));
    end
    
    u = h2e(K * e2h(Y), 1);
    
    idx = find(rs < 0.00001);
    u(1, idx) = K(1, 3);
    u(2, idx) = K(2, 3);
    
else 
    % Rational model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    X = A * [X; ones(1, size(X, 2))];
    X(1,:) = X(1,:) ./ X(3,:);
    X(2,:) = X(2,:) ./ X(3,:);
    
    r2 = X(1,:) .* X(1,:) + X(2,:) .* X(2,:);
    
    if (nargin == 5 && cflag ~= 0)
        r = sqrt(r2);
    else
        r = r2;
    end
    
    d1 = 1 + dist(1)*r + dist(2)*r.^2 + dist(5)*r.^3;
    d2 = 1 + dist(6)*r + dist(7)*r.^2 + dist(8)*r.^3;
    
    p(1,:) = (d1./d2) .* X(1,:) + 2 * dist(3) * X(1,:) .* X(2,:) + dist(4) .* (r2 + 2 * X(1,:) .* X(1,:));
    p(2,:) = (d1./d2) .* X(2,:) + 2 * dist(4) * X(1,:) .* X(2,:) + dist(3) .* (r2 + 2 * X(2,:) .* X(2,:));
    
    u(1,:) = K(1,1) * p(1,:) + K(1,3);
    u(2,:) = K(2,2) * p(2,:) + K(2,3);
end
end

function v = normalize(u)
n = sqrt(sum(u.^2));
v = bsxfun(@mrdivide, u, n);
end

function f = incsin(theta)
i1 = (theta <= pi/2);
i2 = (theta > pi/2) & (theta <= pi);
i3 = (theta > pi);

f = zeros(1, numel(theta));
f(i1) = sin(theta(i1));
f(i2) = sin(theta(i2) - pi) + 2;
f(i3) = theta(i3) - pi + 2;
end

function H = e2h(E)
H = [E; ones(1, size(E, 2))];
end

function E = h2e(H, hflag)
if (nargin > 1 && hflag)
    E = H(1:(end-1), :);
else
    E = bsxfun(@rdivide, H(1:end-1, :), H(end, :));
end
end

function [R, t] = hom2rt(H)
    R = H(1:3, 1:3);
    t = H(1:3, 4);
end
