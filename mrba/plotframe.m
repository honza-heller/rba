function plotframe(T, scale, txt, ds)

    X = [0,1,0,0;
         0,0,1,0;
         0,0,0,1;
         1,1,1,1];
     
    X(1:3,1:4) = scale * X(1:3,1:4);
    
    Xw = inv(T) * X;
    
    if (nargin < 4)
        ds = '-';
    end
    
    vline(Xw(1:3,1), Xw(1:3,2), [ds 'r']);
    vline(Xw(1:3,1), Xw(1:3,3), [ds 'g']);
    vline(Xw(1:3,1), Xw(1:3,4), [ds 'b']);
    
    if (nargin >= 3)
         text(Xw(1,1),Xw(2,1), Xw(3,1), txt);
    end
end

function vline(p1, p2, style)
   if (size(p1, 1) == 1)
       p1 = p1';
   end
   if (size(p2, 1) == 1)
       p2 = p2';
   end
   
   if (size(p1, 1) == 3 && size(p2, 1) == 3)
       p = [p1, p2];
       plot3(p(1,:), p(2,:), p(3,:), style);
   end
end