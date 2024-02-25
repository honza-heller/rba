function subfig(rows, cols, ord, f)
    twidth = 20;
    screen = get(0, 'ScreenSize');
    w = screen(3);
    h = screen(4);
    fw = w / cols;
    fh = h / rows - twidth;
    i = ceil(ord / cols);
    j = rem(ord - 1, cols);
    left = j * fw + screen(1);
    bottom = h - i * fh + screen(2);
    
    if (nargin < 4)
        f = figure('Visible','off');
    end
    
    set(f, 'OuterPosition', [left bottom fw fh]);
    set(f, 'Visible', 'on');
end