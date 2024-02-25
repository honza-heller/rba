function elmark(axes, type, fname, psize)

if (ismac)
    setenv('PATH', [getenv('PATH') ':/usr/local/bin']);
    setenv('FONTCONFIG_PATH', '/opt/X11/lib/X11/fontconfig');
elseif (isunix)
    setenv('LD_LIBRARY_PATH', '/usr/lib/');
end
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[~, ~, ext] = fileparts(fname);

do_pdf = 0;
if (strcmpi('.pdf', ext))
    svg_file = [tempname '.svg'];
    do_pdf = 1;
elseif (strcmpi('.svg', ext))
    svg_file = fname;
else
    error(['Unrecognized file type: ' fname]);
end

do_paper = 0;
if (nargin > 3)
    pdf_file = [tempname '.pdf'];
    do_paper = 1;
else
    pdf_file = fname;
end

if (type == 1)
    axes(1:2) = axes(1:2) - 1;
    axes(3:4) = axes(3:4) + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fid = fopen(svg_file, 'w');

fprintf(fid, '<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n');
fprintf(fid, '<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">\n');

x1 = min([axes(1), axes(3)]);
y1 = min([axes(2), axes(4)]);
x2 = max([axes(1), axes(3)]);
y2 = max([axes(2), axes(4)]);

scale = 100;
bbox = [[x1 y1] - 1, [x2 y2] + 1] * 4;
sbbox = bbox * scale;
l = [sbbox(3) - sbbox(1), sbbox(4) - sbbox(2)];

fprintf(fid, ['<svg viewBox="' d2str(bbox(1)) ' ' d2str(bbox(2)) ' ' d2str(bbox(3)) ' ' d2str(bbox(4)) '" version = "1.1">\n']);
fprintf(fid, svg_rect(bbox(1), bbox(2), bbox(3), bbox(4), 'white', 'white'));

if (type == 1)
    bw = x2 - x1 - 1;
    bh = y2 - y1 - 1;
    str_name = 'ElMark Chessboard Target';
elseif (type == 2)
    bw = x2 - x1 + 1;
    bh = y2 - y1 + 1;
    str_name = 'Elmark Circles Target';
elseif (type == 3)
    bw = x2 - x1 + 1;
    bh = y2 - y1 + 1;
    str_name = 'MagikEye CornerMark Circles Target';
end


if (do_paper)
    gs_size = '';
    if (strcmpi(psize, 'a4'))
         mm = [210 297];
         gs_size = ' -sPAPERSIZE=a4 ';
    elseif (strcmpi(psize, 'a3'))
         mm = [297 420];
         gs_size = ' -sPAPERSIZE=a3 ';
    elseif (strcmpi(psize, 'a0'))
         mm = [841 1189];
         gs_size = ' -sPAPERSIZE=a0 ';         
    elseif (isequal(size(psize), [1, 2]))
         mm = psize;
         xpt = mm(1) * 0.0393700787402 * 72;
         ypt = mm(2) * 0.0393700787402 * 72;
         gs_size = sprintf(' -dDEVICEWIDTHPOINTS=%f -dDEVICEHEIGHTPOINTS=%f ', xpt, ypt);
    end
    
    s = min([max(l ./ mm) max(l ./ fliplr(mm))]);
    stride = 4 * scale * 1/s;
        
    istr = [str_name ': ' sprintf('%dx%d points', bw, bh) ' ' sprintf('([%d, %d], [%d,%d])', x1, y1, x2, y2) ', stride: ' sprintf('%.2fmm', stride)]; 
else
    istr = [str_name ': ' sprintf('%dx%d points', bw, bh)];
end

fprintf(fid, ['<text x="' sprintf('%f', sbbox(1) + (sbbox(3) - sbbox(1)) / 2) ...
                  '" y="' sprintf('%f', -sbbox(4) + 1.5 * scale) ...
                  '" style="text-anchor: middle;" fill="black" font-family = "Times" ' ...
                  'font-size="' sprintf('%f', scale/1.5) '">' istr '</text>\n']);

if (type == 1)
  % ElMark Chessboard
  for i = x1:(x2-1)
      for j = y1:(y2-1)
          if ((mod(i,2) && mod(j,2)) || (~mod(i,2) && ~mod(j,2)))
              fprintf(fid, svg_rect(i*4, j*4, (i+1)*4, (j+1)*4, 'black', 'black'));
          end
      end
  end

  fprintf(fid, svg_circ(-2, -2, 1, 'white'));
  fprintf(fid, svg_circ( 2, -2, 1, 'black'));
  fprintf(fid, svg_circ(-2,  2, 1, 'black'));
  fprintf(fid, svg_circ(-2,  6, 1, 'white'));

elseif (type == 2)
  % ElMark Circles
  R = ones(bw, bh);
  R = set_dot(R, 0, 0, 1.5, x1, y1);
  R = set_dot(R, 1, 0, 1.5, x1, y1);
  R = set_dot(R, 0, 2, 1.5, x1, y1);  
  
  for i = x1:x2
      for j = y1:y2
          r = get_dot(R, i, j, x1, y1);
          if (r > 0)
            fprintf(fid, svg_circ(i*4, j*4, r, 'black'));
          end
      end
  end
elseif (type == 3)
  % CornerMark Circles
  R = zeros(bw, bh);
  B = zeros(bw, bh);

  c1 = [1  ,   1,   0,   0;
        1.5,  -1,-1.5,   0;
        1  ,-1.5,  -1,   1;
        1.5, 1.5,   1,   1];    
    
  c2 = [1.5, 1.5,   1,   1;
        1  ,  -1,-1.5,   1;
        1.5,-1.5,  -1,   0;
        1  ,   1,   0,   0];
    
  c3 = [  1,   1, 1.5, 1.5;
          1,-1.5,  -1,   1;
          0,-1.5,  -1, 1.5;
          0,   0,   1,   1];

  c4 = [  0,  -1,   1,   1;
          0,  -1,-1.5, 1.5;
          1,  -1,-1.5,   1;
          1,   1, 1.5, 1.5];

  bb = [  0,   0,   0,   0;
          0,   1,   1,   0;
          0,   1,   1,   0;
          0,   0,   0,   0];    
    
            
  R(1:end,   1) = 1;
  R(1:end, end) = 1;
  R(  1, 1:end) = 1;
  R(end, 1:end) = 1;
  
  R(1:end,   2) = 1;
  R(1:end, end-1) = 1;
  R(  2, 1:end) = 1;
  R(end-1, 1:end) = 1;  
      
  R = set_rec(R, x1, y1, fliplr(c1'), x1, y1);    
  R = set_rec(R, x1, y2 - 3, fliplr(c2'), x1, y1); 
  R = set_rec(R, x2 - 3, y2 - 3, fliplr(c3'), x1, y1);  
  R = set_rec(R, x2 - 3, y1, fliplr(c4'), x1, y1);  

  B = set_rec(B, x1, y1, fliplr(bb'), x1, y1);    
  B = set_rec(B, x1, y2 - 3, fliplr(bb'), x1, y1); 
  B = set_rec(B, x2 - 3, y2 - 3, fliplr(bb'), x1, y1);  
  B = set_rec(B, x2 - 3, y1, fliplr(bb'), x1, y1);  

  for i = x1:x2
      for j = y1:y2          
          b = get_dot(B, i, j, x1, y1);
          if (b > 0)
              fprintf(fid, svg_rect((i-0.5)*4, (j-0.5)*4, (i+0.5)*4, (j+0.5)*4, 'black', 'black', 1));
          end      
          
          r = get_dot(R, i, j, x1, y1);
          if (r > 0)
            fprintf(fid, svg_circ(i*4, j*4, r, 'black'));
          elseif (r < 0)
            fprintf(fid, svg_circ(i*4, j*4, -r, 'white'));
          end                
      end
  end
end

fprintf(fid, '</svg>\n');
fclose(fid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if (do_pdf)
    [~, log] = system(['inkscape -D --export-pdf="' pdf_file '" ' svg_file]);
    disp(log);
    delete(svg_file);
    
    if (do_paper)
        [~, log] = system(['gs -sOutputFile="' fname '" -sDEVICE=pdfwrite ' gs_size ...
         ' -dCompatibilityLevel=1.4  -dNOPAUSE -dBATCH  -dPDFFitPage -dFIXEDMEDIA ' pdf_file]);
     disp(log);
     delete(pdf_file);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function R = set_rec(R, i, j, r, x, y)
        [w, h] = size(r);
        i = i - x + 1;
        j = j - y + 1;
        
        R(i:(i+w-1), j:(j+h-1)) = r;
    end

    function R = set_dot(R, i, j, r, x, y)
        R(i - x + 1, j - y + 1) = r;
    end

    function r = get_dot(R, i, j, x, y)
        r = R(i - x + 1, j - y + 1);
    end

    function s = d2str(d)
        s = sprintf('%g', d * scale);
    end

    function [v1, v2] = swap(u1, u2)
        v1 = u2;
        v2 = u1;
    end
    
    function circ = svg_circ(x, y, r, col)
          circ = ['<circle cx="' d2str(x) '" cy="' d2str(-y) '" r="' d2str(r) '" fill="' col ...
                '" stroke="' col '"  stroke-width="0" />\n'];  
    end

    function rect = svg_rect(x1, y1, x2, y2, col1, col2, sw)
        if (x1 > x2)
            [x1, x2] = swap(x1, x2);
        end
        y1 = -y1;
        y2 = -y2;
        
        if (y1 > y2)
            [y1, y2] = swap(y1, y2);
        end
        
        w = x2 - x1;
        h = y2 - y1;
        
        if (nargin < 7)
            sw = 0;
        end
        
        rect = ['<rect x="' d2str(x1) '" y="' d2str(y1) '" width="' d2str(w) '" height="' d2str(h) ...
                '" style="fill:' col1 ';stroke:' col2 '; stroke-width:' sprintf('%d', sw) '" />\n'];
    end
end
