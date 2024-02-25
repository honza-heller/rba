function imgnames = list_images(dir_path)

ImageFiles = dir(fullfile(dir_path, '*.*'));
imgnames = cell(0,0);
    
for Index = 1:length(ImageFiles)
    baseFileName = ImageFiles(Index).name;
    [~, ~, extension] = fileparts(baseFileName);
    switch lower(extension)
        case {'.png', '.bmp', '.jpg', '.tif', '.avi'}
            % Allow only PNG, TIF, JPG, or BMP images
            imgnames{end + 1} = fullfile(dir_path, baseFileName); %#ok<AGROW>
        otherwise
    end
    
end
end