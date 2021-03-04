function [ ] = display_env( filename )
%DISPLAY_ENV Summary of this function goes here
%   Detailed explanation goes here
close all;
fid = fopen(filename);

tline = fgets(fid);
sz = strsplit(tline);
m = str2num(sz{1});
n = str2num(sz{2});
num_lines = 0;
tline = fgets(fid);
map = [];
while ischar(tline)
    num_lines = num_lines+1;
    mapline = [];
    for i = 1:m
        mapline = [mapline, str2num(tline(i))];
    end
    map = [map; mapline];
    
    
    
    tline = fgets(fid);
end

fclose(fid);

imagesc(map), hold on;
%colormap('bone');
%p=colorbar;

cmap = bone(4);
cmap = flipud(cmap(1:4,:));
cmap(1,:) = [1,1,1];
colormap(jet(4));
%colorbar
labels = {'Nest','Open','Wall','Food'};
lcolorbar(labels,'fontweight','bold');

end

