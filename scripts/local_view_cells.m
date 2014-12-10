function [] = plot_lv(localview)
% Parameter is a matrix from importdata
% t = Time, y = ID, z = Intensity
t = [];
y = [];
z = [];
cont = 1;
matrix = localview;
aux = size(matrix);
for i=1:aux(1)
    for j=2:2:aux(2)
        if (j == aux(2))
            break
        end
        if (isnan(matrix(i, j)))
            break
        end
        t(cont) = matrix(i,1);
        t(cont+1) = 0;	% Fixing the colormap scale
        y(cont) = matrix(i,j);
        y(cont+1) = 0;
        z(cont) = matrix(i,j+1);
        z(cont+1) = 0;
        cont = cont+2;
    end
    

end

colormap(flipud(gray));
h = scatter(t,y,1000,z, 'fill', 'SizeData', 45);
colorbar;
grid;
xlabel('Time (s)');
ylabel('Local View Cell ID');
