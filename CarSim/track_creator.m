figure; axis equal; axis([-400 400 -300 300]); grid on; hold on
title('Cliquez la ligne centrale, ligne de départ en premier. Entrée = fin')
[x, y] = ginput;
centerline = [x(:), y(:)];
plot([x; x(1)], [y; y(1)], 'o-')

width = 12;
name  = "monCircuit";
save(fullfile('data','tracks','monCircuit.mat'), 'centerline', 'width', 'name');