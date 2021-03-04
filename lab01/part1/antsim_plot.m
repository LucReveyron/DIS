function antsim_plot(i, numants, pher)

subplot(1, 2, 1);
image(numants{i}/50*256);
colormap('bone');
axis square;
title([num2str(i) ': Distribution of the ants']);

subplot(1, 2, 2);
imagesc(pher{i});
colormap('bone');
axis square;
title([num2str(i) ': Pheromone distribution']);

