if runs~=1
	disp('If you perform multiple runs of a simulation, the intermediate steps are not stored.');
	disp('Therefore, you cannot play the simulation.') ;
else
	disp('Press a key to go to the next simulation step.');
	disp('To stop, press Ctrl-C.');
	for i=1:length(numants)
		antsim_plot(i, numants, pher0);
		pause();
	end
end

