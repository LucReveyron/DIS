function testall(cities,localsearch)
if nargin<2, localsearch=0; end;
tsp('random',cities,localsearch);
tsp('guess',cities,localsearch);
tsp('ant',cities,localsearch);
