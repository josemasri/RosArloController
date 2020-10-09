#ifndef SIMULATIONSTATE_H
# define SIMULATIONSTATE_H

class SimulationState {

public:
   SimulationState(double energy = 1000);

   double distanceToGo;
   double boxDistance;
   double distanceTravelled;
   double finishTime;
   double currentTime;
   double currentPosition;
   double position[2];
   double robotEnergy;
   double initialEnergy;
   double robotDamage;
   double averageDistance;
   int iteration;
   bool stuck;
   bool hasTimeRunOut;
	bool finishLineCrossed;

   void resetState();
};

#endif
