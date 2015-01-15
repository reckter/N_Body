/*
 * SimGUIAdapter.h
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 *
 *  Used to interface with the Qt GUI implementation.
 *  You are responsible to complete the methods nside Simulation.cpp in order to get a correct visualization.
 */

#ifndef SIMGUIADAPTER_H_
#define SIMGUIADAPTER_H_


namespace practical {
namespace nbody {

namespace ui {
class BodyVisualization;
}

class SimGUIAdapter {
public:
	/**
	 * Is called at the very beginning to set the min and max mass of the bodies.
	 * Used to change the color of the body. (Call vis to set -> callback)
	 */
	virtual void setMinMaxMass(ui::BodyVisualization& vis) const = 0;

	/**
	 * Calculates one timestep of the simulation. If GUI is used, it will call this method repeatedly to advance the simulation.
	 */
	virtual void nextTimestep() = 0;

	/**
	 * Called after nextTimeStep(). User is supposed to pass current information for each body.
	 * Each body will be visualized with an ellipse, same size, different color based on mass.
	 * (Call vis -> callback)
	 */
	virtual void visualize(ui::BodyVisualization& vis) const = 0;

	virtual ~SimGUIAdapter() {}
};

} /* namespace nbody */
} /* namespace practical */

#endif /* SIMGUIADAPTER_H_ */
