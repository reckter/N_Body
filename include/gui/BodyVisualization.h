/*
 * BodyVisualization.h
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 *
 * Interface for the GUI visualisation. Implemented by SimulationGUI.h.
 * Nothing has to be done here.
 */

#ifndef BODYVISUALIZATION_H_
#define BODYVISUALIZATION_H_

namespace practical {
namespace nbody {
namespace ui {

class BodyVisualization {
public:
	BodyVisualization() {}
	/**
	 * Set min/mass mass of the bodies in the system. Used for coloring based on mass.
	 * --> Black is high(est) mass
	 */
	virtual void setMass(float min, float max) = 0;
	/**
	 * Add a body, which is going to be visualized. px, py needs to be scaled to [-1, 1]
	 */
	virtual void addBody(float px, float py, float mass, float vx=0.0f, float vy=0.0f) = 0;

	virtual ~BodyVisualization() {}
};

} /* namespace ui */
} /* namespace nbody */
} /* namespace practical */

#endif /* BODYVISUALIZATION_H_ */
