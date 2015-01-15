/*
 * SimulationGUI.h
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 *
 * GUI of the Nbody problem. No change is supposed to happen here.
 */

#ifndef SIMULATIONGUI_H_
#define SIMULATIONGUI_H_

#include "BodyVisualization.h"

#include <qwidget.h>

namespace Ui {
	class MainView;
}

class QTimer;
class QGraphicsScene;
class QTime;
class QPen;

namespace practical {
namespace nbody {

class Simulation;

namespace ui {

class SimulationGUI : public QWidget, public BodyVisualization {
	Q_OBJECT
private:
	Simulation& nbody_sim;
	Ui::MainView* view;
	QGraphicsScene* scene;
	QTimer* compute_timer;
	QTime* time;
	QPen* pen_body;
	int frame_counter;
	int time_start;
	float mass_min;
	float mass_factor;

public:
	SimulationGUI(Simulation& nbody_sim);
	void addBody(float px, float py, float mass, float vx=0.0f, float vy=0.0f);
	void setMass(float min, float max);
	virtual ~SimulationGUI();

private:
	qreal sizeOfBody(float mass, float from, float to);

private slots:
	void startComputation();
	void stopComputation();
	void compute();
};

} /* namespace ui */
} /* namespace nbody */
} /* namespace practical */

#endif /* SIMULATIONGUI_H_ */
