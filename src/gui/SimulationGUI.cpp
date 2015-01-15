/*
 * SimulationGUI.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 *
 */

#include "gui/SimulationGUI.h"

#include "ui_nbody.h"

#include "Logger.h"

#include <Simulation.h>

#include <qtimer.h>
#include <qgraphicsitem.h>
#include <qgraphicsscene.h>
#include <QTime>

#include <cmath>

namespace practical {
    namespace nbody {
        namespace ui {

            SimulationGUI::SimulationGUI(Simulation & nbody_sim) :
                    QWidget(0), BodyVisualization(), nbody_sim(nbody_sim), view(
                    new Ui::MainView), frame_counter(0), time_start(0), mass_factor(1.0), mass_min(0.0) {
                view->setupUi(this);
                scene = new QGraphicsScene(view->graphics_view);
                scene->setSceneRect(-0.1, -0.1, .4, .4);
                view->graphics_view->setScene(scene);

                view->button_stop->setEnabled(false);
                connect(view->button_stop, SIGNAL(clicked()), this,
                        SLOT(stopComputation()));

                connect(view->button_start, SIGNAL(clicked()), this,
                        SLOT(startComputation()));

                compute_timer = new QTimer(this);
                connect(compute_timer, SIGNAL(timeout()), this, SLOT(compute()));

                view->graphics_view->fitInView(scene->sceneRect(), Qt::KeepAspectRatioByExpanding);

                time = new QTime();
                pen_body = new QPen(Qt::white);
                pen_body->setCosmetic(true);
            }

            void SimulationGUI::setMass(float min, float max) {
                mass_min = min;
                mass_factor = max - min;
                if (fabs(mass_factor) < 0.0001) {
                    mass_factor = 1.0;
                }
            }

            qreal SimulationGUI::sizeOfBody(float mass, float from, float to) {
                return std::min((to - from) * (mass - mass_min) / mass_factor + from, to);
            }

            void SimulationGUI::addBody(float px, float py, float mass, float vx, float vy) {
                if (mass > 0.0) {
                    const qreal size = 0.08;// sizeOfBody(mass, 0.06, 0.11);
                    qreal h = 0.5 * size;
                    const int cOffset = 255 - sizeOfBody(mass, 0, 255);
                    scene->addEllipse(px - h, py - h, size, size, *pen_body, QBrush(QColor(cOffset, 40, cOffset)));


                    double length = std::sqrt(vx * vx + vy * vy);
                    qreal x1 = px;
                    qreal y1 = py;
                    qreal x2 = x1 + h * (vx / length);
                    qreal y2 = y1 + h * (vy / length);
                    scene->addLine(x1, y1, x2, y2, *pen_body);
                } else {
                    LOG_ERROR("Mass less or equal to zero.");
                }
            }

            void SimulationGUI::compute() {
                nbody_sim.nextTimestep();
                scene->clear();
                nbody_sim.visualize(*this);
                scene->update();
                view->graphics_view->update();
                view->lcd_fps->display(double(frame_counter) / (time->elapsed() - time_start) * 1000.0);
                view->lcd_timestep->display(++frame_counter);
            }

            void SimulationGUI::startComputation() {
                view->button_stop->setEnabled(true);
                view->button_start->setEnabled(false);
                view->lcd_timestep->display(frame_counter);
                time->start();
                time_start = time->restart();
                compute_timer->start(30);
            }

            void SimulationGUI::stopComputation() {
                view->button_stop->setEnabled(false);
                view->button_start->setEnabled(true);
                compute_timer->stop();
            }

            SimulationGUI::~SimulationGUI() {
                delete time;
            }

        } /* namespace ui */
    } /* namespace nbody */
} /* namespace practical */
