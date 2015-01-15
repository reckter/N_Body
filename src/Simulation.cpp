/*
 * Simulation.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 */

#include "Simulation.h"
#include "Config.h"
#include "Body.h"

#include <gui/BodyVisualization.h>

#include <limits>
#include <cmath>
#include <iostream>

namespace practical {
    namespace nbody {


        Simulation::Simulation(const Config &config) :
                config(config), num_steps(0u), num_bodies(0u), dt(0.0f), scale(
                1.0f), distance(0.05f) {

        }

        void Simulation::run() {
            for (unsigned int step = 0; step < num_steps; ++step) {
                nextTimestep();
            }
        }

        void Simulation::nextTimestep() {
            handleCollisions();

#pragma omp parallel for
            for (std::vector<Body>::iterator iter = bodies.begin(); iter != bodies.end(); iter++) {

                float accelerationX = 0;
                float accelerationY = 0;
                if (iter->getMass() != 0) {
                    for (std::vector<Body>::iterator iter2 = bodies.begin(); iter2 != bodies.end(); iter2++) {
                        if (iter != iter2 && iter2->getMass()) {
                            float G = 6.673e-11f;
                            float diffx = iter2->getX() - iter->getX();
                            float diffy = iter2->getY() - iter->getY();
                            float delta = std::sqrt(diffx * diffx + diffy * diffy);

                            float fac = std::pow(delta, -3);
                            float tmpAccelerationX = G * iter2->getMass() * fac * diffx;
                            float tmpAccelerationY = G * iter2->getMass() * fac * diffy;
                            accelerationX += tmpAccelerationX;
                            accelerationY += tmpAccelerationY;


                        }
                    }
                }

                iter->setVelocityX(iter->getVelocityX() + accelerationX * dt);
                iter->setVelocityY(iter->getVelocityY() + accelerationY * dt);

            }

#pragma omp parallel for
            for (std::vector<Body>::iterator iter = bodies.begin(); iter != bodies.end(); iter++) {
                iter->setX(iter->getX() + iter->getVelocityX() * dt);
                iter->setY(iter->getY() + iter->getVelocityY() * dt);
            }

        }

        void Simulation::handleCollisions() {
            for (std::vector<Body>::iterator iter = bodies.begin(); iter != bodies.end(); iter++) {

                if (iter->getMass() != 0) {
                    for (std::vector<Body>::iterator iter2 = bodies.begin(); iter2 != bodies.end(); iter2++) {
                        if (iter != iter2 && iter->collidesWith(*iter2)) {

                            std::vector<Body>::iterator bodyA;
                            std::vector<Body>::iterator bodyB;
                            if (iter->getMass() >= iter2->getMass()) {
                                bodyA = iter;
                                bodyB = iter2;
                            } else {
                                bodyA = iter;
                                bodyB = iter2;
                            }
                            float newMass = bodyA->getMass() + bodyB->getMass();
                            float newVelocityX = (bodyA->getMass() * bodyA->getVelocityX() + bodyB->getMass() * bodyB->getVelocityX()) / newMass;
                            float newVelocityY = (bodyA->getMass() * bodyA->getVelocityY() + bodyB->getMass() * bodyB->getVelocityY()) / newMass;
                            bodyA->setMass(newMass);
                            bodyB->setMass(0);

                            bodyA->setVelocityX(newVelocityX);
                            bodyA->setVelocityY(newVelocityY);

                            bodyB->setVelocityX(0);
                            bodyB->setVelocityY(0);
                        }
                    }
                }
            }

        }

        void Simulation::setMinMaxMass(ui::BodyVisualization &vis) const {
            float min = std::numeric_limits<float>::max();
            float max = std::numeric_limits<float>::min();
            // finds min and max mass of all bodies
            for (unsigned int i = 0; i < num_bodies; ++i) {
                const float mass = bodies[i].getMass();
                if (mass > max) {
                    max = mass;
                }
                if (mass < min) {
                    min = mass;
                }
            }
            vis.setMass(min, max);
        }

        void Simulation::visualize(ui::BodyVisualization &vis) const {
            const float p_scaler = 1.0f / scale;
            for (unsigned int i = 0; i < num_bodies; ++i) {
                if (bodies[i].getMass() > 0) {
                    /* TODO visualize all bodies (with mass > 0.0) here
                 * use p_scaler to scale positional coordinates to ~[-1,1]
                 * */


                    vis.addBody(bodies[i].getX() * p_scaler, bodies[i].getY() * p_scaler, bodies[i].getMass(), bodies[i].getVelocityX() * p_scaler, bodies[i].getVelocityY() * p_scaler);
                }
            }

        }

        bool Simulation::initialize() {
            return config.read(this, num_steps, num_bodies, scale, dt, distance);
        }

        bool Simulation::writeResults() const {
            return config.write(this, num_steps, num_bodies, scale, dt, distance);
        }

        void Simulation::addBody(Body body) {
            this->bodies.push_back(body);
        }

        Simulation::~Simulation() {
        }

    } /* namespace nbody */
} /* namespace practical */
