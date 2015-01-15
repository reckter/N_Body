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

                // pretty outputs
                #ifdef PRETTY_OUT
                if(step % 1000 == 0) {
                    float p = (float) step / (float) num_steps;
                    int num = (int) (50.0 * p);

                    std::cout << "\r";
                    std::cout << "|";
                    for(int i = 0; i < num; i++) {
                        std::cout << "#";
                    }
                    for(int i = num; i < 50; i++) {
                        std::cout << " ";
                    }
                    std::cout << "| " << p * 100 << "%";

                    std::cout.flush();
                }
                #endif
            }
            // pretty outputs
            #ifdef PRETTY_OUT
            float p = 1.0;
            int num = (int) (50.0 * p);

            std::cout << "\r";
            std::cout << "|";
            for(int i = 0; i < num; i++) {
                std::cout << "#";
            }
            for(int i = num; i < 50; i++) {
                std::cout << " ";
            }
            std::cout << "| " << p * 100 << "%" << std::endl;
            #endif
        }

        void Simulation::nextTimestep() {
            handleCollisions();

            // iterate through all bodies to calculate the acceleration for each
#pragma omp parallel for
            for (int i = 0; i < bodies.size(); i++) {

                Body *iter = &(bodies[i]);

                // the accumulators of the acceleration
                float accelerationX = 0;
                float accelerationY = 0;

                // only calculate the acceleration if the current body is still a valid body
                if (iter->getMass() != 0) {

                    // go through all other bodies to calculate the acceleration they cause on the current one
                    for (int j = 0; j < bodies.size(); j++) {

                        // only calculate the acceletration if the other body is different from the current one
                        // and if its mass isnÄt 0
                        Body *iter2 = &(bodies[j]);
                        if (i != j && iter2->getMass() != 0) {

                            // gravitational constant
                            float G = 6.673e-11f;

                            // distance
                            float diffx = iter2->getX() - iter->getX();
                            float diffy = iter2->getY() - iter->getY();
                            float delta = std::sqrt(diffx * diffx + diffy * diffy);

                            // the acceleration factor
                            float fac = std::pow(delta, -3);

                            float tmpAccelerationX = G * iter2->getMass() * fac * diffx;
                            float tmpAccelerationY = G * iter2->getMass() * fac * diffy;

                            // adding the calculated acceleration
                            accelerationX += tmpAccelerationX;
                            accelerationY += tmpAccelerationY;

                        }

                    }
                }

                // adding the acceleration to the movement
                iter->setVelocityX(iter->getVelocityX() + accelerationX * dt);
                iter->setVelocityY(iter->getVelocityY() + accelerationY * dt);

            }

            // moving each body according to his velocity
            #pragma omp parallel for
            for (int i = 0; i < bodies.size(); i++) {
                Body *iter = &(bodies[i]);
                iter->setX(iter->getX() + iter->getVelocityX() * dt);
                iter->setY(iter->getY() + iter->getVelocityY() * dt);
            }

        }

        void Simulation::handleCollisions() {

            // all the collisions that we found
            std::vector<std::pair<Body *, Body *> > collisions;

            // we go through all the bodies and check if the current one collides with the second one
            #pragma omp parallel for collapse(2)
            for (int i = 0; i < bodies.size(); i++) {
                for (int j = 0; j < bodies.size(); j++) {
                    Body *iter = &(bodies[i]);
                    Body *iter2 = &(bodies[j]);
                    if (iter != iter2 && iter->collidesWith(*iter2)) {

                        // we collide so we at both bodies to the list
                    #pragma omp critical
                        collisions.push_back(std::pair<Body *, Body *>(&(*iter), &(*iter2)));
                    }
                }
            }

            // we go over every collision and handle it
            for (int i = 0; i < collisions.size(); i++) {
                std::pair<Body *, Body *> pair = collisions[i];
                handleSingleCollision(pair.first, pair.second);
            }
        }

        void Simulation::handleSingleCollision(Body *body1, Body *body2) {

            // the suriving body is the bigger one, so if two bodies collide and one of them is zero
            // the bigger survives.
            Body *bodyA;
            Body *bodyB;
            if (body1->getMass() >= body2->getMass()) {
                bodyA = body1;
                bodyB = body2;
            } else {
                bodyA = body2;
                bodyB = body1;
            }

            // the new mass is the two masses
            float newMass = bodyA->getMass() + bodyB->getMass();

            // the velocity is = (v_i * m_i + v_j * m_j) / m
            float newVelocityX = (bodyA->getMass() * bodyA->getVelocityX() + bodyB->getMass() * bodyB->getVelocityX()) / newMass;
            float newVelocityY = (bodyA->getMass() * bodyA->getVelocityY() + bodyB->getMass() * bodyB->getVelocityY()) / newMass;
            bodyA->setMass(newMass);
            bodyB->setMass(0);

            bodyA->setVelocityX(newVelocityX);
            bodyA->setVelocityY(newVelocityY);

            bodyB->setVelocityX(0);
            bodyB->setVelocityY(0);
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
