/*
 * Simulation.h
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "SimGUIAdapter.h"
#include "../src/Body.h"
#include "Config.h"

#include <vector>
#include <string>
#include <utility>

namespace practical {
    namespace nbody {

        class Config;

        class Simulation : public SimGUIAdapter {
        private:
            const Config &config;
            /**
            * Number of (time) steps for the simulation
            * Ignored when the GUI is active
            */
            unsigned int num_steps;
            /**
            * Number of bodies in the system
            */
            unsigned int num_bodies;
            /**
            * Time step
            */
            float dt;
            /**
            * Used to scale position variables to ~[-1, 1] in order to fit them into the GUI window
            */
            float scale;
            /**
            * Distance when bodies are supposed to collide
            */
            float distance;

            std::vector<Body> bodies;

        public:
            Simulation(const Config &conf);

            /**
            * Reads simulation data from a file.
            * Uses Config.
            * TODO needs to be completed for chosen data structure
            */
            bool initialize();

            /**
            * Run simulation "num_step" times
            */
            void run();

            /**
            * @see SimGUIAdapter.h
            * TODO needs to be completed
            */
            void nextTimestep();

            /**
            * @see SimGUIAdapter.h
            * TODO Needs to be completed
            */
            void setMinMaxMass(ui::BodyVisualization &vis) const;

            /**
            * @see SimGUIAdapter.h
            * TODO Needs to be completed
            */
            void visualize(ui::BodyVisualization &vis) const;

            /**
            * Simply writes results of the simulation to a file.
            * Uses Config.
            * TODO needs to be completed for chosen data structure
            */
            bool writeResults() const;

            void addBody(Body body);


            std::vector<Body> const &getBodies() const {
                return bodies;
            }

            Config const &getConfig() const {
                return config;
            }

            unsigned int getNum_steps() const {
                return num_steps;
            }

            void setNum_steps(unsigned int num_steps) {
                Simulation::num_steps = num_steps;
            }

            unsigned int getNum_bodies() const {
                return num_bodies;
            }

            void setNum_bodies(unsigned int num_bodies) {
                Simulation::num_bodies = num_bodies;
            }

            float getDt() const {
                return dt;
            }

            void setDt(float dt) {
                Simulation::dt = dt;
            }

            float getScale() const {
                return scale;
            }

            void setScale(float scale) {
                Simulation::scale = scale;
            }

            float getDistance() const {
                return distance;
            }

            void setDistance(float distance) {
                Simulation::distance = distance;
            }

            virtual ~Simulation();

        private:
            /**
            * Handles collision between bodies before a time step
            * TODO Needs to be completed
            */
            void handleCollisions();
        };

    } /* namespace nbody */
} /* namespace practical */

#endif /* SIMULATION_H_ */
