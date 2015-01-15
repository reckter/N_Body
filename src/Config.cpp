/*
 * Config.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 */

#include "Config.h"

#include "Logger.h"
#include "Body.h"
#include "Simulation.h"

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <time.h>
#include <math.h>

namespace practical {
    namespace nbody {

        Config::Config()
                : infile(""), outfile("") {

        }

        Config::Config(const std::string &infile, const std::string &outfile)
                : infile(infile), outfile(outfile) {

        }

        bool Config::read(Simulation *sim, unsigned &num_steps, unsigned &num_bodies, float &scale, float &dt, float &distance) const {
            //using namespace nmath;
            if (infile == "") {
                LOG_INFO("Empty string for file path.");
                return false;
            }
            std::ifstream sim_file;
            sim_file.open(infile.c_str());
            if (!sim_file.good()) {
                LOG_ERROR("Reading file '" << infile << "' failed.");
                return false;
            }
            unsigned int rcounter = 0;
            float px, py, vx, vy, mass;
            sim_file >> num_steps >> num_bodies >> dt >> scale >> distance;

            // setting all the variables
            sim->setNum_steps(num_steps);
            sim->setNum_bodies(num_bodies);
            sim->setDt(dt);
            sim->setScale(scale);
            sim->setDistance(distance);

            while (rcounter++ < num_bodies && sim_file >> px >> py >> vx >> vy >> mass) {
                // adding a body to the simulation
                Body tmp(px, py, distance / 2, vx, vy, mass);
                sim->addBody(tmp);
            }
            sim_file.close();

            return true;
        }

        bool Config::randomGeneration(const std::string &file, unsigned num_steps, unsigned num_bodies) const {
            static const float MAX_POS = 2.0f;
            static const float MAX_V = 0.25f;
            static const float MAX_MASS = 1.0f;
            static const float DELTA_T = 0.01f;
            static const float SCALE = 1.0f;
            static const float DIST = 0.01f;

            std::ofstream sim_file;
            sim_file.open(file.c_str());
            if (!sim_file.good()) {
                LOG_ERROR("Writing to file '" << file << "' failed.");
                return false;
            }
            srand(time(NULL));
            sim_file << num_steps << std::endl << num_bodies << std::endl << DELTA_T << std::endl << SCALE << std::endl << DIST << std::endl;
            for (int i = 0; i < num_bodies; ++i) {
                sim_file << (MAX_POS * rand() / RAND_MAX) - MAX_POS * 0.5f << " ";
                sim_file << (MAX_POS * rand() / RAND_MAX) - MAX_POS * 0.5f << " ";

                sim_file << 2.0f * MAX_V * rand() / RAND_MAX - MAX_V << " ";
                sim_file << 2.0f * MAX_V * rand() / RAND_MAX - MAX_V << " ";

                sim_file << MAX_MASS * rand() / RAND_MAX;
                sim_file << std::endl;
            }

            sim_file.close();

            return true;
        }

        bool Config::write(Simulation const *sim, unsigned num_steps, unsigned num_bodies, float scale, float dt, float distance) const {
            // using namespace nmath;
            if (outfile == "") {
                LOG_INFO("Empty string for file path.");
                return false;
            }
            std::ofstream sim_file;
            sim_file.open(outfile.c_str());
            if (!sim_file.good()) {
                LOG_ERROR("Writing to file '" << outfile << "' failed.");
                return false;
            }
            sim_file << num_steps << std::endl << num_bodies << std::endl << dt << std::endl << scale << std::endl << distance << std::endl;
            for (unsigned int i = 0; i < num_bodies; ++i) {
                // writing a body to the file
                Body tmp = sim->getBodies()[i];
                sim_file << tmp.getX() << " " << tmp.getY() << " " << tmp.getVelocityX() << " " << tmp.getVelocityY() << " " << tmp.getMass() << std::endl;
            }

            sim_file.close();

            return true;
        }


        Config::~Config() {
        }

    } /* namespace nbody */
} /* namespace practical */

