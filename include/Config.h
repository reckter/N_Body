/*
 * Config.h
 *
 *  Created on: Dec 15, 2014
 *      Author: ahueck
 *
 * Reads/Writes simulation files. Can also write randomly generated simulation files.
 */
#ifndef CONFIG_H_
#define CONFIG_H_

#include <vector>
#include <string>

#include "Simulation.h"

namespace practical {
    namespace nbody {

        class Simulation;

        class Config {
        private:
            std::string infile;
            std::string outfile;

        public:
            Config();

            Config(const std::string &infile, const std::string &outfile);

            /**
            * Read simulation file and set the passed references to the content of the file
            * Uses variable "infile"
            */
            virtual bool read(Simulation *sim, unsigned &num_steps, unsigned &num_bodies, float &scale, float &dt, float &distance) const;

            /**
            * Randomly generate content of a simulation and write to a file
            */
            virtual bool randomGeneration(const std::string &file, unsigned num_steps, unsigned num_bodies) const;

            /**
            * Write simulation file with passed values
            * Uses variable "outfile"
            */
            virtual bool write(Simulation const *sim, unsigned num_steps, unsigned num_bodies, float scale, float dt, float distance) const;

            virtual ~Config();
        };

    } /* namespace nbody */
} /* namespace practical */

#endif /* CONFIG_H_ */
