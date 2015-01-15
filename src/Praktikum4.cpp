//============================================================================
// Name        : Praktikum4.cpp
// Author      :
// Version     :
// Description : Main entry point. In principle nothing to be done here.
//============================================================================

#include "Config.h"
#include "Simulation.h"
#include "PTime.h"
#include "Logger.h"

//#include <omp.h>

#ifdef BUILD_QT
#include <qapplication.h>
#include <gui/SimulationGUI.h>
#endif

/**
* No change should be needed.
*/
int main(int argc, char **argv) {
    if (argc < 2) {
        LOG_INFO("Needs input file for simulation.");
        return 1;
    }
    std::string outfile = "";
    if (argc > 2) {
        outfile = argv[2];
    }
#ifdef _OPENMP
	LOG_INFO("With OpenMP support compiled. Max. # threads: " << omp_get_max_threads());
#endif
    practical::nbody::Config c(argv[1], outfile);
    practical::nbody::Simulation sim(c);
    LOG_MSG("Initializing data (1 = success): " << sim.initialize());
#ifdef BUILD_QT
	QApplication app(argc, argv);
	practical::nbody::ui::SimulationGUI view(sim);
	sim.setMinMaxMass(view);
	view.show();
	return 	app.exec();
#else
    double start = practical::nbody::now();
    sim.run();
    double end = practical::nbody::now();
    LOG_MSG("Time for simulation: " << end - start << "s");
    if (outfile != "") {
        sim.writeResults();
    }
    return 0;
#endif
}
