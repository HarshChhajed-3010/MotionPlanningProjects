#include "KinematicChain.h"
#include "RTP.h"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " <num_links>\n";
        exit(0);
    }

    auto numLinks = std::stoul(argv[1]); // number of links in the kinematic chain
    Environment env = createHornEnvironment(numLinks, log((double)numLinks) / (double)numLinks);
    auto chain(std::make_shared<KinematicChainSpace>(numLinks, 1. / (double)numLinks, &env));
    ompl::geometric::SimpleSetup ss(chain);

    ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));

    ompl::base::ScopedState<> start(chain), goal(chain);
    std::vector<double> startVec(numLinks, boost::math::constants::pi<double>() / (double)numLinks);
    std::vector<double> goalVec(numLinks, 0);

    startVec[0] = 0.;
    goalVec[0] = boost::math::constants::pi<double>() - .001;
    chain->setup();
    chain->copyFromReals(start.get(), startVec);
    chain->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    // SEKRIT BONUS FEATURE:
    // if you specify a second command line argument, it will solve the
    // problem just once with STRIDE and print out the solution path.
    if (argc > 2)
    {
        ss.setPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
        ss.setup();
        ss.print();
        ss.solve(3600);
        ss.simplifySolution();

        std::ofstream pathfile(boost::str(boost::format("kinematic_path_%i.dat") % numLinks).c_str());
        ss.getSolutionPath().printAsMatrix(pathfile);
        exit(0);
    }

    // by default, use the Benchmark class
    double runtime_limit = 60, memory_limit = 1024; 
    int run_count = 20;  
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "KinematicChain");
    b.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation()));

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    exit(0);
}
