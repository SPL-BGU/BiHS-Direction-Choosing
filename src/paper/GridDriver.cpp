#include "GridDriver.h"
#include "ScenarioLoader.h"
#include "Map.h"
#include "Map2DEnvironment.h"
#include "BAE.h"
#include "BAEBFD.h"
#include "DBBS/DBBS.h"
#include "TwoLevelBAE.h"


namespace direction_grid {

std::string getFileName(const std::string &path) {
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? path : path.substr(pos + 1);
}

void testGrid(const ArgParameters &ap) {
    ScenarioLoader s(ap.scenario.c_str());
    Map m(ap.map.c_str());
    MapEnvironment env(&m);
    env.SetDiagonalCost(1.5);

    Timer timer;
    std::vector<xyLoc> solutionPath;
    xyLoc start, goal;

    std::cout << "[D] domain: " << ap.domain << "; map: " << getFileName(ap.map) << std::endl;

    for (int i: ap.instances) {
        if (i < 0 || i >= s.GetNumExperiments()) {
            continue;
        }

        std::cout << "[I] ID: " << i << "; start: " << start << "; goal: " << goal << std::endl;
        start.x = s.GetNthExperiment(i).GetStartX();
        start.y = s.GetNthExperiment(i).GetStartY();
        goal.x = s.GetNthExperiment(i).GetGoalX();
        goal.y = s.GetNthExperiment(i).GetGoalY();

        if (ap.hasAlgorithm("BAE-a")) {
            BAE<xyLoc, tDirection, MapEnvironment> bae(true, 1.0, 0.5);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-a; solution %1.1f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-a")) {
            BAEBFD<xyLoc, tDirection, MapEnvironment> bae(true, 1.0, 0.5);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-a; solution %1.1f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-f")) {
            BAEBFD<xyLoc, tDirection, MapEnvironment> bae(false, 1.0, 0.5);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-f; solution %1.1f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("TLBAE")) {
            TwoLevelBAE<xyLoc, tDirection, MapEnvironment> bae(0.5);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = bae.GetSolLen();
            printf("[R] alg: TLBAE; solution %1.1f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("DBBS")) {
            DBBS<xyLoc, tDirection, MapEnvironment, MinCriterion::MinB> dbbs(true, true, 1.0, 0.5);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: dbbs; solution %1.1f; expanded: %llu; fabove: 0; time: %1.6fs\n",
                   solLen, dbbs.GetNodesExpanded(),
                   timer.GetElapsedTime());
        }
    }
}

}
