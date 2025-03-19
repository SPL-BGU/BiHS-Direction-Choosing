#include "PancakeDriver.h"
#include "PancakePuzzle.h"
#include "PancakeInstances.h"
#include "BAE.h"
#include "BAEBFD.h"
#include "TwoLevelBAE.h"
#include "DBBS/DBBS.h"


namespace direction_pancake {
const int N = 16;

int getGap(const std::string &h) {
    try {
        return std::stoi(h);
    } catch (...) {
        return std::stoi(h.substr(h.find('-') + 1));
    }

}

void testPancake(const ArgParameters &ap) {
    int gap = getGap(ap.heuristic);
    printf("[D] Pancake-%d GAP-%d\n", N, gap);
    PancakePuzzle<N> env(gap);
    PancakePuzzleState<N> start;
    PancakePuzzleState<N> goal;
    std::vector<PancakePuzzleState<N>> solutionPath;
    Timer timer;

    for (int i: ap.instances) {
        if (!GetPancakeInstance(start, i)) {
            std::cerr << "Error: Invalid Pancake Instance: " << i << std::endl;
            exit(EXIT_FAILURE);
        }

        std::cout << "[I] id: " << i << "; instance: " << start << std::endl;

        if (ap.hasAlgorithm("BAE-a")) {
            BAE<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bae;
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-a; solution %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-a")) {
            BAEBFD<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bae;
            timer.StartTimer();
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-a; solution %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-f")) {
            BAEBFD<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bae(false);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-f; solution %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("TLBAE")) {
            TwoLevelBAE<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>> bae;
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = bae.GetSolLen();
            printf("[R] alg: TLBAE; solution %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("DBBS")) {
            DBBS<PancakePuzzleState<N>, PancakePuzzleAction, PancakePuzzle<N>, MinCriterion::MinB> dbbs(true, true, 1.0,
                                                                                                        0.5);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: dbbs; solution %1.0f; expanded: %llu; fabove: 0; time: %1.6fs\n",
                   solLen, dbbs.GetNodesExpanded(),
                   timer.GetElapsedTime());
        }

    }
}
}
