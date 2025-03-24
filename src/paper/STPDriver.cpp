//
// Created by Lior Siag on 24/03/2025.
//

#include "STPDriver.h"
#include "MNPuzzle.h"
#include "STPInstances.h"
#include "BAE.h"
#include "BAEBFD.h"
#include "TwoLevelBAE.h"
#include "DBBS/MinCriterion.h"
#include "DBBS/DBBS.h"

namespace direction_stp {
void testSTP(const ArgParameters &ap) {
    printf("[D] domain: stp; heuristic: MD\n");
    MNPuzzleState<4, 4> goal;
    std::vector<MNPuzzleState<4, 4>> solutionPath;
    MNPuzzle<4, 4> env;
    Timer timer;

    for (int i: ap.instances) {
        MNPuzzleState<4, 4> start = STP::GetKorfInstance(i);
        std::cout << "[I] id: " << i << "; instance: " << start << std::endl;

        if (ap.hasAlgorithm("BAE-a")) {
            BAE<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae;
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-a; solution: %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-p")) {
            BAE<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae(false);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-p; solution: %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-a")) {
            BAEBFD<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae(BaeDirStrategy::BFD_Alternating);
            timer.StartTimer();
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-a; solution: %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-f")) {
            BAEBFD<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae(BaeDirStrategy::BFD_Forward);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-f; solution: %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("BAE-bfd-b")) {
            BAEBFD<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae(BaeDirStrategy::BFD_Backward);
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: BAE-bfd-b; solution: %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("TLBAE")) {
            TwoLevelBAE<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> bae;
            timer.StartTimer();
            bae.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = bae.GetSolLen();
            printf("[R] alg: TLBAE; solution: %1.0f; expanded: %llu; fabove: %d; time: %1.6fs\n",
                   solLen, bae.GetNodesExpanded(), bae.GetNumOfExpandedWithFGreaterC(solLen),
                   timer.GetElapsedTime());
        }

        if (ap.hasAlgorithm("DBBS")) {
            DBBS<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, MinCriterion::MinB> dbbs(true);
            timer.StartTimer();
            dbbs.GetPath(&env, start, goal, &env, &env, solutionPath);
            timer.EndTimer();
            double solLen = env.GetPathLength(solutionPath);
            printf("[R] alg: dbbs; solution: %1.0f; expanded: %llu; fabove: 0; time: %1.6fs\n",
                   solLen, dbbs.GetNodesExpanded(),
                   timer.GetElapsedTime());
        }
    }
}
}
