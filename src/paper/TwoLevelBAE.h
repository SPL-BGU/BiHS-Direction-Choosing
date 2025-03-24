#ifndef SRC_PAPER_TWOLEVELBAE_H
#define SRC_PAPER_TWOLEVELBAE_H

#include <cmath>
#include <iostream>
#include "BDOpenClosedBAE.h"
#include "FPUtil.h"
#include "Heuristic.h"

// Comparators which return true if i2 is preferred over i1

template<class state>
struct BTLFCompare { // B-comparator
    bool operator()(const BDOpenClosedBAEData<state> &i1, const BDOpenClosedBAEData<state> &i2) const {
        double p1 = i1.g + i1.h;
        double p2 = i2.g + i2.h;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // high g-cost over low
        }
        return (fgreater(p1, p2)); // low priority over high
    }
};

template<class state>
struct BTLBCompare { // F-comparator
    bool operator()(const BDOpenClosedBAEData<state> &i1, const BDOpenClosedBAEData<state> &i2) const {
        double p1 = 2 * i1.g + i1.h - i1.rh;
        double p2 = 2 * i2.g + i2.h - i2.rh;
        if (fequal(p1, p2)) {
            return (fless(i1.g, i2.g)); // high g-cost over low
        }
        return (fgreater(p1, p2)); // low priority over high
    }
};

template<class state, class action, class environment>
class TwoLevelBAE {
public:
    TwoLevelBAE(double gcd_ = 1.0) {
        forwardHeuristic = 0;
        backwardHeuristic = 0;
        env = 0;
        gcd = gcd_;
        Reset();
    }

    ~TwoLevelBAE() {}

    void ResetNodeCount() { nodesExpanded = nodesTouched = uniqueNodesExpanded = 0; }

    void Reset() {
        ResetNodeCount();
        forwardQueue.Reset(0);
        backwardQueue.Reset(0);
    }

    void GetPath(environment *env, const state &from, const state &to,
                 Heuristic<state> *forward, Heuristic<state> *backward, std::vector<state> &thePath);

    bool DoSingleSearchStep(std::vector<state> &thePath);

    uint64_t GetNodesExpanded() {
        return nodesExpanded;
    }

    float GetSolLen() {
        return currentCost;
    }

    uint64_t GetUniqueNodesExpanded() {
        return uniqueNodesExpanded;
    }

    int GetNumOfExpandedWithFGreaterC(float cstar);

private:
    double GetCurrentBBound();

    void UpdateReadyQueue();

    void Expand(BDOpenClosedBAE<state, BTLBCompare<state>, BTLFCompare<state>> &current,
                BDOpenClosedBAE<state, BTLBCompare<state>, BTLFCompare<state>> &opposite,
                Heuristic<state> *heuristic, Heuristic<state> *revHeuristic, const state &target, const state &source);

    uint64_t nodesTouched, nodesExpanded, uniqueNodesExpanded;
    state goal, start;

    BDOpenClosedBAE<state, BTLBCompare<state>, BTLFCompare<state>> forwardQueue;
    BDOpenClosedBAE<state, BTLBCompare<state>, BTLFCompare<state>> backwardQueue;

    Heuristic<state> *forwardHeuristic;
    Heuristic<state> *backwardHeuristic;
    environment *env;

    double gcd;

    double cLowerBound;

    double currentCost;

    state middleNode;

    bool expandForward;
};

template<class state, class action, class environment>
int TwoLevelBAE<state, action, environment>::GetNumOfExpandedWithFGreaterC(float cstar) {
    int count = 0;
    for (int i = 0; i < forwardQueue.size(); ++i) {
        auto &n = forwardQueue.Lookup(i);
        if (n.where == kClosed && n.g + forwardHeuristic->HCost(n.data, goal) > cstar) {
            count++;
        }
    }
    for (int i = 0; i < backwardQueue.size(); ++i) {
        auto &n = backwardQueue.Lookup(i);
        if (n.where == kClosed && n.g + backwardHeuristic->HCost(n.data, start) > cstar) {
            count++;
        }
    }
    return count;
}


template<class state, class action, class environment>
void TwoLevelBAE<state, action, environment>::GetPath(environment *env_, const state &from, const state &to,
                                                      Heuristic<state> *forward, Heuristic<state> *backward,
                                                      std::vector<state> &thePath) {
    Reset();
    env = env_;
    forwardHeuristic = forward;
    backwardHeuristic = backward;
    thePath.resize(0);
    start = from;
    goal = to;
    expandForward = true;
    currentCost = DBL_MAX;
    if (start == goal)
        return;

    forwardQueue.AddOpenNode(start, env->GetStateHash(start), 0, forwardHeuristic->HCost(start, goal), 0, kTBDNoNode,
                             kOpenReady);
    backwardQueue.AddOpenNode(goal, env->GetStateHash(goal), 0, backwardHeuristic->HCost(goal, start), 0, kTBDNoNode,
                              kOpenReady);
    cLowerBound = max(forwardHeuristic->HCost(start, goal), backwardHeuristic->HCost(goal, start));

    while (!DoSingleSearchStep(thePath)) {}
}

template<class state, class action, class environment>
bool TwoLevelBAE<state, action, environment>::DoSingleSearchStep(std::vector<state> &thePath) {
    UpdateReadyQueue();

    if (currentCost <= cLowerBound) {
        //solution found
        return true;
    }

    if (expandForward) {
        Expand(forwardQueue, backwardQueue, forwardHeuristic, backwardHeuristic, goal, start);
        expandForward = false;
    } else {
        Expand(backwardQueue, forwardQueue, backwardHeuristic, forwardHeuristic, start, goal);
        expandForward = true;
    }

    // If the solution lead to emptying one of the Open, which will lead to an infinite loop.
    if (currentCost < DBL_MAX && (forwardQueue.OpenSize() == 0 || backwardQueue.OpenSize() == 0)) {
        //solution found
        return true;
    }
    return false;
}

template<class state, class action, class environment>
double TwoLevelBAE<state, action, environment>::GetCurrentBBound() {
    if (forwardQueue.OpenReadySize() == 0 || backwardQueue.OpenReadySize() == 0) {
        return DBL_MAX;
    }
    auto &n1 = forwardQueue.Lookup(forwardQueue.Peek(kOpenReady));
    auto &n2 = backwardQueue.Lookup(backwardQueue.Peek(kOpenReady));
    double lb = ((2 * n1.g + n1.h - n1.rh) + (2 * n2.g + n2.h - n2.rh)) / 2;
    return ceil(lb / gcd) * gcd;
}

template<class state, class action, class environment>
void TwoLevelBAE<state, action, environment>::UpdateReadyQueue() {
    auto ff = forwardQueue.OpenWaitingSize() == 0 ? DBL_MAX :
              forwardQueue.Lookup(forwardQueue.Peek(kOpenWaiting)).g +
              forwardQueue.Lookup(forwardQueue.Peek(kOpenWaiting)).h;
    auto fb = backwardQueue.OpenWaitingSize() == 0 ? DBL_MAX :
              backwardQueue.Lookup(backwardQueue.Peek(kOpenWaiting)).g +
              backwardQueue.Lookup(backwardQueue.Peek(kOpenWaiting)).h;
    double minf = min(ff, fb);
    while (minf <= GetCurrentBBound()) {
        cLowerBound = minf;

        // Move nextF layer into ready
        while (forwardQueue.OpenWaitingSize() > 0 && forwardQueue.Lookup(forwardQueue.Peek(kOpenWaiting)).g +
                                                     forwardQueue.Lookup(forwardQueue.Peek(kOpenWaiting)).h ==
                                                     cLowerBound) {
            forwardQueue.PutToReady();
        }

        while (backwardQueue.OpenWaitingSize() > 0 && backwardQueue.Lookup(backwardQueue.Peek(kOpenWaiting)).g +
                                                      backwardQueue.Lookup(backwardQueue.Peek(kOpenWaiting)).h ==
                                                      cLowerBound) {
            backwardQueue.PutToReady();
        }

        // Check next f value
        ff = forwardQueue.OpenWaitingSize() == 0 ? DBL_MAX :
                  forwardQueue.Lookup(forwardQueue.Peek(kOpenWaiting)).g +
                  forwardQueue.Lookup(forwardQueue.Peek(kOpenWaiting)).h;
        fb = backwardQueue.OpenWaitingSize() == 0 ? DBL_MAX :
                  backwardQueue.Lookup(backwardQueue.Peek(kOpenWaiting)).g +
                  backwardQueue.Lookup(backwardQueue.Peek(kOpenWaiting)).h;
        minf = min(ff, fb);
    }

    // If both waiting are empty, minf is DBL_MAX, but if the bound need to increase, it will never enter the loop thus
    // not increasing the bound. This verifies that it will increase if needed regardless of waiting.
    if (minf == DBL_MAX) {
        cLowerBound = max(cLowerBound, GetCurrentBBound());
    }
}

template<class state, class action, class environment>
void
TwoLevelBAE<state, action, environment>::Expand(BDOpenClosedBAE<state, BTLBCompare<state>, BTLFCompare<state>> &current,
                                                BDOpenClosedBAE<state, BTLBCompare<state>, BTLFCompare<state>> &opposite,
                                                Heuristic<state> *heuristic, Heuristic<state> *revHeuristic,
                                                const state &target, const state &source) {
    uint64_t nextID = current.Close();
    nodesExpanded++;

    if (current.Lookup(nextID).reopened == false)
        uniqueNodesExpanded++;

    static std::vector<state> neighbors;
    env->GetSuccessors(current.Lookup(nextID).data, neighbors);
    for (auto &succ: neighbors) {
        nodesTouched++;
        uint64_t childID;
        uint64_t hash = env->GetStateHash(succ);
        auto loc = current.Lookup(hash, childID);
        auto &childData = current.Lookup(childID);
        auto &parentData = current.Lookup(nextID);

        double edgeCost = env->GCost(parentData.data, succ);

        // ignore states with greater cost than best solution
        if (fgreatereq(parentData.g + edgeCost + heuristic->HCost(succ, target), currentCost))
            continue;

        switch (loc) {
            case kClosed: {
                uint64_t oppositeID;
                auto oppositeLoc = opposite.Lookup(env->GetStateHash(succ), oppositeID);
                if (fless(parentData.g + edgeCost, childData.g) && oppositeLoc != kClosed) {
                    uint64_t currLoopID = nextID;
                    while (currLoopID != 0) {
                        std::cout << currLoopID << ", ";
                        currLoopID = current.Lookup(currLoopID).parentID;
                    }
                    std::cout << std::endl;
                    std::cout << "Non optimal g" << std::endl;
                    std::cerr << "Non optimal g" << std::endl;
                    current.Lookup(childID).parentID = nextID;
                    current.Lookup(childID).g = current.Lookup(nextID).g + edgeCost;
                    double childF = current.Lookup(childID).g + current.Lookup(childID).h;
                    current.Reopen(childID, flesseq(childF, cLowerBound) ? kOpenReady : kOpenWaiting);
                }
                break;
            }
            case kOpenReady:
            case kOpenWaiting: {
                if (fless(current.Lookup(nextID).g + edgeCost, current.Lookup(childID).g)) {
                    current.Lookup(childID).parentID = nextID;
                    current.Lookup(childID).g = current.Lookup(nextID).g + edgeCost;
                    current.KeyChanged(childID);
                    if (loc == kOpenWaiting) {
                        if (current.Lookup(current.Peek(kOpenWaiting)).g +
                            current.Lookup(current.Peek(kOpenWaiting)).h <= cLowerBound) {
                            current.PutToReady();
                        }
                    }
                    // We postpone moving the node to Ready to the next cycle. It will happen at the very start of the
                    // cycle, where this is the last step.
                    // Try to find it on the opposite OPEN. If it's there, we got a solution - check if it's better
                    uint64_t oppositeID;
                    auto oppositeLoc = opposite.Lookup(env->GetStateHash(succ), oppositeID);
                    if ((oppositeLoc == kOpenReady || oppositeLoc == kOpenWaiting) &&
                        fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(oppositeID).g, currentCost)) {
                        currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(oppositeID).g;
                        middleNode = succ;
                        // Prune the node if it has already been expanded in the opposite direction
                    } else if (oppositeLoc == kClosed) {
                        current.Remove(childID);
                    }
                }
                break;
            }
            case kUnseen: {
                uint64_t oppositeID;
                auto oppositeLoc = opposite.Lookup(env->GetStateHash(succ), oppositeID);
                // Do not expand a node which has already been expanded in the opposite direction
                if (oppositeLoc == kClosed) {
                    break;
                }
                double newNodeF = current.Lookup(nextID).g + edgeCost + heuristic->HCost(succ, target);
                auto newLoc = flesseq(newNodeF, cLowerBound) ? kOpenReady : kOpenWaiting;
                current.AddOpenNode(succ, env->GetStateHash(succ),
                                    current.Lookup(nextID).g + edgeCost,
                                    heuristic->HCost(succ, target),
                                    revHeuristic->HCost(succ, source),
                                    nextID, newLoc);

                if (oppositeLoc == kOpenReady || oppositeLoc == kOpenWaiting) {
                    if (fless(current.Lookup(nextID).g + edgeCost + opposite.Lookup(oppositeID).g, currentCost)) {
                        currentCost = current.Lookup(nextID).g + edgeCost + opposite.Lookup(oppositeID).g;
                        middleNode = succ;
                    }
                }
                break;
            }
        }
    }
}

#endif //SRC_PAPER_TWOLEVELBAE_H
