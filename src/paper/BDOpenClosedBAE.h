/*
 *  BDOpenClosedBAE.h
 */

#ifndef BDOpenClosedBAEBAE_H
#define BDOpenClosedBAEBAE_H

#include <cassert>
#include <vector>
#include <unordered_map>
#include <stdint.h>
#include <unordered_map>
#include "AStarOpenClosed.h"

enum stateLocation {
    kOpenReady = 0,//priority queue 0, low g -> low f
    kOpenWaiting = 1,//priority queue 1, low f -> low g
    kClosed,
    kUnseen
};


const uint64_t kTBDNoNode = 0xFFFFFFFFFFFFFFFFull;

template<typename state>
class BDOpenClosedBAEData {
public:
    BDOpenClosedBAEData() {}

    BDOpenClosedBAEData(const state &theData, double gCost, double hCost, double rhCost, uint64_t parent,
                        uint64_t openLoc,
                        stateLocation location)
            : data(theData), g(gCost), h(hCost), rh(rhCost), parentID(parent), openLocation(openLoc),
              where(location) { reopened = false; }

    state data;
    double g;
    double h;
    double rh;
    uint64_t parentID;
    uint64_t openLocation;
    bool reopened;
    stateLocation where;
};

template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure = BDOpenClosedBAEData<state> >
class BDOpenClosedBAE {
public:
    BDOpenClosedBAE();

    ~BDOpenClosedBAE();

    void Reset(int);

    uint64_t AddOpenNode(const state &val, uint64_t hash, double g, double h, double rh, uint64_t parent = kTBDNoNode,
                         stateLocation whichQueue = kOpenWaiting);

    uint64_t AddClosedNode(state &val, uint64_t hash, double g, double h, double rh, uint64_t parent = kTBDNoNode);

    void KeyChanged(uint64_t objKey);

    void Remove(uint64_t objKey);

    //void IncreaseKey(uint64_t objKey);
    stateLocation Lookup(uint64_t hashKey, uint64_t &objKey) const;

    inline dataStructure &Lookup(uint64_t objKey) { return elements[objKey]; }

    inline const dataStructure &Lookat(uint64_t objKey) const { return elements[objKey]; }

    uint64_t Peek(stateLocation whichQueue) const;

    inline const dataStructure &PeekAt(stateLocation whichQueue) const;

    //if exist min pair, return true, else(one queue is empty) return false;
    //if it returns true, left and right should be set to the pair that is supposed to be returned.
    //bool ExtractMinPair(uint64_t& left, uint64_t& right) ;
    uint64_t Close();

    uint64_t PutToReady();

    void Reopen(uint64_t objKey, stateLocation where=kOpenWaiting);

    uint64_t GetOpenItem(unsigned int which, stateLocation where) { return priorityQueues[where][which]; }

    size_t OpenReadySize() const { return priorityQueues[kOpenReady].size(); }

    size_t OpenWaitingSize() const { return priorityQueues[kOpenWaiting].size(); }

    size_t OpenSize() const { return priorityQueues[kOpenReady].size() + priorityQueues[kOpenWaiting].size(); }

    size_t ClosedSize() const { return size() - OpenReadySize() - OpenWaitingSize(); }

    size_t size() const { return elements.size(); }

    bool ValidateOpenReady(int index = 0) {
        stateLocation whichQ = kOpenReady;
        CmpKey0 compare;
        if (index >= priorityQueues[whichQ].size())
            return true;
        int child1 = index * 2 + 1;
        int child2 = index * 2 + 2;
        if (priorityQueues[whichQ].size() > child1 &&
            compare(elements[priorityQueues[whichQ][index]], elements[priorityQueues[whichQ][child1]]))
            return false;
        if (priorityQueues[whichQ].size() > child2 &&
            compare(elements[priorityQueues[whichQ][index]], elements[priorityQueues[whichQ][child2]]))
            return false;
        return ValidateOpenReady(child1) && ValidateOpenReady(child2);
    }

    bool ValidateOpenWaiting(int index = 0) {
        stateLocation whichQ = kOpenWaiting;
        CmpKey1 compare;
        if (index >= priorityQueues[whichQ].size())
            return true;
        int child1 = index * 2 + 1;
        int child2 = index * 2 + 2;
        if (priorityQueues[whichQ].size() > child1 &&
            compare(elements[priorityQueues[whichQ][index]], elements[priorityQueues[whichQ][child1]]))
            return false;
        if (priorityQueues[whichQ].size() > child2 &&
            compare(elements[priorityQueues[whichQ][index]], elements[priorityQueues[whichQ][child2]]))
            return false;
        return ValidateOpenWaiting(child1) && ValidateOpenWaiting(child2);
    }

private:
    bool HeapifyUp(unsigned int index, stateLocation whichQueue);

    void HeapifyDown(unsigned int index, stateLocation whichQueue);

    //2 queues:
    //priorityQueues[0] is openReady, priorityQueues[1] is openWaiting
    std::vector<std::vector<uint64_t>> priorityQueues;

    // storing the element id; looking up with hash
    std::unordered_map<uint64_t, size_t> table;
    //all the elements, open or closed
    std::vector<dataStructure> elements;
};

template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
void BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::Reopen(uint64_t objKey, stateLocation where)
{
    assert(elements[objKey].where == kClosed);
    elements[objKey].reopened = true;
    elements[objKey].where = where;
    elements[objKey].openLocation = priorityQueues[where].size();
    priorityQueues[where].push_back(objKey);
    HeapifyUp(priorityQueues[where].size() - 1, where);
}


template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::BDOpenClosedBAE() {
    std::vector<uint64_t> queue;
    queue.resize(0);
    priorityQueues.push_back(queue);
    priorityQueues.push_back(queue);
}

template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::~BDOpenClosedBAE() {
}

/**
 * Remove all objects from queue.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
void BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::Reset(int) {
    table.clear();
    elements.clear();
    priorityQueues[0].resize(0);
    priorityQueues[1].resize(0);
}

/**
 * Add object into open list.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
uint64_t
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::AddOpenNode(const state &val, uint64_t hash, double g,
                                                                     double h,
                                                                     double rh, uint64_t parent,
                                                                     stateLocation whichQueue) {
    // should do lookup here...
    if (table.find(hash) != table.end()) {
        assert(false);
    }
    if (whichQueue == kOpenReady) {
        elements.push_back(dataStructure(val, g, h, rh, parent, priorityQueues[0].size(), kOpenReady));

    } else if (whichQueue == kOpenWaiting) {
        elements.push_back(dataStructure(val, g, h, rh, parent, priorityQueues[1].size(), kOpenWaiting));
    }

    if (parent == kTBDNoNode)
        elements.back().parentID = elements.size() - 1;
    table[hash] = elements.size() - 1; // hashing to element list location

    priorityQueues[whichQueue].push_back(elements.size() - 1);
    HeapifyUp(priorityQueues[whichQueue].size() - 1, whichQueue);

    return elements.size() - 1;
}

/**
 * Add object into closed list.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
uint64_t
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::AddClosedNode(state &val, uint64_t hash, double g, double h,
                                                                       double rh, uint64_t parent) {
    // should do lookup here...
    assert(table.find(hash) == table.end());
    elements.push_back(dataStructure(val, g, h, rh, parent, 0, kClosed));
    if (parent == kTBDNoNode)
        elements.back().parentID = elements.size() - 1;
    table[hash] = elements.size() - 1; // hashing to element list location
    return elements.size() - 1;
}

/**
 * Indicate that the key for a particular object has changed.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
void BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::KeyChanged(uint64_t val) {
    if (elements[val].where == kOpenReady) {
        if (!HeapifyUp(elements[val].openLocation, kOpenReady))
            HeapifyDown(elements[val].openLocation, kOpenReady);
    } else if (elements[val].where == kOpenWaiting) {
        if (!HeapifyUp(elements[val].openLocation, kOpenWaiting))
            HeapifyDown(elements[val].openLocation, kOpenWaiting);
    }
}

template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
void BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::Remove(uint64_t val) {

    int index = elements[val].openLocation;
    stateLocation whichQueue = elements[val].where;
    elements[val].where = kClosed;
    priorityQueues[whichQueue][index] = priorityQueues[whichQueue][priorityQueues[whichQueue].size() - 1];
    elements[priorityQueues[whichQueue][index]].openLocation = index;
    priorityQueues[whichQueue].pop_back();

    if (!HeapifyUp(index, whichQueue))
        HeapifyDown(index, whichQueue);


}

/**
 * Returns location of object as well as object key.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
stateLocation
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::Lookup(uint64_t hashKey, uint64_t &objKey) const {
    auto it = table.find(hashKey);
    if (it == table.end())
        return kUnseen;

    objKey = it->second;
    return elements[objKey].where;
}


/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
uint64_t BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::Peek(stateLocation whichQueue) const {
    if (whichQueue == kOpenReady) {
        assert(OpenReadySize() != 0);
    } else if (whichQueue == kOpenWaiting) {
        assert(OpenWaitingSize() != 0);
    }
    return priorityQueues[whichQueue][0];
}

/**
 * Peek at the next item to be expanded.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
inline const dataStructure &
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::PeekAt(stateLocation whichQueue) const {
    if (whichQueue == kOpenReady) {
        assert(OpenReadySize() != 0);
    } else if (whichQueue == kOpenWaiting) {
        assert(OpenWaitingSize() != 0);
    }
    return elements[priorityQueues[whichQueue][0]];
}


/**
 * Move the best item to the closed list and return key.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
uint64_t BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::Close() {
    assert(OpenReadySize() != 0);

    uint64_t ans = priorityQueues[0][0];
    elements[ans].where = kClosed;
    priorityQueues[0][0] = priorityQueues[0][OpenReadySize() - 1];
    elements[priorityQueues[0][0]].openLocation = 0;
    priorityQueues[0].pop_back();

    HeapifyDown(0, kOpenReady);

    return ans;
}

template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
uint64_t BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::PutToReady() {
    assert(OpenWaitingSize() != 0);

    //remove it from openWaiting
    uint64_t ans = priorityQueues[kOpenWaiting][0];
    uint64_t back = priorityQueues[kOpenWaiting].back();
    priorityQueues[kOpenWaiting][0] = back;
    priorityQueues[kOpenWaiting].pop_back();
    elements[back].openLocation = 0;

    HeapifyDown(0, kOpenWaiting);

    //put it to openReady
    priorityQueues[kOpenReady].push_back(ans);
    elements[ans].where = kOpenReady;
    elements[ans].openLocation = priorityQueues[kOpenReady].size() - 1;

    HeapifyUp(priorityQueues[kOpenReady].size() - 1, kOpenReady);

    return ans;
}

/**
 * Moves a node up the heap. Returns true if the node was moved, false otherwise.
 */
template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
bool BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::HeapifyUp(unsigned int index, stateLocation whichQueue) {
    if (index == 0) return false;
    int parent = (index - 1) / 2;

    if (whichQueue == kOpenReady) {
        CmpKey0 compare;
        if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]])) {
            unsigned int tmp = priorityQueues[whichQueue][parent];
            priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
            priorityQueues[whichQueue][index] = tmp;
            elements[priorityQueues[whichQueue][parent]].openLocation = parent;
            elements[priorityQueues[whichQueue][index]].openLocation = index;
            HeapifyUp(parent, whichQueue);
            return true;
        }
    } else if (whichQueue == kOpenWaiting) {
        CmpKey1 compare;
        if (compare(elements[priorityQueues[whichQueue][parent]], elements[priorityQueues[whichQueue][index]])) {
            unsigned int tmp = priorityQueues[whichQueue][parent];
            priorityQueues[whichQueue][parent] = priorityQueues[whichQueue][index];
            priorityQueues[whichQueue][index] = tmp;
            elements[priorityQueues[whichQueue][parent]].openLocation = parent;
            elements[priorityQueues[whichQueue][index]].openLocation = index;
            HeapifyUp(parent, whichQueue);
            return true;
        }
    }

    return false;
}

template<typename state, typename CmpKey0, typename CmpKey1, class dataStructure>
void
BDOpenClosedBAE<state, CmpKey0, CmpKey1, dataStructure>::HeapifyDown(unsigned int index, stateLocation whichQueue) {

    unsigned int child1 = index * 2 + 1;
    unsigned int child2 = index * 2 + 2;

    if (whichQueue == kOpenReady) {
        CmpKey0 compare;
        int which;
        unsigned int count = priorityQueues[whichQueue].size();
        // find smallest child
        if (child1 >= count)
            return;
        else if (child2 >= count)
            which = child1;
        else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
            which = child1;
        else
            which = child2;

        if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]]))) {
            unsigned int tmp = priorityQueues[whichQueue][which];
            priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
            priorityQueues[whichQueue][index] = tmp;
            elements[priorityQueues[whichQueue][which]].openLocation = which;
            elements[priorityQueues[whichQueue][index]].openLocation = index;
            HeapifyDown(which, whichQueue);
        }
    } else if (whichQueue == kOpenWaiting) {
        CmpKey1 compare;
        int which;
        unsigned int count = priorityQueues[whichQueue].size();
        // find smallest child
        if (child1 >= count) // no children; done
            return;
        else if (child2 >= count) // one child - compare there
            which = child1;
            // find larger child to move up
        else if (!(compare(elements[priorityQueues[whichQueue][child1]], elements[priorityQueues[whichQueue][child2]])))
            which = child1;
        else
            which = child2;

        if (!(compare(elements[priorityQueues[whichQueue][which]], elements[priorityQueues[whichQueue][index]]))) {
            unsigned int tmp = priorityQueues[whichQueue][which];
            priorityQueues[whichQueue][which] = priorityQueues[whichQueue][index];
            priorityQueues[whichQueue][index] = tmp;
            elements[priorityQueues[whichQueue][which]].openLocation = which;
            elements[priorityQueues[whichQueue][index]].openLocation = index;
            HeapifyDown(which, whichQueue);
        }
    }

}

#endif
