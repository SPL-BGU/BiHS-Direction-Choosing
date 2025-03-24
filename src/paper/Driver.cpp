#include <iostream>
#include "ArgParameters.h"
#include "PancakeDriver.h"
#include "GridDriver.h"
#include "STPDriver.h"

void printRunLine(int argc, char *argv[]) {
    std::cout << "[L] ";
    for (int i = 0; i < argc - 1; ++i) {
        std::cout << argv[i] << " ";
    }
    std::cout << argv[argc - 1] << std::endl;
}

int main(int argc, char *argv[]) {
    printRunLine(argc, argv);
    ArgParameters ap(argc, argv);
    if (ap.domain == "pancake") {
        direction_pancake::testPancake(ap);
    } else if (ap.domain == "grid") {
        direction_grid::testGrid(ap);
    } else if (ap.domain == "stp") {
        direction_stp::testSTP(ap);
    } else {
        std::cerr << "Error: Unknown domain: " << ap.domain << std::endl;
        exit(EXIT_FAILURE);
    }
}
