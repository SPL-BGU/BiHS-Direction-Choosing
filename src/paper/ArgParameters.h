#ifndef SRC_PAPER_ARGPARAMETERS_H
#define SRC_PAPER_ARGPARAMETERS_H


#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

class ArgParameters {
public:
    ArgParameters(int argc, char *argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg == "--help") {
                ArgParameters::help();
            } else if (arg == "-d" || arg == "--domain") {
                ArgParameters::verifyValidFlagValue(argc, argv, ++i);
                this->domain = argv[i];
            } else if (arg == "-h" || arg == "--heuristic") {
                ArgParameters::verifyValidFlagValue(argc, argv, ++i);
                this->heuristic = argv[i];
            } else if (arg == "-m" || arg == "--map") {
                ArgParameters::verifyValidFlagValue(argc, argv, ++i);
                this->map = argv[i];
            } else if (arg == "-s" || arg == "--scenario") {
                ArgParameters::verifyValidFlagValue(argc, argv, ++i);
                this->scenario = argv[i];
            } else if (arg == "-i" || arg == "--instances") {
                ArgParameters::verifyValidFlagValue(argc, argv, ++i);
                std::vector<std::string> lineInstances;
                while (i < argc && argv[i][0] != '-') {
                    lineInstances.emplace_back(argv[i]);
                    ++i;
                }
                --i; // Adjust for the loop increment
                this->parseInstanceRanges(lineInstances);
            } else if (arg == "-a" || arg == "--algorithms") {
                ArgParameters::verifyValidFlagValue(argc, argv, ++i);
                while (i < argc && argv[i][0] != '-') {
                    this->algs.emplace_back(argv[i]);
                    ++i;
                }
                --i; // Adjust for the loop increment
            } else {
                std::cerr << "Error: Unknown argument: " << arg << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        // Lowercase domain and heuristic, so it's easier to work with them later
        std::transform(this->domain.begin(), this->domain.end(), this->domain.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        std::transform(this->heuristic.begin(), this->heuristic.end(), this->heuristic.begin(),
                       [](unsigned char c) { return std::tolower(c); });
    }

    bool hasAlgorithm(const std::string &alg) const {
        return std::find(algs.begin(), algs.end(), alg) != algs.end();
    }

    void parseInstanceRanges(const std::vector<std::string> &input) {
        for (const std::string &part: input) {
            size_t dashPos = part.find('-');

            if (dashPos == std::string::npos) {
                // It's a single number
                try {
                    int number = std::stoi(part);
                    this->instances.push_back(number);
                } catch (const std::invalid_argument &e) {
                    std::cerr << "Error: Invalid instance: " << part << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                // It's a range
                try {
                    int start = std::stoi(part.substr(0, dashPos));
                    int end = std::stoi(part.substr(dashPos + 1));

                    if (start >= end) {
                        std::cerr << "Error: Invalid range: " << part << std::endl;
                        exit(EXIT_FAILURE);
                    }

                    for (int i = start; i < end; ++i) {
                        this->instances.push_back(i);
                    }
                } catch (const std::invalid_argument &e) {
                    std::cerr << "Error: Invalid input: " << part << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
    }

    friend std::ostream &operator<<(std::ostream &os, const ArgParameters &params) {
        os << "Domain: " << params.domain << "\n";

        os << "Heuristic: " << params.heuristic << "\n";

        os << "Algorithms: ";
        for (const auto &alg: params.algs) { os << alg << " "; }
        os << "\n";

        os << "Instances: ";
        for (const auto &instance: params.instances) { os << instance << " "; }
        os << "\n";

        return os;
    }

    static void help() {
        std::cout << "Usage: program [options]\n"
                  << "Options:\n"
                  << "  --help                   Show this help message and exit\n"
                  << "  -d, --domain <name>      Specify the domain\n"
                  << "  -h, --heuristic <name>   Specify the heuristic\n"
                  << "  -m, --map <file>         Specify the map file\n"
                  << "  -s, --scenario <file>    Specify the scenario file\n"
                  << "  -i, --instances <list>   Specify instances (single or range, e.g., 1 2 5-10)\n"
                  << "  -a, --algorithms <list>  Specify algorithms (space-separated)\n"
                  << "\nExamples:\n"
                  << "  program -d grid -h od -i 0-1000 -a BAE-a TLBAE -m maps/orz302d.map -s scenarios/orz302d.map.scen\n"
                  << "  program -d pancake -h 0 -i 0-100 -a BAE-a TLBAE\n";
        exit(EXIT_SUCCESS);
    }

    std::string domain;
    std::string heuristic;
    std::vector<std::string> algs;
    std::vector<int> instances;
    std::string map;
    std::string scenario;

private:
    static void verifyValidFlagValue(int argc, char *argv[], int index) {
        if (index >= argc || argv[index][0] == '-') {
            std::cerr << "Missing values for: " << argv[index - 1] << std::endl;
            exit(EXIT_FAILURE);
        }
    }
};

#endif //SRC_PAPER_ARGPARAMETERS_H
