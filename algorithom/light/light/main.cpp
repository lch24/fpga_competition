#include "experiment.h"
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--self-test") {
            selfTest();
            return 0;
        }
        if (argc != 3 && argc != 5) {
            std::cerr << "Usage: light INPUT_DIRECTORY NEW_OUTPUT_DIRECTORY [COLUMNS ROWS]\n"
                         "       light --self-test\n";
            return 2;
        }
        const int columns = argc == 5 ? std::stoi(argv[3]) : 8;
        const int rows = argc == 5 ? std::stoi(argv[4]) : 5;
        return runExperiment(argv[1], argv[2], columns, rows);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
