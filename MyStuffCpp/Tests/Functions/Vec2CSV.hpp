#ifndef CSV_EXPORTER_HPP
#define CSV_EXPORTER_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

/**
 * Writes a 2D vector to a CSV file.
 * Each inner vector represents one row.
 */
template <typename T>
void writeToCSV(const std::string& filename, const std::vector<std::vector<T>>& data) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return;
    }

    for (const auto& row : data) {
        for (size_t i = 0; i < row.size(); ++i) {
            outFile << row[i];
            
            // Add a comma after the element, unless it's the last one in the row
            if (i < row.size() - 1) {
                outFile << ",";
            }
        }
        outFile << "\n"; // New line for the next vector
    }

    outFile.close();
    std::cout << "Successfully wrote to " << filename << std::endl;
}

#endif