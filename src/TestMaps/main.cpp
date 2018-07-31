#include <QCoreApplication>

#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

void appendLineToFile(std::string filepath, std::string line)
{
    std::fstream fs;
    fs.open (filepath, std::fstream::in | std::fstream::out | std::fstream::app);
    fs << line << "\n";
    fs.close();
}

int main(int argc, char *argv[])
{

    // Write to file in loop with row index as data:
    int counter = 0;
    while(true) {
        std::string line = std::to_string(counter) + ", 1, 2, 3";
        std::cout << "Writing line #" << counter << " to file" << std::endl;
        // Write to file:
        appendLineToFile("C:/Code/MACE/MATLABtest.csv", line);

        // Increment counter:
        counter++;

        // Sleep for a second
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
