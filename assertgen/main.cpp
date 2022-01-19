#include <iostream>
#include <fstream>

#include "../box2d/lib.cpp"

int main() {
    ofstream myfile;
    myfile.open("../asserts.jai");
    myfile << "Writing this to a file.\n";
    myfile.close();

    return 0;
}