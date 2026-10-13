// Run OndselSolver's static solve (what FreeCAD's AssemblyObject::solve calls) on an
// ASMT file and print the parts' placements.
//
// Build against the OndselSolver sources (FreeCAD: src/3rdParty/OndselSolver), see run.sh.
// run.sh links it with a copy of Part.cpp whose fillqsuWeights takes aJi/aJiMax from the
// AJ_RATIO environment variable (FreeCAD 1.1.3's OndselSolver 1.0.1 behaves like 1/3).
#include "ASMTAssembly.h"
#include "ASMTPart.h"
#include <iomanip>
#include <iostream>
using namespace MbD;
int main(int argc, char** argv) {
    auto a = ASMTAssembly::assemblyFromFile(argv[1]);
    a->runPreDrag();
    std::cout << std::setprecision(17);
    for (auto& p : *a->parts) {
        std::cout << "PART " << p->name << " pos";
        for (int i = 0; i < 3; i++) std::cout << " " << p->position3D->at(i);
        std::cout << " rot";
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++) std::cout << " " << p->rotationMatrix->at(i)->at(j);
        std::cout << "\n";
    }
}
