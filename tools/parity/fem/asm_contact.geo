// geo file for meshing with Gmsh meshing software created by FreeCAD

// enable multi-core processing
General.NumThreads = 1;

// open brep geometry
Merge "Compound_Geometry.brep";

// group data
Physical Line("Edge1") = {1};
Physical Line("Edge10") = {10};
Physical Line("Edge11") = {11};
Physical Line("Edge12") = {12};
Physical Line("Edge13") = {13};
Physical Line("Edge14") = {14};
Physical Line("Edge15") = {15};
Physical Line("Edge16") = {16};
Physical Line("Edge17") = {17};
Physical Line("Edge18") = {18};
Physical Line("Edge19") = {19};
Physical Line("Edge2") = {2};
Physical Line("Edge20") = {20};
Physical Line("Edge21") = {21};
Physical Line("Edge22") = {22};
Physical Line("Edge23") = {23};
Physical Line("Edge24") = {24};
Physical Line("Edge3") = {3};
Physical Line("Edge4") = {4};
Physical Line("Edge5") = {5};
Physical Line("Edge6") = {6};
Physical Line("Edge7") = {7};
Physical Line("Edge8") = {8};
Physical Line("Edge9") = {9};
Physical Surface("Face1") = {1};
Physical Surface("Face10") = {10};
Physical Surface("Face11") = {11};
Physical Surface("Face12") = {12};
Physical Surface("Face2") = {2};
Physical Surface("Face3") = {3};
Physical Surface("Face4") = {4};
Physical Surface("Face5") = {5};
Physical Surface("Face6") = {6};
Physical Surface("Face7") = {7};
Physical Surface("Face8") = {8};
Physical Surface("Face9") = {9};
Physical Volume("Solid1") = {1};
Physical Volume("Solid2") = {2};

// Characteristic Length
// no boundary layer settings for this mesh
// min, max Characteristic Length
Mesh.CharacteristicLengthMax = 2.5;
Mesh.CharacteristicLengthMin = 0.0;
Mesh.MeshSizeFromCurvature = 12; // number of elements per 2*pi radians, 0 to deactivate

// optimize the mesh
Mesh.Optimize = 1;
Mesh.OptimizeNetgen = 0;
// High-order meshes optimization (0=none, 1=optimization, 2=elastic+optimization, 3=elastic, 4=fast curving)
Mesh.HighOrderOptimize = 0;

// mesh order
Mesh.ElementOrder = 2;
Mesh.SecondOrderLinear = 0; // Second order nodes are created by linear interpolation instead by curvilinear

// mesh algorithm, only a few algorithms are usable with 3D boundary layer generation
// 2D mesh algorithm (1=MeshAdapt, 2=Automatic, 5=Delaunay, 6=Frontal, 7=BAMG, 8=DelQuad, 9=Packing Parallelograms, 11=Quasi-structured Quad)
Mesh.Algorithm = 2;
// 3D mesh algorithm (1=Delaunay, 2=New Delaunay, 4=Frontal, 7=MMG3D, 9=R-tree, 10=HTX)
Mesh.Algorithm3D = 1;

// subdivision algorithm
Mesh.SubdivisionAlgorithm = 0;

// incomplete second order elements
Mesh.SecondOrderIncomplete = 0;

// meshing
Mesh  3;

// save
// For each group save not only the elements but the nodes too.;
Mesh.SaveGroupsOfNodes = 1;
// Needed for Group meshing too, because for one material there is no group defined;
// Ignore Physical definitions and save all elements;
Mesh.SaveAll = 1;
Save "Compound_Mesh.unv";


// **********************************************************************
// Gmsh documentation:
// https://gmsh.info/doc/texinfo/gmsh.html#Mesh
//
// We do not check if something went wrong, like negative jacobians etc. You can run Gmsh manually yourself: 
//
// to see full Gmsh log, run in bash:
// /Applications/FreeCAD.app/Contents/Resources/bin/gmsh - /private/tmp/claude-501/-Users-fernandosuarez-Projects/c094560d-f3fd-415e-9d62-62c56483cde0/scratchpad/femref/asm_contact/gmsh/shape2mesh.geo
//
// to run Gmsh and keep file in Gmsh GUI (with log), run in bash:
// /Applications/FreeCAD.app/Contents/Resources/bin/gmsh /private/tmp/claude-501/-Users-fernandosuarez-Projects/c094560d-f3fd-415e-9d62-62c56483cde0/scratchpad/femref/asm_contact/gmsh/shape2mesh.geo
