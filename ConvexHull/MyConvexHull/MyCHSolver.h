#ifndef MYDCEL_H
#define MYDCEL_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <lib/common/point.h>
#include <lib/dcel/drawable_dcel.h>
#include <GUI/mainwindow.h>
#include <eigen3/Eigen/Dense>
#include <MyConvexHull/myconflictgraph.h>
#include <algorithm>

class MyCHSolver
{
private:
    DrawableDcel *dcel;
    MainWindow *mainWindow;
    bool showPhases;
    int a, b, c, d;
    double det = 0.0;    

    int returnCoplanarity(std::vector<Pointd>);
    int extractFourPoints();
    std::vector<Dcel::HalfEdge*> initializeTetrahedron(int);
    std::vector<Dcel::Face*> addFaces(std::vector<Dcel::HalfEdge*>, Pointd);
    void setTwins(std::vector<Dcel::Face*>);
    void randomizeVertexArray();    
    std::vector<Dcel::HalfEdge*> computeHorizon(std::set<Dcel::Face*>*);

public:
    MyCHSolver(DrawableDcel *dcel, MainWindow *mainWindow, bool const &showPhases);
    void buildCH();
    std::vector<Dcel::Face*> facesList;
    std::vector<Pointd> vertexArray;
    void deleteFacesFromDcel(std::set<Dcel::Face *> *visibleFaces);
};

#endif // MYDCEL_H
