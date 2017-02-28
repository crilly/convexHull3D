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
    bool updateModel;
    int a, b, c, d;
    MyConflictGraph *conflictGraph;

    int returnCoplanarity(const std::vector<Pointd>) const;
    int extractFourPoints();
    std::vector<Dcel::HalfEdge*> initializeTetrahedron(const int);
    std::vector<Dcel::Face*> addFaces(std::vector<Dcel::HalfEdge*>, const Pointd);
    void setTwins(const std::vector<Dcel::Face*>) const;
    void randomizeVertexArray();    
    std::vector<Dcel::HalfEdge*> computeHorizon(const std::set<Dcel::Face*>*);
    void updateCanvas();

public:
    MyCHSolver(DrawableDcel *, MainWindow *, bool const &);
    ~MyCHSolver();
    void buildCH();
    std::vector<Dcel::Face*> facesList;
    std::vector<Pointd> vertexArray;
    void deleteFacesFromDcel(Dcel::Face *);
};

#endif // MYDCEL_H
