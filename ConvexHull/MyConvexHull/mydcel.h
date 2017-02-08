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

class MyDcel
{
private:
    DrawableDcel *dcel;
    MainWindow *mainWindow;
    int a, b, c, d;
    double det = 0.0;    

    int returnCoplanarity(std::vector<Pointd>);
    int tetrahedronBuilder();
    std::list<Dcel::HalfEdge*> initializeTetrahedron(int);
    std::vector<Dcel::Face*> addFacesTetrahedron(std::list<Dcel::HalfEdge*>, Pointd);
    void setTwins(std::vector<Dcel::Face*>);

public:
    MyDcel(DrawableDcel *dcel, MainWindow *mainWindow);
    void buildCH();
    std::vector<Dcel::Face*> facesList;
    std::vector<Pointd> vertexArray;
};

#endif // MYDCEL_H
