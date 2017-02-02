#ifndef MYDCEL_H
#define MYDCEL_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <lib/common/point.h>
#include <lib/dcel/drawable_dcel.h>
#include <GUI/mainwindow.h>
#include <eigen3/Eigen/Dense>

class MyDcel
{
private:
    DrawableDcel *dcel;
    MainWindow *mainWindow;
    int a, b, c, d;
    double det = 0.0;
    std::vector<Pointd> vertexArray;

    int returnCoplanarity(std::vector<Pointd>);
    int tetrahedronBuilder();
    std::list<Dcel::HalfEdge*> initializeDcel(int);
    std::list<Dcel::Face*> addFacesTetrahedron(std::list<Dcel::HalfEdge*>, Pointd);
    void setTwins(std::list<Dcel::Face*>);

public:
    MyDcel(DrawableDcel *dcel, MainWindow *mainWindow);
    void buildCH();
};

#endif // MYDCEL_H
