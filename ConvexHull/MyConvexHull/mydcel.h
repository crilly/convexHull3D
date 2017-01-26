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
    int a, b, c, d;
    double det = 0.0;
    int returnCoplanarity(std::vector<Pointd> myVertexArray);

public:
    MyDcel(DrawableDcel *dcel, MainWindow *mainWindow);
};

#endif // MYDCEL_H
