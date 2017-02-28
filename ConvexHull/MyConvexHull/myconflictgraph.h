#ifndef MYCONFLICTGRAPH_H
#define MYCONFLICTGRAPH_H

#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Dense>
#include <lib/common/point.h>
#include <lib/dcel/drawable_dcel.h>

class MyConflictGraph
{
private:
    void createMatrixForFace(int, Eigen::Matrix4d &);
    void addFaceToVConflict(Pointd &, Dcel::Face *);
    void addVertexToFConflict(Dcel::Face *, Pointd &);

public:
    MyConflictGraph(DrawableDcel*, std::vector<Pointd>);
    ~MyConflictGraph();
    DrawableDcel *dcel;
    std::vector<Pointd> vertexArray;    
    std::map<Dcel::Face*, std::set<Pointd>*> conflictFaces;
    std::map<Pointd, std::set<Dcel::Face*>*> conflictVertices;

    void initializeCG();    
    std::set<Dcel::Face*>* getFacesInConflict(Pointd &);
    std::set<Pointd> *getVerticesInConflict(Dcel::Face *);
    std::map<Dcel::HalfEdge*, std::set<Pointd>*> lookForVerticesInConflict(std::vector<Dcel::HalfEdge*>);
    void deleteFacesFromCG(Dcel::Face *);
    void deleteVertexFromCG(Pointd &);
    bool isVisible(Dcel::Face *, Pointd) const;
    void updateBothCG(Dcel::Face *, std::set<Pointd> *);
};

#endif // MYCONFLICTGRAPH_H
