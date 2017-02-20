#ifndef MYCONFLICTGRAPH_H
#define MYCONFLICTGRAPH_H

#include <stdio.h>
#include <stdlib.h>
#include <eigen3/Eigen/Dense>
#include <lib/common/point.h>
#include <lib/dcel/drawable_dcel.h>

class MyConflictGraph
{
public:
    DrawableDcel *dcel;
    std::vector<Pointd> vertexArray;
    MyConflictGraph(DrawableDcel*, std::vector<Pointd>);
    std::map<Dcel::Face*, std::set<Pointd>*> conflictFaces;
    std::map<Pointd, std::set<Dcel::Face*>*> conflictVertices;
    void initializeCG();    
    std::set<Dcel::Face*>* getFacesInConflict(Pointd &);
    std::set<Pointd> *getVerticesInConflict(Dcel::Face *f);
    std::map<Dcel::HalfEdge *, std::set<Pointd> *> lookForVerticesInConflict(std::vector<Dcel::HalfEdge *> myHorizon);
    void deleteFacesFromCG(std::set<Dcel::Face *> *visibleFaces);
    void deleteVertexFromCG(Pointd &point);
    bool isVisible(Dcel::Face *face, Pointd point) const;
    void updateBothCG(Dcel::Face *newFace, std::set<Pointd> *verticesSet);

private:
    void createMatrixForFace(int, Eigen::Matrix4d &);
    void addFaceToVConflict(Pointd &, Dcel::Face *);
    void addVertexToFConflict(Dcel::Face *, Pointd &);
};

#endif // MYCONFLICTGRAPH_H
