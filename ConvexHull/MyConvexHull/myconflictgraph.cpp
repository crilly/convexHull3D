#include "myconflictgraph.h"

/**
 * @brief MyConflictGraph::MyConflictGraph costruttore della classe
 * @param dcel la nostra dcel del modello
 * @param vertexArray array di Pointd contenente i vertici del modello
 */
MyConflictGraph::MyConflictGraph(DrawableDcel *dcel, std::vector<Pointd> vertexArray)
{
    this->dcel = dcel;
    this->vertexArray = vertexArray;
}

/**
 * @brief MyConflictGraph::createMatrixForFace questo metodo si occupa di inizializzare le prime 3 righe
 *          della matrice corrispondenti alle coordinate dei 3 vertici della faccia passata con l'ID
 * @param faceID ID della faccia dalla quale prendere i vertici
 * @param matrix riferimento alla matrice che vogliamo popolare (serve come indice del vettore di matrici "matrices")
 */
void MyConflictGraph::createMatrixForFace(int faceID, Eigen::Matrix4d &matrix)
{
    //parto dall'outerHE della faccia corrente e mi recupero le coordinate del suo fromVertex
    Dcel::HalfEdge *currentHE = dcel->getFace(faceID)->getOuterHalfEdge();
    for(int i = 0; i < 3; i++)
    {
        matrix(i, 0) = currentHE->getFromVertex()->getCoordinate().x();
        matrix(i, 1) = currentHE->getFromVertex()->getCoordinate().y();
        matrix(i, 2) = currentHE->getFromVertex()->getCoordinate().z();
        matrix(i, 3) = 1;

        //prendo il successivo half-edge rispetto a quello corrente
        currentHE = currentHE->getNext();
    }

    //setto l'ultima riga a zero per leggibilità
    matrix(3, 0) = 0;
    matrix(3, 1) = 0;
    matrix(3, 2) = 0;
    matrix(3, 3) = 0;
}

/**
 * @brief MyConflictGraph::initializeCG metodo che permette di inizializzare i conflictGraph dei vertici e delle facce
 */
void MyConflictGraph::initializeCG()
{
    std::vector<Eigen::Matrix4d> matrices(4);

    int size = vertexArray.size();
    int indexVertexArray = 0;

    createMatrixForFace(0, matrices[0]);
    createMatrixForFace(1, matrices[1]);
    createMatrixForFace(2, matrices[2]);
    createMatrixForFace(3, matrices[3]);

    /* per tutte le facce della dcel, scorro tutti i vertici del modello rimanenti
    ** in particolare aggiungo alla matrice della faccia già inizializzata in ultima riga le coordinate del nuovo vertice da controllare:
    **  */
    for(auto iterator = dcel->faceBegin(); iterator != dcel->faceEnd(); ++iterator)
    {
        indexVertexArray = 0;
        Dcel::Face *currentFace = (*iterator);

        //scorro il mio vettore di vertici del modello (tranne i primi 4 che fanno parte del tetraedro)

        for (auto iterVert = vertexArray.begin(); iterVert != vertexArray.end(); iterVert++ )
        {

            matrices[currentFace->getId()](3, 0) = (*iterVert).x();
            matrices[currentFace->getId()](3, 1) = (*iterVert).y();
            matrices[currentFace->getId()](3, 2) = (*iterVert).z();
            matrices[currentFace->getId()](3, 3) = 1;

            auto det = matrices[currentFace->getId()].determinant();

            /*se il determinante è minore di -epsilon macchina, allora la faccia vede il punto (così come mostrato nelle slides del corso)
            ** e dovrò andare a inserirli nel CG, altrimenti non faccio nulla*/
            if(det < -std::numeric_limits<double>::epsilon())
            {
                //metodi d'appoggio per andare a inserire facce e vertici nei rispettivi CG
                addFaceToVConflict((*iterVert), currentFace);
                addVertexToFConflict(currentFace, (*iterVert));
            }

        }
    }
}

/**
 * @brief MyConflictGraph::addFaceToVConflict metodo d'appoggio per inserire nel v_conflict la faccia
 * @param v vertice chiave della mappa
 * @param f faccia in conflitto col vertice da inserire nel set con chiave v
 */
void MyConflictGraph::addFaceToVConflict(Pointd &v, Dcel::Face* f)
{
    //controllo se la mia chiave "f" esiste: se si, inserisco il vertice nel set di vertici
    if(conflictVertices.find(v) == conflictVertices.end())
    {

        std::set<Dcel::Face*>* set = new std::set<Dcel::Face*>();
        set->insert(f);

        conflictVertices[v] =  set;

    } else {

        std::set<Dcel::Face*>* set = conflictVertices[v];
        set->insert(f);
    }
}

/**
 * @brief MyConflictGraph::addVertexToFConflict metodo d'appoggio per inserire nel f_conflict il vertice
 * @param f
 * @param v
 */
void MyConflictGraph::addVertexToFConflict(Dcel::Face *f, Pointd &v)
{
    //controllo se la mia chiave "f" esiste: se si, inserisco il vertice nel set di vertici
    if(conflictFaces.find(f) == conflictFaces.end())
    {
        conflictFaces[f] = new std::set<Pointd>();
    }
     //altrimenti istanzio il set di vertici, aggiungo il vertice al set e infine inserisco faccia e vertice nella mappa
      conflictFaces[f]->insert(v);
}

/**
 * @brief MyConflictGraph::getVisibleFaces
 * @param point
 * @return
 */
std::set<Dcel::Face*>* MyConflictGraph::getFacesInConflict(Pointd &vertex){

    std::set<Dcel::Face*> *facesSet;

    // se non ha trovato elementi torno un set vuoto (può servirmi più avanti)
    if(conflictVertices[vertex] == nullptr)
    {
        facesSet = new std::set<Dcel::Face*>;

        //inserisco il set di facce nella conflict list dei punti
        conflictVertices[vertex]=facesSet;
    } else {
        facesSet = conflictVertices[vertex];
    }

    return new std::set<Dcel::Face*>(*facesSet);
}


/**
 * @brief MyConflictGraph::getVerticesInConflict
 * @param point
 * @return
 */
std::set<Pointd>* MyConflictGraph::getVerticesInConflict(Dcel::Face* f){

    std::set<Pointd> *verticesSet = conflictFaces[f];

    // se non ha trovato elementi torno un set vuoto (può servirmi più avanti)
    if(verticesSet == nullptr)
    {
        verticesSet = new std::set<Pointd>;

        //inserisco il set di facce nella conflict list dei punti
        conflictFaces[f] = verticesSet;
    }

    return new std::set<Pointd>(*verticesSet);
}


/**
 * @brief MyConflictGraph::lookForVerticesInConflict
 * @param myHorizon
 * @return
 */
std::map<Dcel::HalfEdge*, std::set<Pointd>*> MyConflictGraph::lookForVerticesInConflict(std::vector<Dcel::HalfEdge*> myHorizon)
{
    std::map<Dcel::HalfEdge*, std::set<Pointd>*> mapOfVerticesForCG;

    for(auto horizonIter = myHorizon.begin(); horizonIter != myHorizon.end(); horizonIter++)
    {
        std::set<Pointd> *setPointsHorizon = getVerticesInConflict((*horizonIter)->getFace());
        std::set<Pointd> *setPointsHorizonTwin = getVerticesInConflict((*horizonIter)->getTwin()->getFace());

        setPointsHorizon->insert(setPointsHorizonTwin->begin(), setPointsHorizonTwin->end());

        mapOfVerticesForCG[*horizonIter] = setPointsHorizon;
    }

    return mapOfVerticesForCG;
}


/**
 * @brief MyConflictGraph::deleteFacesFromCG
 * @param visibleFaces
 */
void MyConflictGraph::deleteFacesFromCG(Dcel::Face *visibleFace)
{

        std::set<Pointd> *visiblePoints = conflictFaces[visibleFace];

        //infine elimino la faccia dal CG delle facce
        conflictFaces.erase(visibleFace);

//        //se trovo punti, allora li devo eliminare (ovvero se la faccia da distruggere era visibile da qualche punto)
//        if(visiblePoints != nullptr)
//        {
            //per ogni punto trovato, vado ad eliminare la faccia da distruggere dalla sua lista
            for(auto pointIter = visiblePoints->begin(); pointIter != visiblePoints->end(); pointIter++)
            {
                conflictVertices[*pointIter]->erase(visibleFace);
            }


//        }

}


/**
 * @brief MyConflictGraph::isVisible
 * @param vertex
 * @param face
 * @return
 */
bool MyConflictGraph::isVisible(Dcel::Face *face, Pointd point) const{

    Eigen::Matrix4d matrix;
    int i=0;

    for(auto heIter = face->incidentHalfEdgeBegin(); heIter != face->incidentHalfEdgeEnd(); heIter++, i++)
    {
        matrix(i,0) = (*heIter)->getFromVertex()->getCoordinate().x();
        matrix(i,1) = (*heIter)->getFromVertex()->getCoordinate().y();
        matrix(i,2) = (*heIter)->getFromVertex()->getCoordinate().z();
        matrix(i,3) = 1;
    }

    matrix(3,0) = point.x();
    matrix(3,1) = point.y();
    matrix(3,2) = point.z();
    matrix(3,3) = 1;

    if(matrix.determinant() < -std::numeric_limits<double>::epsilon()) return true;
    else return false;
}


/**
 * @brief MyConflictGraph::updateBothCG
 * @param newFace
 * @param verticesSet
 */
void MyConflictGraph::updateBothCG(Dcel::Face *newFace, std::set<Pointd> *verticesSet)
{
    for(auto pointIter = verticesSet->begin(); pointIter != verticesSet->end(); pointIter++)
    {
        Pointd point = (*pointIter);
        if(isVisible(newFace, point))
        {
            addFaceToVConflict(point, newFace);
            addVertexToFConflict(newFace, point);
        }
    }
}


/**
 * @brief MyConflictGraph::deleteVertexFromCG
 * @param point
 */
void MyConflictGraph::deleteVertexFromCG(Pointd &point)
{
    std::set<Dcel::Face*> *facesInConflict = getFacesInConflict(point);

    conflictVertices.erase(point);

    for(auto faceIter = facesInConflict->begin(); faceIter != facesInConflict->end(); faceIter++)
    {
        conflictFaces[*faceIter]->erase(point);
    }

}
