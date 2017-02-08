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
 * della matrice corrispondenti alle coordinate dei 3 vertici della faccia passata con l'ID
 * @param faceID ID della faccia dalla quale prendere i vertici
 * @param matrix riferimento alla matrice che vogliamo popolare (serve come indice del vettore di matrici "matrices")
 */
void MyConflictGraph::createMatrixForFace(int faceID, Eigen::Matrix4d &matrix)
{
    Dcel::HalfEdge *currentHE = dcel->getFace(faceID)->getOuterHalfEdge();
    for(int i = 0; i < 3; i++)
    {
        matrix(i, 0) = currentHE->getFromVertex()->getCoordinate().x();
        matrix(i, 1) = currentHE->getFromVertex()->getCoordinate().y();
        matrix(i, 2) = currentHE->getFromVertex()->getCoordinate().z();
        matrix(i, 3) = 1;
        currentHE = currentHE->getNext();
    }
    //setto l'ultima riga a zero per leggibilità
    matrix(3, 0) = 0;
    matrix(3, 1) = 0;
    matrix(3, 2) = 0;
    matrix(3, 3) = 0;
}

/**
 * @brief MyConflictGraph::initializeCG
 */
void MyConflictGraph::initializeCG()
{
    std::vector<Eigen::Matrix4d> matrices(4);

    int size = vertexArray.size();
    int indexVertexArray = 4;

    createMatrixForFace(0, matrices[0]);
    createMatrixForFace(1, matrices[1]);
    createMatrixForFace(2, matrices[2]);
    createMatrixForFace(3, matrices[3]);

    //trasformo il mio vettore di vertici di tipo Pointd in vertici di tipo Dcel::Vertex per inserirli nel CG
    Dcel::Vertex *currentVertex = new Dcel::Vertex(vertexArray[indexVertexArray]);

    /* per tutte le facce della dcel, scorro tutti i vertici del modello rimanenti
    ** in particolare aggiungo alla matrice della faccia già inizializzata in ultima riga le coordinate del nuovo vertice da controllare:
    **  */
    for(auto iterator = dcel->faceBegin(); iterator != dcel->faceEnd(); iterator++)
    {
        indexVertexArray = 4;

        //scorro il mio vettore di vertici del modello (tranne i primi 4 che fanno parte del tetraedro)
        while(currentVertex->getCoordinate() != vertexArray[size-1])
        {
            matrices[(*iterator)->getId()](3, 0) = currentVertex->getCoordinate().x();
            matrices[(*iterator)->getId()](3, 1) = currentVertex->getCoordinate().y();
            matrices[(*iterator)->getId()](3, 2) = currentVertex->getCoordinate().z();
            matrices[(*iterator)->getId()](3, 3) = 1;

            /*se il determinante è minore di -epsilon macchina, allora la faccia vede il punto (così come mostrato nelle slides del corso)
            ** e dovrò andare a inserirli nel CG, altrimenti non faccio nulla*/
            if(matrices[(*iterator)->getId()].determinant() < (-std::numeric_limits<double>::epsilon()))
            {
                //metodi d'appoggio per andare a inserire facce e vertici nei rispettivi CG
                addFaceToVConflict(currentVertex, (*iterator));
                addVertexToFConflict((*iterator), currentVertex);
            }

            //passo al vertice successivo (se sono arrivata all'ultimo, uscirò dal while)
            indexVertexArray++;
            currentVertex->setCoordinate(vertexArray[indexVertexArray]);
        }
    }
}

void MyConflictGraph::addFaceToVConflict(Dcel::Vertex *v, Dcel::Face *f)
{

}

void MyConflictGraph::addVertexToFConflict(Dcel::Face *f, Dcel::Vertex *v)
{

}
