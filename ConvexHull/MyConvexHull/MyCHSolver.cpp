#include "MyCHSolver.h"

/**
 * @brief MyCHSolver::MyCHSolver costruttore della classe principale
 * @param dcel consiste della dcel dei punti del modello 3D in input
 * @param mainwindow
 */
MyCHSolver::MyCHSolver(DrawableDcel *dcel, MainWindow *mainWindow, bool const &showPhases)
{
    this->dcel = dcel;
    this->mainWindow = mainWindow;
    this->showPhases = showPhases;
}


/**
 * @brief MyCHSolver::buildCH è il mio main method che si occupa di costruire la ConvexHull in diversi step:
 *      1. estrae 4 punti random dalla Dcel del modello in input (controllando che non siano complanari)
 *      2. inizializza la Dcel con i primi 3 punti estratti
 *      3. aggiunge il quarto punto costruendo il primo tetraedro (settando rispettivi half-edges, twind, faces ecc.
 *      4. a partire dal tetraedro, inizializzo il mio conflict graph per tenere traccia delle facce e vertici in conflitto
 *      5. elimino i 4 vertici del tetraedro dal mio vettore di vertici (Pointd) e chiamo la funzione shuffle per disordinare i punti
 *      6. scorrendo tutti i punti rimanenti, controllo per ognuno quale sia il suo orizzonte e aggiorno la Dcel (quindi la CH)
 */
void MyCHSolver::buildCH()
{
    //estraggo i primi 4 punti
    int coplanarity = extractFourPoints();

    //creo la prima faccia del tetraedro e rendo una lista dei primi 3 half-edge
    std::vector<Dcel::HalfEdge*> list = initializeTetrahedron(coplanarity);

    //aggiungo le altre 3 facce per chiudere il tetraedro
    addFaces(list, vertexArray[3]);

    randomizeVertexArray();

    if(showPhases){
        dcel->update();
        this->mainWindow->updateGlCanvas();
    }

    //inizializzo il conflict graph
    MyConflictGraph conflictGraph(dcel, vertexArray);
    //inizializzo il conflict graph
    conflictGraph.initializeCG();

    std::vector<Dcel::HalfEdge*> horizon;

    //per ciascun vertice rimasto del modello in input, calcolo il corrispettivo orizzonte e computo la nuova CH
    for(auto pointIter = vertexArray.begin(); pointIter != vertexArray.end(); pointIter++)
    {
        Pointd vertex = (*pointIter);
        //recupero il set di facce visibili dal vertice tramite un metodo d'appoggio
        std::set<Dcel::Face*> *visibleFaces = conflictGraph.getFacesInConflict(vertex);

        //computo l'orizzonte solo se il set di facce visibili ha almeno un elemento, altrimenti il punto è interno al CH
        if(!visibleFaces->empty())
        {
            horizon = computeHorizon(visibleFaces);

            std::map<Dcel::HalfEdge*, std::set<Pointd>*> mapOfVerticesForCG = conflictGraph.lookForVerticesInConflict(horizon);

            for(auto faceVisibleIter = visibleFaces->begin(); faceVisibleIter != visibleFaces->end(); faceVisibleIter++)
            {
                conflictGraph.deleteFacesFromCG(*faceVisibleIter);
                deleteFacesFromDcel(*faceVisibleIter);
            }

            //creo le nuove facce a partire dagli half-edge dell'orizzonte e il nuovo vertice in esame
            std::vector<Dcel::Face*> newFaces = addFaces(horizon, vertex);

            //per tutti gli half-edge dell'orizzonte, e per ogni faccia appena creata, aggiorno entrambi i CG con le nuove facce
            for(auto horizonIter = horizon.begin(); horizonIter != horizon.end(); horizonIter++)
            {
                for(auto faceIter = newFaces.begin(); faceIter != newFaces.end(); faceIter++)
                {
                    conflictGraph.updateBothCG((*faceIter), mapOfVerticesForCG[*horizonIter]);
                }
            }

            if(showPhases){
                dcel->update();
                this->mainWindow->updateGlCanvas();
            }
        }
        conflictGraph.deleteVertexFromCG(vertex);
    }
}


/**
 * @brief MyCHSolver::extractFourPoints estrae 4 punti random dalla dcel come vertici inziali del tetraedro
 * @return torna 1 o -1 a seconda del segno della matrice (ovvero indica in quale semispazio si trova il quarto punto rispetto
 *          al semipiano formato dai primi 3 punti)
 */
int MyCHSolver::extractFourPoints()
{
    srand(time(NULL));
    int coplanarity, size = 0;
    std::vector<Pointd> fourPoints(4);

    //popolo un vettore di Pointd con le coordinate dei vertici della DCEL in input
    for(auto vertexIterator = dcel->vertexBegin(); vertexIterator != dcel->vertexEnd(); ++vertexIterator)
    {
        Dcel::Vertex *v = *vertexIterator;
        vertexArray.push_back(v->getCoordinate());
    }

    size = vertexArray.size();

    //estraggo random 4 punti dividendo in 4 parti il vettore dei vertici (estraggo un punto in ognuno dei 4 range)
    //finché i punti non sono complanari
    do
    {
        //estraggo 4 interi random che saranno l'indice del vettore che contiene i miei vertici
        a = rand() % size/4;
        b = size/4 + rand()% (size/2 - size/4);
        c = size/2 + rand()% (size/4*3 - size/2);
        d = size/4*3 + rand()% (size - size/4*3 + 1);

        //inserisco nel mio vettore provvisorio, nelle prime 4 posizioni, i miei 4 punti estratti
        fourPoints[0] = vertexArray[a];
        fourPoints[1] = vertexArray[b];
        fourPoints[2] = vertexArray[c];
        fourPoints[3] = vertexArray[d];

        //checko la complanarità dei 4 punti
        coplanarity = returnCoplanarity(fourPoints);

    } while(coplanarity == 0);

    //posiziono i 4 punti estratti nelle prime 4 posizioni del mio array (sarà più comodo scorrere tutti gli altri punti in seguito)
    std::swap(vertexArray[0], vertexArray[a]);
    std::swap(vertexArray[1], vertexArray[b]);
    std::swap(vertexArray[2], vertexArray[c]);
    std::swap(vertexArray[3], vertexArray[d]);

    return coplanarity;
}


/**
 * @brief MyCHSolver::initializeTetrahedron questo metodo mi permette di costruire in maniera corretta la prima faccia del tetraedro
 *          con annessi 3 vertici e 3 half-edges grazie al segno del determinante calcolato fra i primi 4 vertici. Infatti, se
 *          il determinante è < 0 allora dobbiamo settare le proprietà (next, fromVertex ecc) in un modo, altrimenti in verso opposto.
 * @param coplanarity mi serve per tenere il segno del determinante (< 0 oppure > 0)
 * @return torna una lista di half-edges che mi sarà utile per settare in maniera corretta e più semplice i twin degli half-edges
 */
std::vector<Dcel::HalfEdge*> MyCHSolver::initializeTetrahedron(int coplanarity)
{
    //svuoto la Dcel
    dcel->reset();

    //inizializzo la mia Dcel con i primi 3 vertici estratti
    Dcel::Vertex *v1 = dcel->addVertex(vertexArray[0]);
    Dcel::Vertex *v2 = dcel->addVertex(vertexArray[1]);
    Dcel::Vertex *v3 = dcel->addVertex(vertexArray[2]);

    //aggiungo la prima faccia
    Dcel::Face *f = dcel->addFace();

    //aggiungo i primi 3 half-edges
    Dcel::HalfEdge *hf1 = dcel->addHalfEdge();
    Dcel::HalfEdge *hf2 = dcel->addHalfEdge();
    Dcel::HalfEdge *hf3 = dcel->addHalfEdge();

    //se il determinante è > 0 allora per preservare il senso antiorario, dobbiamo settare le proprietà nel seguente modo
    if(coplanarity == 1)
    {
        hf1->setFromVertex(v1);
        hf1->setToVertex(v2);
        hf1->setNext(hf2);
        hf1->setPrev(hf3);

        hf2->setFromVertex(v2);
        hf2->setToVertex(v3);
        hf2->setNext(hf3);
        hf2->setPrev(hf1);

        hf3->setFromVertex(v3);
        hf3->setToVertex(v1);
        hf3->setNext(hf1);
        hf3->setPrev(hf2);

        v1->setIncidentHalfEdge(hf1);
        v2->setIncidentHalfEdge(hf2);
        v3->setIncidentHalfEdge(hf3);

    } else if(coplanarity == -1) //altrimenti in senso opposto
    {
        hf1->setFromVertex(v3);
        hf1->setToVertex(v2);
        hf1->setNext(hf2);
        hf1->setPrev(hf3);

        hf2->setFromVertex(v2);
        hf2->setToVertex(v1);
        hf2->setNext(hf3);
        hf2->setPrev(hf1);

        hf3->setFromVertex(v1);
        hf3->setToVertex(v3);
        hf3->setNext(hf1);
        hf3->setPrev(hf2);

        v1->setIncidentHalfEdge(hf3);
        v2->setIncidentHalfEdge(hf2);
        v3->setIncidentHalfEdge(hf1);
    }

    //setto la cardinalità di ciascun vertice (corrisponde al numero di half-edges connessi ad esso)
    v1->setCardinality(2);
    v2->setCardinality(2);
    v3->setCardinality(2);

    //setto il vertice di partenza per la prima faccia
    f->setOuterHalfEdge(hf1);

    //setto per ogni half-edge la propria faccia di appartenenza
    hf1->setFace(f);
    hf2->setFace(f);
    hf3->setFace(f);

    //inserisco i miei primi 3 half-edges in una lista di half-edge (mi sarà più comodo più avanti per i twin)
    std::vector<Dcel::HalfEdge*> halfEdgeList;
    halfEdgeList.push_back(hf1);
    halfEdgeList.push_back(hf2);
    halfEdgeList.push_back(hf3);
    return halfEdgeList;
}


/**
 * @brief MyCHSolver::returnCoplanarity metodo che, presi 4 punti, mi rende 0 se i punti sono complanari, 1 o -1 se i punti non sono
 *          complanari e il segno mi dice in quale semispazio (sinistro o destro) sta il quarto punto rispetto al semipiano formato
 *          dagli altri 3 assieme
 * @param myVertexArray vettore di Pointd che memorizza tutti i vertici del modello in input (le prime 4 posizioni sono occupate proprio
 *          dai 4 punti di interesse)
 * @return 0, 1 o -1
 */
int MyCHSolver::returnCoplanarity(std::vector<Pointd> myVertexArray)
{
    Eigen::Matrix4d mat;

    //popolo la matrice con le coordinate dei primi 4 punti
    for(int i=0; i<4; i++)
    {
        mat(i,0) = myVertexArray[i].x();
        mat(i,1) = myVertexArray[i].y();
        mat(i,2) = myVertexArray[i].z();
        mat(i,3) = 1;
    }

    //calcolo il determinante
    det = mat.determinant();

    //se il determinante è compreso in questo intervallo, i punti sono complanari e resetto il valore del determinante per comodità
    bool sign = det > -std::numeric_limits<double>::epsilon() && det < std::numeric_limits<double>::epsilon();

    if(!sign){
        if(det < -std::numeric_limits<double>::epsilon()){
            return -1;
        } else {
            return  1;
        }
    } else {
        return 0;
    }
}


/**
 * @brief MyCHSolver::addFacesTetrahedron questo metodo permette di settare correttamente le rimanenti 3 facce del tetraedro
 *          con annessi half-edges e vertici
 * @param list mi servo della lista dei primi 3 half-edge creati e inseriti in un vettore (poichè andrò a eseguire un ciclo basandomi
 *          sul numero di half-edges iniziale e non posso basarmi sul numero di half-edge nella Dcel perchè quello aumenta ad ogni iterazione)
 * @param newVertex è il quarto vertice da aggiungere per chiudere l'inizializzazione del tetraedro, corrisponde al quarto punto estratto
 * @return torno un vettore di facce contenente le nuove tre facce appena create (verrà utilizzato per settare correttamente i twin)
 */
std::vector<Dcel::Face*> MyCHSolver::addFaces(std::vector<Dcel::HalfEdge*> list, Pointd newVertex)
{
    //inizializzo una lista di facce che mi servirà per settare correttamente i twin del tetraedro
    std::vector<Dcel::Face*> faceList;
    //aggiungo il mio quarto punto alla Dcel
    Dcel::Vertex *vertex = dcel->addVertex(newVertex);

    //ciclo per i primi 3 half-edge precedentemente inseriti
    for(auto halfEdgeIterator = list.begin(); halfEdgeIterator != list.end(); halfEdgeIterator++)
    {
        //creo una nuova faccia e i 3 half-edges corrispondenti
        Dcel::Face *currentFace = dcel->addFace();
        Dcel::HalfEdge *he1 = dcel->addHalfEdge();
        Dcel::HalfEdge *he2 = dcel->addHalfEdge();
        Dcel::HalfEdge *he3 = dcel->addHalfEdge();

        //setto tutte le proprietà necessarie per ogni half-edge
        he1->setToVertex((*halfEdgeIterator)->getFromVertex());
        he1->setFromVertex((*halfEdgeIterator)->getToVertex());
        he1->setNext(he2);
        he1->setPrev(he3);
        he1->setTwin((*halfEdgeIterator));
        (*halfEdgeIterator)->setTwin(he1);

        he2->setFromVertex(he1->getToVertex());
        he2->setToVertex(vertex);
        he2->setNext(he3);
        he2->setPrev(he1);

        he3->setFromVertex(vertex);
        he3->setToVertex(he1->getFromVertex());
        he3->setNext(he1);
        he3->setPrev(he2);

        (*halfEdgeIterator)->getToVertex()->setIncidentHalfEdge(he1);
        (*halfEdgeIterator)->getFromVertex()->setIncidentHalfEdge(he2);
        vertex->setIncidentHalfEdge(he3);

        he1->getToVertex()->incrementCardinality();
        he1->getFromVertex()->incrementCardinality();
        he2->getToVertex()->incrementCardinality();
        he2->getFromVertex()->incrementCardinality();
        he3->getToVertex()->incrementCardinality();
        he3->getFromVertex()->incrementCardinality();

        he1->setFace(currentFace);
        he2->setFace(currentFace);
        he3->setFace(currentFace);

        currentFace->setOuterHalfEdge(he1);

        //inserisco la nuova faccia nel vettore di facce
        faceList.push_back(currentFace);
    }

    setTwins(faceList);
    return faceList;
}


/**
 * @brief MyCHSolver::setTwins metodo che mi permette di settare i twin per ogni half-edge presente nel tetraedro (sono in tutto 12)
 * @param listFace vettore che consta delle ultime 3 facce aggiunte alla Dcel
 */
void MyCHSolver::setTwins(std::vector<Dcel::Face*> listFace)
{
    //ciclo iterato sulla dimensione della listFace (sono 3 facce)
    for(auto faceIterator = listFace.begin(); faceIterator != listFace.end(); faceIterator++)
    {
        //prendo la faccia successiva a quella in esame ora
        auto next = std::next(faceIterator, 1);
        Dcel::HalfEdge *nextFaceOuterHE;
        //recupero l'outerHalfEdge della faccia corrente
        Dcel::HalfEdge *faceOuterHE = (*faceIterator)->getOuterHalfEdge();

        //se sono arrivata all'ultima faccia, allora recupero l'outerHalfEdge della faccia iniziale
        if(next == listFace.end())
        {
            nextFaceOuterHE = listFace.front()->getOuterHalfEdge();

        } else //altrimenti recupero l'outerHalfEdge della faccia successiva
        {
            nextFaceOuterHE = (*next)->getOuterHalfEdge();
        }

        //setto come twin l'outerHalfEdge della faccia attuale con quello della prossima faccia
        nextFaceOuterHE->getNext()->setTwin(faceOuterHE->getPrev());
        faceOuterHE->getPrev()->setTwin(nextFaceOuterHE->getNext());
    }
}


/**
 * @brief MyCHSolver::randomizeVertexArray breve metodo che mi permette di eliminare dal mio vettore di vertici i vertici
 *          facenti parte del tetraedro e di randomizzare i restanti vertici per andare a costruire la vera e propria CH
 */
void MyCHSolver::randomizeVertexArray()
{
    //elimino i primi 4 vertici dell'array, che sono quelli del tetraedro
    vertexArray.erase(vertexArray.begin(), vertexArray.begin() + 4);

    //randomizzo i punti restanti del vettore per andare a costruire il CH
    std::random_shuffle(vertexArray.begin(), vertexArray.end());
}


/**
 * @brief MyCHSolver::computeHorizon metodo che computa l'orizzonte per un punto
 * @param setOfFaces insieme delle facce visibili dal punto in esame
 * @return torna il vettore di half-edge che compongono l'orizzonte del punto
 */

std::vector<Dcel::HalfEdge*> MyCHSolver::computeHorizon(std::set<Dcel::Face*>* setOfFaces)
{
    std::vector<Dcel::HalfEdge*> horizon;
    Dcel::HalfEdge* firstHE= new Dcel::HalfEdge;

    //per tutte le facce nel conflict graph del vertice corrente, controllo quali fanno parte dell'orizzonte
    for(auto faceIterator = setOfFaces->begin(); faceIterator != setOfFaces->end(); faceIterator++)
    {
        //scorro gli half-edge della faccia corrente. Se uno di loro ha il twin appartenente a una faccia non visibile dal punto
        //allora quell'half-edge fa parte dell'orizzonte
        for(auto halfEdgeIterator = (*faceIterator)->incidentHalfEdgeBegin(); halfEdgeIterator != (*faceIterator)->incidentHalfEdgeEnd(); halfEdgeIterator++)
        {
            if((*halfEdgeIterator)->getTwin() != nullptr)
            {
                //se la faccia del twin del mio half-edge corrente non è nella lista delle facce visibili dal vertice
                //allora inserisco l'half-edge corrente nell'orizzonte (che è ancora disordinato)
                auto element = setOfFaces->find((*halfEdgeIterator)->getTwin()->getFace());
                if(element == setOfFaces->end())
                {
                    firstHE = (*halfEdgeIterator)->getTwin();
                    break;
                }
            }
        }
        //controllo se effettivamente è inizializzato
        if(firstHE->getFromVertex() != nullptr) break;
    }

    //inserisco il l'half-edge di partenza nel vettore che conterrà l'horizon
    horizon.push_back(firstHE);
    Dcel::HalfEdge* next= new Dcel::HalfEdge;
    if(firstHE->getNext()->getTwin() != nullptr){
        //inizializzo un half-edge che mi servirà per scorrere il boundary dell'orizzonte
        next = firstHE->getNext()->getTwin();
    }
    //finchè non sono arrivata all'half-edge iniziale (quindi non ho chiuso il giro)
    while(next->getNext()!= firstHE)  //&& next->getNext()->getTwin()->getNext() != firstHE)
    {
        //se la faccia dell'half-edge corrente fa parte dell'insieme delle facce visibili, allora quell'HE fa parte dell'orizzonte
        if(setOfFaces->find(next->getFace()) != setOfFaces->end())
        {
            //inserisco l'HE attuale nell'orizzonte
            horizon.push_back(next->getTwin());
            //l'HE attuale diventa il twin di quello appena pushato nell'orizzonte
            next = next->getTwin();
        } else { //se la faccia non è tra quelle visibili
            //l'HE attuale diventa il twin del next di quello precedente
            next = next->getNext()->getTwin();
        }
    }


    return horizon;
}



/**
 * @brief MyCHSolver::deleteFacesFromDcel
 * @param visibleFaces
 */
void MyCHSolver::deleteFacesFromDcel(Dcel::Face* visibleFace)
{
    for(auto heIter = visibleFace->incidentHalfEdgeBegin(); heIter != visibleFace->incidentHalfEdgeEnd(); heIter++)
    {
        Dcel::Vertex *fromVertex = (*heIter)->getFromVertex();
        Dcel::Vertex *toVertex = (*heIter)->getToVertex();

        dcel->deleteHalfEdge(*heIter);

        fromVertex->decrementCardinality();
        toVertex->decrementCardinality();

        if(fromVertex->getCardinality() == 0)
        {
            dcel->deleteVertex(fromVertex);
        }
        if(toVertex->getCardinality() == 0)
        {
            dcel->deleteVertex(toVertex);
        }

    }
    dcel->deleteFace(visibleFace);

}
