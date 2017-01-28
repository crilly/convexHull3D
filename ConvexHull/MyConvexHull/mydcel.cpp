#include "mydcel.h"

/**
 * @brief MyDcel::MyDcel costruttore della classe principale
 * @param dcel consiste della dcel dei punti del modello 3D in input
 * @param mainwindow
 */
MyDcel::MyDcel(DrawableDcel *dcel, MainWindow *mainWindow)
{
    this->dcel = dcel;
    this->mainWindow = mainWindow;
}


/**
 * @brief MyDcel::tetrahedronBuilder costruisce il primo tetraedro estraendo 4 punti random dalla dcel
 */
void MyDcel::tetrahedronBuilder()
{
    srand(time(NULL));
    auto endDcel = dcel->vertexEnd();
    int coplanarity, size = 0;

    //popolo un vettore con le coordinate dei vertici della DCEL in input
    for(auto beginDcel = dcel->vertexBegin(); beginDcel != endDcel; ++beginDcel)
    {
        Dcel::Vertex *v = *beginDcel;
        vertexArray.push_back(v->getCoordinate());
    }

    size = vertexArray.size();

    do
    {
        //estraggo 4 vertici random dividendo il numero totale di vertici in 4 parti
        a = rand() % size/4;
        b = size/4 + rand()% (size/2 - size/4);
        c = size/2 + rand()% (size/4*3 - size/2);
        d = size/4*3 + rand()% (size - size/4*3 + 1);
        printf("%d\t%d\t%d\t%d", a, b, c, d);

        coplanarity = returnCoplanarity(vertexArray);

    } while(coplanarity == 0);

    //posiziono i 4 punti estratti nelle prime 4 posizioni del mio array (sarà più comodo scorrere tutti gli altri punti in seguito)
    Pointd temp = vertexArray[0];
    vertexArray[0] = vertexArray[a];
    vertexArray[a] = temp;

    temp = vertexArray[1];
    vertexArray[1] = vertexArray[b];
    vertexArray[b] = temp;

    temp = vertexArray[2];
    vertexArray[2] = vertexArray[c];
    vertexArray[c] = temp;

    temp = vertexArray[3];
    vertexArray[3] = vertexArray[d];
    vertexArray[d] = temp;
}

/**
 * @brief MyDcel::initializeDcel
 */
void MyDcel::initializeDcel()
{
    //svuoto la Dcel
    dcel->clear();

    //inizializzo la mia Dcel con i primi 4 vertici estratti
    Dcel::Vertex *v1 = dcel->addVertex(vertexArray[a]);
    Dcel::Vertex *v2 = dcel->addVertex(vertexArray[b]);
    Dcel::Vertex *v3 = dcel->addVertex(vertexArray[c]);
    Dcel::Vertex *v4 = dcel->addVertex(vertexArray[d]);

    //aggiungo la prima faccia
    Dcel::Face *f = dcel->addFace();

    //aggiungo gli half-edges
    Dcel::HalfEdge *hf1 = dcel->addHalfEdge();
    Dcel::HalfEdge *hf2 = dcel->addHalfEdge();
    Dcel::HalfEdge *hf3 = dcel->addHalfEdge();

    //se il determinante è > 0 allora dobbiamo muoverci in senso antiorario
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

    } else if(coplanarity == -1) //altrimenti ci muoviamo in senso orario
    {
        hf1->setFromVertex(v2);
        hf1->setToVertex(v1);
        hf1->setNext(hf2);
        hf1->setPrev(hf1);

        hf2->setFromVertex(v1);
        hf2->setToVertex(v3);
        hf2->setNext(hf3);
        hf2->setPrev(hf1);

        hf3->setFromVertex(v3);
        hf3->setToVertex(v2);
        hf3->setNext(hf1);
        hf3->setPrev(hf2);

        v1->setIncidentHalfEdge(hf2);
        v2->setIncidentHalfEdge(hf1);
        v3->setIncidentHalfEdge(hf3);
    }

    //setto la cardinalità di ciascun vertice
    v1->setCardinality(2);
    v2->setCardinality(2);
    v3->setCardinality(2);

    //setto il vertice di partenza per la prima faccia
    f->setOuterHalfEdge(hf1);

    //setto per ogni half-edge la propria faccia di appartenenza
    hf1->setFace(f);
    hf2->setFace(f);
    hf3->setFace(f);


    bool miao = true;
}

/**
 * @brief MyDcel::returnCoplanarity
 * @param myVertexArray
 * @return
 */
int MyDcel::returnCoplanarity(std::vector<Pointd> myVertexArray)
{
    Eigen::Matrix4d mat;

    for(int i=0; i<4; i++)
    {
        mat(i,0) = myVertexArray[i].x();
        mat(i,1) = myVertexArray[i].y();
        mat(i,2) = myVertexArray[i].z();
        mat(i,3) = 1;
    }

    //calcolo il determinante
    det = mat.determinant();

    //se il determinante è compreso in questo intervallo, i punti sono complanari e resetto il valore del determinante
    if(det > - std::numeric_limits<double>::epsilon() && det < std::numeric_limits<double>::epsilon())
    {
        det = 0.0;
    }

    if(det > 0.0) return 1;
    else if(det == 0.0) return 0;
    else return -1;
}

/**
 * @brief MyDcel::addFourthPoint
 */
void MyDcel::addFourthPoint()
{

}


/**
 * @brief MyDcel::buildCH metodo che si occupa di costruire la ConvexHull in diversi step:
 * 1. estrae 4 punti random dalla Dcel del modello in input (controllando che non siano complanari)
 * 2. inizializza la Dcel con i primi 3 punti estratti
 * 3. aggiunge il quarto punto costruendo il primo tetraedro
 * TODO //finish description
 */
void MyDcel::buildCH()
{
    tetrahedronBuilder();
    initializeDcel();
    addFourthPoint();
}
