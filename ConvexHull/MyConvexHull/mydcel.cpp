#include "mydcel.h"

MyDcel::MyDcel(DrawableDcel *dcel, MainWindow *mainwindow)
{
    srand(time(NULL));
    std::vector<Pointd> vertexArray;
    auto endDcel = dcel->vertexEnd();
    int coplanarity, arraySize = 0;

    //popolo un vettore con le coordinate dei vertici della DCEL in input
    for(auto beginDcel = dcel->vertexBegin(); beginDcel != endDcel; ++beginDcel)
    {
        Dcel::Vertex *v = *beginDcel;
        vertexArray.push_back(v->getCoordinate());
    }

    arraySize = vertexArray.size();

    do
    {
        //estraggo 4 vertici random dividendo il numero totale di vertici in 4 parti
        a = rand() % arraySize/4;
        b = arraySize/4 + rand()% (arraySize/2 - arraySize/4);
        c = arraySize/2 + rand()% (arraySize/4*3 - arraySize/2);
        d = arraySize/4*3 + rand()% (arraySize - arraySize/4*3 + 1);
        printf("%d\t%d\t%d\t%d", a, b, c, d);

        coplanarity = returnCoplanarity(vertexArray);

    }while(coplanarity == 0);

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

    //svuoto la Dcel
    dcel->clear();

    //inizializzo la mia Dcel con i primi 3 vertici estratti
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
    if(coplanarity == 1){
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

int MyDcel::returnCoplanarity(std::vector<Pointd> myVertexArray)
{
    Eigen::Matrix4d mat;
    mat(0,0) = myVertexArray[0].x();
    mat(0,1) = myVertexArray[0].y();
    mat(0,2) = myVertexArray[0].z();
    mat(0,3) = 1;
    mat(1,0) = myVertexArray[b].x();
    mat(1,1) = myVertexArray[b].y();
    mat(1,2) = myVertexArray[b].z();
    mat(1,3) = 1;
    mat(2,0) = myVertexArray[c].x();
    mat(2,1) = myVertexArray[c].y();
    mat(2,2) = myVertexArray[c].z();
    mat(2,3) = 1;
    mat(3,0) = myVertexArray[d].x();
    mat(3,1) = myVertexArray[d].y();
    mat(3,2) = myVertexArray[d].z();
    mat(3,3) = 1;

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
