#include "mydcel.h"

MyDcel::MyDcel(DrawableDcel *dcel, MainWindow *mainwindow)
{
    srand(time(NULL));
    std::vector<Pointd> vertexArray;
    auto endDcel = dcel->vertexEnd();
    int a, b, c, d;
    double det = 0.0;

    //popolo un vettore con le coordinate dei vertici della DCEL in input
    for(auto beginDcel = dcel->vertexBegin(); beginDcel != endDcel; ++beginDcel)
    {
        Dcel::Vertex *v = *beginDcel;
        vertexArray.push_back(v->getCoordinate());
    }

    int arraySize = vertexArray.size();

    do
    {

    //estraggo 4 vertici random dividendo il numero totale di vertici in 4 parti
    a = rand() % arraySize/4;
    b = arraySize/4 + rand()% (arraySize/2 - arraySize/4);
    c = arraySize/2 + rand()% (arraySize/4*3 - arraySize/2);
    d = arraySize/4*3 + rand()% (arraySize - arraySize/4*3 + 1);
    printf("%d\t%d\t%d\t%d", a, b, c, d);

    //popolo la mia matrice di 4 righe e 4 colonne per verificare se i punti sono complanari (det = 0) o meno (det != 0)
    Eigen::Matrix4d mat;
    mat(0,0) = vertexArray[a].x();
    mat(0,1) = vertexArray[a].y();
    mat(0,2) = vertexArray[a].z();
    mat(0,3) = 1;
    mat(1,0) = vertexArray[b].x();
    mat(1,1) = vertexArray[b].y();
    mat(1,2) = vertexArray[b].z();
    mat(1,3) = 1;
    mat(2,0) = vertexArray[c].x();
    mat(2,1) = vertexArray[c].y();
    mat(2,2) = vertexArray[c].z();
    mat(2,3) = 1;
    mat(3,0) = vertexArray[d].x();
    mat(3,1) = vertexArray[d].y();
    mat(3,2) = vertexArray[d].z();
    mat(3,3) = 1;

    //calcolo il determinante
    det = mat.determinant();

    //se il determinante è compreso in questo intervallo, i punti sono complanari e resetto il valore del determinante
    if(det > - std::numeric_limits<double>::epsilon() && det < std::numeric_limits<double>::epsilon())
    {
        det = 0.0;
    }

    }while(det == 0.0);

    bool sburro = true;
}
