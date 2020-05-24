#ifndef LINK_H
#define LINK_H


class Link
{
public:
    Link();
    void setParentLink(Link* pLink){parentLink = pLink;}
    void setChildLink1(Link* cLink){childLink1 = cLink;}
    void setChildLink2(Link* cLink){childLink2 = cLink;}
    Link* parentLink = nullptr;
    Link* childLink1 = nullptr;
    Link* childLink2 = nullptr;
    char axis;

    void setAxis(char componentAxis){axis = componentAxis;}
};

#endif // LINK_H



#ifndef KINEMATICCHAIN_H
#define KINEMATICCHAIN_H

#include <QVector>


class KinematicChain
{
public:
    KinematicChain(QVector<char> components);
    int n_link;

private:
    Link baseLink;
};

#endif // KINEMATICCHAIN_H


