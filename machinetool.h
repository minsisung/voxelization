#ifndef READURDF
#define READURDF

#include <QVector>
#include<QDebug>
#include "joint.h"


class MachineTool
{

public:
    MachineTool();
    int readURDF(QString);
    Link* find_link(std::string linkName, QVector<Link> &myVector);
    void assignLinkType(Joint &joint);

    QVector<Link> LinkVector;                 // create a vector for links
    QVector<Joint> JointVector;               // create a vector for joints
    QString m_name;
    Link* baseLink;
    Link* xLink;
    Link* yLink;
    Link* zLink;
    Link* firstRotaryLink;
    Link* secondRotaryLink;
    QHash<QString, stl_reader::StlMesh <float, unsigned int>> componentsHash;
    QHash<QString, QMatrix4x4> tranMatrixHash;

    friend QDebug operator<<(QDebug stream, const MachineTool& MT);   //overloading operator <<

    static inline double strToDouble(const char *in)
    {
        std::stringstream ss;
        ss.imbue(std::locale::classic());

        ss << in;

        double out;
        ss >> out;

        if (ss.fail() || !ss.eof()) {
            throw std::runtime_error("Failed converting string to double");
        }

        return out;
    }

private:
    bool hasAssignFirstRotaryLink = false;

};
#endif
