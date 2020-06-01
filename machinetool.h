#ifndef READURDF
#define READURDF

#include <QVector>
#include<QDebug>
#include "joint.h"


class MachineTool
{

public:
    MachineTool();
    int readURDF(const char* filename);
    QVector<Link> LinkVector;                 // create a vector for links
    QVector<Joint> JointVector;               // create a vector for joints
    QString m_name;
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
    Link* find_link(std::string linkName, QVector<Link> &myVector);
    void assignLinkType(Joint &joint);

};
#endif
