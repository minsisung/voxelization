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

    double getBoundingBox_X_min(){
        return bounding_x_min;
    }
    double getBoundingBox_X_max(){
        return bounding_x_max;
    }
    double getBoundingBox_Y_min(){
        return bounding_y_min;
    }
    double getBoundingBox_Y_max(){
        return bounding_y_max;
    }
    double getBoundingBox_Z_min(){
        return bounding_z_min;
    }
    double getBoundingBox_Z_max(){
        return bounding_z_max;
    }

private:
    Link* find_link(std::string linkName, QVector<Link> &myVector);
    void assignLinkType(Joint &joint);
    double bounding_x_min;
    double bounding_x_max;
    double bounding_y_min;
    double bounding_y_max;
    double bounding_z_min;
    double bounding_z_max;

};
#endif
