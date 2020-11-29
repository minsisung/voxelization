#ifndef URDFEXPORTER_H
#define URDFEXPORTER_H
#include <QString>
#include <QDir>
#include <QXmlStreamWriter>
#include "machinetool.h"
#include "stl_reader.h"

class UrdfExporter
{
public:
    UrdfExporter(MachineTool& MT);
    void start();

private:
    void exportSTL();
    void STLFacetOut(QTextStream &out, stl_reader::StlMesh <float, unsigned int>& mesh, Vector3& offsetVector);
    void exportXML();
    void writeLink(QXmlStreamWriter& xmlWriter, Link& link);
    void writeJoint(QXmlStreamWriter& xmlWriter, Joint& joint, Joint* prev_joint);
    Joint *findJoint(QString parentLinkName, QString childLinkName, QVector<Joint>& JointVector);

    MachineTool m_MT;
    QString m_MTName;
};


#endif // URDFEXPORTER_H
