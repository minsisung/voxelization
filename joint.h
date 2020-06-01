#ifndef JOINT_H
#define JOINT_H

#include <link.h>

class Joint
{
private:
    std::string m_name;
    std::string m_type;
    Vector3 m_origin_xyz;
    Vector3 m_origin_rpy;
    Vector3 m_axis;
    Link *m_parent_link;
    Link *m_child_link;
    float m_lower_limit;
    float m_upper_limit;

public:
    Joint();            //constructor
    Joint(std::string name , std::string type,Link *parent_link,  Link *child_link);
    Joint(std::string name,std::string type, Vector3 origin_xyz,Vector3 origin_rpy, Vector3 axis,
          Link *parent_link,  Link *child_link);
    ~Joint(){}

    Vector3 getOrigin_xyz() {return m_origin_xyz;}
    Vector3 getOrigin_rpy() {return m_origin_rpy;}
    Vector3 getAxis() {return m_axis;}
    std::string getName() {return m_name;}
    std::string getType() {return m_type;}
    Link* getParentLink() {return m_parent_link;}
    Link* getChildLink() {return m_child_link;}
    float translational_motion = 0.0;
    float rotational_motion = 0.0;
};

#endif // JOINT_H
