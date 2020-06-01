#include "joint.h"

Joint::Joint(){}
Joint::Joint(std::string name , std::string type, Link *parent_link,  Link *child_link)
    :m_name(name), m_type(type), m_parent_link(parent_link) ,m_child_link(child_link){}

Joint::Joint(std::string name,std::string type, Vector3 origin_xyz,Vector3 origin_rpy, Vector3 axis,
             Link *parent_link,  Link *child_link)
    :m_name(name), m_type(type), m_origin_xyz(origin_xyz), m_origin_rpy(origin_rpy),
      m_axis(axis), m_parent_link(parent_link) ,m_child_link(child_link){}
