//
// Created by twang on 03/07/2023.
//
#include <urdf/model.h>
#include <tinyxml/txml.h>
#include <string>
#include <iostream>
#include <fstream>
#include "urdf.h"
#include "PxPhysicsAPI.h"


using namespace urdf;

std::string readURDFFileToString(const std::string& filepath)
{
    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();

    return buffer.str();
}

//physx::PxQuat ToQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
//{
//    // Abbreviations for the various angular functions
//    double cr = cos(roll * 0.5);
//    double sr = sin(roll * 0.5);
//    double cp = cos(pitch * 0.5);
//    double sp = sin(pitch * 0.5);
//    double cy = cos(yaw * 0.5);
//    double sy = sin(yaw * 0.5);
//
//    physx::PxQuat q;
//    q.w = cr * cp * cy + sr * sp * sy;
//    q.x = sr * cp * cy - cr * sp * sy;
//    q.y = cr * sp * cy + sr * cp * sy;
//    q.z = cr * cp * sy - sr * sp * cy;
//
//    return q;
//}



void
parse_urdf(const string &filename, std::vector<linkData> &links, std::vector<jointData> &joints, std::shared_ptr<UrdfModel>& modelpointer) {

    //load urdf string from file
    std::string urdf_str = readURDFFileToString(filename);

    std::shared_ptr<UrdfModel> model;
    model = UrdfModel::fromUrdfStr(urdf_str);

    modelpointer = model;

    std::cout << "base link name: " << model->getRoot()->name << std::endl;

    for (const auto& link : model->link_map)
    {
        linkData px_link;
        auto link_obj = link.second;

        px_link.name = link.first;

        if (!link_obj->visuals.empty()) {
            auto visual_mesh = (std::shared_ptr<Mesh> &) link_obj->visuals[0]->geometry;
            px_link.visual_mesh_filename = visual_mesh->filename;
        }
//        if (!link_obj->collisions.empty()) {
//            auto collision_mesh = (std::shared_ptr<Mesh> &) link_obj->collisions[0]->geometry;
//            px_link.collision_mesh_filename = collision_mesh->filename;
//        }

        px_link.mass = link_obj->inertial->mass;
        physx::PxVec3 col0 = physx::PxVec3(link_obj->inertial->ixx, link_obj->inertial->ixy, link_obj->inertial->ixz);
        physx::PxVec3 col1 = physx::PxVec3(link_obj->inertial->ixy, link_obj->inertial->iyy, link_obj->inertial->iyz);
        physx::PxVec3 col2 = physx::PxVec3(link_obj->inertial->ixz, link_obj->inertial->iyz, link_obj->inertial->izz);

        px_link.inertia = physx::PxMat33(col0, col1, col2);
        px_link.position = physx::PxVec3(link_obj->inertial->origin.position.x, link_obj->inertial->origin.position.y, link_obj->inertial->origin.position.z);
        px_link.orientation = physx::PxQuat(link_obj->inertial->origin.rotation.x,
                                            link_obj->inertial->origin.rotation.y,
                                            link_obj->inertial->origin.rotation.z,
                                            link_obj->inertial->origin.rotation.w);

        links.push_back(px_link);
    }

    for (const auto& joint : model->joint_map) {
        jointData px_joint;
        auto joint_obj = joint.second;
        px_joint.name = joint.first;
        px_joint.joint_type = joint_obj->type;
        px_joint.parent_link_name = joint_obj->parent_link_name;
        px_joint.child_link_name = joint_obj->child_link_name;
        px_joint.position = physx::PxVec3(joint_obj->parent_to_joint_transform.position.x,
                                          joint_obj->parent_to_joint_transform.position.y,
                                          joint_obj->parent_to_joint_transform.position.z);
        px_joint.orientation = physx::PxQuat(joint_obj->parent_to_joint_transform.rotation.x,
                                             joint_obj->parent_to_joint_transform.rotation.y,
                                             joint_obj->parent_to_joint_transform.rotation.z,
                                             joint_obj->parent_to_joint_transform.rotation.w);
        px_joint.axis = physx::PxVec3(joint_obj->axis.x, joint_obj->axis.y, joint_obj->axis.z);
        px_joint.rotation = physx::PxVec4T(joint_obj->parent_to_joint_transform.rotation.x,
                                          joint_obj->parent_to_joint_transform.rotation.y,
                                          joint_obj->parent_to_joint_transform.rotation.z,
                                          joint_obj->parent_to_joint_transform.rotation.w);
        if (joint_obj->limits != nullopt) {
            px_joint.lower_limit = joint_obj->limits->get()->lower;
            px_joint.upper_limit = joint_obj->limits->get()->upper;
        }
        px_joint.transform = physx::PxTransform(px_joint.position, px_joint.orientation);
        joints.push_back(px_joint);
    }
}
