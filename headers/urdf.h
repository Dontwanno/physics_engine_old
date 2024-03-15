//
// Created by twang on 04/07/2023.
//

#ifndef PHYSICS_ENGINE_URDF_H
#define PHYSICS_ENGINE_URDF_H

#include <PxPhysicsAPI.h>
#include <urdf/model.h>

struct jointData {
    std::string name;
    int joint_type;
    std::string parent_link_name;
    std::string child_link_name;

    physx::PxQuat orientation;
    physx::PxVec3 position;
    physx::PxVec3 axis;
    physx::PxVec4T<double> rotation;
    physx::PxTransform transform;

    float lower_limit;
    float upper_limit;
};

struct linkData {
    std::string name;
    std::string collision_mesh_filename;
    std::string visual_mesh_filename;

    double mass;
    physx::PxVec3 axis;
    physx::PxMat33 inertia;
    physx::PxVec3 scale;
    physx::PxVec3 position;
    physx::PxQuat orientation;

    physx::PxConvexMeshGeometry convex_geom;
};


void
parse_urdf(const string &filename, std::vector<linkData> &links, std::vector<jointData> &joints, std::shared_ptr<urdf::UrdfModel>& modelpointer);

#endif //PHYSICS_ENGINE_URDF_H
