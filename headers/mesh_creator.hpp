//
// Created by twang on 19/06/2023.
//

#ifndef PHYSICS_ENGINE_MESH_CREATOR_HPP
#define PHYSICS_ENGINE_MESH_CREATOR_HPP

#include "PxPhysicsAPI.h"

using namespace physx;

static float rand(float loVal, float hiVal);
PxU64 getCurrentTimeCounterValue();
PxReal getElapsedTimeInMilliseconds(const PxU64 elapsedTime);

template<PxConvexMeshCookingType::Enum convexMeshCookingType, bool directInsertion, PxU32 gaussMapLimit>
void createConvex(PxU32 numVerts, const PxVec3* verts, PxPhysics* gPhysics, const std::string &filename);
void createConvexMesh(const std::string& filename, PxPhysics* gPhysics, const std::string &meshName);

void setupCommonCookingParams(PxCookingParams& params, bool skipMeshCleanup, bool skipEdgeData);

void createBV34TriangleMesh(PxU32 numVertices, const PxVec3* vertices, PxU32 numTriangles, const PxU32* indices,
                                              bool skipMeshCleanup, bool skipEdgeData, bool inserted, const PxU32 numTrisPerLeaf, PxPhysics *gPhysics, PxCooking *gCooking);

void createTriangleMesh(const std::string& filename, PxPhysics *gPhysics, PxCooking *gCooking);

#endif //PHYSICS_ENGINE_MESH_CREATOR_HPP
