#include <ctype.h>
#include "PxPhysicsAPI.h"
#include "foundation/PxTime.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "mesh_creator.hpp"
#include <fstream>


using namespace physx;


static float rand(float loVal, float hiVal)
{
    return loVal + (float(rand())/float(RAND_MAX))*(hiVal - loVal);
}

PxU64 getCurrentTimeCounterValue()
{
    return PxTime::getCurrentCounterValue();
}

PxReal getElapsedTimeInMilliseconds(const PxU64 elapsedTime)
{
    return PxTime::getCounterFrequency().toTensOfNanos(elapsedTime)/(100.0f * 1000.0f);
}

void SaveToFile(const std::string& filename, const PxDefaultMemoryOutputStream& outputStream) {
    std::ofstream outFile("../resources/generated/" + filename, std::ios::binary);
    if (!outFile.is_open()) {
        // Handle file opening error
    }

    constexpr std::size_t HeaderSize = sizeof(std::uint64_t);  // Size of the file size header in bytes

    const void* meshData = outputStream.getData();
    unsigned int meshSize = outputStream.getSize();
    printf("Mesh size: %d\n", meshSize);
    std::uint64_t sizeToWrite = static_cast<std::uint64_t>(meshSize);
    printf("Size to write: %d\n", sizeToWrite);
    printf(reinterpret_cast<const char*>(&sizeToWrite));
    printf("\n");
    outFile.write(reinterpret_cast<const char*>(&sizeToWrite), HeaderSize);;
    outFile.write(static_cast<const char *>(meshData), meshSize);
    outFile.close();
}


template<PxConvexMeshCookingType::Enum convexMeshCookingType, bool directInsertion, PxU32 gaussMapLimit>
void createConvex(PxU32 numVerts, const PxVec3 *verts, PxPhysics *gPhysics, const std::string& filename) {
    PxTolerancesScale tolerances;
    PxCookingParams params(tolerances);

    // Use the new (default) PxConvexMeshCookingType::eQUICKHULL
    params.convexMeshCookingType = convexMeshCookingType;

    // If the gaussMapLimit is chosen higher than the number of output vertices, no gauss map is added to the convex mesh data (here 256).
    // If the gaussMapLimit is chosen lower than the number of output vertices, a gauss map is added to the convex mesh data (here 16).
    params.gaussMapLimit = gaussMapLimit;

    // Setup the convex mesh descriptor
    PxConvexMeshDesc desc;

    // We provide points only, therefore the PxConvexFlag::eCOMPUTE_CONVEX flag must be specified
    desc.points.data = verts;
    desc.points.count = numVerts;
    desc.points.stride = sizeof(PxVec3);
    desc.flags = PxConvexFlag::eCOMPUTE_CONVEX;
    desc.quantizedCount = 65535;

    PxU32 meshSize = 0;
    PxConvexMesh* convex = NULL;

    PxU64 startTime = getCurrentTimeCounterValue();

    if(directInsertion)
    {
        // Directly insert mesh into PhysX
        convex = PxCreateConvexMesh(params, desc, gPhysics->getPhysicsInsertionCallback());
        PX_ASSERT(convex);
    }
    else
    {
        // Serialize the cooked mesh into a stream.
        PxDefaultMemoryOutputStream outStream;
        bool res = PxCookConvexMesh(params, desc, outStream);
        PX_UNUSED(res);
        PX_ASSERT(res);
        meshSize = outStream.getSize();

        SaveToFile(filename + ".mesh", outStream);

        // Create the mesh from a stream.
        PxDefaultMemoryInputData inStream(outStream.getData(), outStream.getSize());
        convex = gPhysics->createConvexMesh(inStream);
        PX_ASSERT(convex);
    }

    // Print the elapsed time for comparison
    PxU64 stopTime = getCurrentTimeCounterValue();
    float elapsedTime = getElapsedTimeInMilliseconds(stopTime - startTime);
    printf("\t -----------------------------------------------\n");
    printf("\t Create convex mesh with %d triangles: \n", numVerts);
    directInsertion ? printf("\t\t Direct mesh insertion enabled\n") : printf("\t\t Direct mesh insertion disabled\n");
    printf("\t\t Gauss map limit: %d \n", gaussMapLimit);
    printf("\t\t Created hull number of vertices: %d \n", convex->getNbVertices());
    printf("\t\t Created hull number of polygons: %d \n", convex->getNbPolygons());
    printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
    if (!directInsertion)
    {
        printf("\t Mesh size: %d \n", meshSize);
    }

    convex->release();
}


void createConvexMesh(const std::string &filename, PxPhysics *gPhysics, const std::string& meshName) {
    //get vertices from file with assimp
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    if (!scene)
    {
        printf("Error loading file: %s\n", importer.GetErrorString());
        ;
    }
    aiMesh* mesh = scene->mMeshes[0];
    PxU32 numVerts = mesh->mNumVertices;
    PxVec3* vertices = new PxVec3[numVerts];
    for (PxU32 i = 0; i < numVerts; i++)
    {
        vertices[i] = PxVec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
    }



    // Create convex mesh using the quickhull algorithm with different settings
    printf("-----------------------------------------------\n");
    printf("Create convex mesh using the quickhull algorithm: \n\n");

    // The default convex mesh creation serializing to a stream, useful for offline cooking.
//    createConvex<PxConvexMeshCookingType::eQUICKHULL, false, 16>(numVerts, vertices, gPhysics, meshName);

    // The default convex mesh creation without the additional gauss map data.
    createConvex<PxConvexMeshCookingType::eQUICKHULL, false, 256>(numVerts, vertices, gPhysics, meshName);

    // Convex mesh creation inserting the mesh directly into PhysX.
    // Useful for runtime cooking.
//    PxConvexMeshGeometry mesh3 = createConvex<PxConvexMeshCookingType::eQUICKHULL, true, 16>(numVerts, vertices, gPhysics);

    // Convex mesh creation inserting the mesh directly into PhysX, without gauss map data.
    // Useful for runtime cooking.
//    PxConvexMeshGeometry mesh1 = createConvex<PxConvexMeshCookingType::eQUICKHULL, true, 256>(numVerts, vertices, gPhysics);

    delete [] vertices;
}

// Setup common cooking params
void setupCommonCookingParams(PxCookingParams &params, bool skipMeshCleanup, bool skipEdgeData) {
    // we suppress the triangle mesh remap table computation to gain some speed, as we will not need it
    // in this snippet
    params.suppressTriangleMeshRemapTable = true;

    // If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking. The input mesh must be valid.
    // The following conditions are true for a valid triangle mesh :
    //  1. There are no duplicate vertices(within specified vertexWeldTolerance.See PxCookingParams::meshWeldTolerance)
    //  2. There are no large triangles(within specified PxTolerancesScale.)
    // It is recommended to run a separate validation check in debug/checked builds, see below.

    if (!skipMeshCleanup)
        params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH);
    else
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_CLEAN_MESH;

    // If eDISABLE_ACTIVE_EDGES_PRECOMPUTE is set, the cooking does not compute the active (convex) edges, and instead
    // marks all edges as active. This makes cooking faster but can slow down contact generation. This flag may change
    // the collision behavior, as all edges of the triangle mesh will now be considered active.
    if (!skipEdgeData)
        params.meshPreprocessParams &= ~static_cast<PxMeshPreprocessingFlags>(PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE);
    else
        params.meshPreprocessParams |= PxMeshPreprocessingFlag::eDISABLE_ACTIVE_EDGES_PRECOMPUTE;
}

// Creates a triangle mesh using BVH34 midphase with different settings.
void createBV34TriangleMesh(PxU32 numVertices, const PxVec3 *vertices, PxU32 numTriangles, const PxU32 *indices,
                       bool skipMeshCleanup, bool skipEdgeData, bool inserted, const PxU32 numTrisPerLeaf,
                       PxPhysics *gPhysics, PxCooking *gCooking) {
    PxU64 startTime = getCurrentTimeCounterValue();

    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count = numVertices;
    meshDesc.points.data = vertices;
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.triangles.count = numTriangles;
    meshDesc.triangles.data = indices;
    meshDesc.triangles.stride = 3 * sizeof(PxU32);

    PxCookingParams params = gCooking->getParams();

    // Create BVH34 midphase
    params.midphaseDesc = PxMeshMidPhase::eBVH34;

    // setup common cooking params
    setupCommonCookingParams(params, skipMeshCleanup, skipEdgeData);

    // Cooking mesh with less triangles per leaf produces larger meshes with better runtime performance
    // and worse cooking performance. Cooking time is better when more triangles per leaf are used.
    params.midphaseDesc.mBVH34Desc.numPrimsPerLeaf = numTrisPerLeaf;

    gCooking->setParams(params);

#if defined(PX_CHECKED) || defined(PX_DEBUG)
    // If DISABLE_CLEAN_MESH is set, the mesh is not cleaned during the cooking.
    // We should check the validity of provided triangles in debug/checked builds though.
    if (skipMeshCleanup)
    {
        PX_ASSERT(gCooking->validateTriangleMesh(meshDesc));
    }
#endif // DEBUG


    PxTriangleMesh* triMesh = NULL;
    PxU32 meshSize = 0;

    // The cooked mesh may either be saved to a stream for later loading, or inserted directly into PxPhysics.
    if (inserted)
    {
        triMesh = gCooking->createTriangleMesh(meshDesc, gPhysics->getPhysicsInsertionCallback());
    }
    else
    {
        PxDefaultMemoryOutputStream outBuffer;
        gCooking->cookTriangleMesh(meshDesc, outBuffer);

        SaveToFile("link0trimesh.mesh", outBuffer);

        PxDefaultMemoryInputData stream(outBuffer.getData(), outBuffer.getSize());
        triMesh = gPhysics->createTriangleMesh(stream);

        meshSize = outBuffer.getSize();
    }

    // Print the elapsed time for comparison
    PxU64 stopTime = getCurrentTimeCounterValue();
    float elapsedTime = getElapsedTimeInMilliseconds(stopTime - startTime);
    printf("\t -----------------------------------------------\n");
    printf("\t Create triangle mesh with %d triangles: \n", numTriangles);
    inserted ? printf("\t\t Mesh inserted on\n") : printf("\t\t Mesh inserted off\n");
    !skipEdgeData ? printf("\t\t Precompute edge data on\n") : printf("\t\t Precompute edge data off\n");
    !skipMeshCleanup ? printf("\t\t Mesh cleanup on\n") : printf("\t\t Mesh cleanup off\n");
    printf("\t\t Num triangles per leaf: %d \n", numTrisPerLeaf);
    printf("\t Elapsed time in ms: %f \n", double(elapsedTime));
    if (!inserted)
    {
        printf("\t Mesh size: %d \n", meshSize);
    }

    triMesh->release();
}



void createTriangleMesh(const std::string &filename, PxPhysics *gPhysics, PxCooking *gCooking) {
    //get vertices from file with assimp
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename, aiProcess_Triangulate);
    if (!scene)
    {
        printf("Error loading file: %s\n", importer.GetErrorString());
        ;
    }
    aiMesh* mesh = scene->mMeshes[0];
    PxU32 numVertices = mesh->mNumVertices;
    PxVec3* vertices = new PxVec3[numVertices];
    PxU32 numIndices = mesh->mNumFaces * 3;
    PxU32* indices = new PxU32[numIndices];
    PxU32 numTriangles = mesh->mNumFaces;

    for (PxU32 i = 0; i < numVertices; i++)
    {
        vertices[i] = PxVec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
    }

    // Iterate over each triangle
    for (unsigned int i = 0; i < numTriangles; i++) {
        aiFace& face = mesh->mFaces[i];

        // Assume each face is a triangle
        if (face.mNumIndices == 3) {
            unsigned int index1 = face.mIndices[0];
            unsigned int index2 = face.mIndices[1];
            unsigned int index3 = face.mIndices[2];

            // Add indices
            indices[i] = index1;
            indices[i + 1] = index2;
            indices[i + 2] = index3;
        }
    }


    // Create triangle mesh using BVH34 midphase with different settings
    printf("-----------------------------------------------\n");
    printf("Create triangles mesh using BVH34 midphase: \n\n");

    // Favor runtime speed, cleaning the mesh and precomputing active edges. Store the mesh in a stream.
    // These are the default settings, suitable for offline cooking.
//    (void)createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, 4, gPhysics, gCooking);

    // Favor mesh size, cleaning the mesh and precomputing active edges. Store the mesh in a stream.
    createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, false, false, false, 15, gPhysics, gCooking);

    // Favor cooking speed, skip mesh cleanup, but precompute active edges. Insert into PxPhysics.
    // These settings are suitable for runtime cooking, although selecting more triangles per leaf may reduce
    // runtime performance of simulation and queries. We still need to ensure the triangles
    // are valid, so we perform a validation check in debug/checked builds.
//    createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, true, false, true, 15);

    // Favor cooking speed, skip mesh cleanup, and don't precompute the active edges. Insert into PxPhysics.
    // This is the fastest possible solution for runtime cooking, but all edges are marked as active, which can
    // further reduce runtime performance, and also affect behavior.
//    createBV34TriangleMesh(numVertices, vertices, numTriangles, indices, false, true, true, 15);

    delete [] vertices;
    delete [] indices;
}
