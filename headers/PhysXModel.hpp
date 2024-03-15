//
// Created by twang on 23/06/2023.
//
#ifndef PHYSICS_ENGINE_PHYSXMODEL_HPP
#define PHYSICS_ENGINE_PHYSXMODEL_HPP

#include "glad/glad.h" // holds all OpenGL type declarations

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "learnopengl/shader.h"

#include <string>
#include <PxPhysicsAPI.h>
#include <vector>

struct PxGeomType
{
    enum Enum
    {
        eSPHERE,
        ePLANE,
        eCAPSULE,
        eBOX,
        eCONVEXMESH,
        ePARTICLESYSTEM,
        eTETRAHEDRONMESH,
        eTRIANGLEMESH,
        eHEIGHTFIELD,
        eHAIRSYSTEM,
        eCUSTOM,

        eGEOMETRY_COUNT,	//!< internal use only!
        eINVALID = -1		//!< internal use only!
    };
};

struct PhysXVertex {
    // position
    glm::vec3 Position;
    // normal
    glm::vec3 Normal;
    // texCoords
};

struct PhysXTexture {
    unsigned int id;
    std::string type;
    std::string path;
};

class PhysXModel {
public:
// mesh Data
    std::vector<PhysXVertex>       vertices;
    std::vector<unsigned int>       indices;
    std::vector<PhysXTexture>      textures;
    unsigned int VAO;

    // constructor
    PhysXModel(physx::PxGeometry& geom)
    {
        loadModel(geom);
        setupMesh();
    }

    void loadAfter(physx::PxGeometry& geom)
    {
        loadModel(geom);
        setupMesh();
    }

    // empty constructor
    PhysXModel()
    {
        VAO = 0;
        VBO = 0;
        EBO = 0;
    }


    // render the mesh
    void Draw(Shader& shader)
    {
        // draw mesh
        glBindVertexArray(VAO);
//        glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()), GL_UNSIGNED_INT, 0);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<unsigned int>(vertices.size()));
        glBindVertexArray(0);
    }

private:
    // render data
    unsigned int VBO, EBO;

    void loadModel (const physx::PxGeometry &geom) {
        using namespace physx;
        switch (geom.getType()) {
            case PxGeomType::eCONVEXMESH:
            {
                const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom);

                //Compute triangles for each polygon.
                const PxVec3& scale = convexGeom.scale.scale;
                PxConvexMesh* mesh = convexGeom.convexMesh;
                const PxU32 nbPolys = mesh->getNbPolygons();
                const PxU8* polygons = mesh->getIndexBuffer();
                const PxVec3* verts = mesh->getVertices();
                PxU32 nbVerts = mesh->getNbVertices();
                PX_UNUSED(nbVerts);

                std::vector<PhysXVertex> vertices2;

                for (int i = 0; i < nbVerts; i++) {
                    PxVec3 vert = verts[i];
                    vertices2.push_back({glm::vec3(vert.x, vert.y, vert.z), glm::vec3(0.0f, 0.0f, 0.0f)});
                }

                PxU32 numTotalTriangles = 0;
                for(PxU32 i = 0; i < nbPolys; i++)
                {
                    PxHullPolygon data;
                    mesh->getPolygonData(i, data);

                    const PxU32 nbTris = PxU32(data.mNbVerts - 2);
                    const PxU8 vref0 = polygons[data.mIndexBase + 0];
                    PX_ASSERT(vref0 < nbVerts);
                    for(PxU32 j=0;j<nbTris;j++)
                    {
                        const PxU32 vref1 = polygons[data.mIndexBase + 0 + j + 1];
                        const PxU32 vref2 = polygons[data.mIndexBase + 0 + j + 2];

                        //generate face normal:
                        PxVec3 e0 = verts[vref1] - verts[vref0];
                        PxVec3 e1 = verts[vref2] - verts[vref0];

                        PX_ASSERT(vref1 < nbVerts);
                        PX_ASSERT(vref2 < nbVerts);

                        PxVec3 fnormal = e0.cross(e1);
                        fnormal.normalize();
                        glm::vec3 normal = glm::vec3(fnormal.x, fnormal.y, fnormal.z);


                        vertices2[vref0].Normal = normal;
                        vertices2[vref1].Normal = normal;
                        vertices2[vref2].Normal = normal;
//
                        indices.push_back(vref0);
                        indices.push_back(vref1);
                        indices.push_back(vref2);

                        vertices.push_back(PhysXVertex({glm::vec3(verts[vref0].x, verts[vref0].y, verts[vref0].z), normal}));
                        vertices.push_back(PhysXVertex({glm::vec3(verts[vref1].x, verts[vref1].y, verts[vref1].z), normal}));
                        vertices.push_back(PhysXVertex({glm::vec3(verts[vref2].x, verts[vref2].y, verts[vref2].z), normal}));

                    }
                }
//                std::vector<PhysXVertex> vertices3;
//                for (auto index: indices) {vertices3.push_back(vertices2[index]);}
//                std::cout << "vertices size: " << vertices.size() << std::endl;
//                std::cout << "vertices3 size: " << vertices3.size() << std::endl;
//                // check if vertices and vertices3 are equal
//                for (int i = 0; i < vertices.size(); i++) {
//                    if (vertices[i].Position != vertices3[i].Position) {
//                        std::cout << "vertices not equal" << std::endl;
//                        std::cout << vertices[i].Position.x << " " << vertices[i].Position.y << " " << vertices[i].Position.z << std::endl;
//                        std::cout << vertices3[i].Position.x << " " << vertices3[i].Position.y << " " << vertices3[i].Position.z << std::endl;
//                    }
//                    if (vertices[i].Normal != vertices3[i].Normal) {
//                        std::cout << "normals not equal" << std::endl;
//                        std::cout << vertices[i].Normal.x << " " << vertices[i].Normal.y << " " << vertices[i].Normal.z << std::endl;
//                        std::cout << vertices3[i].Normal.x << " " << vertices3[i].Normal.y << " " << vertices3[i].Normal.z << std::endl;
//                    }
//                    std::cout << "i is: " << i << std::endl;
//                }
            }
                break;

            case PxGeomType::eTRIANGLEMESH: {
                const auto &triGeom = static_cast<const PxTriangleMeshGeometry &>(geom);

                const PxTriangleMesh &mesh = *triGeom.triangleMesh;
                const PxVec3 scale = triGeom.scale.scale;

                const PxU32 triangleCount = mesh.getNbTriangles();
                const PxU32 has16BitIndices = mesh.getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;
                const void *indexBuffer = mesh.getTriangles();

                const PxVec3 *localVertices = mesh.getVertices();
                const PxU32 verticesCount = mesh.getNbVertices();


                const PxU32 *intIndices = reinterpret_cast<const PxU32 *>(indexBuffer);
                const PxU16 *shortIndices = reinterpret_cast<const PxU16 *>(indexBuffer);
                PxU32 numTotalTriangles = 0;

                for (int i; i < verticesCount; i++) {
                    PxVec3 v = localVertices[i];
                    PhysXVertex vertex = {
                            glm::vec3(v.x, v.y, v.z),
                            glm::vec3(0.0f, 0.0f, 0.0f)
                    };
                    vertices.push_back(vertex);
                }

                for (PxU32 i = 0; i < triangleCount; ++i) {
                    PxU32 vref0, vref1, vref2;
                    if (has16BitIndices) {
                        vref0 = *shortIndices++;
                        vref1 = *shortIndices++;
                        vref2 = *shortIndices++;
                    } else {
                        vref0 = *intIndices++;
                        vref1 = *intIndices++;
                        vref2 = *intIndices++;
                    }


                    const PxVec3 &v0 = localVertices[vref0];
                    const PxVec3 &v1 = localVertices[vref1];
                    const PxVec3 &v2 = localVertices[vref2];

                    PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
                    (void)fnormal.normalize();
                    glm::vec3 normal = glm::vec3(fnormal.x, fnormal.y, fnormal.z);

                    vertices[vref0].Normal = normal;
                    vertices[vref1].Normal = normal;
                    vertices[vref2].Normal = normal;


                    indices.push_back(vref0);
                    indices.push_back(vref1);
                    indices.push_back(vref2);

                    numTotalTriangles++;
                }


                break;

            }
        }
    }

    // initializes all the buffer objects/arrays
    void setupMesh()
    {
        // create buffers/arrays
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
//        glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);
        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // A great thing about structs is that their memory layout is sequential for all its items.
        // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
        // again translates to 3/2 floats which translates to a byte array.
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(PhysXVertex), &vertices[0], GL_STATIC_DRAW);

//        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(PhysXVertex), (void*)0);
        // vertex normals
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(PhysXVertex), (void*)offsetof(PhysXVertex, Normal));


        glBindVertexArray(0);
    }

};

#endif //PHYSICS_ENGINE_PHYSXMODEL_HPP
