//
// Created by twang on 18/06/2023.
//

#include "render_funcs.hpp"
#include "foundation/PxPreprocessor.h"
#include "cuda.h"
#include "cudaGL.h"
#include "foundation/PxArray.h"
#include "foundation/PxMathUtils.h"
#include <vector>
#include <optional>
#include "sphere.hpp"
#include "cube.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "learnopengl/shader.h"
#include "PhysXModel.hpp"


#define MAX_NUM_ACTOR_SHAPES	128


using namespace physx;

//static GLFontRenderer	gTexter;

static float gCylinderData[]={
        1.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,0.0f,0.0f,
        0.866025f,0.500000f,1.0f,0.866025f,0.500000f,1.0f,0.866025f,0.500000f,0.0f,0.866025f,0.500000f,0.0f,
        0.500000f,0.866025f,1.0f,0.500000f,0.866025f,1.0f,0.500000f,0.866025f,0.0f,0.500000f,0.866025f,0.0f,
        -0.0f,1.0f,1.0f,-0.0f,1.0f,1.0f,-0.0f,1.0f,0.0f,-0.0f,1.0f,0.0f,
        -0.500000f,0.866025f,1.0f,-0.500000f,0.866025f,1.0f,-0.500000f,0.866025f,0.0f,-0.500000f,0.866025f,0.0f,
        -0.866025f,0.500000f,1.0f,-0.866025f,0.500000f,1.0f,-0.866025f,0.500000f,0.0f,-0.866025f,0.500000f,0.0f,
        -1.0f,-0.0f,1.0f,-1.0f,-0.0f,1.0f,-1.0f,-0.0f,0.0f,-1.0f,-0.0f,0.0f,
        -0.866025f,-0.500000f,1.0f,-0.866025f,-0.500000f,1.0f,-0.866025f,-0.500000f,0.0f,-0.866025f,-0.500000f,0.0f,
        -0.500000f,-0.866025f,1.0f,-0.500000f,-0.866025f,1.0f,-0.500000f,-0.866025f,0.0f,-0.500000f,-0.866025f,0.0f,
        0.0f,-1.0f,1.0f,0.0f,-1.0f,1.0f,0.0f,-1.0f,0.0f,0.0f,-1.0f,0.0f,
        0.500000f,-0.866025f,1.0f,0.500000f,-0.866025f,1.0f,0.500000f,-0.866025f,0.0f,0.500000f,-0.866025f,0.0f,
        0.866026f,-0.500000f,1.0f,0.866026f,-0.500000f,1.0f,0.866026f,-0.500000f,0.0f,0.866026f,-0.500000f,0.0f,
        1.0f,0.0f,1.0f,1.0f,0.0f,1.0f,1.0f,0.0f,0.0f,1.0f,0.0f,0.0f
};

static std::vector<PxVec3>* gVertexBuffer = NULL;
static int gLastTime = 0;
static int gFrameCounter = 0;
static char gTitle[256];
static bool gWireFrame = false;

static PX_FORCE_INLINE void prepareVertexBuffer()
{
    if(!gVertexBuffer)
        gVertexBuffer = new std::vector<PxVec3>;
    gVertexBuffer->clear();
}

static PX_FORCE_INLINE void pushVertex(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& n)
{
    PX_ASSERT(gVertexBuffer);
    gVertexBuffer->push_back(n);	gVertexBuffer->push_back(v0);
    gVertexBuffer->push_back(n);	gVertexBuffer->push_back(v1);
    gVertexBuffer->push_back(n);	gVertexBuffer->push_back(v2);
}

static PX_FORCE_INLINE void pushVertex(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& n0, const PxVec3& n1, const PxVec3& n2, float normalSign = 1)
{
    PX_ASSERT(gVertexBuffer);
    gVertexBuffer->push_back(normalSign * n0); gVertexBuffer->push_back(v0);
    gVertexBuffer->push_back(normalSign * n1); gVertexBuffer->push_back(v1);
    gVertexBuffer->push_back(normalSign * n2); gVertexBuffer->push_back(v2);
}

static PX_FORCE_INLINE const PxVec3* getVertexBuffer()
{
    PX_ASSERT(gVertexBuffer);
    return &(*gVertexBuffer)[0];
}

static void releaseVertexBuffer()
{
    if(gVertexBuffer)
    {
        delete gVertexBuffer;
        gVertexBuffer = NULL;
    }
}

static void renderSoftBodyGeometry(const PxTetrahedronMesh& mesh, const PxArray<PxVec4>& deformedPositionsInvMass)
{
    const int tetFaces[4][3] = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };

    //Get the deformed vertices
    //const PxVec3* vertices = mesh.getVertices();
    const PxU32 tetCount = mesh.getNbTetrahedrons();
    const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
    const void* indexBuffer = mesh.getTetrahedrons();

    prepareVertexBuffer();

    const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
    const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
    PxU32 numTotalTriangles = 0;
    for (PxU32 i = 0; i < tetCount; ++i)
    {
        PxU32 vref[4];
        if (has16BitIndices)
        {
            vref[0] = *shortIndices++;
            vref[1] = *shortIndices++;
            vref[2] = *shortIndices++;
            vref[3] = *shortIndices++;
        }
        else
        {
            vref[0] = *intIndices++;
            vref[1] = *intIndices++;
            vref[2] = *intIndices++;
            vref[3] = *intIndices++;
        }

        for (PxU32 j = 0; j < 4; ++j)
        {
            const PxVec4& v0 = deformedPositionsInvMass[vref[tetFaces[j][0]]];
            const PxVec4& v1 = deformedPositionsInvMass[vref[tetFaces[j][1]]];
            const PxVec4& v2 = deformedPositionsInvMass[vref[tetFaces[j][2]]];

            PxVec3 fnormal = (v1.getXYZ() - v0.getXYZ()).cross(v2.getXYZ() - v0.getXYZ());
            fnormal.normalize();

            pushVertex(v0.getXYZ(), v1.getXYZ(), v2.getXYZ(), fnormal);
            numTotalTriangles++;
        }
    }
    glPushMatrix();
    glScalef(1.0f, 1.0f, 1.0f);
    glEnableClientState(GL_NORMAL_ARRAY);
    glEnableClientState(GL_VERTEX_ARRAY);
    const PxVec3* vertexBuffer = getVertexBuffer();
    glNormalPointer(GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer);
    glVertexPointer(3, GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer + 1);
    glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glPopMatrix();
}

static void renderGeometry(const PxGeometry& geom, PxRigidActor* actor= nullptr, Shader* shader = nullptr)
{
    if (gWireFrame)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    switch(geom.getType())
    {
        case PxGeometryType::eBOX:
        {
            const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom);
            Cube cube(glm::vec3(boxGeom.halfExtents.x, boxGeom.halfExtents.y, boxGeom.halfExtents.z));
            cube.draw();
        }
            break;

        case PxGeometryType::eSPHERE:
        {
            const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom);
            Sphere sphere = Sphere(sphereGeom.radius, 10);
            sphere.draw();
        }
            break;

        case PxGeometryType::eCAPSULE:
        {
            const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom);
            const PxF32 radius = capsuleGeom.radius;
            const PxF32 halfHeight = capsuleGeom.halfHeight;

            //Sphere
            glPushMatrix();
            glTranslatef(halfHeight, 0.0f, 0.0f);
            glScalef(radius,radius,radius);
            Sphere sphere = Sphere(capsuleGeom.radius, 10);
            sphere.draw();
//            glutSolidSphere(1, 10, 10);
            glPopMatrix();

            //Sphere
            glPushMatrix();
            glTranslatef(-halfHeight, 0.0f, 0.0f);
            glScalef(radius,radius,radius);
            sphere.draw();
//            glutSolidSphere(1, 10, 10);
            glPopMatrix();

            //Cylinder
            glPushMatrix();
            glTranslatef(-halfHeight, 0.0f, 0.0f);
            glScalef(2.0f*halfHeight, radius,radius);
            glRotatef(90.0f,0.0f,1.0f,0.0f);
            glEnableClientState(GL_VERTEX_ARRAY);
            glEnableClientState(GL_NORMAL_ARRAY);
            glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), gCylinderData);
            glNormalPointer(GL_FLOAT, 2*3*sizeof(float), gCylinderData+3);
            glDrawArrays(GL_TRIANGLE_STRIP, 0, 13*2);
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glPopMatrix();
        }
            break;

        case PxGeometryType::eCONVEXMESH:
        {
            auto model = static_cast<PhysXModel*>(actor->userData);
            model->Draw(*shader);
        }
            break;

        case PxGeometryType::eTRIANGLEMESH:
        {
            auto model = static_cast<PhysXModel*>(actor->userData);
            model->Draw(*shader);
        }
            break;

        case PxGeometryType::eTETRAHEDRONMESH:
        {
            const int tetFaces[4][3] = { {0,2,1}, {0,1,3}, {0,3,2}, {1,2,3} };

            const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(geom);

            const PxTetrahedronMesh& mesh = *tetGeom.tetrahedronMesh;

            //Get the deformed vertices
            const PxVec3* vertices = mesh.getVertices();
            const PxU32 tetCount = mesh.getNbTetrahedrons();
            const PxU32 has16BitIndices = mesh.getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES;
            const void* indexBuffer = mesh.getTetrahedrons();

            prepareVertexBuffer();

            const PxU32* intIndices = reinterpret_cast<const PxU32*>(indexBuffer);
            const PxU16* shortIndices = reinterpret_cast<const PxU16*>(indexBuffer);
            PxU32 numTotalTriangles = 0;
            for (PxU32 i = 0; i < tetCount; ++i)
            {
                PxU32 vref[4];
                if (has16BitIndices)
                {
                    vref[0] = *shortIndices++;
                    vref[1] = *shortIndices++;
                    vref[2] = *shortIndices++;
                    vref[3] = *shortIndices++;
                }
                else
                {
                    vref[0] = *intIndices++;
                    vref[1] = *intIndices++;
                    vref[2] = *intIndices++;
                    vref[3] = *intIndices++;
                }

                for (PxU32 j = 0; j < 4; ++j)
                {
                    const PxVec3& v0 = vertices[vref[tetFaces[j][0]]];
                    const PxVec3& v1 = vertices[vref[tetFaces[j][1]]];
                    const PxVec3& v2 = vertices[vref[tetFaces[j][2]]];

                    PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
                    fnormal.normalize();

                    pushVertex(v0, v1, v2, fnormal);
                    numTotalTriangles++;
                }
            }
            glPushMatrix();
            glScalef(1.0f, 1.0f, 1.0f);
            glEnableClientState(GL_NORMAL_ARRAY);
            glEnableClientState(GL_VERTEX_ARRAY);
            const PxVec3* vertexBuffer = getVertexBuffer();
            glNormalPointer(GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer);
            glVertexPointer(3, GL_FLOAT, 2 * 3 * sizeof(float), vertexBuffer + 1);
            glDrawArrays(GL_TRIANGLES, 0, int(numTotalTriangles * 3));
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glPopMatrix();
        }
            break;

        default:
            break;
    }

    if (gWireFrame)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

static PxU32 gScreenWidth	= 0;
static PxU32 gScreenHeight	= 0;


namespace Snippets
{
//    void initFPS()
//    {
//        gLastTime = glutGet(GLUT_ELAPSED_TIME);
//    }
//
//    void showFPS(int updateIntervalMS, const char* info)
//    {
//        ++gFrameCounter;
//        int currentTime = glutGet(GLUT_ELAPSED_TIME);
//        if (currentTime - gLastTime > updateIntervalMS)
//        {
//            if (info)
//                sprintf(gTitle, " FPS : %4.0f%s", gFrameCounter * 1000.0 / (currentTime - gLastTime), info);
//            else
//                sprintf(gTitle, " FPS : %4.0f", gFrameCounter * 1000.0 / (currentTime - gLastTime));
//            glutSetWindowTitle(gTitle);
//            gLastTime = currentTime;
//            gFrameCounter = 0;
//        }
//    }

    PxU32 getScreenWidth()
    {
        return gScreenWidth;
    }

    PxU32 getScreenHeight()
    {
        return gScreenHeight;
    }

    void enableVSync(bool vsync)
    {
#if PX_WIN32 || PX_WIN64
        typedef void (APIENTRY * PFNWGLSWAPINTERVALPROC) (GLenum interval);
        PFNWGLSWAPINTERVALPROC wglSwapIntervalEXT = (PFNWGLSWAPINTERVALPROC)glfwGetProcAddress("wglSwapIntervalEXT");
        wglSwapIntervalEXT(vsync);
#else
        PX_UNUSED(vsync);
#endif
    }


    static float	gNearClip = 0.0f;
    static float	gFarClip = 0.0f;
    static float	gFOV = 60.0f;

    static float	gTextScale = 0.0f;
    static float	gTextY = 0.0f;

    PxVec3 computeWorldRayF(float xs, float ys, const PxVec3& camDir)
    {
        const float Width = float(gScreenWidth);
        const float Height = float(gScreenHeight);

        // Recenter coordinates in camera space ([-1, 1])
        const float u = ((xs - Width*0.5f)/Width)*2.0f;
        const float v = -((ys - Height*0.5f)/Height)*2.0f;

        // Adjust coordinates according to camera aspect ratio
        const float HTan = tanf(0.25f * fabsf(PxDegToRad(gFOV * 2.0f)));
        const float VTan = HTan*(Width/Height);

        // Ray in camera space
        const PxVec3 CamRay(VTan*u, HTan*v, 1.0f);

        // Compute ray in world space
        PxVec3 Right, Up;
        PxComputeBasisVectors(camDir, Right, Up);

        const PxMat33 invView(-Right, Up, camDir);

        return invView.transform(CamRay).getNormalized();
    }

//    void print(const char* text)
//    {
//        gTexter.print(0.0f, gTextY, gTextScale, text);
//        gTextY -= gTextScale;
//    }

    const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
    const PxReal shadowMat[] = { 1,0,0,0, -shadowDir.x / shadowDir.y,0,-shadowDir.z / shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

    void renderSoftBody(PxSoftBody* softBody, const PxArray<PxVec4>& deformedPositionsInvMass, bool shadows, const PxVec3& color)
    {
        PxShape* shape = softBody->getShape();

        const PxMat44 shapePose(PxIdentity); // (PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
        const PxGeometry& geom = shape->getGeometry();

        const PxTetrahedronMeshGeometry& tetGeom = static_cast<const PxTetrahedronMeshGeometry&>(geom);

        const PxTetrahedronMesh& mesh = *tetGeom.tetrahedronMesh;

        glPushMatrix();
        glMultMatrixf(&shapePose.column0.x);
        glColor4f(color.x, color.y, color.z, 1.0f);
        renderSoftBodyGeometry(mesh, deformedPositionsInvMass);
        glPopMatrix();

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    }


//    void renderHairSystem(physx::PxHairSystem* /*hairSystem*/, const physx::PxVec4* vertexPositionInvMass, PxU32 numVertices)
//    {
//        const PxVec3 color{ 1.0f, 0.0f, 0.0f };
//        const PxSphereGeometry geom(0.05f);
//
//        // draw the volume
//        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//        for(PxU32 j=0;j<numVertices;j++)
//        {
//            const PxMat44 shapePose(PxTransform(reinterpret_cast<const PxVec3&>(vertexPositionInvMass[j])));
//
//            glPushMatrix();
//            glMultMatrixf(&shapePose.column0.x);
//            glColor4f(color.x, color.y, color.z, 1.0f);
//            renderGeometry(geom);
//            glPopMatrix();
//        }
//
//        // draw the cage lines
//        const GLdouble aspect = GLdouble(glutGet(GLUT_WINDOW_WIDTH)) / GLdouble(glutGet(GLUT_WINDOW_HEIGHT));
//        glMatrixMode(GL_PROJECTION);
//        glLoadIdentity();
//        gluPerspective(60.0, aspect, GLdouble(gNearClip * 1.005f), GLdouble(gFarClip));
//        glMatrixMode(GL_MODELVIEW);
//
//        glDisable(GL_LIGHTING);
//        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
//
//        for (PxU32 j = 0; j < numVertices; j++)
//        {
//            const PxMat44 shapePose(PxTransform(reinterpret_cast<const PxVec3&>(vertexPositionInvMass[j])));
//
//            glPushMatrix();
//            glMultMatrixf(&shapePose.column0.x);
//            renderGeometry(geom);
//            glPopMatrix();
//        }
//
//        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//        glEnable(GL_LIGHTING);
//
//        glMatrixMode(GL_PROJECTION);
//        glLoadIdentity();
//        gluPerspective(60.0, aspect, GLdouble(gNearClip), GLdouble(gFarClip));
//        glMatrixMode(GL_MODELVIEW);
//    }

    void renderActors(PxRigidActor** actors, const PxU32 numActors, Shader shader, bool shadows, const PxVec3& color, TriggerRender* cb,
                      bool changeColorForSleepingActors, bool wireframePass)
    {
        PxShape* shapes[MAX_NUM_ACTOR_SHAPES];
        for(PxU32 i=0;i<numActors;i++)
        {
            const PxU32 nbShapes = actors[i]->getNbShapes();
            PX_ASSERT(nbShapes <= MAX_NUM_ACTOR_SHAPES);
            actors[i]->getShapes(shapes, nbShapes);
            bool sleeping;
            if (changeColorForSleepingActors)
                sleeping = actors[i]->is<PxRigidDynamic>() ? actors[i]->is<PxRigidDynamic>()->isSleeping() : false;
            else
                sleeping = false;
            for(PxU32 j=0;j<nbShapes;j++)
            {
                const PxMat44 shapePose(PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
                const PxTransform shapePose2 = PxShapeExt::getGlobalPose(*shapes[j], *actors[i]);
                const PxGeometry& geom = shapes[j]->getGeometry();

                const bool isTrigger = cb ? cb->isTrigger(shapes[j]) : shapes[j]->getFlags() & PxShapeFlag::eTRIGGER_SHAPE;
                if(isTrigger)
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

                auto quat = shapePose2.q;
                auto pos = shapePose2.p;

                glm::mat4 modelMatrix = glm::mat4(1.0f);
                modelMatrix = glm::translate(modelMatrix, glm::vec3(pos.x, pos.y, pos.z));
                modelMatrix = glm::rotate(modelMatrix, quat.getAngle(), glm::vec3(quat.x, quat.y, quat.z));

                shader.setMat4("model", modelMatrix);

                if(sleeping)
                {
                } else {}

                renderGeometry(geom, actors[i], &shader);

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            }
        }

        if(wireframePass)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            shader.setVec3("material.ambient", glm::vec3(0.0f, 0.0f, 0.0f));
            for(PxU32 i=0;i<numActors;i++)
            {
                const PxU32 nbShapes = actors[i]->getNbShapes();
                PX_ASSERT(nbShapes <= MAX_NUM_ACTOR_SHAPES);
                actors[i]->getShapes(shapes, nbShapes);

                for(PxU32 j=0;j<nbShapes;j++)
                {
//                    const PxMat44 shapePose2(PxShapeExt::getGlobalPose(*shapes[j], *actors[i]));
                    const PxTransform shapePose2 = PxShapeExt::getGlobalPose(*shapes[j], *actors[i]);
                    auto quat = shapePose2.q;
                    auto pos = shapePose2.p;

                    glm::mat4 modelMatrix = glm::mat4(1.0f);
                    modelMatrix = glm::translate(modelMatrix, glm::vec3(pos.x, pos.y, pos.z));
                    modelMatrix = glm::rotate(modelMatrix, quat.getAngle(), glm::vec3(quat.x, quat.y, quat.z));
                    shader.setMat4("model", modelMatrix);

                    renderGeometry(shapes[j]->getGeometry(), actors[i], &shader);
                }
            }
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }

/*static const PxU32 gGeomSizes[] = {
	sizeof(PxSphereGeometry),
	sizeof(PxPlaneGeometry),
	sizeof(PxCapsuleGeometry),
	sizeof(PxBoxGeometry),
	sizeof(PxConvexMeshGeometry),
	sizeof(PxTriangleMeshGeometry),
	sizeof(PxHeightFieldGeometry),
};

void renderGeoms(const PxU32 nbGeoms, const PxGeometry* geoms, const PxTransform* poses, bool shadows, const PxVec3& color)
{
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
	const PxReal shadowMat[]={ 1,0,0,0, -shadowDir.x/shadowDir.y,0,-shadowDir.z/shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

	const PxU8* stream = reinterpret_cast<const PxU8*>(geoms);
	for(PxU32 j=0;j<nbGeoms;j++)
	{
		const PxMat44 shapePose(poses[j]);

		const PxGeometry& geom = *reinterpret_cast<const PxGeometry*>(stream);
		stream += gGeomSizes[geom.getType()];

		// render object
		glPushMatrix();
		glMultMatrixf(&shapePose.column0.x);
		glColor4f(color.x, color.y, color.z, 1.0f);
		renderGeometry(geom);
		glPopMatrix();

		if(shadows)
		{
			glPushMatrix();
			glMultMatrixf(shadowMat);
			glMultMatrixf(&shapePose.column0.x);
			glDisable(GL_LIGHTING);
			glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
			renderGeometry(geom);
			glEnable(GL_LIGHTING);
			glPopMatrix();
		}
	}
}*/

    void renderGeoms(const PxU32 nbGeoms, const PxGeometryHolder* geoms, const PxTransform* poses, bool shadows, const PxVec3& color, glm::mat4 projection, Shader shader)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        //const PxVec3 shadowDir(0.0f, -0.7071067f, -0.7071067f);
        //const PxReal shadowMat[]={ 1,0,0,0, -shadowDir.x/shadowDir.y,0,-shadowDir.z/shadowDir.y,0, 0,0,1,0, 0,0,0,1 };

        for(PxU32 j=0;j<nbGeoms;j++)
        {
            const PxMat44 shapePose(poses[j]);

            const PxGeometry& geom = geoms[j].any();

            // render object
            glPushMatrix();
            glMultMatrixf(&shapePose.column0.x);
            glColor4f(color.x, color.y, color.z, 1.0f);
            renderGeometry(geom);
            glPopMatrix();

            if(shadows)
            {
                glPushMatrix();
                glMultMatrixf(shadowMat);
                glMultMatrixf(&shapePose.column0.x);
                glDisable(GL_LIGHTING);
                //glColor4f(0.1f, 0.2f, 0.3f, 1.0f);
                glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
                renderGeometry(geom);
                glEnable(GL_LIGHTING);
                glPopMatrix();
            }
        }

        if(1)
        {
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
//            gluPerspective(60.0, aspect, GLdouble(gNearClip*1.005f), GLdouble(gFarClip));
            shader.setMat4("projection", projection);
            glMatrixMode(GL_MODELVIEW);


            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);


            for(PxU32 j=0;j<nbGeoms;j++)
            {
                const PxMat44 shapePose(poses[j]);

                const PxGeometry& geom = geoms[j].any();

                // render object
                glPushMatrix();
                glMultMatrixf(&shapePose.column0.x);
                renderGeometry(geom);
                glPopMatrix();
            }


            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
//            gluPerspective(60.0, aspect, GLdouble(gNearClip), GLdouble(gFarClip));
            shader.setMat4("projection", projection);
            glMatrixMode(GL_MODELVIEW);
        }
    }

    PX_FORCE_INLINE PxVec3 getVec3(const physx::PxU8* data, const PxU32 index, const PxU32 sStrideInBytes)
    {
        return *reinterpret_cast<const PxVec3*>(data + index * sStrideInBytes);
    }

    void renderMesh(physx::PxU32 /*nbVerts*/, const physx::PxU8* verts, const PxU32 vertsStrideInBytes, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color,
                    const physx::PxU8* normals, const PxU32 normalsStrideInBytes, bool flipFaceOrientation, Shader shader, glm::mat4 projection)
    {
        if (nbTris == 0)
            return;

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        const PxMat44 idt(PxIdentity);
        glPushMatrix();
        glMultMatrixf(&idt.column0.x);
        glColor4f(color.x, color.y, color.z, 1.0f);
        {
            prepareVertexBuffer();

            PxU32 numTotalTriangles = 0;
            for(PxU32 i=0; i <nbTris; ++i)
            {
                const PxU32 vref0 = *indices++;
                const PxU32 vref1 = *indices++;
                const PxU32 vref2 = *indices++;

                const PxVec3& v0 = getVec3(verts, vref0, vertsStrideInBytes);
                const PxVec3& v1 = flipFaceOrientation ? getVec3(verts, vref2, vertsStrideInBytes) : getVec3(verts, vref1, vertsStrideInBytes);
                const PxVec3& v2 = flipFaceOrientation ? getVec3(verts, vref1, vertsStrideInBytes) : getVec3(verts, vref2, vertsStrideInBytes);

                if (normals)
                {
                    const PxVec3& n0 = getVec3(normals, vref0, normalsStrideInBytes);
                    const PxVec3& n1 = flipFaceOrientation ? getVec3(normals, vref2, normalsStrideInBytes) : getVec3(normals, vref1, normalsStrideInBytes);
                    const PxVec3& n2 = flipFaceOrientation ? getVec3(normals, vref1, normalsStrideInBytes) : getVec3(normals, vref2, normalsStrideInBytes);
                    pushVertex(v0, v1, v2, n0, n1, n2, flipFaceOrientation ? -1.0f : 1.0f);
                }
                else
                {
                    PxVec3 fnormal = (v1 - v0).cross(v2 - v0);
                    fnormal.normalize();
                    pushVertex(v0, v1, v2, fnormal);
                }
                numTotalTriangles++;
            }
            glScalef(1.0f, 1.0f, 1.0f);
            glEnableClientState(GL_NORMAL_ARRAY);
            glEnableClientState(GL_VERTEX_ARRAY);
            const PxVec3* vertexBuffer = getVertexBuffer();
            glNormalPointer(GL_FLOAT, 2*3*sizeof(float), vertexBuffer);
            glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), vertexBuffer+1);
            glDrawArrays(GL_TRIANGLES, 0, int(nbTris * 3));
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
        }
        glPopMatrix();

        if(0)
        {
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
//            gluPerspective(60.0, aspect, GLdouble(gNearClip*1.005f), GLdouble(gFarClip));
            shader.setMat4("projection", projection);
            glMatrixMode(GL_MODELVIEW);

            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

            glPushMatrix();
            glMultMatrixf(&idt.column0.x);

            glEnableClientState(GL_NORMAL_ARRAY);
            glEnableClientState(GL_VERTEX_ARRAY);
            const PxVec3* vertexBuffer = getVertexBuffer();
            glNormalPointer(GL_FLOAT, 2*3*sizeof(float), vertexBuffer);
            glVertexPointer(3, GL_FLOAT, 2*3*sizeof(float), vertexBuffer+1);
            glDrawArrays(GL_TRIANGLES, 0, int(nbTris * 3));
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);

            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
//            gluPerspective(60.0, aspect, GLdouble(gNearClip), GLdouble(gFarClip));
            shader.setMat4("projection", projection);
            glMatrixMode(GL_MODELVIEW);
        }
    }

//    void renderMesh(physx::PxU32 nbVerts, const physx::PxVec3* verts, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color, const physx::PxVec3* normals, bool flipFaceOrientation)
//    {
//        renderMesh(nbVerts, reinterpret_cast<const PxU8*>(verts), sizeof(PxVec3), nbTris, indices, color, reinterpret_cast<const PxU8*>(normals), sizeof(PxVec3), flipFaceOrientation);
//    }
//
//    void renderMesh(physx::PxU32 nbVerts, const physx::PxVec4* verts, physx::PxU32 nbTris, const physx::PxU32* indices, const physx::PxVec3& color, const physx::PxVec4* normals, bool flipFaceOrientation)
//    {
//        renderMesh(nbVerts, reinterpret_cast<const PxU8*>(verts), sizeof(PxVec4), nbTris, indices, color, reinterpret_cast<const PxU8*>(normals), sizeof(PxVec4), flipFaceOrientation);
//    }


    const physx::PxVec3 icosahedronPoints[12] = { PxVec3(0, -0.525731, 0.850651),
                                                  PxVec3(0.850651, 0, 0.525731),
                                                  PxVec3(0.850651, 0, -0.525731),
                                                  PxVec3(-0.850651, 0, -0.525731),
                                                  PxVec3(-0.850651, 0, 0.525731),
                                                  PxVec3(-0.525731, 0.850651, 0),
                                                  PxVec3(0.525731, 0.850651, 0),
                                                  PxVec3(0.525731, -0.850651, 0),
                                                  PxVec3(-0.525731, -0.850651, 0),
                                                  PxVec3(0, -0.525731, -0.850651),
                                                  PxVec3(0, 0.525731, -0.850651),
                                                  PxVec3(0, 0.525731, 0.850651) };
    const PxU32 icosahedronIndices[3 * 20] = { 1  ,2  ,6  ,
                                               1  ,7  ,2  ,
                                               3  ,4  ,5  ,
                                               4  ,3  ,8  ,
                                               6  ,5  ,11 ,
                                               5  ,6  ,10 ,
                                               9  ,10 ,2  ,
                                               10 ,9  ,3  ,
                                               7  ,8  ,9  ,
                                               8  ,7  ,0  ,
                                               11 ,0  ,1  ,
                                               0  ,11 ,4  ,
                                               6  ,2  ,10 ,
                                               1  ,6  ,11 ,
                                               3  ,5  ,10 ,
                                               5  ,4  ,11 ,
                                               2  ,7  ,9  ,
                                               7  ,1  ,0  ,
                                               3  ,9  ,8  ,
                                               4  ,8  ,0 };


#if PX_SUPPORT_GPU_PHYSX

    namespace
    {
        void createVBO(GLuint* vbo, PxU32 size)
        {
            PX_ASSERT(vbo);

            // create buffer object
            glGenBuffers(1, vbo);
            glBindBuffer(GL_ARRAY_BUFFER, *vbo);

            // initialize buffer object
            glBufferData(GL_ARRAY_BUFFER, size, 0, GL_DYNAMIC_DRAW);
            glBindBuffer(GL_ARRAY_BUFFER, 0);
        }

        void deleteVBO(GLuint* vbo)
        {
            glBindBuffer(1, *vbo);
            glDeleteBuffers(1, vbo);
            *vbo = 0;
        }

#if USE_CUDA_INTEROP
        //Returns the pointer to the cuda buffer
	void* mapCudaGraphicsResource(CUgraphicsResource* vbo_resource, size_t& numBytes, CUstream stream = 0)
	{
		CUresult result0 = cuGraphicsMapResources(1, vbo_resource, stream);
		PX_UNUSED(result0);
		void* dptr;
		CUresult result1 = cuGraphicsResourceGetMappedPointer((CUdeviceptr*)&dptr, &numBytes, *vbo_resource);
		PX_UNUSED(result1);
		return dptr;
	}

	void unmapCudaGraphicsResource(CUgraphicsResource* vbo_resource, CUstream stream = 0)
	{
		CUresult result2 = cuGraphicsUnmapResources(1, vbo_resource, stream);
		PX_UNUSED(result2);
	}
#endif

    } // namespace

    SharedGLBuffer::SharedGLBuffer() : vbo_res(NULL), devicePointer(NULL), vbo(0), size(0)
    {
    }

    void SharedGLBuffer::initialize(PxCudaContextManager* contextManager)
    {
        cudaContextManager = contextManager;
    }

    void SharedGLBuffer::allocate(PxU32 sizeInBytes)
    {
        release();
        createVBO(&vbo, sizeInBytes);
#if USE_CUDA_INTEROP
        physx::PxCudaInteropRegisterFlags flags = physx::PxCudaInteropRegisterFlags();
	cudaContextManager->acquireContext();
	CUresult result = cuGraphicsGLRegisterBuffer(reinterpret_cast<CUgraphicsResource*>(&vbo_res), vbo, flags);
	PX_UNUSED(result);
	cudaContextManager->releaseContext();
#endif
        size = sizeInBytes;
    }

    void SharedGLBuffer::release()
    {
        if (vbo)
        {
            deleteVBO(&vbo);
            vbo = 0;
        }
#if USE_CUDA_INTEROP
        if (vbo_res)
	{
		cudaContextManager->acquireContext();
		CUresult result = cuGraphicsUnregisterResource(reinterpret_cast<CUgraphicsResource>(vbo_res));
		PX_UNUSED(result);
		cudaContextManager->releaseContext();
		vbo_res = NULL;
	}
#endif
    }

    SharedGLBuffer::~SharedGLBuffer()
    {
        //release();
    }

    void* SharedGLBuffer::map()
    {
        if (devicePointer)
            return devicePointer;

#if USE_CUDA_INTEROP
            size_t numBytes;
	cudaContextManager->acquireContext();
	devicePointer = mapCudaGraphicsResource(reinterpret_cast<CUgraphicsResource*>(&vbo_res), numBytes, 0);
	cudaContextManager->releaseContext();
#else
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        devicePointer = glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif
        return devicePointer;
    }

    void SharedGLBuffer::unmap()
    {
        if (!devicePointer)
            return;
#if USE_CUDA_INTEROP
            cudaContextManager->acquireContext();
	unmapCudaGraphicsResource(reinterpret_cast<CUgraphicsResource*>(&vbo_res), 0);
	cudaContextManager->releaseContext();
#else
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glUnmapBuffer(GL_ARRAY_BUFFER);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif
        devicePointer = NULL;
    }

#endif // PX_SUPPORT_GPU_PHYSX

} //namespace Snippets

