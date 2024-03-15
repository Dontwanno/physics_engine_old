#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <stb_image.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <learnopengl/shader_m.h>
#include <learnopengl/camera.h>

#include "PxPhysicsAPI.h"
#include "SnippetPVD.h"
#include "render_funcs.hpp"
#include "mesh_creator.hpp"
#include <filesystem>
//#include "PhysXModel.hpp"
//#include "sphere.hpp"
//#include "cube.hpp"
//#include "learnopengl/model.h"
#include "urdf.h"
#include <urdf/model.h>



#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
unsigned int loadTexture(const char *path);


// settings
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 800;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

void renderScene(const Shader &shader);
void renderCube();
unsigned int planeVAO;

void renderQuad();


using namespace physx;

static PxDefaultAllocator		                gAllocator;
static PxDefaultErrorCallback	                gErrorCallback;
static PxFoundation*			                gFoundation     = NULL;
static PxPhysics*				                gPhysics	    = NULL;
static PxCooking*				                gCooking	    = NULL;
static PxDefaultCpuDispatcher*	                gDispatcher     = NULL;
static PxScene*					                gScene		    = NULL;
static PxMaterial*				                gMaterial	    = NULL;
static PxPvd*					                gPvd            = NULL;
static PxArticulationReducedCoordinate*			gArticulation	= NULL;
static PxArticulationJointReducedCoordinate*	gDriveJoint		= NULL;
static PxArticulationJointReducedCoordinate*	gDriveJoint2	= NULL;
static std::vector<PxArticulationJointReducedCoordinate*> jointVector;


static PxRigidDynamic* createDynamic(const PxTransform& t, const PxGeometry& geometry, const PxVec3& velocity=PxVec3(0))
{
    PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics, t, geometry, *gMaterial, 10.0f);
    dynamic->setAngularDamping(0.5f);
    dynamic->setLinearVelocity(velocity);
    gScene->addActor(*dynamic);
    return dynamic;
}

static PxFilterFlags scissorFilter(	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
                                       PxFilterObjectAttributes attributes1, PxFilterData filterData1,
                                       PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
    PX_UNUSED(attributes0);
    PX_UNUSED(attributes1);
    PX_UNUSED(constantBlock);
    PX_UNUSED(constantBlockSize);
    if (filterData0.word2 != 0 && filterData0.word2 == filterData1.word2)
        return PxFilterFlag::eKILL;
    pairFlags |= PxPairFlag::eCONTACT_DEFAULT;
    return PxFilterFlag::eDEFAULT;
}

void readGeometry(const std::string& filename, PxConvexMeshGeometry& geom) {
//    PxVec3 scale(1000.0f);
    PxVec3 scale(1.0f);

    std::ifstream file(filename, std::ios::binary);
    const std::uint64_t HeaderSize = sizeof(std::uint64_t);
    std::uint64_t fileSize;

    file.read(reinterpret_cast<char*>(&fileSize), HeaderSize);

    file.seekg(HeaderSize, std::ios::beg);

    std::unique_ptr<char[]> buffer = std::make_unique<char[]>(fileSize);
    char* bufferPtr = buffer.get();
    (void)file.read(bufferPtr, fileSize);

    file.close();

    PxDefaultMemoryInputData inputData(reinterpret_cast<physx::PxU8*>(bufferPtr), fileSize);

    auto geom_type = geom.getType();

    switch(geom_type) {
        case PxGeometryType::eCONVEXMESH: {
            PxConvexMesh* convexMesh = gPhysics->createConvexMesh(inputData);
            PxConvexMeshGeometry meshGeom = PxConvexMeshGeometry(convexMesh);
            meshGeom.scale.scale = scale;
            geom = meshGeom;
            break;
        }
//        case PxGeometryType::eTRIANGLEMESH: {
//            PxTriangleMesh* triangleMesh = gPhysics->createTriangleMesh(inputData);
//            PxTriangleMeshGeometry triGeom = PxTriangleMeshGeometry(triangleMesh);
//            triGeom.scale.scale = scale;
//           break;
//            geom = triGeom;
////         }
        default: {
            std::cout << "Geometry type not supported" << std::endl;
            break;
        }
    }
}

void swap_family(std::string& parent, std::string& child, int& parent_index, int& child_index, int& joint_index, const std::vector<jointData>& joints, const std::vector<linkData>& links) {
    int j = 0;
    for (auto joint_n: joints) {
        if (joint_n.parent_link_name == child) {
            parent = joint_n.parent_link_name;
            child = joint_n.child_link_name;
            joint_index = j;
            break;
        }
        j++;
    }
    int i = 0;
    for (auto link: links) {
        if (link.name == parent) {
            parent_index = i;
        }
        else if (link.name == child) {
            child_index = i;
        }
        i++;
    }
}

static void createRobot()
{
    std::vector<linkData> links;
    std::vector<jointData> joints;
    std::shared_ptr<urdf::UrdfModel> model;

    parse_urdf("../resources/robots/edo_sim/edo_sim.urdf", links, joints, model);

    std::cout << "baselink: " << model->getRoot()->name << std::endl;


    std::cout << "links: " << links.size() << std::endl;
    std::cout << "joints: " << joints.size() << std::endl;

    for (linkData& link: links) {
        if (link.visual_mesh_filename != "") {
            std::string split_string = link.visual_mesh_filename.substr(link.visual_mesh_filename.find("package://") + 10);
            std::string mesh_path = "../resources/robots/" + split_string;
            if (std::filesystem::exists("../resources/generated/" + link.name + ".mesh")) {
                readGeometry("../resources/generated/" + link.name + ".mesh", link.convex_geom);
            } else {
                createConvexMesh(mesh_path, gPhysics, link.name);
                readGeometry("../resources/generated/" + link.name + ".mesh", link.convex_geom);}
        }
    }

    gArticulation->setSolverIterationCounts(32);
    gArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, true);

    int parent_index, child_index, joint_index, i , j;

    std::string parent, child = model->getRoot()->name;
    PxArticulationLink* parent_link = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.f, 0.f)));
    PxRigidActorExt::createExclusiveShape(*parent_link, links[child_index].convex_geom, *gMaterial);
    PxRigidBodyExt::updateMassAndInertia(*parent_link, 1.f);


    for (int z = 0 ; z < joints.size(); z++) {
        swap_family(parent, child, parent_index, child_index, joint_index, joints, links);

        auto transform = joints[joint_index].transform;
        PxArticulationLink* link = gArticulation->createLink(parent_link, PxTransform(PxVec3(0.f, 0.f, 0.f)));
        PxRigidActorExt::createExclusiveShape(*link, links[child_index].convex_geom, *gMaterial);
        PxRigidBodyExt::updateMassAndInertia(*link, 1.f);

        PxArticulationLimit limits;
        limits.low  = joints[joint_index].lower_limit;
        limits.high = joints[joint_index].upper_limit;

        PxArticulationJointType::Enum type;

        switch (joints[joint_index].joint_type) {
            case urdf::REVOLUTE: {
                type = PxArticulationJointType::eREVOLUTE;
                break;}
            case urdf::CONTINUOUS: {
                type = PxArticulationJointType::eREVOLUTE;
                break;}
            case urdf::PRISMATIC: {
                type = PxArticulationJointType::ePRISMATIC;
                break;}
        }

        int final_idx;
        for (int idx = 0; idx < 3; idx++) {
            if (joints[joint_index].axis[idx] == 1 or joints[joint_index].axis[idx] == -1) {
                final_idx = idx;
                break;
            }
        };

        //Set up the drive joint...
        auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link->getInboundJoint());
        joint->setJointType(type);
        joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
        joint->setLimitParams(PxArticulationAxis::eTWIST, limits);
        joint->setDrive(PxArticulationAxis::eTWIST, 1.f, 0.f, PX_MAX_F32);

        if (joints[joint_index].axis.magnitude() == 1) {
            joint->setParentPose(transform);
            joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
        } else {
            joint->setParentPose(PxTransform(transform.p));
            joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f), transform.getInverse().q));
        }
        parent_link = link;
    }

//    swap_family(parent, child, parent_index, child_index, joint_index, joints, links);
//
//    auto transform1 = joints[joint_index].transform;
//
//    //(1) Create base...
////    PxArticulationLink* base = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.f, 0.f)));
//    PxArticulationLink* base = gArticulation->createLink(NULL, transform1);
//    PxRigidActorExt::createExclusiveShape(*base, links[parent_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*base, 1.f);
//
//    PxArticulationLink* link1 = gArticulation->createLink(base, transform1);
//    PxRigidActorExt::createExclusiveShape(*link1, links[child_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*link1, 1.f);
//
//    //Set up the drive joint...
//    gDriveJoint = static_cast<PxArticulationJointReducedCoordinate*>(link1->getInboundJoint());
//    gDriveJoint->setJointType(PxArticulationJointType::eREVOLUTE);
//    gDriveJoint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eLIMITED);
//    PxArticulationLimit limits;
//    limits.low = -PxPiDivFour;  // in rad for a rotational motion
//    limits.high = PxPiDivFour;
//    gDriveJoint->setLimitParams(PxArticulationAxis::eSWING1, limits);
//    gDriveJoint->setDrive(PxArticulationAxis::eSWING1, 100000.f, 0.f, PX_MAX_F32);
//
//    gDriveJoint->setParentPose(base->getGlobalPose());
//    gDriveJoint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
//
//    swap_family(parent, child, parent_index, child_index, joint_index, joints, links);
//
//    auto transform2 = joints[joint_index].transform;
//
//    limits.low = -PxPiDivFour/10;  // in rad for a rotational motion
//    limits.high = PxPiDivFour/10;
//
//    PxArticulationLink* link2 = gArticulation->createLink(link1, transform2);
//    PxRigidActorExt::createExclusiveShape(*link2, links[child_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*link2, 1.f);
//
//    //Set up the drive joint...
//    gDriveJoint2 = static_cast<PxArticulationJointReducedCoordinate*>(link2->getInboundJoint());
//    gDriveJoint2->setJointType(PxArticulationJointType::eREVOLUTE);
//    gDriveJoint2->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
//    gDriveJoint2->setLimitParams(PxArticulationAxis::eTWIST, limits);
//    gDriveJoint2->setDrive(PxArticulationAxis::eTWIST, 100000.f, 0.f, PX_MAX_F32);
//
//    gDriveJoint2->setParentPose(PxTransform(transform2.p));
//    gDriveJoint2->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f), transform2.getInverse().q));
//
//    swap_family(parent, child, parent_index, child_index, joint_index, joints, links);
//
//    auto transform3 = joints[joint_index].transform;
//
//    PxArticulationLink* link3 = gArticulation->createLink(link2, PxTransform(PxVec3(0.f, 0.f, 0.f)));
//    PxRigidActorExt::createExclusiveShape(*link3, links[child_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*link3, 1.f);
//
//    //Set up the drive joint...
//    auto joint = static_cast<PxArticulationJointReducedCoordinate*>(link3->getInboundJoint());
//    joint->setJointType(PxArticulationJointType::eREVOLUTE);
//    joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
//    joint->setLimitParams(PxArticulationAxis::eTWIST, limits);
//    joint->setDrive(PxArticulationAxis::eTWIST, 1.f, 0.f, PX_MAX_F32);
//
//    joint->setParentPose(transform3);
//    joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
//
//    swap_family(parent, child, parent_index, child_index, joint_index, joints, links);
//
//    auto transform4 = joints[joint_index].transform;
//
//    PxArticulationLink* link4 = gArticulation->createLink(link3, PxTransform(PxVec3(0.f, 0.f, 0.f)));
//    PxRigidActorExt::createExclusiveShape(*link4, links[child_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*link4, 1.f);
//
//    //Set up the drive joint...
//    auto joint2 = static_cast<PxArticulationJointReducedCoordinate*>(link4->getInboundJoint());
//    joint2->setJointType(PxArticulationJointType::eREVOLUTE);
//    joint2->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
//    joint2->setLimitParams(PxArticulationAxis::eTWIST, limits);
//    joint2->setDrive(PxArticulationAxis::eTWIST, 1.f, 0.f, PX_MAX_F32);
//
//    joint2->setParentPose(transform4);
//    joint2->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
//
//    swap_family(parent, child, parent_index, child_index, joint_index, joints, links);
//
//    auto transform5 = joints[joint_index].transform;
//
//    PxArticulationLink* link5 = gArticulation->createLink(link4, PxTransform(PxVec3(0.f, 0.f, 0.f)));
//    PxRigidActorExt::createExclusiveShape(*link5, links[child_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*link5, 1.f);
//
//    //Set up the drive joint...
//    auto joint3 = static_cast<PxArticulationJointReducedCoordinate*>(link5->getInboundJoint());
//    joint3->setJointType(PxArticulationJointType::eREVOLUTE);
//    joint3->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
//    joint3->setLimitParams(PxArticulationAxis::eTWIST, limits);
//    joint3->setDrive(PxArticulationAxis::eTWIST, 1.f, 0.f, PX_MAX_F32);
//
//    joint3->setParentPose(transform5);
//    joint3->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));
//
//    swap_family(parent, child, parent_index, child_index, joint_index, joints, links);
//
//    auto transform6 = joints[joint_index].transform;
//
//    PxArticulationLink* link6 = gArticulation->createLink(link5, PxTransform(PxVec3(0.f, 0.f, 0.f)));
//    PxRigidActorExt::createExclusiveShape(*link6, links[child_index].convex_geom, *gMaterial);
//    PxRigidBodyExt::updateMassAndInertia(*link6, 1.f);
//
//    //Set up the drive joint...
//    auto joint4 = static_cast<PxArticulationJointReducedCoordinate*>(link6->getInboundJoint());
//    joint4->setJointType(PxArticulationJointType::eREVOLUTE);
//    joint4->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eLIMITED);
//    joint4->setLimitParams(PxArticulationAxis::eSWING1, limits);
//    joint4->setDrive(PxArticulationAxis::eSWING1, 1.f, 0.f, PX_MAX_F32);
//
//    joint4->setParentPose(transform6);
//    joint4->setChildPose(PxTransform(PxVec3(0.f, 0.f, 0.f)));


    //**************************************************//

    gScene->addArticulation(*gArticulation);

    for (PxU32 i = 0; i < gArticulation->getNbLinks(); ++i)
    {
        PxArticulationLink* link;
        gArticulation->getLinks(&link, 1, i);

        link->setLinearDamping(0.2f);
        link->setAngularDamping(0.2f);

        link->setMaxAngularVelocity(20.f);
        link->setMaxLinearVelocity(100.f);

        for (PxU32 b = 0; b < link->getNbShapes(); ++b)
        {
            PxShape* shape;
            link->getShapes(&shape, 1, b);

            shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
        }
    }
}

void initPhysics(bool interactive)
{
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
    gPvd = PxCreatePvd(*gFoundation);
    PxPvdTransport* transport = PxDefaultPvdSocketTransportCreate(PVD_HOST, 5425, 10);
    gPvd->connect(*transport,PxPvdInstrumentationFlag::eALL);

    gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, gPvd);
    PxInitExtensions(*gPhysics, gPvd);

    gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(PxTolerancesScale()));

    PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
    sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

    PxU32 numCores = PxThread::getNbPhysicalCores();
    gDispatcher = PxDefaultCpuDispatcherCreate(numCores == 0 ? 0 : numCores - 1);
    sceneDesc.cpuDispatcher	= gDispatcher;
    sceneDesc.filterShader	= PxDefaultSimulationFilterShader;

    sceneDesc.solverType = PxSolverType::eTGS;
    sceneDesc.filterShader = scissorFilter;

    gScene = gPhysics->createScene(sceneDesc);
    PxPvdSceneClient* pvdClient = gScene->getScenePvdClient();
    if(pvdClient)
    {
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONSTRAINTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_CONTACTS, true);
        pvdClient->setScenePvdFlag(PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES, true);
    }

    gMaterial = gPhysics->createMaterial(0.5f, 0.5f, 0.f);

    PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics, PxPlane(0,1,0,0), *gMaterial);
    gScene->addActor(*groundPlane);

    gArticulation = gPhysics->createArticulationReducedCoordinate();

    createRobot();



}

static bool gClosing = true;

void stepPhysics(bool /*interactive*/)
{
    const PxReal dt = 1.0f / 60.f;
    PxReal driveValue = gDriveJoint->getDriveTarget(PxArticulationAxis::eSWING1);

    if (gClosing && driveValue < -1.f)
        gClosing = false;
    else if (!gClosing && driveValue > 1.f)
        gClosing = true;

    if (gClosing)
        driveValue -= dt*0.1f;
//        driveValue -= dt;
    else
        driveValue += dt*0.1f;
//        driveValue += dt;
    gDriveJoint->setDriveTarget(PxArticulationAxis::eSWING1, driveValue);
    gDriveJoint2->setDriveTarget(PxArticulationAxis::eTWIST, driveValue);

    (void)gScene->simulate(1.0f/60.0f);
    (void)gScene->fetchResults(true);
}

void cleanupPhysics(bool /*interactive*/)
{
    PX_RELEASE(gScene);
    PX_RELEASE(gDispatcher);
    PX_RELEASE(gPhysics);
    if(gPvd)
    {
        PxPvdTransport* transport = gPvd->getTransport();
        gPvd->release();	gPvd = NULL;
        PX_RELEASE(transport);
    }
    PX_RELEASE(gFoundation);

    (void)printf("SnippetHelloWorld done.\n");
}



int main()
{
//    std::string filename = "../resources/robots/edo_sim/edo_sim.urdf";
//    parse_urdf(filename);

    initPhysics(false);

    while (true) {
        stepPhysics(false);
    }


//    // glfw: initialize and configure
//    // ------------------------------
//    glfwInit();
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//
//    // glfw window creation
//    // --------------------
//    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
//    if (window == NULL)
//    {
//        std::cout << "Failed to create GLFW window" << std::endl;
//        glfwTerminate();
//        return -1;
//    }
//    glfwMakeContextCurrent(window);
//    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
//    glfwSetCursorPosCallback(window, mouse_callback);
//    glfwSetScrollCallback(window, scroll_callback);
//
//    // tell GLFW to capture our mouse
//    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
//
//    // glad: load all OpenGL function pointers
//    // ---------------------------------------
//    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
//    {
//        std::cout << "Failed to initialize GLAD" << std::endl;
//        return -1;
//    }
//
//    // tell stb_image.h to flip loaded texture's on the y-axis (before loading model).
//    stbi_set_flip_vertically_on_load(true);
//
//    // configure global opengl state
//    // -----------------------------
//    glEnable(GL_DEPTH_TEST);
//    glCullFace(GL_BACK);
//
//    // build and compile shaders
//    // -------------------------
//    Shader screenShader("../shaders/screenvert.glsl", "../shaders/screenfrag.glsl");
//    Shader depthShader("../shaders/depthTest.vs", "../shaders/depthTest.fs");
//
//    // setup framebuffer for quad and depth map
//    // -----------------------------------------
//    unsigned int framebuffer;
//    glGenFramebuffers(1, &framebuffer);
//    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
//    // color + specular color buffer
//    unsigned int gColor;
//    glGenTextures(1, &gColor);
//    glBindTexture(GL_TEXTURE_2D, gColor);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gColor, 0);
//    // create a normal attachment texture
//    unsigned int gNormal;
//    glGenTextures(1, &gNormal);
//    glBindTexture(GL_TEXTURE_2D, gNormal);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, SCR_WIDTH, SCR_HEIGHT, 0, GL_RGBA, GL_FLOAT, NULL);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, gNormal, 0);
//    // create depth Texture
//    unsigned int depthTexture;
//    glGenTextures(1, &depthTexture);
//    glBindTexture(GL_TEXTURE_2D, depthTexture);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SCR_WIDTH, SCR_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL); // no need to specify data
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // use linear filtering
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // use linear filtering
//    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTexture, 0);
//    // tell OpenGL which color attachments we'll use (of this framebuffer) for rendering
//    unsigned int attachments[2] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
//    glDrawBuffers(2, attachments);
//    // finally check if framebuffer is complete
//    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
//        std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
//    glGetError();
//
//    // load textures
//    // -------------
//    unsigned int cubeTexture = loadTexture("../resources/textures/wood.png");
//
//    // shader configuration
//    // --------------------
//    screenShader.use();
//    screenShader.setInt("colorTexture", 0);
//    screenShader.setInt("normalTexture", 1);
//    screenShader.setInt("depthTexture", 2);
//
//
//    Cube cube(glm::vec3(1.0f));
//
//
////    Model stepper = Model("../resources/myobjects/character.obj");
//
//    while (!glfwWindowShouldClose(window))
//    {
//        // per-frame time logic
//        // --------------------
//        float currentFrame = static_cast<float>(glfwGetTime());
//        deltaTime = currentFrame - lastFrame;
//        lastFrame = currentFrame;
//
//        glGetError();
//
//        // input
//        // -----
//        processInput(window);
//
//        // render
//        // ------
//        // first pass: render scene to depth and colour map
//        // ------------------------------------------------
//        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
//        glEnable(GL_DEPTH_TEST); // enable depth testing (is disabled for rendering screen-space quad)
//        float r = 249/255; float g  = 132/255; float b = 74/255;
//        glClearColor(r, g, b, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // clear depth buffer
//        depthShader.use();
//        glm::mat4 model = glm::mat4(1.0f);
//        glm::mat4 view = camera.GetViewMatrix();
//        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
//        depthShader.setMat4("view", view);
//        depthShader.setMat4("projection", projection);
////        renderScene(depthShader);
//
//        depthShader.setMat4("model", model);
//        cube.draw();
//        glBindFramebuffer(GL_FRAMEBUFFER, 0);
//
//
//
//
//        // second pass: render quad with scene's visuals as its texture image
//        // ------------------------------------------------------------------
//        glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
//        glClearColor(r, g, b, 1.0f);
//        glClear(GL_COLOR_BUFFER_BIT); // clear colour buffer//
//        screenShader.use();
//
//        // bind color map
//        glActiveTexture(GL_TEXTURE0);
//        glBindTexture(GL_TEXTURE_2D, gColor);
//        // bind diffuse map
//        glActiveTexture(GL_TEXTURE1);
//        glBindTexture(GL_TEXTURE_2D, gNormal);
//        // bind depth map
//        glActiveTexture(GL_TEXTURE2);
//        glBindTexture(GL_TEXTURE_2D, depthTexture);
//        renderQuad();
//
//        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
//        // -------------------------------------------------------------------------------
//        glfwSwapBuffers(window);
//        glfwPollEvents();
//    }
//
//    glfwTerminate();
//    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);

    static bool nKeyPressed = false;
    if (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS && !nKeyPressed)
    {
        nKeyPressed = true;

        // Perform the action associated with the 'N' key
        createDynamic(PxTransform(PxVec3(camera.Position.x, camera.Position.y, camera.Position.z)), PxSphereGeometry(3.0f), PxVec3(camera.Front.x, camera.Front.y, camera.Front.z) * 200);
    }
    else if (glfwGetKey(window, GLFW_KEY_N) == GLFW_RELEASE)
    {
        nKeyPressed = false;
    }
}

// renders the 3D scene
// --------------------
void renderScene(const Shader &shader)
{
    // floor
    // set up vertex data (and buffer(s)) and configure vertex attributes
    // ------------------------------------------------------------------
    float planeVertices[] = {
            // positions            // normals         // texcoords
            25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
            -25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,   0.0f,  0.0f,
            -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,

            25.0f, -0.5f,  25.0f,  0.0f, 1.0f, 0.0f,  25.0f,  0.0f,
            -25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,   0.0f, 25.0f,
            25.0f, -0.5f, -25.0f,  0.0f, 1.0f, 0.0f,  25.0f, 25.0f
    };
    // plane VAO
    unsigned int planeVBO;
    glGenVertexArrays(1, &planeVAO);
    glGenBuffers(1, &planeVBO);
    glBindVertexArray(planeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glBindVertexArray(0);
    glm::mat4 model = glm::mat4(1.0f);
    shader.setMat4("model", model);
    glBindVertexArray(planeVAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    // cubes
    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(0.0f, 1.5f, 0.0));
    model = glm::scale(model, glm::vec3(0.5f));
    shader.setMat4("model", model);
    renderCube();
    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(2.0f, 0.0f, 1.0));
    model = glm::scale(model, glm::vec3(0.5f));
    shader.setMat4("model", model);
    renderCube();
    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(-1.0f, 0.0f, 2.0));
    model = glm::rotate(model, glm::radians(60.0f), glm::normalize(glm::vec3(1.0, 0.0, 1.0)));
    model = glm::scale(model, glm::vec3(0.25));
    shader.setMat4("model", model);
    renderCube();

    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(5.0f, 2.0f, 4.0));
    model = glm::scale(model, glm::vec3(1.4f));
    shader.setMat4("model", model);
    renderCube();

    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(-3.0f, 1.0f, 0.0));
    model = glm::scale(model, glm::vec3(0.5f));
    shader.setMat4("model", model);
    renderCube();

}



// renderCube() renders a 1x1 3D cube in NDC.
// -------------------------------------------------
unsigned int cubeVAO = 0;
unsigned int cubeVBO = 0;
void renderCube()
{
    // initialize (if necessary)
    if (cubeVAO == 0)
    {
        float vertices[] = {
                // back face
                -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
                1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
                1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, // bottom-right
                1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, // top-right
                -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, // bottom-left
                -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, // top-left
                // front face
                -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
                1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, // bottom-right
                1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
                1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, // top-right
                -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, // top-left
                -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, // bottom-left
                // left face
                -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
                -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-left
                -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
                -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-left
                -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-right
                -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-right
                // right face
                1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
                1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
                1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 1.0f, // top-right
                1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 1.0f, // bottom-right
                1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f, 0.0f, // top-left
                1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f, 0.0f, // bottom-left
                // bottom face
                -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
                1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 1.0f, // top-left
                1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
                1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f, 0.0f, // bottom-left
                -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 0.0f, // bottom-right
                -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f, 1.0f, // top-right
                // top face
                -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
                1.0f,  1.0f , 1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
                1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 1.0f, // top-right
                1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f, 0.0f, // bottom-right
                -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 1.0f, // top-left
                -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f, 0.0f  // bottom-left
        };
        glGenVertexArrays(1, &cubeVAO);
        glGenBuffers(1, &cubeVBO);
        // fill buffer
        glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        // link vertex attributes
        glBindVertexArray(cubeVAO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    // render Cube
    glBindVertexArray(cubeVAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);
}



// renderQuad() renders a 1x1 XY quad in NDC
// -----------------------------------------
unsigned int quadVAO = 0;
unsigned int quadVBO;
void renderQuad()
{
    if (quadVAO == 0)
    {
        float quadVertices[] = {
                // positions        // texture Coords
                -1.0f,  1.0f, 0.0f, 0.0f, 1.0f,
                -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
                1.0f,  1.0f, 0.0f, 1.0f, 1.0f,
                1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
        };
        // setup plane VAO
        glGenVertexArrays(1, &quadVAO);
        glGenBuffers(1, &quadVBO);
        glBindVertexArray(quadVAO);
        glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
    }
    glBindVertexArray(quadVAO);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

void renderPhyxsActors(Shader& shader) {
    PxScene* scene;
    PxGetPhysics().getScenes(&scene,1);
    PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);
        if(nbActors)
        {
            std::vector<PxRigidActor*> actors(nbActors);
            scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, reinterpret_cast<PxActor**>(&actors[0]), nbActors);
            Snippets::renderActors(&actors[0], static_cast<PxU32>(actors.size()), shader, true, physx::PxVec3(0.0f, 0.75f, 0.0f), NULL, true, true);
        }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const * path)
{
    unsigned int textureID;
    glGenTextures(1, &textureID);

    int width, height, nrComponents;
    unsigned char *data = stbi_load(path, &width, &height, &nrComponents, 0);
    if (data)
    {
        GLenum format;
        if (nrComponents == 1)
            format = GL_RED;
        else if (nrComponents == 3)
            format = GL_RGB;
        else if (nrComponents == 4)
            format = GL_RGBA;

        glBindTexture(GL_TEXTURE_2D, textureID);
        glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT); // for this tutorial: use GL_CLAMP_TO_EDGE to prevent semi-transparent borders. Due to interpolation it takes texels from next repeat
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, format == GL_RGBA ? GL_CLAMP_TO_EDGE : GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        stbi_image_free(data);
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free(data);
    }

    return textureID;
}