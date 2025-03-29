#include <stdio.h>
#include <stdlib.h>
#include "joltc.h"
#include "raylib.h"
#include "raymath.h" // For quaternion and matrix operations
#include "rlgl.h"    // For rlPushMatrix, rlTranslatef, etc.
#include "flecs.h"

// Layer definitions
enum {
    NON_MOVING = 0,
    MOVING = 1,
    NUM_LAYERS = 2
};

// Broad-phase layer definitions
enum {
    NON_MOVING_BP = 0,
    MOVING_BP = 1,
    NUM_LAYERS_BP = 2
};

// Transform3D component
typedef struct {
  Vector3 position;
  Quaternion rotation;
  Vector3 scale;
  Matrix localMatrix;
  Matrix worldMatrix;
} Transform3D;
ECS_COMPONENT_DECLARE(Transform3D);

// Pointer component for raylib Model
typedef struct {
  Model* model;
} ModelComponent;
ECS_COMPONENT_DECLARE(ModelComponent);

// PhysicsBody component for Joltc
typedef struct {
  JPH_BodyID body;
} PhysicsBody;
ECS_COMPONENT_DECLARE(PhysicsBody);

// JoltcPhysics struct to encapsulate Joltc globals
typedef struct {
  JPH_PhysicsSystem* physicsSystem;
  JPH_BodyInterface* bodyInterface;
  JPH_JobSystem* jobSystem;
} JoltcPhysics;

// WorldContext struct to hold both Camera3D and later add variable later.
typedef struct {
  JoltcPhysics *physics;
  Camera3D *camera;
} WorldContext;


// Custom trace handler
static void TraceImpl(const char* message) {
    printf("Trace: %s\n", message);
}

// Physics simulation step time
#define PHYSICS_STEP (1.0f / 60.0f)


// Logic update system
void LogicUpdateSystem(ecs_iter_t *it) {
  //...
  // printf("update\n");
}

// Render begin system
void RenderBeginSystem(ecs_iter_t *it) {
  // printf("RenderBeginSystem\n");
  BeginDrawing();
  ClearBackground(RAYWHITE);
  //ClearBackground(GRAY);
}

// Begin camera system
void BeginCameraSystem(ecs_iter_t *it) {
  // printf("BeginCameraSystem\n");
  WorldContext *ctx = (WorldContext *)ecs_get_ctx(it->world);
  if (!ctx || !ctx->camera) return;
  BeginMode3D(*(ctx->camera));
}

// Camera Render system
// 3D model render only
// void CameraRenderSystem(ecs_iter_t *it) {
//   printf("CameraRenderSystem\n");
//   Transform3D *t = ecs_field(it, Transform3D, 0);
//   ModelComponent *m = ecs_field(it, ModelComponent, 1);
  
//   for (int i = 0; i < it->count; i++) {
//       if (m[i].model) {
//           // Update model transform with world matrix from physics
//           m[i].model->transform = t[i].worldMatrix;
//           DrawModelWires(*m[i].model, (Vector3){0,0,0}, 1.0f, RED);
//       }
//   }
//   DrawGrid(10, 1.0f);
// }

void CameraRenderSystem(ecs_iter_t *it) {
  // printf("CameraRenderSystem\n");
  Transform3D *t = ecs_field(it, Transform3D, 0);
  ModelComponent *m = ecs_field(it, ModelComponent, 1);
  
  for (int i = 0; i < it->count; i++) {
      if (m[i].model) {
          // Get entity name
          const char *name = ecs_get_name(it->world, it->entities[i]);
          Color color = RED; // Default color
          
          if (name) {
              if (strcmp(name, "Cube") == 0) {
                  color = RED;
              } else if (strcmp(name, "Floor") == 0) {
                  color = GRAY;
              }
          }
          
          // Update model transform with world matrix from physics
          m[i].model->transform = t[i].worldMatrix;
          DrawModelWires(*m[i].model, (Vector3){0,0,0}, 1.0f, color);
      }
  }
  DrawGrid(10, 1.0f);
}


// End camera system
void EndCameraSystem(ecs_iter_t *it) {
  // printf("EndCameraSystem\n");
  WorldContext *ctx = (WorldContext *)ecs_get_ctx(it->world);
  if (!ctx || !ctx->camera) return;
  EndMode3D();
}

// Render system
//2D only can't use 3D
void RenderSystem(ecs_iter_t *it) {
  //...
  // printf("RenderSystem\n");
  DrawFPS(10, 10);
}

// Physics system
void PhysicsSystem(ecs_iter_t *it) {
  Transform3D *t = ecs_field(it, Transform3D, 0);
  PhysicsBody *p = ecs_field(it, PhysicsBody, 1);
  WorldContext *ctx = (WorldContext *)ecs_get_ctx(it->world);
  
  if (!ctx || !ctx->physics || !ctx->physics->physicsSystem || 
      !ctx->physics->bodyInterface || !ctx->physics->jobSystem) {
      return;
  }

  // Update physics simulation
  JPH_PhysicsSystem_Update(ctx->physics->physicsSystem, PHYSICS_STEP, 1, ctx->physics->jobSystem);

  // Sync physics state to Transform3D
  for (int i = 0; i < it->count; i++) {
      JPH_RVec3 position;
      JPH_Quat rotation;
      
      JPH_BodyInterface_GetCenterOfMassPosition(ctx->physics->bodyInterface, 
          p[i].body, &position);
      JPH_BodyInterface_GetRotation(ctx->physics->bodyInterface, 
          p[i].body, &rotation);

      // Update Transform3D with physics data
      t[i].position = (Vector3){position.x, position.y, position.z};
      t[i].rotation = (Quaternion){rotation.x, rotation.y, rotation.z, rotation.w};
      
      // Update matrices
      Matrix matScale = MatrixScale(t[i].scale.x, t[i].scale.y, t[i].scale.z);
      Matrix matRot = QuaternionToMatrix(t[i].rotation);
      Matrix matTrans = MatrixTranslate(t[i].position.x, t[i].position.y, t[i].position.z);
      
      t[i].localMatrix = MatrixMultiply(matScale, matRot);
      t[i].worldMatrix = MatrixMultiply(t[i].localMatrix, matTrans);
  }
}

// Render end system
void RenderEndSystem(ecs_iter_t *it) {
  // printf("RenderEndSystem\n");
  EndDrawing();
}


int main() {

    // Initialize Flecs world
    ecs_world_t *world = ecs_init();

    // Declare components
    ECS_COMPONENT(world, Transform3D);
    ECS_COMPONENT(world, ModelComponent);
    ECS_COMPONENT(world, PhysicsBody);


    printf("init jolt physics\n");

    if (!JPH_Init()) {
        printf("Failed to initialize Jolt Physics\n");
        return 1;
    }

    JPH_SetTraceHandler(TraceImpl);

    JPH_JobSystem* jobSystem = JPH_JobSystemThreadPool_Create(NULL);
    if (!jobSystem) {
        printf("Failed to create job system, proceeding without it\n");
    } else {
        printf("Job system created\n");
    }

    JPH_ObjectLayerPairFilter* objectLayerPairFilterTable = JPH_ObjectLayerPairFilterTable_Create(NUM_LAYERS);
    JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, NON_MOVING, MOVING);
    JPH_ObjectLayerPairFilterTable_EnableCollision(objectLayerPairFilterTable, MOVING, NON_MOVING);

    JPH_BroadPhaseLayerInterface* broadPhaseLayerInterfaceTable = JPH_BroadPhaseLayerInterfaceTable_Create(NUM_LAYERS, NUM_LAYERS_BP);
    JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, NON_MOVING, NON_MOVING_BP);
    JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broadPhaseLayerInterfaceTable, MOVING, MOVING_BP);

    JPH_ObjectVsBroadPhaseLayerFilter* objectVsBroadPhaseLayerFilter = JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
        broadPhaseLayerInterfaceTable, NUM_LAYERS_BP, objectLayerPairFilterTable, NUM_LAYERS);

    JPH_PhysicsSystemSettings settings = {0};
    settings.maxBodies = 65536;
    settings.numBodyMutexes = 0;
    settings.maxBodyPairs = 65536;
    settings.maxContactConstraints = 65536;
    settings.broadPhaseLayerInterface = broadPhaseLayerInterfaceTable;
    settings.objectLayerPairFilter = objectLayerPairFilterTable;
    settings.objectVsBroadPhaseLayerFilter = objectVsBroadPhaseLayerFilter;
    JPH_PhysicsSystem* physicsSystem = JPH_PhysicsSystem_Create(&settings);
    JPH_BodyInterface* bodyInterface = JPH_PhysicsSystem_GetBodyInterface(physicsSystem);

    JPH_Vec3 gravity = { 0.0f, -9.81f, 0.0f };
    JPH_PhysicsSystem_SetGravity(physicsSystem, &gravity);
    JPH_PhysicsSystem_GetGravity(physicsSystem, &gravity);
    printf("Gravity: (%.2f, %.2f, %.2f)\n", gravity.x, gravity.y, gravity.z);

    // Floor creation
    printf("Create floor\n");
    JPH_RVec3 floorPos = { 0.0f, -1.0f, 0.0f }; // Top at -0.5f
    JPH_Quat floorRot = { 0.0f, 0.0f, 0.0f, 1.0f };
    JPH_Vec3 floorHalfExtents = { 10.0f, 0.5f, 10.0f };
    JPH_BoxShapeSettings* floorShapeSettings = JPH_BoxShapeSettings_Create(&floorHalfExtents, 0.0f);
    JPH_Shape* floorShape = (JPH_Shape*)JPH_BoxShapeSettings_CreateShape(floorShapeSettings);
    JPH_BodyCreationSettings* floorSettings = JPH_BodyCreationSettings_Create3(floorShape, &floorPos, &floorRot, JPH_MotionType_Static, NON_MOVING);
    JPH_Body* floorBody = JPH_BodyInterface_CreateBody(bodyInterface, floorSettings);
    JPH_BodyID floorId = JPH_Body_GetID(floorBody);
    JPH_BodyInterface_AddBody(bodyInterface, floorId, 0);

    // Sphere creation // not add
    // printf("Create sphere\n");
    // JPH_RVec3 spherePos = { 0.0f, 2.0f, 0.0f };
    // JPH_Quat sphereRot = { 0.0f, 0.0f, 0.0f, 1.0f };
    // JPH_SphereShapeSettings* sphereShapeSettings = JPH_SphereShapeSettings_Create(0.5f);
    // JPH_Shape* sphereShape = (JPH_Shape*)JPH_SphereShapeSettings_CreateShape(sphereShapeSettings);
    // JPH_BodyCreationSettings* sphereSettings = JPH_BodyCreationSettings_Create3(sphereShape, &spherePos, &sphereRot, JPH_MotionType_Dynamic, MOVING);
    // JPH_BodyID sphereId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, sphereSettings, JPH_Activation_Activate);

    printf("Create cube\n");// cube test needed
    JPH_RVec3 cubePos = { 0.0f, 2.0f, 0.0f };
    JPH_Quat cubeRot = { 0.0f, 0.0f, 0.0f, 1.0f };
    JPH_Vec3 cubeHalfExtents = { 0.5f, 0.5f, 0.5f };
    JPH_BoxShapeSettings* cubeShapeSettings = JPH_BoxShapeSettings_Create(&cubeHalfExtents, 0.0f);
    JPH_Shape* cubeShape = (JPH_Shape*)JPH_BoxShapeSettings_CreateShape(cubeShapeSettings);
    JPH_BodyCreationSettings* cubeSettings = JPH_BodyCreationSettings_Create3(cubeShape, &cubePos, &cubeRot, JPH_MotionType_Dynamic, MOVING);
    JPH_BodyID cubeId = JPH_BodyInterface_CreateAndAddBody(bodyInterface, cubeSettings, JPH_Activation_Activate);


    //improve physics
    JPH_PhysicsSystem_OptimizeBroadPhase(physicsSystem);

    // Initialize raylib
    InitWindow(800, 600, "Jolt + Raylib Test");
    SetTargetFPS(60);

    // Adjust camera to properly view the scene
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };  // Moved up and back to see floor and cube
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };      // Looking at origin
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Flecs set up
    // Define custom phases
    ecs_entity_t LogicUpdatePhase = ecs_new_w_id(world, EcsPhase);
    ecs_entity_t BeginRenderPhase = ecs_new_w_id(world, EcsPhase);
    ecs_entity_t BeginCameraPhase = ecs_new_w_id(world, EcsPhase);
    ecs_entity_t UpdateCameraPhase = ecs_new_w_id(world, EcsPhase);
    ecs_entity_t EndCameraPhase = ecs_new_w_id(world, EcsPhase);
    ecs_entity_t RenderPhase = ecs_new_w_id(world, EcsPhase);
    ecs_entity_t EndRenderPhase = ecs_new_w_id(world, EcsPhase);

    // Set phase dependencies
    ecs_add_pair(world, LogicUpdatePhase, EcsDependsOn, EcsPreUpdate);
    ecs_add_pair(world, BeginRenderPhase, EcsDependsOn, LogicUpdatePhase);
    ecs_add_pair(world, BeginCameraPhase, EcsDependsOn, BeginRenderPhase);
    ecs_add_pair(world, UpdateCameraPhase, EcsDependsOn, BeginCameraPhase);
    ecs_add_pair(world, EndCameraPhase, EcsDependsOn, UpdateCameraPhase);
    ecs_add_pair(world, RenderPhase, EcsDependsOn, EndCameraPhase);
    ecs_add_pair(world, EndRenderPhase, EcsDependsOn, RenderPhase);

    // Initialize systems
    ecs_system_init(world, &(ecs_system_desc_t){
      .entity = ecs_entity(world, { .name = "LogicUpdateSystem", .add = ecs_ids(ecs_dependson(LogicUpdatePhase)) }),
      //.query.terms = {
          //{ .id = ecs_id(Transform3D), .src.id = EcsSelf },
          //{ .id = ecs_id(PhysicsBody), .src.id = EcsSelf }
      //},
      .callback = LogicUpdateSystem
    });
    //physics update.
    ecs_system_init(world, &(ecs_system_desc_t){
        .entity = ecs_entity(world, { .name = "PhysicsSystem", .add = ecs_ids(ecs_dependson(LogicUpdatePhase)) }),
        .query.terms = {
            { .id = ecs_id(Transform3D), .src.id = EcsSelf },
            { .id = ecs_id(PhysicsBody), .src.id = EcsSelf }
        },
        .callback = PhysicsSystem
    });
    // render the screen
    ecs_system_init(world, &(ecs_system_desc_t){
        .entity = ecs_entity(world, { .name = "RenderBeginSystem", .add = ecs_ids(ecs_dependson(BeginRenderPhase)) }),
        .callback = RenderBeginSystem
    });
    //render started for camera 3d model only
    ecs_system_init(world, &(ecs_system_desc_t){
        .entity = ecs_entity(world, { .name = "BeginCameraSystem", .add = ecs_ids(ecs_dependson(BeginCameraPhase)) }),
        .callback = BeginCameraSystem
    });
    //this for mesh model only
    ecs_system_init(world, &(ecs_system_desc_t){
        .entity = ecs_entity(world, { .name = "CameraRenderSystem", .add = ecs_ids(ecs_dependson(UpdateCameraPhase)) }),
        .query.terms = {
            { .id = ecs_id(Transform3D), .src.id = EcsSelf },
            { .id = ecs_id(ModelComponent), .src.id = EcsSelf }
        }, 
        .callback = CameraRenderSystem
    });
    //finish camera render
    ecs_system_init(world, &(ecs_system_desc_t){
      .entity = ecs_entity(world, { .name = "EndCameraSystem", .add = ecs_ids(ecs_dependson(EndCameraPhase)) }),
      .callback = EndCameraSystem
    });
    //render 2d screen
    ecs_system_init(world, &(ecs_system_desc_t){
      .entity = ecs_entity(world, { .name = "RenderSystem", .add = ecs_ids(ecs_dependson(RenderPhase)) }),
      .callback = RenderSystem
    });
    //finish render
    ecs_system_init(world, &(ecs_system_desc_t){
        .entity = ecs_entity(world, { .name = "RenderEndSystem", .add = ecs_ids(ecs_dependson(EndRenderPhase)) }),
        .callback = RenderEndSystem
    });

    JoltcPhysics jphysics = {0};
    jphysics.physicsSystem = physicsSystem;
    jphysics.bodyInterface = bodyInterface;
    jphysics.jobSystem = jobSystem;



    // Combine into WorldContext
    WorldContext world_ctx = {
      .physics = &jphysics, //joltc physics
      .camera = &camera //raylib camera
    };
    ecs_set_ctx(world, &world_ctx, NULL);

    // // Load Raylib model
    // Model cube = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));

    // // Create entity with physics
    // ecs_entity_t node1 = ecs_new(world);
    // ecs_set_name(world, node1, "NodeMesh");
    // ecs_set(world, node1, Transform3D, {
    //     .position = (Vector3){0.0f, 5.0f, 0.0f},
    //     .rotation = QuaternionIdentity(),
    //     .scale = (Vector3){1.0f, 1.0f, 1.0f},
    //     .localMatrix = MatrixIdentity(),
    //     .worldMatrix = MatrixIdentity()
    // });
    // ecs_set(world, node1, ModelComponent, {&cube});


    // Create floor entity
    ecs_entity_t floor = ecs_new(world);
    ecs_set_name(world, floor, "Floor");
    ecs_set(world, floor, Transform3D, {
       .position = (Vector3){0.0f, -1.0f, 0.0f},
       .rotation = QuaternionIdentity(),
       .scale = (Vector3){20.0f, 0.5f, 20.0f},
       .localMatrix = MatrixIdentity(),
       .worldMatrix = MatrixIdentity()
    });
    ecs_set(world, floor, PhysicsBody, {floorId});
    // Generate mesh with full dimensions (200x1x200) since scale is applied in transform
    Model floorModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f)); // Double extents
    ecs_set(world, floor, ModelComponent, {&floorModel});

    // Create cube entity
    ecs_entity_t cube = ecs_new(world);
    ecs_set_name(world, cube, "Cube");
    ecs_set(world, cube, Transform3D, {
        .position = (Vector3){0.0f, 2.0f, 0.0f},
        .rotation = QuaternionIdentity(),
        .scale = (Vector3){1.0f, 1.0f, 1.0f},
        .localMatrix = MatrixIdentity(),
        .worldMatrix = MatrixIdentity()
    });
    ecs_set(world, cube, PhysicsBody, {cubeId});
    Model cubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    ecs_set(world, cube, ModelComponent, {&cubeModel});


    printf("Starting main loop\n");
    float time = 0.0f;
    while (!WindowShouldClose()) {
        //time += PHYSICS_STEP;
        //update physics
        //JPH_PhysicsSystem_Update(physicsSystem, PHYSICS_STEP, 1, jobSystem);

        //this is Flecs
        ecs_progress(world, 0);

        JPH_RVec3 sphereCurrentPos;
        JPH_Vec3 sphereVelocity;
        // JPH_BodyInterface_GetCenterOfMassPosition(bodyInterface, sphereId, &sphereCurrentPos);
        // JPH_BodyInterface_GetLinearVelocity(bodyInterface, sphereId, &sphereVelocity);

        JPH_BodyInterface_GetCenterOfMassPosition(bodyInterface, cubeId, &sphereCurrentPos);
        JPH_BodyInterface_GetLinearVelocity(bodyInterface, cubeId, &sphereVelocity);

        // BeginDrawing();
        // ClearBackground(RAYWHITE);

        // BeginMode3D(camera);

        // DrawCubeV((Vector3){floorPos.x, floorPos.y, floorPos.z}, 
        //           (Vector3){floorHalfExtents.x * 2, floorHalfExtents.y * 2, floorHalfExtents.z * 2}, 
        //           GRAY);
        // DrawCube((Vector3){sphereCurrentPos.x, sphereCurrentPos.y, sphereCurrentPos.z}, 1.0f, 1.0f, 1.0f, BLUE);
        // // DrawSphere((Vector3){sphereCurrentPos.x, sphereCurrentPos.y, sphereCurrentPos.z}, 0.5f, BLUE);
        // DrawGrid(10, 1.0f);

        // EndMode3D();

        // Reset cube with 'R' key
        if (IsKeyPressed(KEY_R)) {
          JPH_RVec3 newPos = {
            (float)(rand() % 5 - 5), // Random X: -10 to 10
            (float)(rand() % 10 + 5),  // Random Y: 5 to 15
            (float)(rand() % 5 - 5)  // Random Z: -10 to 10
          };
          JPH_Quat newRot = {
              (float)(rand() % 360) / 360.0f,
              (float)(rand() % 360) / 360.0f,
              (float)(rand() % 360) / 360.0f,
              (float)(rand() % 360) / 360.0f
          };

          float norm = sqrt(newRot.x * newRot.x + newRot.y * newRot.y + newRot.z * newRot.z + newRot.w * newRot.w);
          newRot.x /= norm; newRot.y /= norm; newRot.z /= norm; newRot.w /= norm; // Normalize quaternion

          JPH_BodyInterface_SetPosition(bodyInterface, cubeId, &newPos, JPH_Activation_Activate);
          JPH_BodyInterface_SetRotation(bodyInterface, cubeId, &newRot, JPH_Activation_Activate);
          JPH_BodyInterface_SetLinearVelocity(bodyInterface, cubeId, &(JPH_Vec3){ 0.0f, 0.0f, 0.0f });

          // JPH_BodyInterface_SetPosition(bodyInterface, sphereId, &newPos, JPH_Activation_Activate);
          // JPH_BodyInterface_SetRotation(bodyInterface, sphereId, &newRot, JPH_Activation_Activate);
          // JPH_BodyInterface_SetLinearVelocity(bodyInterface, sphereId, &(JPH_Vec3){ 0.0f, 0.0f, 0.0f });
        }


        // DrawFPS(10, 10);
        // DrawText(TextFormat("Sphere Y: %.2f, Vel Y: %.2f", sphereCurrentPos.y, sphereVelocity.y), 10, 30, 20, BLACK);
        // DrawText(TextFormat("Time: %.2f s", time), 10, 50, 20, BLACK);

        //EndDrawing();

        // if ((int)(time * 60) % 60 == 0) {
        //     printf("Time: %.2f, Sphere Y: %.2f, Vel Y: %.2f\n", time, sphereCurrentPos.y, sphereVelocity.y);
        // }
    }

    printf("clean up\n");

    CloseWindow();

    // Cleanup
    UnloadModel(floorModel);
    UnloadModel(cubeModel);

    JPH_BodyInterface_RemoveBody(bodyInterface, floorId);
    JPH_BodyInterface_DestroyBody(bodyInterface, floorId);
    // JPH_BodyInterface_RemoveBody(bodyInterface, sphereId);
    // JPH_BodyInterface_DestroyBody(bodyInterface, sphereId);

    JPH_BodyInterface_RemoveBody(bodyInterface, cubeId);
    JPH_BodyInterface_DestroyBody(bodyInterface, cubeId);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)floorShapeSettings);
    JPH_Shape_Destroy(floorShape);
    // JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)sphereShapeSettings);
    JPH_ShapeSettings_Destroy((JPH_ShapeSettings*)cubeShapeSettings);
    // JPH_Shape_Destroy(sphereShape);
    JPH_Shape_Destroy(cubeShape);
    JPH_BodyCreationSettings_Destroy(floorSettings);
    // JPH_BodyCreationSettings_Destroy(sphereSettings);
    JPH_BodyCreationSettings_Destroy(cubeSettings);

    JPH_JobSystem_Destroy(jobSystem);
    JPH_PhysicsSystem_Destroy(physicsSystem);
    JPH_Shutdown();

    ecs_fini(world);

    printf("finish\n");

    return 0;
}