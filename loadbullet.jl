const bullet_lib_path = "/home/schmrlng/code/oss/bullet3/src"
const bullet_include_paths = ["/home/schmrlng/code/oss/bullet3/src", dirname(@__FILE__())]

for p in bullet_include_paths
    addHeaderDir(p, kind=C_System)
end
Libdl.dlopen(bullet_lib_path * "/BulletCollision/libBulletCollision.so", Libdl.RTLD_GLOBAL)
Libdl.dlopen(bullet_lib_path * "/LinearMath/libLinearMath.so", Libdl.RTLD_GLOBAL)

cxx"""
#include<iostream>
#include<sstream>
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "distanceComputation.h"
#include "LinearMath/btConvexHull.h"

#ifndef BT_BINARY_COLLISION_CALLBACKS
#define BT_BINARY_COLLISION_CALLBACKS

struct BinaryCollisionCallback : public btCollisionWorld::ContactResultCallback
{
    bool is_collision;

    BinaryCollisionCallback() : btCollisionWorld::ContactResultCallback(), is_collision(false) {}

    virtual btScalar addSingleResult(btManifoldPoint& cp,
        const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
        const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
    {
        is_collision = true;
        return 0;
    }
};

struct BinarySweptConvexCollisionCallback : public btCollisionWorld::ConvexResultCallback
{
    bool is_collision;

    BinarySweptConvexCollisionCallback() : btCollisionWorld::ConvexResultCallback(), is_collision(false) {}
    
    virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
    {
        is_collision = true;
        return 0;
    }
};
#endif // BT_BINARY_COLLISION_CALLBACKS
"""
