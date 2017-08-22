module BT

using Cxx
using StaticArrays
using MotionPlanning
include("loadbullet.jl")

import MotionPlanning: is_free_state, is_free_motion, Polygon, Circle

# typealias BulletCollisionObject
#     Cxx.CppPtr{Cxx.CxxQualType{Cxx.CppBaseType{:btCollisionObject},(false,false,false)},(false,false,false)}
# typealias BulletConvexShape
#     Cxx.CppPtr{Cxx.CxxQualType{Cxx.CppBaseType{:btConvexShape},(false,false,false)},(false,false,false)}
# typealias BulletCompoundShape
#     Cxx.CppPtr{Cxx.CxxQualType{Cxx.CppBaseType{:btCompoundShape},(false,false,false)},(false,false,false)}
# typealias BulletCollisionWorld
#     Cxx.CppPtr{Cxx.CxxQualType{Cxx.CppBaseType{:btCollisionWorld},(false,false,false)},(false,false,false)}
typealias BulletCollisionObjectPtr cxxt"""btCollisionObject*"""
typealias BulletCollisionShapePtr cxxt"""btCollisionShape*"""
# typealias BulletConvexShapePtr cxxt"""btConvexShape*"""
# typealias BulletCompoundShapePtr cxxt"""btCompoundShape*"""
typealias BulletCollisionWorldPtr cxxt"""btCollisionWorld*"""
typealias BulletTransformPtr cxxt"""btTransform*"""
typealias BulletTransformValue cxxt"""btTransform"""
typealias BulletScalar cxxt"""btScalar"""
const BulletInf = icxx"""BT_LARGE_FLOAT;"""

set_margin(obj::BulletCollisionObjectPtr, dx = 0.) = set_margin(get_collision_shape(obj), dx)
function set_margin(cs::BulletCollisionShapePtr, dx = 0.)
    if icxx"""$cs->isConvex();"""
        icxx"""$cs->setMargin($dx);"""
    elseif icxx"""$cs->isCompound();"""
        cs = icxx"""btCompoundShape* cm = (btCompoundShape*)$cs; cm;"""
        num_children = icxx"""$cs->getNumChildShapes();"""
        for i in 0:num_children-1
            set_margin(icxx"""$cs->getChildShape($i);""", dx)
        end
    end
end
get_collision_shape(obj::BulletCollisionObjectPtr) = icxx"""$obj->getCollisionShape();"""
get_world_transform(obj::BulletCollisionObjectPtr) = icxx"""btTransform tr = $obj->getWorldTransform(); tr;"""
identity_transform() = icxx"""btTransform tr; tr.setIdentity(); tr;"""

function box(lo::AbstractVector, hi::AbstractVector)
    halfextents = (hi - lo)/2
    mid = (hi + lo)/2
    icxx"""
    btCollisionObject* box = new btCollisionObject();
    btBoxShape* box_shape = new btBoxShape(btVector3($(halfextents[1]), $(halfextents[2]), $(halfextents[3])));
    box_shape->setMargin(0.);
    box->getWorldTransform().setOrigin(btVector3($(mid[1]), $(mid[2]), $(mid[3])));
    box->setCollisionShape(box_shape);
    box;
    """
end

function convex_hull{V<:AbstractVector}(pts::Vector{V})
    convex_hull = icxx"""
    btCollisionObject* convex_hull = new btCollisionObject();
    btConvexHullShape* convex_hull_shape = new btConvexHullShape();
    convex_hull_shape->setMargin(0.);
    convex_hull->getWorldTransform().setIdentity();
    convex_hull->setCollisionShape(convex_hull_shape);
    convex_hull;
    """
    for v in pts
        icxx"""((btConvexHullShape*)($convex_hull->getCollisionShape()))->addPoint(btVector3($(v[1]), $(v[2]), $(v[3])));"""
    end
    convex_hull
end
convex_hull{V<:AbstractVector}(pts::V...) = convex_hull(collect(pts))
convex_hull_box(lo::AbstractVector, hi::AbstractVector) = convex_hull([SVector([lo hi][[1,2,3] + 3*[i,j,k]]) for i=0:1, j=0:1, k=0:1]...)
function convex_hull_cylinder(p1::AbstractVector, p2::AbstractVector, r::AbstractFloat, n::Int=24)
    Q, _ = qr((p2-p1)'', thin=false)
    d1, d2 = Q[:,2], Q[:,3]
    convex_hull((p1 + r*cos(2*pi*i/n)*d1 + r*sin(2*pi*i/n)*d2 for i in 1:n)...,
                (p2 + r*cos(2*pi*i/n)*d1 + r*sin(2*pi*i/n)*d2 for i in 1:n)...)
end

function sphere(c::AbstractVector, r::AbstractFloat)
    icxx"""
    btCollisionObject* sphere = new btCollisionObject();
    btSphereShape* sphere_shape = new btSphereShape($r);
    sphere_shape->setMargin(0.);
    sphere->getWorldTransform().setOrigin(btVector3($(c[1]), $(c[2]), $(c[3])));
    sphere->setCollisionShape(sphere_shape);
    sphere;
    """
end

function compound_collision_object(objs::Vector{BulletCollisionObjectPtr})
    compound = icxx"""
    btCollisionObject* compound = new btCollisionObject();
    btCompoundShape* compound_shape = new btCompoundShape(true, $(length(objs)));
    compound_shape->setMargin(0.);
    compound->getWorldTransform().setIdentity();
    compound->setCollisionShape(compound_shape);
    compound;
    """
    for o in objs
        icxx"""((btCompoundShape*)($compound->getCollisionShape()))->addChildShape($o->getWorldTransform(),
                                                                                   $o->getCollisionShape());"""
    end
    icxx"""((btCompoundShape*)($compound->getCollisionShape()))->recalculateLocalAabb();"""
    compound
end
compound_collision_object(objs::BulletCollisionObjectPtr...) = compound_collision_object(collect(objs))

function tree(base::AbstractVector, b::AbstractFloat, h::AbstractFloat, r::AbstractFloat, R::AbstractFloat, n::Int = 24)
    if n > 0
        trunk = convex_hull((base + [r*cos(2*pi*i/n), r*sin(2*pi*i/n), 0.] for i in 1:n)...,
                            (base + [r*cos(2*pi*i/n), r*sin(2*pi*i/n), b] for i in 1:n)...)
        branches = convex_hull((base + [R*cos(2*pi*i/n), R*sin(2*pi*i/n), b] for i in 1:n)...,
                               (base + [0., 0., h] for i in 1:n)...)
        compound_collision_object(trunk, branches)
    else
        error("TODO")
    end
end

function forest(lo::AbstractVector, hi::AbstractVector, N, b, h, r, R; n = 24, d_min = R, seed = 0)
    seed == 0 || srand(seed)
    tree_locations = [[lo[2] + rand()*hi[2], lo[2] + rand()*hi[2], 0]]
    i = 1
    while length(tree_locations) < N && i < 10000
        i += 1
        new_loc = [lo[2] + rand()*hi[2], lo[2] + rand()*hi[2], 0]
        if all(norm(new_loc - loc) >= d_min for loc in tree_locations)
            push!(tree_locations, new_loc)
        end
    end
    [tree(loc, b, h, r, R, n) for loc in tree_locations]
end

function make_2d(co::BulletCollisionObjectPtr)
    icxx"""
    btConvex2dShape* shape2d = new btConvex2dShape((btConvexShape*)($co->getCollisionShape()));
    shape2d->setMargin(0.);
    $co->setCollisionShape(shape2d);
    """
    co
end

polygon{V<:AbstractVector}(pts::Vector{V}) = make_2d(convex_hull([SVector(p[1], p[2], 0) for p in pts]))
polygon(P::Polygon) = polygon(P.points)
circle(c::AbstractVector, r) = make_2d(sphere(SVector(c[1], c[2], 0), r))
circle(C::Circle) = circle(C.c, C.r)

function distance(co1::BulletCollisionObjectPtr, co2::BulletCollisionObjectPtr, max_d2 = BulletInf)
    result_ptr = icxx"""
    btScalar* result = new btScalar[7];
    result[0] = btScalar(BT_LARGE_FLOAT);
    compute_distance($co1->getCollisionShape(), $co1->getWorldTransform(),
                     $co2->getCollisionShape(), $co2->getWorldTransform(),
                     result, $max_d2);
    result;
    """
    result = unsafe_load(convert(Ptr{SVector{7,Float32}}, result_ptr))
    icxx"""delete[] $result_ptr;"""
    result[1], result[(2,3,4)], result[(5,6,7)]
end

function distance(co1::BulletCollisionObjectPtr, co2::BulletCollisionObjectPtr, W::AbstractMatrix, max_d2 = BulletInf)
    U = chol(W)
    result_ptr = icxx"""
    btMatrix3x3 U($(U[1,1]), $(U[1,2]), $(U[1,3]),
                  $(U[2,1]), $(U[2,2]), $(U[2,3]),
                  $(U[3,1]), $(U[3,2]), $(U[3,3]));
    btScalar* result = new btScalar[7];
    result[0] = btScalar(BT_LARGE_FLOAT);
    compute_weighted_distance($co1->getCollisionShape(), $co1->getWorldTransform(),
                              $co2->getCollisionShape(), $co2->getWorldTransform(),
                              U, result, $max_d2);
    result;
    """
    result = unsafe_load(convert(Ptr{SVector{7,Float32}}, result_ptr))
    icxx"""delete[] $result_ptr;"""
    result[1], result[(2,3,4)], result[(5,6,7)]
end

function collision_world(lo::AbstractVector, hi::AbstractVector, maxobjects = 100)
    icxx"""
    btCollisionConfiguration* collision_configuration = new btDefaultCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_configuration);
    
    btVector3 worldAabbMin($(lo[1]), $(lo[2]), $(lo[3]));
    btVector3 worldAabbMax($(hi[1]), $(hi[2]), $(hi[3]));
    unsigned int max_objects = $maxobjects;
    
    btBroadphaseInterface* broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, false);
    btCollisionWorld* cw = new btCollisionWorld(dispatcher, broadphase, collision_configuration);
    cw;
    """
end
collision_world(SS::BoundedStateSpace, maxobj = 100) =
    collision_world(state2workspace(SS.lo, SS.s2w), state2workspace(SS.hi, SS.s2w), maxobj)
add_collision_object!(cw::BulletCollisionWorldPtr, co::BulletCollisionObjectPtr) =
    icxx"""$cw->addCollisionObject($co); $cw->updateAabbs();"""

get_convex_components(co::BulletCollisionObjectPtr) = get_convex_components(get_collision_shape(co), get_world_transform(co))
function get_convex_components(cs::BulletCollisionShapePtr, tr::BulletTransformValue = identity_transform())
    if icxx"""$cs->isConvex();"""
        [icxx"""btCollisionObject* cc = new btCollisionObject(); cc->setCollisionShape($cs); cc->setWorldTransform($tr); cc;"""]
    elseif icxx"""$cs->isCompound();"""
        cs = icxx"""btCompoundShape* cm = (btCompoundShape*)$cs; cm;"""
        num_children = icxx"""$cs->getNumChildShapes();"""
        vcat((get_convex_components(icxx"""$cs->getChildShape($i);""",
                                    icxx"""btTransform comp = $tr * $cs->getChildTransform($i); comp;""")
              for i in 0:num_children-1)...)
    end
end

type BulletStaticEnvironment <: SweptCollisionChecker
    robot::BulletCollisionObjectPtr
    convex_robot_components::Vector{BulletCollisionObjectPtr}    # when robot's world transform is the identity; TODO: set_transformation is unsafe
    environment::BulletCollisionWorldPtr
    convex_env_components::Vector{BulletCollisionObjectPtr}
    count::Int
end
function BulletStaticEnvironment(r::BulletCollisionObjectPtr, e::BulletCollisionWorldPtr)
    num_co = icxx"""$e->getNumCollisionObjects();"""
    e_components = vcat((get_convex_components(icxx"""(btCollisionObject *)($e->getCollisionObjectArray()[$i]);""") for i in 0:num_co-1)...)
    BulletStaticEnvironment(r, get_convex_components(get_collision_shape(r)), e, e_components, 0)
end
BulletStaticEnvironment(r::BulletCollisionObjectPtr, SS::StateSpace) = BulletStaticEnvironment(r, collision_world(SS))
function add_collision_object!(CC::BulletStaticEnvironment, co::BulletCollisionObjectPtr)
    add_collision_object!(CC.environment, co)
    append!(CC.convex_env_components, get_convex_components(co))
end
set_margin(CC::BulletStaticEnvironment, dx = 0.) = set_margin(CC.robot, dx)
# function BulletStaticEnvironment(SS::StateSpace, CC::PointRobotNDBoxes, dx = 0.01, convex_hull_obs = true) # TODO: REDO
#     robot = icxx"""
#     btCollisionObject* robot = new btCollisionObject();
#     btConvexShape* robot_shape;
#     if ($dx > 0) {
#         robot_shape = new btBoxShape(btVector3($dx, $dx, $dx));
#         robot_shape->setMargin(0.);
#     } else {
#         robot_shape = new btConvexHullShape();
#         robot_shape->setMargin(0.);
#         ((btConvexHullShape*) robot_shape)->addPoint(btVector3(0., 0., 0.));
#     }
#     btCompoundShape* robot_compound = new btCompoundShape(true, 1);
#     robot_compound->setMargin(0.);
#     btTransform t;
#     t.setIdentity();
#     robot_compound->addChildShape(t, robot_shape);
#     robot_compound->recalculateLocalAabb();
#     robot->setCollisionShape(robot_compound);
#     robot;
#     """
#     env_lo = state2workspace(SS.lo, SS)
#     env_hi = state2workspace(SS.hi, SS)
#     cw = collision_world(env_lo, env_hi)
#     cwo = BulletCollisionObjectPtr[]
#     for BB in CC.boxes
#         box = convex_hull_obs ? convex_hull_box(BB.lo+dx, BB.hi-dx) : box(BB.lo+dx, BB.hi-dx)
#         icxx"""
#         $cw->addCollisionObject($box);
#         """
#         push!(cwo, box)
#     end
#     icxx"""
#     $cw->updateAabbs();
#     """
#     BulletStaticEnvironment(robot, cw, cwo)
# end

function set_transformation(o::BulletCollisionObjectPtr, v::SVector{3})
    icxx"""$o->getWorldTransform().setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));"""
end
function set_transformation(o::BulletCollisionObjectPtr, v::SVector{6})
    icxx"""
    $o->getWorldTransform().setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));
    btQuaternion rot;
    rot.setEulerZYX($(v[4]), $(v[5]), $(v[6]));
    $o->getWorldTransform().setRotation(rot);
    """
end
set_transformation(o::BulletCollisionObjectPtr, v::AbstractVector) = set_transformation(o, SVector(v))

function is_free_state(v::AbstractVector, CC::BulletStaticEnvironment)
    set_transformation(CC.robot, v)
    icxx"""
    BinaryCollisionCallback result;
    $(CC.environment)->contactTest($(CC.robot), result);
    !result.is_collision;
    """
end

function is_free_motion(v::SVector{3}, w::SVector{3}, CC::BulletStaticEnvironment)
    CC.count += 1
    icxx"""
    bool free_motion = true;
    btCompoundShape* robot_compound = (btCompoundShape*) ($(CC.robot)->getCollisionShape());
    btTransform v, w;
    v.setIdentity();
    w.setIdentity();
    v.setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));
    w.setOrigin(btVector3($(w[1]), $(w[2]), $(w[3])));
    for (int i = 0; i < robot_compound->getNumChildShapes(); i++) {
        BinarySweptConvexCollisionCallback result;
        btConvexShape* robot_piece = (btConvexShape*) (robot_compound->getChildShape(i));
        $(CC.environment)->convexSweepTest(robot_piece, v * robot_compound->getChildTransform(i), w * robot_compound->getChildTransform(i), result);
        if (result.is_collision) {
            free_motion = false;
            break;
        }
    }
    free_motion;
    """
end
function is_free_motion(v::SVector{6}, w::SVector{6}, CC::BulletStaticEnvironment)
    CC.count += 1
    icxx"""
    bool free_motion = true;
    btCompoundShape* robot_compound = (btCompoundShape*) ($(CC.robot)->getCollisionShape());
    btTransform v, w;
    btQuaternion vrot, wrot;
    vrot.setEulerZYX($(v[4]), $(v[5]), $(v[6]));
    wrot.setEulerZYX($(w[4]), $(w[5]), $(w[6]));
    v.setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));
    w.setOrigin(btVector3($(w[1]), $(w[2]), $(w[3])));
    v.setRotation(vrot);
    w.setRotation(wrot);
    for (int i = 0; i < robot_compound->getNumChildShapes(); i++) {
        BinarySweptConvexCollisionCallback result;
        btConvexShape* robot_piece = (btConvexShape*) (robot_compound->getChildShape(i));
        $(CC.environment)->convexSweepTest(robot_piece, v * robot_compound->getChildTransform(i), w * robot_compound->getChildTransform(i), result);
        if (result.is_collision) {
            free_motion = false;
            break;
        }
    }
    free_motion;
    """
end
is_free_motion(v::AbstractVector, w::AbstractVector, CC::BulletStaticEnvironment) = is_free_motion(SVector(v), SVector(w), CC)

function pairwise_convex_convex_distances{T}(v::AbstractVector{T}, CC::BulletStaticEnvironment, threshold = T(Inf))
    foreach(rc -> set_transformation(rc, v), CC.convex_robot_components)
    pairwise_iter = ((ri, ei, distance(rc, ec)) for (ri, rc) in enumerate(CC.convex_robot_components),
                                                    (ei, ec) in enumerate(CC.convex_env_components))
    collect(filter(x -> x[3][1] < threshold, pairwise_iter))    # note: filter must be collected now because of set_transformation
end

function pairwise_convex_convex_distances{T}(v::AbstractVector{T}, CC::BulletStaticEnvironment, W::AbstractMatrix{T}, threshold = T(Inf))
    foreach(rc -> set_transformation(rc, v), CC.convex_robot_components)
    pairwise_iter = ((ri, ei, distance(rc, ec, W)) for (ri, rc) in enumerate(CC.convex_robot_components),
                                                       (ei, ec) in enumerate(CC.convex_env_components))
    collect(filter(x -> x[3][1] < threshold, pairwise_iter))    # note: filter must be collected now because of set_transformation
end

function distance(CC::BulletStaticEnvironment, ri::Int, tr::AbstractVector, ei::Int)
    rc, ec = CC.convex_robot_components[ri], CC.convex_env_components[ei]
    set_transformation(rc, tr)
    distance(rc, ec)[1]
end

function distance_gradient_finite_difference{T}(CC::BulletStaticEnvironment, ri::Int, tr::SVector{6,T}, ei::Int, dx::T = T(.02))
    trm = MVector(tr)
    function gradi(i)
        tri = trm[i]
        trm[i] = tri + dx; fp = distance(CC, ri, trm, ei)
        trm[i] = tri - dx; fm = distance(CC, ri, trm, ei)
        trm[i] = tri
        (fp - fm) / (dx + dx)
    end
    SVector{6}(gradi(i) for i in 1:6)
end

function distance(CC::BulletStaticEnvironment, ri::Int, tr::AbstractVector, ei::Int, W::AbstractMatrix)
    rc, ec = CC.convex_robot_components[ri], CC.convex_env_components[ei]
    set_transformation(rc, tr)
    distance(rc, ec, W)[1]
end

function distance_gradient_finite_difference{T}(CC::BulletStaticEnvironment, ri::Int, tr::SVector{6,T}, ei::Int, W::AbstractMatrix, dx::T = T(.02))
    trm = MVector(tr)
    function gradi(i)
        tri = trm[i]
        trm[i] = tri + dx; fp = distance(CC, ri, trm, ei, W)
        trm[i] = tri - dx; fm = distance(CC, ri, trm, ei, W)
        trm[i] = tri
        (fp - fm) / (dx + dx)
    end
    SVector{6}(gradi(i) for i in 1:6)
end

function distance_hessian_finite_difference{T}(CC::BulletStaticEnvironment, ri::Int, tr::SVector{6,T}, ei::Int, dx::T = T(.02))
    f = distance(CC, ri, tr, ei)
    trm = MVector(tr)
    function hessij(i, j)
        if i == j
            tri = trm[i]
            trm[i] = tri + dx; fp = distance(CC, ri, trm, ei)
            trm[i] = tri - dx; fm = distance(CC, ri, trm, ei)
            trm[i] = tri
            (fp + fm - f - f) / (2*dx*dx)
        elseif i > j
            tri, trj = trm[i], trm[j]
            trm[i] = tri + dx; trm[j] = trj + dx; fpp = distance(CC, ri, trm, ei)
            trm[i] = tri + dx; trm[j] = trj - dx; fpm = distance(CC, ri, trm, ei)
            trm[i] = tri - dx; trm[j] = trj + dx; fmp = distance(CC, ri, trm, ei)
            trm[i] = tri - dx; trm[j] = trj - dx; fmm = distance(CC, ri, trm, ei)
            trm[i], trm[j] = tri, trj
            (fpp - fpm - fmp + fmm) / (4*dx*dx)
        else
            T(0)
        end
    end
    Hu = SMatrix{6,6}((hessij(i, j) for i in 1:6, j in 1:6)...)
    Hu + Hu'
end

end

# function distance(co1::BulletCollisionObjectPtr, co2::BulletCollisionObjectPtr)  # co1 and co2 must be convex
#     result = icxx"""
#     btVoronoiSimplexSolver sGjkSimplexSolver;
#     sGjkSimplexSolver.setEqualVertexThreshold(0.f);
#     btGjkPairDetector convexConvex((btConvexShape*)($co1->getCollisionShape()),
#                                    (btConvexShape*)($co2->getCollisionShape()),
#                                    &sGjkSimplexSolver, 0);
#     btGjkPairDetector::ClosestPointInput input;
#     btPointCollector output;
#     input.m_transformA = $co1->getWorldTransform();
#     input.m_transformB = $co2->getWorldTransform();

#     convexConvex.getClosestPoints(input, output, 0);
#     btScalar* result = new btScalar[7];  // very hacky; must be a better way pass values from Cxx to julia
#     btVector3 pt1, pt2;
#     if (output.m_hasResult) {
#         result[0] = output.m_distance;
#         pt1 = output.m_pointInWorld + output.m_normalOnBInWorld*output.m_distance;
#         pt2 = output.m_pointInWorld;
#     } else {
#         result[0] = -btScalar(BT_LARGE_FLOAT);
#         pt1.setZero();
#         pt2.setZero();
#     }
#     result[1] = pt1.x();
#     result[2] = pt1.y();
#     result[3] = pt1.z();
#     result[4] = pt2.x();
#     result[5] = pt2.y();
#     result[6] = pt2.z();
#     result;
#     """::Ptr{BulletScalar}
#     d = icxx"""btScalar f = ($result)[0]; f;"""::BulletScalar
#     p1 = Vec(icxx"""btScalar f = ($result)[1]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[2]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[3]; f;"""::BulletScalar)
#     p2 = Vec(icxx"""btScalar f = ($result)[4]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[5]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[6]; f;"""::BulletScalar)
#     icxx"""delete [] $result;"""
#     d, p1, p2
# end

# function distance(co1::BulletCollisionObjectPtr, co2::BulletCollisionObjectPtr, W)  # co1 and co2 must be convex
#     U = chol(W)
#     result = icxx"""
#     btMatrix3x3 U($(U[1,1]), $(U[1,2]), $(U[1,3]),
#                   $(U[2,1]), $(U[2,2]), $(U[2,3]),
#                   $(U[3,1]), $(U[3,2]), $(U[3,3]));
#     btVoronoiSimplexSolver sGjkSimplexSolver;
#     sGjkSimplexSolver.setEqualVertexThreshold(0.f);
#     btWeightedGjkPairDetector convexConvex((btConvexShape*)($co1->getCollisionShape()),
#                                            (btConvexShape*)($co2->getCollisionShape()),
#                                            &sGjkSimplexSolver, U);
#     btGjkPairDetector::ClosestPointInput input;
#     btPointCollector output;
#     input.m_transformA = $co1->getWorldTransform();
#     input.m_transformB = $co2->getWorldTransform();

#     convexConvex.getClosestPoints(input, output, 0);
#     btScalar* result = new btScalar[7];  // very hacky; must be a better way pass values from Cxx to julia
#     btVector3 pt1, pt2;
#     if (output.m_hasResult) {
#         result[0] = output.m_distance;
#         btVector3 Unormal = U * output.m_normalOnBInWorld;
#         pt1 = output.m_pointInWorld + output.m_normalOnBInWorld * (output.m_distance / Unormal.length());
#         pt2 = output.m_pointInWorld;
#     } else {
#         result[0] = -btScalar(BT_LARGE_FLOAT);
#         pt1.setZero();
#         pt2.setZero();
#     }
#     result[1] = pt1.x();
#     result[2] = pt1.y();
#     result[3] = pt1.z();
#     result[4] = pt2.x();
#     result[5] = pt2.y();
#     result[6] = pt2.z();
#     result;
#     """::Ptr{BulletScalar}
#     d = icxx"""btScalar f = ($result)[0]; f;"""::BulletScalar
#     p1 = Vec(icxx"""btScalar f = ($result)[1]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[2]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[3]; f;"""::BulletScalar)
#     p2 = Vec(icxx"""btScalar f = ($result)[4]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[5]; f;"""::BulletScalar,
#              icxx"""btScalar f = ($result)[6]; f;"""::BulletScalar)
#     icxx"""delete [] $result;"""
#     d, p1, p2
# end