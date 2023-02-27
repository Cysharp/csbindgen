// auto-generated via csbindgen

#[allow(unused)]
use ::std::os::raw::*;




#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConnectSharedMemory(
    key:  c_int    
) ->  b3PhysicsClientHandle
{
    unsafe {
        return b3ConnectSharedMemory(
            key
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConnectSharedMemory2(
    key:  c_int    
) ->  b3PhysicsClientHandle
{
    unsafe {
        return b3ConnectSharedMemory2(
            key
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConnectPhysicsDirect(
    
) ->  b3PhysicsClientHandle
{
    unsafe {
        return b3ConnectPhysicsDirect(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3DisconnectSharedMemory(
    physClient:  b3PhysicsClientHandle    
)
{
    unsafe {
        return b3DisconnectSharedMemory(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CanSubmitCommand(
    physClient:  b3PhysicsClientHandle    
) ->  c_int
{
    unsafe {
        return b3CanSubmitCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SubmitClientCommandAndWaitStatus(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryStatusHandle
{
    unsafe {
        return b3SubmitClientCommandAndWaitStatus(
            physClient,
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SubmitClientCommand(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  c_int
{
    unsafe {
        return b3SubmitClientCommand(
            physClient,
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ProcessServerStatus(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryStatusHandle
{
    unsafe {
        return b3ProcessServerStatus(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusType(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusType(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCustomCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateCustomCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CustomCommandLoadPlugin(
    commandHandle:  b3SharedMemoryCommandHandle,
    pluginPath: *const c_char    
)
{
    unsafe {
        return b3CustomCommandLoadPlugin(
            commandHandle,
            pluginPath
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CustomCommandLoadPluginSetPostFix(
    commandHandle:  b3SharedMemoryCommandHandle,
    postFix: *const c_char    
)
{
    unsafe {
        return b3CustomCommandLoadPluginSetPostFix(
            commandHandle,
            postFix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusPluginUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusPluginUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusPluginCommandResult(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusPluginCommandResult(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusPluginCommandReturnData(
    physClient:  b3PhysicsClientHandle,
    valueOut: *mut b3UserDataValue    
) ->  c_int
{
    unsafe {
        return b3GetStatusPluginCommandReturnData(
            physClient,
            valueOut
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CustomCommandUnloadPlugin(
    commandHandle:  b3SharedMemoryCommandHandle,
    pluginUniqueId:  c_int    
)
{
    unsafe {
        return b3CustomCommandUnloadPlugin(
            commandHandle,
            pluginUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CustomCommandExecutePluginCommand(
    commandHandle:  b3SharedMemoryCommandHandle,
    pluginUniqueId:  c_int,
    textArguments: *const c_char    
)
{
    unsafe {
        return b3CustomCommandExecutePluginCommand(
            commandHandle,
            pluginUniqueId,
            textArguments
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CustomCommandExecuteAddIntArgument(
    commandHandle:  b3SharedMemoryCommandHandle,
    intVal:  c_int    
)
{
    unsafe {
        return b3CustomCommandExecuteAddIntArgument(
            commandHandle,
            intVal
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CustomCommandExecuteAddFloatArgument(
    commandHandle:  b3SharedMemoryCommandHandle,
    floatVal:  f32    
)
{
    unsafe {
        return b3CustomCommandExecuteAddFloatArgument(
            commandHandle,
            floatVal
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusBodyIndices(
    statusHandle:  b3SharedMemoryStatusHandle,
    bodyIndicesOut: *mut c_int,
    bodyIndicesCapacity:  c_int    
) ->  c_int
{
    unsafe {
        return b3GetStatusBodyIndices(
            statusHandle,
            bodyIndicesOut,
            bodyIndicesCapacity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusBodyIndex(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusBodyIndex(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusActualState(
    statusHandle:  b3SharedMemoryStatusHandle,
    bodyUniqueId: *mut c_int,
    numDegreeOfFreedomQ: *mut c_int,
    numDegreeOfFreedomU: *mut c_int,
    rootLocalInertialFrame: *mut *mut f64,
    actualStateQ: *mut *mut f64,
    actualStateQdot: *mut *mut f64,
    jointReactionForces: *mut *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusActualState(
            statusHandle,
            bodyUniqueId,
            numDegreeOfFreedomQ,
            numDegreeOfFreedomU,
            rootLocalInertialFrame,
            actualStateQ,
            actualStateQdot,
            jointReactionForces
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusActualState2(
    statusHandle:  b3SharedMemoryStatusHandle,
    bodyUniqueId: *mut c_int,
    numLinks: *mut c_int,
    numDegreeOfFreedomQ: *mut c_int,
    numDegreeOfFreedomU: *mut c_int,
    rootLocalInertialFrame: *mut *mut f64,
    actualStateQ: *mut *mut f64,
    actualStateQdot: *mut *mut f64,
    jointReactionForces: *mut *mut f64,
    linkLocalInertialFrames: *mut *mut f64,
    jointMotorForces: *mut *mut f64,
    linkStates: *mut *mut f64,
    linkWorldVelocities: *mut *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusActualState2(
            statusHandle,
            bodyUniqueId,
            numLinks,
            numDegreeOfFreedomQ,
            numDegreeOfFreedomU,
            rootLocalInertialFrame,
            actualStateQ,
            actualStateQdot,
            jointReactionForces,
            linkLocalInertialFrames,
            jointMotorForces,
            linkStates,
            linkWorldVelocities
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCollisionInfoCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestCollisionInfoCommandInit(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusAABB(
    statusHandle:  b3SharedMemoryStatusHandle,
    linkIndex:  c_int,
    aabbMin: *mut f64,
    aabbMax: *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusAABB(
            statusHandle,
            linkIndex,
            aabbMin,
            aabbMax
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitSyncBodyInfoCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitSyncBodyInfoCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestBodyInfoCommand(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestBodyInfoCommand(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRemoveBodyCommand(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRemoveBodyCommand(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetNumBodies(
    physClient:  b3PhysicsClientHandle    
) ->  c_int
{
    unsafe {
        return b3GetNumBodies(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetBodyUniqueId(
    physClient:  b3PhysicsClientHandle,
    serialIndex:  c_int    
) ->  c_int
{
    unsafe {
        return b3GetBodyUniqueId(
            physClient,
            serialIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetBodyInfo(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    info: *mut b3BodyInfo    
) ->  c_int
{
    unsafe {
        return b3GetBodyInfo(
            physClient,
            bodyUniqueId,
            info
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetNumJoints(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3GetNumJoints(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetNumDofs(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3GetNumDofs(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ComputeDofCount(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3ComputeDofCount(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetJointInfo(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    jointIndex:  c_int,
    info: *mut b3JointInfo    
) ->  c_int
{
    unsafe {
        return b3GetJointInfo(
            physClient,
            bodyUniqueId,
            jointIndex,
            info
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitSyncUserDataCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitSyncUserDataCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3AddBodyToSyncUserDataRequest(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int    
)
{
    unsafe {
        return b3AddBodyToSyncUserDataRequest(
            commandHandle,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitAddUserDataCommand(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    visualShapeIndex:  c_int,
    key: *const c_char,
    valueType:  UserDataValueType,
    valueLength:  c_int,
    valueData: *const c_void    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitAddUserDataCommand(
            physClient,
            bodyUniqueId,
            linkIndex,
            visualShapeIndex,
            key,
            valueType,
            valueLength,
            valueData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRemoveUserDataCommand(
    physClient:  b3PhysicsClientHandle,
    userDataId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRemoveUserDataCommand(
            physClient,
            userDataId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetUserData(
    physClient:  b3PhysicsClientHandle,
    userDataId:  c_int,
    valueOut: *mut b3UserDataValue    
) ->  c_int
{
    unsafe {
        return b3GetUserData(
            physClient,
            userDataId,
            valueOut
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetUserDataId(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    visualShapeIndex:  c_int,
    key: *const c_char    
) ->  c_int
{
    unsafe {
        return b3GetUserDataId(
            physClient,
            bodyUniqueId,
            linkIndex,
            visualShapeIndex,
            key
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetUserDataIdFromStatus(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetUserDataIdFromStatus(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetNumUserData(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3GetNumUserData(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetUserDataInfo(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    userDataIndex:  c_int,
    keyOut: *mut *mut c_char,
    userDataIdOut: *mut c_int,
    linkIndexOut: *mut c_int,
    visualShapeIndexOut: *mut c_int    
)
{
    unsafe {
        return b3GetUserDataInfo(
            physClient,
            bodyUniqueId,
            userDataIndex,
            keyOut,
            userDataIdOut,
            linkIndexOut,
            visualShapeIndexOut
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetDynamicsInfoCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3GetDynamicsInfoCommandInit(
            physClient,
            bodyUniqueId,
            linkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetDynamicsInfoCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3GetDynamicsInfoCommandInit2(
            commandHandle,
            bodyUniqueId,
            linkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetDynamicsInfo(
    statusHandle:  b3SharedMemoryStatusHandle,
    info: *mut b3DynamicsInfo    
) ->  c_int
{
    unsafe {
        return b3GetDynamicsInfo(
            statusHandle,
            info
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeDynamicsInfo(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitChangeDynamicsInfo(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeDynamicsInfo2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitChangeDynamicsInfo2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetMass(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    mass:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetMass(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            mass
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetLocalInertiaDiagonal(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    localInertiaDiagonal: *const f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetLocalInertiaDiagonal(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            localInertiaDiagonal
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetAnisotropicFriction(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    anisotropicFriction: *const f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetAnisotropicFriction(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            anisotropicFriction
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetJointLimit(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    jointLowerLimit:  f64,
    jointUpperLimit:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetJointLimit(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            jointLowerLimit,
            jointUpperLimit
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetJointLimitForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    jointLimitForce:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetJointLimitForce(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            jointLimitForce
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetDynamicType(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    dynamicType:  c_int    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetDynamicType(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            dynamicType
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetSleepThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    sleepThreshold:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetSleepThreshold(
            commandHandle,
            bodyUniqueId,
            sleepThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetLateralFriction(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    lateralFriction:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetLateralFriction(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            lateralFriction
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetSpinningFriction(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    friction:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetSpinningFriction(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            friction
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetRollingFriction(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    friction:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetRollingFriction(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            friction
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetRestitution(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    restitution:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetRestitution(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            restitution
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetLinearDamping(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linearDamping:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetLinearDamping(
            commandHandle,
            bodyUniqueId,
            linearDamping
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetAngularDamping(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    angularDamping:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetAngularDamping(
            commandHandle,
            bodyUniqueId,
            angularDamping
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetJointDamping(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    jointDamping:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetJointDamping(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            jointDamping
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetContactStiffnessAndDamping(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    contactStiffness:  f64,
    contactDamping:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetContactStiffnessAndDamping(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            contactStiffness,
            contactDamping
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetFrictionAnchor(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    frictionAnchor:  c_int    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetFrictionAnchor(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            frictionAnchor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetCcdSweptSphereRadius(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    ccdSweptSphereRadius:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetCcdSweptSphereRadius(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            ccdSweptSphereRadius
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetContactProcessingThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    contactProcessingThreshold:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetContactProcessingThreshold(
            commandHandle,
            bodyUniqueId,
            linkIndex,
            contactProcessingThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetActivationState(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    activationState:  c_int    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetActivationState(
            commandHandle,
            bodyUniqueId,
            activationState
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetMaxJointVelocity(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    maxJointVelocity:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetMaxJointVelocity(
            commandHandle,
            bodyUniqueId,
            maxJointVelocity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ChangeDynamicsInfoSetCollisionMargin(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    collisionMargin:  f64    
) ->  c_int
{
    unsafe {
        return b3ChangeDynamicsInfoSetCollisionMargin(
            commandHandle,
            bodyUniqueId,
            collisionMargin
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitCreateUserConstraintCommand(
    physClient:  b3PhysicsClientHandle,
    parentBodyUniqueId:  c_int,
    parentJointIndex:  c_int,
    childBodyUniqueId:  c_int,
    childJointIndex:  c_int,
    info: *mut b3JointInfo    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitCreateUserConstraintCommand(
            physClient,
            parentBodyUniqueId,
            parentJointIndex,
            childBodyUniqueId,
            childJointIndex,
            info
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitCreateUserConstraintCommand2(
    commandHandle:  b3SharedMemoryCommandHandle,
    parentBodyUniqueId:  c_int,
    parentJointIndex:  c_int,
    childBodyUniqueId:  c_int,
    childJointIndex:  c_int,
    info: *mut b3JointInfo    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitCreateUserConstraintCommand2(
            commandHandle,
            parentBodyUniqueId,
            parentJointIndex,
            childBodyUniqueId,
            childJointIndex,
            info
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusUserConstraintUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusUserConstraintUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintCommand(
    physClient:  b3PhysicsClientHandle,
    userConstraintUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitChangeUserConstraintCommand(
            physClient,
            userConstraintUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetPivotInB(
    commandHandle:  b3SharedMemoryCommandHandle,
    jointChildPivot: *const f64    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetPivotInB(
            commandHandle,
            jointChildPivot
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetFrameInB(
    commandHandle:  b3SharedMemoryCommandHandle,
    jointChildFrameOrn: *const f64    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetFrameInB(
            commandHandle,
            jointChildFrameOrn
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetMaxForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    maxAppliedForce:  f64    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetMaxForce(
            commandHandle,
            maxAppliedForce
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetGearRatio(
    commandHandle:  b3SharedMemoryCommandHandle,
    gearRatio:  f64    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetGearRatio(
            commandHandle,
            gearRatio
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetGearAuxLink(
    commandHandle:  b3SharedMemoryCommandHandle,
    gearAuxLink:  c_int    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetGearAuxLink(
            commandHandle,
            gearAuxLink
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetRelativePositionTarget(
    commandHandle:  b3SharedMemoryCommandHandle,
    relativePositionTarget:  f64    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetRelativePositionTarget(
            commandHandle,
            relativePositionTarget
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitChangeUserConstraintSetERP(
    commandHandle:  b3SharedMemoryCommandHandle,
    erp:  f64    
) ->  c_int
{
    unsafe {
        return b3InitChangeUserConstraintSetERP(
            commandHandle,
            erp
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRemoveUserConstraintCommand(
    physClient:  b3PhysicsClientHandle,
    userConstraintUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRemoveUserConstraintCommand(
            physClient,
            userConstraintUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetNumUserConstraints(
    physClient:  b3PhysicsClientHandle    
) ->  c_int
{
    unsafe {
        return b3GetNumUserConstraints(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitGetUserConstraintStateCommand(
    physClient:  b3PhysicsClientHandle,
    constraintUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitGetUserConstraintStateCommand(
            physClient,
            constraintUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusUserConstraintState(
    statusHandle:  b3SharedMemoryStatusHandle,
    constraintState: *mut b3UserConstraintState    
) ->  c_int
{
    unsafe {
        return b3GetStatusUserConstraintState(
            statusHandle,
            constraintState
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetUserConstraintInfo(
    physClient:  b3PhysicsClientHandle,
    constraintUniqueId:  c_int,
    info: *mut b3UserConstraint    
) ->  c_int
{
    unsafe {
        return b3GetUserConstraintInfo(
            physClient,
            constraintUniqueId,
            info
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetUserConstraintId(
    physClient:  b3PhysicsClientHandle,
    serialIndex:  c_int    
) ->  c_int
{
    unsafe {
        return b3GetUserConstraintId(
            physClient,
            serialIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestDebugLinesCommand(
    physClient:  b3PhysicsClientHandle,
    debugMode:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestDebugLinesCommand(
            physClient,
            debugMode
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetDebugLines(
    physClient:  b3PhysicsClientHandle,
    lines: *mut b3DebugLines    
)
{
    unsafe {
        return b3GetDebugLines(
            physClient,
            lines
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitConfigureOpenGLVisualizer(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitConfigureOpenGLVisualizer(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitConfigureOpenGLVisualizer2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitConfigureOpenGLVisualizer2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetVisualizationFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flag:  c_int,
    enabled:  c_int    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetVisualizationFlags(
            commandHandle,
            flag,
            enabled
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetLightPosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightPosition: *const f32    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetLightPosition(
            commandHandle,
            lightPosition
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetShadowMapResolution(
    commandHandle:  b3SharedMemoryCommandHandle,
    shadowMapResolution:  c_int    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetShadowMapResolution(
            commandHandle,
            shadowMapResolution
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetShadowMapIntensity(
    commandHandle:  b3SharedMemoryCommandHandle,
    shadowMapIntensity:  f64    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetShadowMapIntensity(
            commandHandle,
            shadowMapIntensity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetLightRgbBackground(
    commandHandle:  b3SharedMemoryCommandHandle,
    rgbBackground: *const f32    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetLightRgbBackground(
            commandHandle,
            rgbBackground
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetShadowMapWorldSize(
    commandHandle:  b3SharedMemoryCommandHandle,
    shadowMapWorldSize:  c_int    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetShadowMapWorldSize(
            commandHandle,
            shadowMapWorldSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetRemoteSyncTransformInterval(
    commandHandle:  b3SharedMemoryCommandHandle,
    remoteSyncTransformInterval:  f64    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetRemoteSyncTransformInterval(
            commandHandle,
            remoteSyncTransformInterval
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ConfigureOpenGLVisualizerSetViewMatrix(
    commandHandle:  b3SharedMemoryCommandHandle,
    cameraDistance:  f32,
    cameraPitch:  f32,
    cameraYaw:  f32,
    cameraTargetPosition: *const f32    
)
{
    unsafe {
        return b3ConfigureOpenGLVisualizerSetViewMatrix(
            commandHandle,
            cameraDistance,
            cameraPitch,
            cameraYaw,
            cameraTargetPosition
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestOpenGLVisualizerCameraCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestOpenGLVisualizerCameraCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusOpenGLVisualizerCamera(
    statusHandle:  b3SharedMemoryStatusHandle,
    camera: *mut b3OpenGLVisualizerCameraInfo    
) ->  c_int
{
    unsafe {
        return b3GetStatusOpenGLVisualizerCamera(
            statusHandle,
            camera
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugDrawAddLine3D(
    physClient:  b3PhysicsClientHandle,
    fromXYZ: *const f64,
    toXYZ: *const f64,
    colorRGB: *const f64,
    lineWidth:  f64,
    lifeTime:  f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugDrawAddLine3D(
            physClient,
            fromXYZ,
            toXYZ,
            colorRGB,
            lineWidth,
            lifeTime
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugDrawAddPoints3D(
    physClient:  b3PhysicsClientHandle,
    positionsXYZ: *const f64,
    colorsRGB: *const f64,
    pointSize:  f64,
    lifeTime:  f64,
    pointNum:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugDrawAddPoints3D(
            physClient,
            positionsXYZ,
            colorsRGB,
            pointSize,
            lifeTime,
            pointNum
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugDrawAddText3D(
    physClient:  b3PhysicsClientHandle,
    txt: *const c_char,
    positionXYZ: *const f64,
    colorRGB: *const f64,
    textSize:  f64,
    lifeTime:  f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugDrawAddText3D(
            physClient,
            txt,
            positionXYZ,
            colorRGB,
            textSize,
            lifeTime
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UserDebugTextSetOptionFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    optionFlags:  c_int    
)
{
    unsafe {
        return b3UserDebugTextSetOptionFlags(
            commandHandle,
            optionFlags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UserDebugTextSetOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    orientation: *const f64    
)
{
    unsafe {
        return b3UserDebugTextSetOrientation(
            commandHandle,
            orientation
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UserDebugItemSetReplaceItemUniqueId(
    commandHandle:  b3SharedMemoryCommandHandle,
    replaceItem:  c_int    
)
{
    unsafe {
        return b3UserDebugItemSetReplaceItemUniqueId(
            commandHandle,
            replaceItem
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UserDebugItemSetParentObject(
    commandHandle:  b3SharedMemoryCommandHandle,
    objectUniqueId:  c_int,
    linkIndex:  c_int    
)
{
    unsafe {
        return b3UserDebugItemSetParentObject(
            commandHandle,
            objectUniqueId,
            linkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugAddParameter(
    physClient:  b3PhysicsClientHandle,
    txt: *const c_char,
    rangeMin:  f64,
    rangeMax:  f64,
    startValue:  f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugAddParameter(
            physClient,
            txt,
            rangeMin,
            rangeMax,
            startValue
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugReadParameter(
    physClient:  b3PhysicsClientHandle,
    debugItemUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugReadParameter(
            physClient,
            debugItemUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusDebugParameterValue(
    statusHandle:  b3SharedMemoryStatusHandle,
    paramValue: *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusDebugParameterValue(
            statusHandle,
            paramValue
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugDrawRemove(
    physClient:  b3PhysicsClientHandle,
    debugItemUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugDrawRemove(
            physClient,
            debugItemUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserDebugDrawRemoveAll(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserDebugDrawRemoveAll(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUserRemoveAllParameters(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUserRemoveAllParameters(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitDebugDrawingCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitDebugDrawingCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetDebugObjectColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    objectUniqueId:  c_int,
    linkIndex:  c_int,
    objectColorRGB: *const f64    
)
{
    unsafe {
        return b3SetDebugObjectColor(
            commandHandle,
            objectUniqueId,
            linkIndex,
            objectColorRGB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RemoveDebugObjectColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    objectUniqueId:  c_int,
    linkIndex:  c_int    
)
{
    unsafe {
        return b3RemoveDebugObjectColor(
            commandHandle,
            objectUniqueId,
            linkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetDebugItemUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetDebugItemUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestCameraImage(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestCameraImage(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestCameraImage2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestCameraImage2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetCameraMatrices(
    commandHandle:  b3SharedMemoryCommandHandle,
    viewMatrix: *mut f32,
    projectionMatrix: *mut f32    
)
{
    unsafe {
        return b3RequestCameraImageSetCameraMatrices(
            commandHandle,
            viewMatrix,
            projectionMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetPixelResolution(
    commandHandle:  b3SharedMemoryCommandHandle,
    width:  c_int,
    height:  c_int    
)
{
    unsafe {
        return b3RequestCameraImageSetPixelResolution(
            commandHandle,
            width,
            height
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetLightDirection(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightDirection: *const f32    
)
{
    unsafe {
        return b3RequestCameraImageSetLightDirection(
            commandHandle,
            lightDirection
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetLightColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightColor: *const f32    
)
{
    unsafe {
        return b3RequestCameraImageSetLightColor(
            commandHandle,
            lightColor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetLightDistance(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightDistance:  f32    
)
{
    unsafe {
        return b3RequestCameraImageSetLightDistance(
            commandHandle,
            lightDistance
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetLightAmbientCoeff(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightAmbientCoeff:  f32    
)
{
    unsafe {
        return b3RequestCameraImageSetLightAmbientCoeff(
            commandHandle,
            lightAmbientCoeff
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetLightDiffuseCoeff(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightDiffuseCoeff:  f32    
)
{
    unsafe {
        return b3RequestCameraImageSetLightDiffuseCoeff(
            commandHandle,
            lightDiffuseCoeff
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetLightSpecularCoeff(
    commandHandle:  b3SharedMemoryCommandHandle,
    lightSpecularCoeff:  f32    
)
{
    unsafe {
        return b3RequestCameraImageSetLightSpecularCoeff(
            commandHandle,
            lightSpecularCoeff
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetShadow(
    commandHandle:  b3SharedMemoryCommandHandle,
    hasShadow:  c_int    
)
{
    unsafe {
        return b3RequestCameraImageSetShadow(
            commandHandle,
            hasShadow
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSelectRenderer(
    commandHandle:  b3SharedMemoryCommandHandle,
    renderer:  c_int    
)
{
    unsafe {
        return b3RequestCameraImageSelectRenderer(
            commandHandle,
            renderer
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3RequestCameraImageSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetCameraImageData(
    physClient:  b3PhysicsClientHandle,
    imageData: *mut b3CameraImageData    
)
{
    unsafe {
        return b3GetCameraImageData(
            physClient,
            imageData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetProjectiveTextureMatrices(
    commandHandle:  b3SharedMemoryCommandHandle,
    viewMatrix: *mut f32,
    projectionMatrix: *mut f32    
)
{
    unsafe {
        return b3RequestCameraImageSetProjectiveTextureMatrices(
            commandHandle,
            viewMatrix,
            projectionMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ComputeViewMatrixFromPositions(
    cameraPosition: *const f32,
    cameraTargetPosition: *const f32,
    cameraUp: *const f32,
    viewMatrix: *mut f32    
)
{
    unsafe {
        return b3ComputeViewMatrixFromPositions(
            cameraPosition,
            cameraTargetPosition,
            cameraUp,
            viewMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ComputeViewMatrixFromYawPitchRoll(
    cameraTargetPosition: *const f32,
    distance:  f32,
    yaw:  f32,
    pitch:  f32,
    roll:  f32,
    upAxis:  c_int,
    viewMatrix: *mut f32    
)
{
    unsafe {
        return b3ComputeViewMatrixFromYawPitchRoll(
            cameraTargetPosition,
            distance,
            yaw,
            pitch,
            roll,
            upAxis,
            viewMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ComputePositionFromViewMatrix(
    viewMatrix: *const f32,
    cameraPosition: *mut f32,
    cameraTargetPosition: *mut f32,
    cameraUp: *mut f32    
)
{
    unsafe {
        return b3ComputePositionFromViewMatrix(
            viewMatrix,
            cameraPosition,
            cameraTargetPosition,
            cameraUp
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ComputeProjectionMatrix(
    left:  f32,
    right:  f32,
    bottom:  f32,
    top:  f32,
    nearVal:  f32,
    farVal:  f32,
    projectionMatrix: *mut f32    
)
{
    unsafe {
        return b3ComputeProjectionMatrix(
            left,
            right,
            bottom,
            top,
            nearVal,
            farVal,
            projectionMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ComputeProjectionMatrixFOV(
    fov:  f32,
    aspect:  f32,
    nearVal:  f32,
    farVal:  f32,
    projectionMatrix: *mut f32    
)
{
    unsafe {
        return b3ComputeProjectionMatrixFOV(
            fov,
            aspect,
            nearVal,
            farVal,
            projectionMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetViewMatrix(
    commandHandle:  b3SharedMemoryCommandHandle,
    cameraPosition: *const f32,
    cameraTargetPosition: *const f32,
    cameraUp: *const f32    
)
{
    unsafe {
        return b3RequestCameraImageSetViewMatrix(
            commandHandle,
            cameraPosition,
            cameraTargetPosition,
            cameraUp
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetViewMatrix2(
    commandHandle:  b3SharedMemoryCommandHandle,
    cameraTargetPosition: *const f32,
    distance:  f32,
    yaw:  f32,
    pitch:  f32,
    roll:  f32,
    upAxis:  c_int    
)
{
    unsafe {
        return b3RequestCameraImageSetViewMatrix2(
            commandHandle,
            cameraTargetPosition,
            distance,
            yaw,
            pitch,
            roll,
            upAxis
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetProjectionMatrix(
    commandHandle:  b3SharedMemoryCommandHandle,
    left:  f32,
    right:  f32,
    bottom:  f32,
    top:  f32,
    nearVal:  f32,
    farVal:  f32    
)
{
    unsafe {
        return b3RequestCameraImageSetProjectionMatrix(
            commandHandle,
            left,
            right,
            bottom,
            top,
            nearVal,
            farVal
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestCameraImageSetFOVProjectionMatrix(
    commandHandle:  b3SharedMemoryCommandHandle,
    fov:  f32,
    aspect:  f32,
    nearVal:  f32,
    farVal:  f32    
)
{
    unsafe {
        return b3RequestCameraImageSetFOVProjectionMatrix(
            commandHandle,
            fov,
            aspect,
            nearVal,
            farVal
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestContactPointInformation(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestContactPointInformation(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetContactFilterBodyA(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueIdA:  c_int    
)
{
    unsafe {
        return b3SetContactFilterBodyA(
            commandHandle,
            bodyUniqueIdA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetContactFilterBodyB(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueIdB:  c_int    
)
{
    unsafe {
        return b3SetContactFilterBodyB(
            commandHandle,
            bodyUniqueIdB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetContactFilterLinkA(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndexA:  c_int    
)
{
    unsafe {
        return b3SetContactFilterLinkA(
            commandHandle,
            linkIndexA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetContactFilterLinkB(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndexB:  c_int    
)
{
    unsafe {
        return b3SetContactFilterLinkB(
            commandHandle,
            linkIndexB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetContactPointInformation(
    physClient:  b3PhysicsClientHandle,
    contactPointData: *mut b3ContactInformation    
)
{
    unsafe {
        return b3GetContactPointInformation(
            physClient,
            contactPointData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitClosestDistanceQuery(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitClosestDistanceQuery(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterBodyA(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueIdA:  c_int    
)
{
    unsafe {
        return b3SetClosestDistanceFilterBodyA(
            commandHandle,
            bodyUniqueIdA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterLinkA(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndexA:  c_int    
)
{
    unsafe {
        return b3SetClosestDistanceFilterLinkA(
            commandHandle,
            linkIndexA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterBodyB(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueIdB:  c_int    
)
{
    unsafe {
        return b3SetClosestDistanceFilterBodyB(
            commandHandle,
            bodyUniqueIdB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterLinkB(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndexB:  c_int    
)
{
    unsafe {
        return b3SetClosestDistanceFilterLinkB(
            commandHandle,
            linkIndexB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    distance:  f64    
)
{
    unsafe {
        return b3SetClosestDistanceThreshold(
            commandHandle,
            distance
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterCollisionShapeA(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapeA:  c_int    
)
{
    unsafe {
        return b3SetClosestDistanceFilterCollisionShapeA(
            commandHandle,
            collisionShapeA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterCollisionShapeB(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapeB:  c_int    
)
{
    unsafe {
        return b3SetClosestDistanceFilterCollisionShapeB(
            commandHandle,
            collisionShapeB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterCollisionShapePositionA(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapePositionA: *const f64    
)
{
    unsafe {
        return b3SetClosestDistanceFilterCollisionShapePositionA(
            commandHandle,
            collisionShapePositionA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterCollisionShapePositionB(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapePositionB: *const f64    
)
{
    unsafe {
        return b3SetClosestDistanceFilterCollisionShapePositionB(
            commandHandle,
            collisionShapePositionB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterCollisionShapeOrientationA(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapeOrientationA: *const f64    
)
{
    unsafe {
        return b3SetClosestDistanceFilterCollisionShapeOrientationA(
            commandHandle,
            collisionShapeOrientationA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetClosestDistanceFilterCollisionShapeOrientationB(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapeOrientationB: *const f64    
)
{
    unsafe {
        return b3SetClosestDistanceFilterCollisionShapeOrientationB(
            commandHandle,
            collisionShapeOrientationB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetClosestPointInformation(
    physClient:  b3PhysicsClientHandle,
    contactPointInfo: *mut b3ContactInformation    
)
{
    unsafe {
        return b3GetClosestPointInformation(
            physClient,
            contactPointInfo
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitAABBOverlapQuery(
    physClient:  b3PhysicsClientHandle,
    aabbMin: *const f64,
    aabbMax: *const f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitAABBOverlapQuery(
            physClient,
            aabbMin,
            aabbMax
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetAABBOverlapResults(
    physClient:  b3PhysicsClientHandle,
    data: *mut b3AABBOverlapData    
)
{
    unsafe {
        return b3GetAABBOverlapResults(
            physClient,
            data
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestVisualShapeInformation(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueIdA:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestVisualShapeInformation(
            physClient,
            bodyUniqueIdA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetVisualShapeInformation(
    physClient:  b3PhysicsClientHandle,
    visualShapeInfo: *mut b3VisualShapeInformation    
)
{
    unsafe {
        return b3GetVisualShapeInformation(
            physClient,
            visualShapeInfo
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestCollisionShapeInformation(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestCollisionShapeInformation(
            physClient,
            bodyUniqueId,
            linkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetCollisionShapeInformation(
    physClient:  b3PhysicsClientHandle,
    collisionShapeInfo: *mut b3CollisionShapeInformation    
)
{
    unsafe {
        return b3GetCollisionShapeInformation(
            physClient,
            collisionShapeInfo
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitLoadTexture(
    physClient:  b3PhysicsClientHandle,
    filename: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitLoadTexture(
            physClient,
            filename
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusTextureUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusTextureUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateChangeTextureCommandInit(
    physClient:  b3PhysicsClientHandle,
    textureUniqueId:  c_int,
    width:  c_int,
    height:  c_int,
    rgbPixels: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateChangeTextureCommandInit(
            physClient,
            textureUniqueId,
            width,
            height,
            rgbPixels
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUpdateVisualShape(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    jointIndex:  c_int,
    shapeIndex:  c_int,
    textureUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUpdateVisualShape(
            physClient,
            bodyUniqueId,
            jointIndex,
            shapeIndex,
            textureUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitUpdateVisualShape2(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    jointIndex:  c_int,
    shapeIndex:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitUpdateVisualShape2(
            physClient,
            bodyUniqueId,
            jointIndex,
            shapeIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UpdateVisualShapeTexture(
    commandHandle:  b3SharedMemoryCommandHandle,
    textureUniqueId:  c_int    
)
{
    unsafe {
        return b3UpdateVisualShapeTexture(
            commandHandle,
            textureUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UpdateVisualShapeRGBAColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    rgbaColor: *const f64    
)
{
    unsafe {
        return b3UpdateVisualShapeRGBAColor(
            commandHandle,
            rgbaColor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UpdateVisualShapeFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3UpdateVisualShapeFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3UpdateVisualShapeSpecularColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    specularColor: *const f64    
)
{
    unsafe {
        return b3UpdateVisualShapeSpecularColor(
            commandHandle,
            specularColor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitPhysicsParamCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitPhysicsParamCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitPhysicsParamCommand2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitPhysicsParamCommand2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetGravity(
    commandHandle:  b3SharedMemoryCommandHandle,
    gravx:  f64,
    gravy:  f64,
    gravz:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetGravity(
            commandHandle,
            gravx,
            gravy,
            gravz
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetTimeStep(
    commandHandle:  b3SharedMemoryCommandHandle,
    timeStep:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetTimeStep(
            commandHandle,
            timeStep
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetDefaultContactERP(
    commandHandle:  b3SharedMemoryCommandHandle,
    defaultContactERP:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetDefaultContactERP(
            commandHandle,
            defaultContactERP
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetDefaultNonContactERP(
    commandHandle:  b3SharedMemoryCommandHandle,
    defaultNonContactERP:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetDefaultNonContactERP(
            commandHandle,
            defaultNonContactERP
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetDefaultFrictionERP(
    commandHandle:  b3SharedMemoryCommandHandle,
    frictionERP:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetDefaultFrictionERP(
            commandHandle,
            frictionERP
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetDefaultGlobalCFM(
    commandHandle:  b3SharedMemoryCommandHandle,
    defaultGlobalCFM:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetDefaultGlobalCFM(
            commandHandle,
            defaultGlobalCFM
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetDefaultFrictionCFM(
    commandHandle:  b3SharedMemoryCommandHandle,
    frictionCFM:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetDefaultFrictionCFM(
            commandHandle,
            frictionCFM
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetNumSubSteps(
    commandHandle:  b3SharedMemoryCommandHandle,
    numSubSteps:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetNumSubSteps(
            commandHandle,
            numSubSteps
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetRealTimeSimulation(
    commandHandle:  b3SharedMemoryCommandHandle,
    enableRealTimeSimulation:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetRealTimeSimulation(
            commandHandle,
            enableRealTimeSimulation
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetNumSolverIterations(
    commandHandle:  b3SharedMemoryCommandHandle,
    numSolverIterations:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetNumSolverIterations(
            commandHandle,
            numSolverIterations
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetNumNonContactInnerIterations(
    commandHandle:  b3SharedMemoryCommandHandle,
    numMotorIterations:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetNumNonContactInnerIterations(
            commandHandle,
            numMotorIterations
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetWarmStartingFactor(
    commandHandle:  b3SharedMemoryCommandHandle,
    warmStartingFactor:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetWarmStartingFactor(
            commandHandle,
            warmStartingFactor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetArticulatedWarmStartingFactor(
    commandHandle:  b3SharedMemoryCommandHandle,
    warmStartingFactor:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetArticulatedWarmStartingFactor(
            commandHandle,
            warmStartingFactor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetCollisionFilterMode(
    commandHandle:  b3SharedMemoryCommandHandle,
    filterMode:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetCollisionFilterMode(
            commandHandle,
            filterMode
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetUseSplitImpulse(
    commandHandle:  b3SharedMemoryCommandHandle,
    useSplitImpulse:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetUseSplitImpulse(
            commandHandle,
            useSplitImpulse
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetSplitImpulsePenetrationThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    splitImpulsePenetrationThreshold:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetSplitImpulsePenetrationThreshold(
            commandHandle,
            splitImpulsePenetrationThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetContactBreakingThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    contactBreakingThreshold:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetContactBreakingThreshold(
            commandHandle,
            contactBreakingThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetMaxNumCommandsPer1ms(
    commandHandle:  b3SharedMemoryCommandHandle,
    maxNumCmdPer1ms:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetMaxNumCommandsPer1ms(
            commandHandle,
            maxNumCmdPer1ms
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetEnableFileCaching(
    commandHandle:  b3SharedMemoryCommandHandle,
    enableFileCaching:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetEnableFileCaching(
            commandHandle,
            enableFileCaching
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetRestitutionVelocityThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    restitutionVelocityThreshold:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetRestitutionVelocityThreshold(
            commandHandle,
            restitutionVelocityThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetEnableConeFriction(
    commandHandle:  b3SharedMemoryCommandHandle,
    enableConeFriction:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetEnableConeFriction(
            commandHandle,
            enableConeFriction
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetDeterministicOverlappingPairs(
    commandHandle:  b3SharedMemoryCommandHandle,
    deterministicOverlappingPairs:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetDeterministicOverlappingPairs(
            commandHandle,
            deterministicOverlappingPairs
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetAllowedCcdPenetration(
    commandHandle:  b3SharedMemoryCommandHandle,
    allowedCcdPenetration:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetAllowedCcdPenetration(
            commandHandle,
            allowedCcdPenetration
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetJointFeedbackMode(
    commandHandle:  b3SharedMemoryCommandHandle,
    jointFeedbackMode:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetJointFeedbackMode(
            commandHandle,
            jointFeedbackMode
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetSolverResidualThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    solverResidualThreshold:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetSolverResidualThreshold(
            commandHandle,
            solverResidualThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetContactSlop(
    commandHandle:  b3SharedMemoryCommandHandle,
    contactSlop:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetContactSlop(
            commandHandle,
            contactSlop
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetEnableSAT(
    commandHandle:  b3SharedMemoryCommandHandle,
    enableSAT:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetEnableSAT(
            commandHandle,
            enableSAT
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetConstraintSolverType(
    commandHandle:  b3SharedMemoryCommandHandle,
    constraintSolverType:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetConstraintSolverType(
            commandHandle,
            constraintSolverType
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetMinimumSolverIslandSize(
    commandHandle:  b3SharedMemoryCommandHandle,
    minimumSolverIslandSize:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetMinimumSolverIslandSize(
            commandHandle,
            minimumSolverIslandSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetSolverAnalytics(
    commandHandle:  b3SharedMemoryCommandHandle,
    reportSolverAnalytics:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetSolverAnalytics(
            commandHandle,
            reportSolverAnalytics
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParameterSetSparseSdfVoxelSize(
    commandHandle:  b3SharedMemoryCommandHandle,
    sparseSdfVoxelSize:  f64    
) ->  c_int
{
    unsafe {
        return b3PhysicsParameterSetSparseSdfVoxelSize(
            commandHandle,
            sparseSdfVoxelSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRequestPhysicsParamCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRequestPhysicsParamCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusPhysicsSimulationParameters(
    statusHandle:  b3SharedMemoryStatusHandle,
    params: *mut b3PhysicsSimulationParameters    
) ->  c_int
{
    unsafe {
        return b3GetStatusPhysicsSimulationParameters(
            statusHandle,
            params
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PhysicsParamSetInternalSimFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
) ->  c_int
{
    unsafe {
        return b3PhysicsParamSetInternalSimFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitStepSimulationCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitStepSimulationCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitStepSimulationCommand2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitStepSimulationCommand2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitPerformCollisionDetectionCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitPerformCollisionDetectionCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusForwardDynamicsAnalyticsData(
    statusHandle:  b3SharedMemoryStatusHandle,
    analyticsData: *mut b3ForwardDynamicsAnalyticsArgs    
) ->  c_int
{
    unsafe {
        return b3GetStatusForwardDynamicsAnalyticsData(
            statusHandle,
            analyticsData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitResetSimulationCommand(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitResetSimulationCommand(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitResetSimulationCommand2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitResetSimulationCommand2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitResetSimulationSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
) ->  c_int
{
    unsafe {
        return b3InitResetSimulationSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandInit(
    physClient:  b3PhysicsClientHandle,
    urdfFileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadUrdfCommandInit(
            physClient,
            urdfFileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle,
    urdfFileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadUrdfCommandInit2(
            commandHandle,
            urdfFileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandSetStartPosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    startPosX:  f64,
    startPosY:  f64,
    startPosZ:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadUrdfCommandSetStartPosition(
            commandHandle,
            startPosX,
            startPosY,
            startPosZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandSetStartOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    startOrnX:  f64,
    startOrnY:  f64,
    startOrnZ:  f64,
    startOrnW:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadUrdfCommandSetStartOrientation(
            commandHandle,
            startOrnX,
            startOrnY,
            startOrnZ,
            startOrnW
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandSetUseMultiBody(
    commandHandle:  b3SharedMemoryCommandHandle,
    useMultiBody:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadUrdfCommandSetUseMultiBody(
            commandHandle,
            useMultiBody
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandSetUseFixedBase(
    commandHandle:  b3SharedMemoryCommandHandle,
    useFixedBase:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadUrdfCommandSetUseFixedBase(
            commandHandle,
            useFixedBase
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadUrdfCommandSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadUrdfCommandSetGlobalScaling(
    commandHandle:  b3SharedMemoryCommandHandle,
    globalScaling:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadUrdfCommandSetGlobalScaling(
            commandHandle,
            globalScaling
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SaveStateCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3SaveStateCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRemoveStateCommand(
    physClient:  b3PhysicsClientHandle,
    stateId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRemoveStateCommand(
            physClient,
            stateId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusGetStateId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusGetStateId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadStateCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadStateCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadStateSetStateId(
    commandHandle:  b3SharedMemoryCommandHandle,
    stateId:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadStateSetStateId(
            commandHandle,
            stateId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadStateSetFileName(
    commandHandle:  b3SharedMemoryCommandHandle,
    fileName: *const c_char    
) ->  c_int
{
    unsafe {
        return b3LoadStateSetFileName(
            commandHandle,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadBulletCommandInit(
    physClient:  b3PhysicsClientHandle,
    fileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadBulletCommandInit(
            physClient,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SaveBulletCommandInit(
    physClient:  b3PhysicsClientHandle,
    fileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3SaveBulletCommandInit(
            physClient,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadMJCFCommandInit(
    physClient:  b3PhysicsClientHandle,
    fileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadMJCFCommandInit(
            physClient,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadMJCFCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle,
    fileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadMJCFCommandInit2(
            commandHandle,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadMJCFCommandSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3LoadMJCFCommandSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadMJCFCommandSetUseMultiBody(
    commandHandle:  b3SharedMemoryCommandHandle,
    useMultiBody:  c_int    
)
{
    unsafe {
        return b3LoadMJCFCommandSetUseMultiBody(
            commandHandle,
            useMultiBody
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseDynamicsCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    jointPositionsQ: *const f64,
    jointVelocitiesQdot: *const f64,
    jointAccelerations: *const f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CalculateInverseDynamicsCommandInit(
            physClient,
            bodyUniqueId,
            jointPositionsQ,
            jointVelocitiesQdot,
            jointAccelerations
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseDynamicsCommandInit2(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    jointPositionsQ: *const f64,
    dofCountQ:  c_int,
    jointVelocitiesQdot: *const f64,
    jointAccelerations: *const f64,
    dofCountQdot:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CalculateInverseDynamicsCommandInit2(
            physClient,
            bodyUniqueId,
            jointPositionsQ,
            dofCountQ,
            jointVelocitiesQdot,
            jointAccelerations,
            dofCountQdot
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseDynamicsSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3CalculateInverseDynamicsSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusInverseDynamicsJointForces(
    statusHandle:  b3SharedMemoryStatusHandle,
    bodyUniqueId: *mut c_int,
    dofCount: *mut c_int,
    jointForces: *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusInverseDynamicsJointForces(
            statusHandle,
            bodyUniqueId,
            dofCount,
            jointForces
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateJacobianCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    localPosition: *const f64,
    jointPositionsQ: *const f64,
    jointVelocitiesQdot: *const f64,
    jointAccelerations: *const f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CalculateJacobianCommandInit(
            physClient,
            bodyUniqueId,
            linkIndex,
            localPosition,
            jointPositionsQ,
            jointVelocitiesQdot,
            jointAccelerations
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusJacobian(
    statusHandle:  b3SharedMemoryStatusHandle,
    dofCount: *mut c_int,
    linearJacobian: *mut f64,
    angularJacobian: *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusJacobian(
            statusHandle,
            dofCount,
            linearJacobian,
            angularJacobian
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateMassMatrixCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    jointPositionsQ: *const f64,
    dofCountQ:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CalculateMassMatrixCommandInit(
            physClient,
            bodyUniqueId,
            jointPositionsQ,
            dofCountQ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateMassMatrixSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3CalculateMassMatrixSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusMassMatrix(
    physClient:  b3PhysicsClientHandle,
    statusHandle:  b3SharedMemoryStatusHandle,
    dofCount: *mut c_int,
    massMatrix: *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusMassMatrix(
            physClient,
            statusHandle,
            dofCount,
            massMatrix
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CalculateInverseKinematicsCommandInit(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsAddTargetPurePosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    endEffectorLinkIndex:  c_int,
    targetPosition: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsAddTargetPurePosition(
            commandHandle,
            endEffectorLinkIndex,
            targetPosition
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsAddTargetsPurePosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    numEndEffectorLinkIndices:  c_int,
    endEffectorIndices: *const c_int,
    targetPositions: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsAddTargetsPurePosition(
            commandHandle,
            numEndEffectorLinkIndices,
            endEffectorIndices,
            targetPositions
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    endEffectorLinkIndex:  c_int,
    targetPosition: *const f64,
    targetOrientation: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
            commandHandle,
            endEffectorLinkIndex,
            targetPosition,
            targetOrientation
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsPosWithNullSpaceVel(
    commandHandle:  b3SharedMemoryCommandHandle,
    numDof:  c_int,
    endEffectorLinkIndex:  c_int,
    targetPosition: *const f64,
    lowerLimit: *const f64,
    upperLimit: *const f64,
    jointRange: *const f64,
    restPose: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsPosWithNullSpaceVel(
            commandHandle,
            numDof,
            endEffectorLinkIndex,
            targetPosition,
            lowerLimit,
            upperLimit,
            jointRange,
            restPose
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(
    commandHandle:  b3SharedMemoryCommandHandle,
    numDof:  c_int,
    endEffectorLinkIndex:  c_int,
    targetPosition: *const f64,
    targetOrientation: *const f64,
    lowerLimit: *const f64,
    upperLimit: *const f64,
    jointRange: *const f64,
    restPose: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(
            commandHandle,
            numDof,
            endEffectorLinkIndex,
            targetPosition,
            targetOrientation,
            lowerLimit,
            upperLimit,
            jointRange,
            restPose
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsSetJointDamping(
    commandHandle:  b3SharedMemoryCommandHandle,
    numDof:  c_int,
    jointDampingCoeff: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsSetJointDamping(
            commandHandle,
            numDof,
            jointDampingCoeff
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsSelectSolver(
    commandHandle:  b3SharedMemoryCommandHandle,
    solver:  c_int    
)
{
    unsafe {
        return b3CalculateInverseKinematicsSelectSolver(
            commandHandle,
            solver
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusInverseKinematicsJointPositions(
    statusHandle:  b3SharedMemoryStatusHandle,
    bodyUniqueId: *mut c_int,
    dofCount: *mut c_int,
    jointPositions: *mut f64    
) ->  c_int
{
    unsafe {
        return b3GetStatusInverseKinematicsJointPositions(
            statusHandle,
            bodyUniqueId,
            dofCount,
            jointPositions
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsSetCurrentPositions(
    commandHandle:  b3SharedMemoryCommandHandle,
    numDof:  c_int,
    currentJointPositions: *const f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsSetCurrentPositions(
            commandHandle,
            numDof,
            currentJointPositions
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsSetMaxNumIterations(
    commandHandle:  b3SharedMemoryCommandHandle,
    maxNumIterations:  c_int    
)
{
    unsafe {
        return b3CalculateInverseKinematicsSetMaxNumIterations(
            commandHandle,
            maxNumIterations
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateInverseKinematicsSetResidualThreshold(
    commandHandle:  b3SharedMemoryCommandHandle,
    residualThreshold:  f64    
)
{
    unsafe {
        return b3CalculateInverseKinematicsSetResidualThreshold(
            commandHandle,
            residualThreshold
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CollisionFilterCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CollisionFilterCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetCollisionFilterPair(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueIdA:  c_int,
    bodyUniqueIdB:  c_int,
    linkIndexA:  c_int,
    linkIndexB:  c_int,
    enableCollision:  c_int    
)
{
    unsafe {
        return b3SetCollisionFilterPair(
            commandHandle,
            bodyUniqueIdA,
            bodyUniqueIdB,
            linkIndexA,
            linkIndexB,
            enableCollision
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetCollisionFilterGroupMask(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueIdA:  c_int,
    linkIndexA:  c_int,
    collisionFilterGroup:  c_int,
    collisionFilterMask:  c_int    
)
{
    unsafe {
        return b3SetCollisionFilterGroupMask(
            commandHandle,
            bodyUniqueIdA,
            linkIndexA,
            collisionFilterGroup,
            collisionFilterMask
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSdfCommandInit(
    physClient:  b3PhysicsClientHandle,
    sdfFileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadSdfCommandInit(
            physClient,
            sdfFileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSdfCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle,
    sdfFileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadSdfCommandInit2(
            commandHandle,
            sdfFileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSdfCommandSetUseMultiBody(
    commandHandle:  b3SharedMemoryCommandHandle,
    useMultiBody:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadSdfCommandSetUseMultiBody(
            commandHandle,
            useMultiBody
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSdfCommandSetUseGlobalScaling(
    commandHandle:  b3SharedMemoryCommandHandle,
    globalScaling:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSdfCommandSetUseGlobalScaling(
            commandHandle,
            globalScaling
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SaveWorldCommandInit(
    physClient:  b3PhysicsClientHandle,
    sdfFileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3SaveWorldCommandInit(
            physClient,
            sdfFileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlCommandInit(
    physClient:  b3PhysicsClientHandle,
    controlMode:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3JointControlCommandInit(
            physClient,
            controlMode
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlCommandInit2(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    controlMode:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3JointControlCommandInit2(
            physClient,
            bodyUniqueId,
            controlMode
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlCommandInit2Internal(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    controlMode:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3JointControlCommandInit2Internal(
            commandHandle,
            bodyUniqueId,
            controlMode
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredPosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    qIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredPosition(
            commandHandle,
            qIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredPositionMultiDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    qIndex:  c_int,
    position: *const f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredPositionMultiDof(
            commandHandle,
            qIndex,
            position,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetKp(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetKp(
            commandHandle,
            dofIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetKpMultiDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    kps: *mut f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetKpMultiDof(
            commandHandle,
            dofIndex,
            kps,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetKd(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetKd(
            commandHandle,
            dofIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetKdMultiDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    kds: *mut f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetKdMultiDof(
            commandHandle,
            dofIndex,
            kds,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetMaximumVelocity(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    maximumVelocity:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetMaximumVelocity(
            commandHandle,
            dofIndex,
            maximumVelocity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredVelocity(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredVelocity(
            commandHandle,
            dofIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredVelocityMultiDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    velocity: *const f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredVelocityMultiDof(
            commandHandle,
            dofIndex,
            velocity,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredVelocityMultiDof2(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    velocity: *const f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredVelocityMultiDof2(
            commandHandle,
            dofIndex,
            velocity,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetMaximumForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetMaximumForce(
            commandHandle,
            dofIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredForceTorqueMultiDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    forces: *mut f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredForceTorqueMultiDof(
            commandHandle,
            dofIndex,
            forces,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDamping(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDamping(
            commandHandle,
            dofIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDampingMultiDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    damping: *mut f64,
    dofCount:  c_int    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDampingMultiDof(
            commandHandle,
            dofIndex,
            damping,
            dofCount
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3JointControlSetDesiredForceTorque(
    commandHandle:  b3SharedMemoryCommandHandle,
    dofIndex:  c_int,
    value:  f64    
) ->  c_int
{
    unsafe {
        return b3JointControlSetDesiredForceTorque(
            commandHandle,
            dofIndex,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateCollisionShapeCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddSphere(
    commandHandle:  b3SharedMemoryCommandHandle,
    radius:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddSphere(
            commandHandle,
            radius
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddBox(
    commandHandle:  b3SharedMemoryCommandHandle,
    halfExtents: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddBox(
            commandHandle,
            halfExtents
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddCapsule(
    commandHandle:  b3SharedMemoryCommandHandle,
    radius:  f64,
    height:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddCapsule(
            commandHandle,
            radius,
            height
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddCylinder(
    commandHandle:  b3SharedMemoryCommandHandle,
    radius:  f64,
    height:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddCylinder(
            commandHandle,
            radius,
            height
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddHeightfield(
    commandHandle:  b3SharedMemoryCommandHandle,
    fileName: *const c_char,
    meshScale: *const f64,
    textureScaling:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddHeightfield(
            commandHandle,
            fileName,
            meshScale,
            textureScaling
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddHeightfield2(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    meshScale: *const f64,
    textureScaling:  f64,
    heightfieldData: *mut f32,
    numHeightfieldRows:  c_int,
    numHeightfieldColumns:  c_int,
    replaceHeightfieldIndex:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddHeightfield2(
            physClient,
            commandHandle,
            meshScale,
            textureScaling,
            heightfieldData,
            numHeightfieldRows,
            numHeightfieldColumns,
            replaceHeightfieldIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddPlane(
    commandHandle:  b3SharedMemoryCommandHandle,
    planeNormal: *const f64,
    planeConstant:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddPlane(
            commandHandle,
            planeNormal,
            planeConstant
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddMesh(
    commandHandle:  b3SharedMemoryCommandHandle,
    fileName: *const c_char,
    meshScale: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddMesh(
            commandHandle,
            fileName,
            meshScale
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddConvexMesh(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    meshScale: *const f64,
    vertices: *const f64,
    numVertices:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddConvexMesh(
            physClient,
            commandHandle,
            meshScale,
            vertices,
            numVertices
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeAddConcaveMesh(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    meshScale: *const f64,
    vertices: *const f64,
    numVertices:  c_int,
    indices: *const c_int,
    numIndices:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateCollisionShapeAddConcaveMesh(
            physClient,
            commandHandle,
            meshScale,
            vertices,
            numVertices,
            indices,
            numIndices
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionSetFlag(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int,
    flags:  c_int    
)
{
    unsafe {
        return b3CreateCollisionSetFlag(
            commandHandle,
            shapeIndex,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateCollisionShapeSetChildTransform(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int,
    childPosition: *const f64,
    childOrientation: *const f64    
)
{
    unsafe {
        return b3CreateCollisionShapeSetChildTransform(
            commandHandle,
            shapeIndex,
            childPosition,
            childOrientation
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusCollisionShapeUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusCollisionShapeUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitRemoveCollisionShapeCommand(
    physClient:  b3PhysicsClientHandle,
    collisionShapeId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitRemoveCollisionShapeCommand(
            physClient,
            collisionShapeId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetMeshDataCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    linkIndex:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3GetMeshDataCommandInit(
            physClient,
            bodyUniqueId,
            linkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetTetraMeshDataCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3GetTetraMeshDataCommandInit(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetMeshDataSimulationMesh(
    commandHandle:  b3SharedMemoryCommandHandle    
)
{
    unsafe {
        return b3GetMeshDataSimulationMesh(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3MeshDataSimulationMeshVelocity(
    commandHandle:  b3SharedMemoryCommandHandle    
)
{
    unsafe {
        return b3MeshDataSimulationMeshVelocity(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetMeshDataSetCollisionShapeIndex(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int    
)
{
    unsafe {
        return b3GetMeshDataSetCollisionShapeIndex(
            commandHandle,
            shapeIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetMeshDataSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3GetMeshDataSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetTetraMeshDataSetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3GetTetraMeshDataSetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetMeshData(
    physClient:  b3PhysicsClientHandle,
    meshData: *mut b3MeshData    
)
{
    unsafe {
        return b3GetMeshData(
            physClient,
            meshData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetTetraMeshData(
    physClient:  b3PhysicsClientHandle,
    meshData: *mut b3TetraMeshData    
)
{
    unsafe {
        return b3GetTetraMeshData(
            physClient,
            meshData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ResetMeshDataCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int,
    num_vertices:  c_int,
    vertices: *const f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3ResetMeshDataCommandInit(
            physClient,
            bodyUniqueId,
            num_vertices,
            vertices
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateVisualShapeCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddSphere(
    commandHandle:  b3SharedMemoryCommandHandle,
    radius:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddSphere(
            commandHandle,
            radius
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddBox(
    commandHandle:  b3SharedMemoryCommandHandle,
    halfExtents: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddBox(
            commandHandle,
            halfExtents
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddCapsule(
    commandHandle:  b3SharedMemoryCommandHandle,
    radius:  f64,
    height:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddCapsule(
            commandHandle,
            radius,
            height
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddCylinder(
    commandHandle:  b3SharedMemoryCommandHandle,
    radius:  f64,
    height:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddCylinder(
            commandHandle,
            radius,
            height
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddPlane(
    commandHandle:  b3SharedMemoryCommandHandle,
    planeNormal: *const f64,
    planeConstant:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddPlane(
            commandHandle,
            planeNormal,
            planeConstant
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddMesh(
    commandHandle:  b3SharedMemoryCommandHandle,
    fileName: *const c_char,
    meshScale: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddMesh(
            commandHandle,
            fileName,
            meshScale
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeAddMesh2(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    meshScale: *const f64,
    vertices: *const f64,
    numVertices:  c_int,
    indices: *const c_int,
    numIndices:  c_int,
    normals: *const f64,
    numNormals:  c_int,
    uvs: *const f64,
    numUVs:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateVisualShapeAddMesh2(
            physClient,
            commandHandle,
            meshScale,
            vertices,
            numVertices,
            indices,
            numIndices,
            normals,
            numNormals,
            uvs,
            numUVs
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualSetFlag(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int,
    flags:  c_int    
)
{
    unsafe {
        return b3CreateVisualSetFlag(
            commandHandle,
            shapeIndex,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeSetChildTransform(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int,
    childPosition: *const f64,
    childOrientation: *const f64    
)
{
    unsafe {
        return b3CreateVisualShapeSetChildTransform(
            commandHandle,
            shapeIndex,
            childPosition,
            childOrientation
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeSetSpecularColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int,
    specularColor: *const f64    
)
{
    unsafe {
        return b3CreateVisualShapeSetSpecularColor(
            commandHandle,
            shapeIndex,
            specularColor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateVisualShapeSetRGBAColor(
    commandHandle:  b3SharedMemoryCommandHandle,
    shapeIndex:  c_int,
    rgbaColor: *const f64    
)
{
    unsafe {
        return b3CreateVisualShapeSetRGBAColor(
            commandHandle,
            shapeIndex,
            rgbaColor
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusVisualShapeUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusVisualShapeUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateMultiBodyCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateMultiBodyCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateMultiBodyBase(
    commandHandle:  b3SharedMemoryCommandHandle,
    mass:  f64,
    collisionShapeUnique:  c_int,
    visualShapeUniqueId:  c_int,
    basePosition: *const f64,
    baseOrientation: *const f64,
    baseInertialFramePosition: *const f64,
    baseInertialFrameOrientation: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreateMultiBodyBase(
            commandHandle,
            mass,
            collisionShapeUnique,
            visualShapeUniqueId,
            basePosition,
            baseOrientation,
            baseInertialFramePosition,
            baseInertialFrameOrientation
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateMultiBodyLink(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkMass:  f64,
    linkCollisionShapeIndex:  f64,
    linkVisualShapeIndex:  f64,
    linkPosition: *const f64,
    linkOrientation: *const f64,
    linkInertialFramePosition: *const f64,
    linkInertialFrameOrientation: *const f64,
    linkParentIndex:  c_int,
    linkJointType:  c_int,
    linkJointAxis: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreateMultiBodyLink(
            commandHandle,
            linkMass,
            linkCollisionShapeIndex,
            linkVisualShapeIndex,
            linkPosition,
            linkOrientation,
            linkInertialFramePosition,
            linkInertialFrameOrientation,
            linkParentIndex,
            linkJointType,
            linkJointAxis
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateMultiBodySetBatchPositions(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    batchPositions: *mut f64,
    numBatchObjects:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateMultiBodySetBatchPositions(
            physClient,
            commandHandle,
            batchPositions,
            numBatchObjects
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateMultiBodyUseMaximalCoordinates(
    commandHandle:  b3SharedMemoryCommandHandle    
)
{
    unsafe {
        return b3CreateMultiBodyUseMaximalCoordinates(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateMultiBodySetFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    flags:  c_int    
)
{
    unsafe {
        return b3CreateMultiBodySetFlags(
            commandHandle,
            flags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxShapeCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateBoxShapeCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxCommandSetStartPosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    startPosX:  f64,
    startPosY:  f64,
    startPosZ:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateBoxCommandSetStartPosition(
            commandHandle,
            startPosX,
            startPosY,
            startPosZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxCommandSetStartOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    startOrnX:  f64,
    startOrnY:  f64,
    startOrnZ:  f64,
    startOrnW:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateBoxCommandSetStartOrientation(
            commandHandle,
            startOrnX,
            startOrnY,
            startOrnZ,
            startOrnW
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxCommandSetHalfExtents(
    commandHandle:  b3SharedMemoryCommandHandle,
    halfExtentsX:  f64,
    halfExtentsY:  f64,
    halfExtentsZ:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateBoxCommandSetHalfExtents(
            commandHandle,
            halfExtentsX,
            halfExtentsY,
            halfExtentsZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxCommandSetMass(
    commandHandle:  b3SharedMemoryCommandHandle,
    mass:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateBoxCommandSetMass(
            commandHandle,
            mass
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxCommandSetCollisionShapeType(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionShapeType:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateBoxCommandSetCollisionShapeType(
            commandHandle,
            collisionShapeType
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateBoxCommandSetColorRGBA(
    commandHandle:  b3SharedMemoryCommandHandle,
    red:  f64,
    green:  f64,
    blue:  f64,
    alpha:  f64    
) ->  c_int
{
    unsafe {
        return b3CreateBoxCommandSetColorRGBA(
            commandHandle,
            red,
            green,
            blue,
            alpha
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreatePoseCommandInit(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreatePoseCommandInit2(
            commandHandle,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetBasePosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    startPosX:  f64,
    startPosY:  f64,
    startPosZ:  f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetBasePosition(
            commandHandle,
            startPosX,
            startPosY,
            startPosZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetBaseOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    startOrnX:  f64,
    startOrnY:  f64,
    startOrnZ:  f64,
    startOrnW:  f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetBaseOrientation(
            commandHandle,
            startOrnX,
            startOrnY,
            startOrnZ,
            startOrnW
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetBaseLinearVelocity(
    commandHandle:  b3SharedMemoryCommandHandle,
    linVel: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetBaseLinearVelocity(
            commandHandle,
            linVel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetBaseAngularVelocity(
    commandHandle:  b3SharedMemoryCommandHandle,
    angVel: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetBaseAngularVelocity(
            commandHandle,
            angVel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetBaseScaling(
    commandHandle:  b3SharedMemoryCommandHandle,
    scaling: *mut f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetBaseScaling(
            commandHandle,
            scaling
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetJointPositions(
    commandHandle:  b3SharedMemoryCommandHandle,
    numJointPositions:  c_int,
    jointPositions: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetJointPositions(
            commandHandle,
            numJointPositions,
            jointPositions
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetJointPosition(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    jointIndex:  c_int,
    jointPosition:  f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetJointPosition(
            physClient,
            commandHandle,
            jointIndex,
            jointPosition
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetJointPositionMultiDof(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    jointIndex:  c_int,
    jointPosition: *const f64,
    posSize:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetJointPositionMultiDof(
            physClient,
            commandHandle,
            jointIndex,
            jointPosition,
            posSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetQ(
    commandHandle:  b3SharedMemoryCommandHandle,
    numJointPositions:  c_int,
    q: *const f64,
    hasQ: *const c_int    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetQ(
            commandHandle,
            numJointPositions,
            q,
            hasQ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetQdots(
    commandHandle:  b3SharedMemoryCommandHandle,
    numJointVelocities:  c_int,
    qDots: *const f64,
    hasQdots: *const c_int    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetQdots(
            commandHandle,
            numJointVelocities,
            qDots,
            hasQdots
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetJointVelocities(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    numJointVelocities:  c_int,
    jointVelocities: *const f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetJointVelocities(
            physClient,
            commandHandle,
            numJointVelocities,
            jointVelocities
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetJointVelocity(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    jointIndex:  c_int,
    jointVelocity:  f64    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetJointVelocity(
            physClient,
            commandHandle,
            jointIndex,
            jointVelocity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreatePoseCommandSetJointVelocityMultiDof(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    jointIndex:  c_int,
    jointVelocity: *const f64,
    velSize:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreatePoseCommandSetJointVelocityMultiDof(
            physClient,
            commandHandle,
            jointIndex,
            jointVelocity,
            velSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateSensorCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateSensorCommandInit(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateSensorEnable6DofJointForceTorqueSensor(
    commandHandle:  b3SharedMemoryCommandHandle,
    jointIndex:  c_int,
    enable:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateSensorEnable6DofJointForceTorqueSensor(
            commandHandle,
            jointIndex,
            enable
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateSensorEnableIMUForLink(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndex:  c_int,
    enable:  c_int    
) ->  c_int
{
    unsafe {
        return b3CreateSensorEnableIMUForLink(
            commandHandle,
            linkIndex,
            enable
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestActualStateCommandInit(
    physClient:  b3PhysicsClientHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestActualStateCommandInit(
            physClient,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestActualStateCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestActualStateCommandInit2(
            commandHandle,
            bodyUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestActualStateCommandComputeLinkVelocity(
    commandHandle:  b3SharedMemoryCommandHandle,
    computeLinkVelocity:  c_int    
) ->  c_int
{
    unsafe {
        return b3RequestActualStateCommandComputeLinkVelocity(
            commandHandle,
            computeLinkVelocity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestActualStateCommandComputeForwardKinematics(
    commandHandle:  b3SharedMemoryCommandHandle,
    computeForwardKinematics:  c_int    
) ->  c_int
{
    unsafe {
        return b3RequestActualStateCommandComputeForwardKinematics(
            commandHandle,
            computeForwardKinematics
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetJointState(
    physClient:  b3PhysicsClientHandle,
    statusHandle:  b3SharedMemoryStatusHandle,
    jointIndex:  c_int,
    state: *mut b3JointSensorState    
) ->  c_int
{
    unsafe {
        return b3GetJointState(
            physClient,
            statusHandle,
            jointIndex,
            state
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetJointStateMultiDof(
    physClient:  b3PhysicsClientHandle,
    statusHandle:  b3SharedMemoryStatusHandle,
    jointIndex:  c_int,
    state: *mut b3JointSensorState2    
) ->  c_int
{
    unsafe {
        return b3GetJointStateMultiDof(
            physClient,
            statusHandle,
            jointIndex,
            state
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetLinkState(
    physClient:  b3PhysicsClientHandle,
    statusHandle:  b3SharedMemoryStatusHandle,
    linkIndex:  c_int,
    state: *mut b3LinkState    
) ->  c_int
{
    unsafe {
        return b3GetLinkState(
            physClient,
            statusHandle,
            linkIndex,
            state
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PickBody(
    physClient:  b3PhysicsClientHandle,
    rayFromWorldX:  f64,
    rayFromWorldY:  f64,
    rayFromWorldZ:  f64,
    rayToWorldX:  f64,
    rayToWorldY:  f64,
    rayToWorldZ:  f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3PickBody(
            physClient,
            rayFromWorldX,
            rayFromWorldY,
            rayFromWorldZ,
            rayToWorldX,
            rayToWorldY,
            rayToWorldZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3MovePickedBody(
    physClient:  b3PhysicsClientHandle,
    rayFromWorldX:  f64,
    rayFromWorldY:  f64,
    rayFromWorldZ:  f64,
    rayToWorldX:  f64,
    rayToWorldY:  f64,
    rayToWorldZ:  f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3MovePickedBody(
            physClient,
            rayFromWorldX,
            rayFromWorldY,
            rayFromWorldZ,
            rayToWorldX,
            rayToWorldY,
            rayToWorldZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RemovePickingConstraint(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RemovePickingConstraint(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateRaycastCommandInit(
    physClient:  b3PhysicsClientHandle,
    rayFromWorldX:  f64,
    rayFromWorldY:  f64,
    rayFromWorldZ:  f64,
    rayToWorldX:  f64,
    rayToWorldY:  f64,
    rayToWorldZ:  f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateRaycastCommandInit(
            physClient,
            rayFromWorldX,
            rayFromWorldY,
            rayFromWorldZ,
            rayToWorldX,
            rayToWorldY,
            rayToWorldZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CreateRaycastBatchCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3CreateRaycastBatchCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchSetNumThreads(
    commandHandle:  b3SharedMemoryCommandHandle,
    numThreads:  c_int    
)
{
    unsafe {
        return b3RaycastBatchSetNumThreads(
            commandHandle,
            numThreads
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchAddRay(
    commandHandle:  b3SharedMemoryCommandHandle,
    rayFromWorld: *const f64,
    rayToWorld: *const f64    
)
{
    unsafe {
        return b3RaycastBatchAddRay(
            commandHandle,
            rayFromWorld,
            rayToWorld
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchAddRays(
    physClient:  b3PhysicsClientHandle,
    commandHandle:  b3SharedMemoryCommandHandle,
    rayFromWorld: *const f64,
    rayToWorld: *const f64,
    numRays:  c_int    
)
{
    unsafe {
        return b3RaycastBatchAddRays(
            physClient,
            commandHandle,
            rayFromWorld,
            rayToWorld,
            numRays
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchSetParentObject(
    commandHandle:  b3SharedMemoryCommandHandle,
    parentObjectUniqueId:  c_int,
    parentLinkIndex:  c_int    
)
{
    unsafe {
        return b3RaycastBatchSetParentObject(
            commandHandle,
            parentObjectUniqueId,
            parentLinkIndex
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchSetReportHitNumber(
    commandHandle:  b3SharedMemoryCommandHandle,
    reportHitNumber:  c_int    
)
{
    unsafe {
        return b3RaycastBatchSetReportHitNumber(
            commandHandle,
            reportHitNumber
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchSetCollisionFilterMask(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionFilterMask:  c_int    
)
{
    unsafe {
        return b3RaycastBatchSetCollisionFilterMask(
            commandHandle,
            collisionFilterMask
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RaycastBatchSetFractionEpsilon(
    commandHandle:  b3SharedMemoryCommandHandle,
    fractionEpsilon:  f64    
)
{
    unsafe {
        return b3RaycastBatchSetFractionEpsilon(
            commandHandle,
            fractionEpsilon
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetRaycastInformation(
    physClient:  b3PhysicsClientHandle,
    raycastInfo: *mut b3RaycastInformation    
)
{
    unsafe {
        return b3GetRaycastInformation(
            physClient,
            raycastInfo
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ApplyExternalForceCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3ApplyExternalForceCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ApplyExternalForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkId:  c_int,
    force: *const f64,
    position: *const f64,
    flag:  c_int    
)
{
    unsafe {
        return b3ApplyExternalForce(
            commandHandle,
            bodyUniqueId,
            linkId,
            force,
            position,
            flag
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ApplyExternalTorque(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyUniqueId:  c_int,
    linkId:  c_int,
    torque: *const f64,
    flag:  c_int    
)
{
    unsafe {
        return b3ApplyExternalTorque(
            commandHandle,
            bodyUniqueId,
            linkId,
            torque,
            flag
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyCommandInit(
    physClient:  b3PhysicsClientHandle,
    fileName: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3LoadSoftBodyCommandInit(
            physClient,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetScale(
    commandHandle:  b3SharedMemoryCommandHandle,
    scale:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetScale(
            commandHandle,
            scale
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetMass(
    commandHandle:  b3SharedMemoryCommandHandle,
    mass:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetMass(
            commandHandle,
            mass
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetCollisionMargin(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionMargin:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetCollisionMargin(
            commandHandle,
            collisionMargin
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetStartPosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    startPosX:  f64,
    startPosY:  f64,
    startPosZ:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetStartPosition(
            commandHandle,
            startPosX,
            startPosY,
            startPosZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetStartOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    startOrnX:  f64,
    startOrnY:  f64,
    startOrnZ:  f64,
    startOrnW:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetStartOrientation(
            commandHandle,
            startOrnX,
            startOrnY,
            startOrnZ,
            startOrnW
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyUpdateSimMesh(
    commandHandle:  b3SharedMemoryCommandHandle,
    filename: *const c_char    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyUpdateSimMesh(
            commandHandle,
            filename
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyAddCorotatedForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    corotatedMu:  f64,
    corotatedLambda:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyAddCorotatedForce(
            commandHandle,
            corotatedMu,
            corotatedLambda
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyAddNeoHookeanForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    NeoHookeanMu:  f64,
    NeoHookeanLambda:  f64,
    NeoHookeanDamping:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyAddNeoHookeanForce(
            commandHandle,
            NeoHookeanMu,
            NeoHookeanLambda,
            NeoHookeanDamping
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyAddMassSpringForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    springElasticStiffness:  f64,
    springDampingStiffness:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyAddMassSpringForce(
            commandHandle,
            springElasticStiffness,
            springDampingStiffness
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyAddGravityForce(
    commandHandle:  b3SharedMemoryCommandHandle,
    gravityX:  f64,
    gravityY:  f64,
    gravityZ:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyAddGravityForce(
            commandHandle,
            gravityX,
            gravityY,
            gravityZ
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetCollisionHardness(
    commandHandle:  b3SharedMemoryCommandHandle,
    collisionHardness:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetCollisionHardness(
            commandHandle,
            collisionHardness
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetSelfCollision(
    commandHandle:  b3SharedMemoryCommandHandle,
    useSelfCollision:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetSelfCollision(
            commandHandle,
            useSelfCollision
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetRepulsionStiffness(
    commandHandle:  b3SharedMemoryCommandHandle,
    stiffness:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetRepulsionStiffness(
            commandHandle,
            stiffness
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyUseFaceContact(
    commandHandle:  b3SharedMemoryCommandHandle,
    useFaceContact:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyUseFaceContact(
            commandHandle,
            useFaceContact
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodySetFrictionCoefficient(
    commandHandle:  b3SharedMemoryCommandHandle,
    frictionCoefficient:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodySetFrictionCoefficient(
            commandHandle,
            frictionCoefficient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyUseBendingSprings(
    commandHandle:  b3SharedMemoryCommandHandle,
    useBendingSprings:  c_int,
    bendingStiffness:  f64    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyUseBendingSprings(
            commandHandle,
            useBendingSprings,
            bendingStiffness
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3LoadSoftBodyUseAllDirectionDampingSprings(
    commandHandle:  b3SharedMemoryCommandHandle,
    useAllDirectionDamping:  c_int    
) ->  c_int
{
    unsafe {
        return b3LoadSoftBodyUseAllDirectionDampingSprings(
            commandHandle,
            useAllDirectionDamping
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InitCreateSoftBodyAnchorConstraintCommand(
    physClient:  b3PhysicsClientHandle,
    softBodyUniqueId:  c_int,
    nodeIndex:  c_int,
    bodyUniqueId:  c_int,
    linkIndex:  c_int,
    bodyFramePosition: *const f64    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3InitCreateSoftBodyAnchorConstraintCommand(
            physClient,
            softBodyUniqueId,
            nodeIndex,
            bodyUniqueId,
            linkIndex,
            bodyFramePosition
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestVREventsCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestVREventsCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3VREventsSetDeviceTypeFilter(
    commandHandle:  b3SharedMemoryCommandHandle,
    deviceTypeFilter:  c_int    
)
{
    unsafe {
        return b3VREventsSetDeviceTypeFilter(
            commandHandle,
            deviceTypeFilter
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetVREventsData(
    physClient:  b3PhysicsClientHandle,
    vrEventsData: *mut b3VREventsData    
)
{
    unsafe {
        return b3GetVREventsData(
            physClient,
            vrEventsData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetVRCameraStateCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3SetVRCameraStateCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetVRCameraRootPosition(
    commandHandle:  b3SharedMemoryCommandHandle,
    rootPos: *const f64    
) ->  c_int
{
    unsafe {
        return b3SetVRCameraRootPosition(
            commandHandle,
            rootPos
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetVRCameraRootOrientation(
    commandHandle:  b3SharedMemoryCommandHandle,
    rootOrn: *const f64    
) ->  c_int
{
    unsafe {
        return b3SetVRCameraRootOrientation(
            commandHandle,
            rootOrn
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetVRCameraTrackingObject(
    commandHandle:  b3SharedMemoryCommandHandle,
    objectUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3SetVRCameraTrackingObject(
            commandHandle,
            objectUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetVRCameraTrackingObjectFlag(
    commandHandle:  b3SharedMemoryCommandHandle,
    flag:  c_int    
) ->  c_int
{
    unsafe {
        return b3SetVRCameraTrackingObjectFlag(
            commandHandle,
            flag
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestKeyboardEventsCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestKeyboardEventsCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestKeyboardEventsCommandInit2(
    commandHandle:  b3SharedMemoryCommandHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestKeyboardEventsCommandInit2(
            commandHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetKeyboardEventsData(
    physClient:  b3PhysicsClientHandle,
    keyboardEventsData: *mut b3KeyboardEventsData    
)
{
    unsafe {
        return b3GetKeyboardEventsData(
            physClient,
            keyboardEventsData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RequestMouseEventsCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3RequestMouseEventsCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetMouseEventsData(
    physClient:  b3PhysicsClientHandle,
    mouseEventsData: *mut b3MouseEventsData    
)
{
    unsafe {
        return b3GetMouseEventsData(
            physClient,
            mouseEventsData
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingCommandInit(
    physClient:  b3PhysicsClientHandle    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3StateLoggingCommandInit(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingStart(
    commandHandle:  b3SharedMemoryCommandHandle,
    loggingType:  c_int,
    fileName: *const c_char    
) ->  c_int
{
    unsafe {
        return b3StateLoggingStart(
            commandHandle,
            loggingType,
            fileName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingAddLoggingObjectUniqueId(
    commandHandle:  b3SharedMemoryCommandHandle,
    objectUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingAddLoggingObjectUniqueId(
            commandHandle,
            objectUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetMaxLogDof(
    commandHandle:  b3SharedMemoryCommandHandle,
    maxLogDof:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetMaxLogDof(
            commandHandle,
            maxLogDof
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetLinkIndexA(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndexA:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetLinkIndexA(
            commandHandle,
            linkIndexA
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetLinkIndexB(
    commandHandle:  b3SharedMemoryCommandHandle,
    linkIndexB:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetLinkIndexB(
            commandHandle,
            linkIndexB
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetBodyAUniqueId(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyAUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetBodyAUniqueId(
            commandHandle,
            bodyAUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetBodyBUniqueId(
    commandHandle:  b3SharedMemoryCommandHandle,
    bodyBUniqueId:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetBodyBUniqueId(
            commandHandle,
            bodyBUniqueId
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetDeviceTypeFilter(
    commandHandle:  b3SharedMemoryCommandHandle,
    deviceTypeFilter:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetDeviceTypeFilter(
            commandHandle,
            deviceTypeFilter
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingSetLogFlags(
    commandHandle:  b3SharedMemoryCommandHandle,
    logFlags:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingSetLogFlags(
            commandHandle,
            logFlags
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetStatusLoggingUniqueId(
    statusHandle:  b3SharedMemoryStatusHandle    
) ->  c_int
{
    unsafe {
        return b3GetStatusLoggingUniqueId(
            statusHandle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3StateLoggingStop(
    commandHandle:  b3SharedMemoryCommandHandle,
    loggingUid:  c_int    
) ->  c_int
{
    unsafe {
        return b3StateLoggingStop(
            commandHandle,
            loggingUid
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3ProfileTimingCommandInit(
    physClient:  b3PhysicsClientHandle,
    name: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3ProfileTimingCommandInit(
            physClient,
            name
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetProfileTimingDuractionInMicroSeconds(
    commandHandle:  b3SharedMemoryCommandHandle,
    duration:  c_int    
)
{
    unsafe {
        return b3SetProfileTimingDuractionInMicroSeconds(
            commandHandle,
            duration
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetProfileTimingType(
    commandHandle:  b3SharedMemoryCommandHandle,
    type_:  c_int    
)
{
    unsafe {
        return b3SetProfileTimingType(
            commandHandle,
            type_
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PushProfileTiming(
    physClient:  b3PhysicsClientHandle,
    timingName: *const c_char    
)
{
    unsafe {
        return b3PushProfileTiming(
            physClient,
            timingName
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3PopProfileTiming(
    physClient:  b3PhysicsClientHandle    
)
{
    unsafe {
        return b3PopProfileTiming(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetTimeOut(
    physClient:  b3PhysicsClientHandle,
    timeOutInSeconds:  f64    
)
{
    unsafe {
        return b3SetTimeOut(
            physClient,
            timeOutInSeconds
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetTimeOut(
    physClient:  b3PhysicsClientHandle    
) ->  f64
{
    unsafe {
        return b3GetTimeOut(
            physClient
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3SetAdditionalSearchPath(
    physClient:  b3PhysicsClientHandle,
    path: *const c_char    
) ->  b3SharedMemoryCommandHandle
{
    unsafe {
        return b3SetAdditionalSearchPath(
            physClient,
            path
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3MultiplyTransforms(
    posA: *const f64,
    ornA: *const f64,
    posB: *const f64,
    ornB: *const f64,
    outPos: *mut f64,
    outOrn: *mut f64    
)
{
    unsafe {
        return b3MultiplyTransforms(
            posA,
            ornA,
            posB,
            ornB,
            outPos,
            outOrn
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3InvertTransform(
    pos: *const f64,
    orn: *const f64,
    outPos: *mut f64,
    outOrn: *mut f64    
)
{
    unsafe {
        return b3InvertTransform(
            pos,
            orn,
            outPos,
            outOrn
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3QuaternionSlerp(
    startQuat: *const f64,
    endQuat: *const f64,
    interpolationFraction:  f64,
    outOrn: *mut f64    
)
{
    unsafe {
        return b3QuaternionSlerp(
            startQuat,
            endQuat,
            interpolationFraction,
            outOrn
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetQuaternionFromAxisAngle(
    axis: *const f64,
    angle:  f64,
    outQuat: *mut f64    
)
{
    unsafe {
        return b3GetQuaternionFromAxisAngle(
            axis,
            angle,
            outQuat
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetAxisAngleFromQuaternion(
    quat: *const f64,
    axis: *mut f64,
    angle: *mut f64    
)
{
    unsafe {
        return b3GetAxisAngleFromQuaternion(
            quat,
            axis,
            angle
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetQuaternionDifference(
    startQuat: *const f64,
    endQuat: *const f64,
    outOrn: *mut f64    
)
{
    unsafe {
        return b3GetQuaternionDifference(
            startQuat,
            endQuat,
            outOrn
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3GetAxisDifferenceQuaternion(
    startQuat: *const f64,
    endQuat: *const f64,
    axisOut: *mut f64    
)
{
    unsafe {
        return b3GetAxisDifferenceQuaternion(
            startQuat,
            endQuat,
            axisOut
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3CalculateVelocityQuaternion(
    startQuat: *const f64,
    endQuat: *const f64,
    deltaTime:  f64,
    angVelOut: *mut f64    
)
{
    unsafe {
        return b3CalculateVelocityQuaternion(
            startQuat,
            endQuat,
            deltaTime,
            angVelOut
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_bullet3_b3RotateVector(
    quat: *const f64,
    vec: *const f64,
    vecOut: *mut f64    
)
{
    unsafe {
        return b3RotateVector(
            quat,
            vec,
            vecOut
        )
    }
}

    