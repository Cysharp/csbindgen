/// enum for empty constructor tag
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxEMPTY {
    PxEmpty = 0,
}

/// enum for zero constructor tag for vectors and matrices
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxZERO {
    PxZero = 0,
}

/// enum for identity constructor flag for quaternions, transforms, and matrices
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxIDENTITY {
    PxIdentity = 0,
}

/// Error codes
///
/// These error codes are passed to [`PxErrorCallback`]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxErrorCode {
    NoError = 0,
    /// An informational message.
    DebugInfo = 1,
    /// a warning message for the user to help with debugging
    DebugWarning = 2,
    /// method called with invalid parameter(s)
    InvalidParameter = 4,
    /// method was called at a time when an operation is not possible
    InvalidOperation = 8,
    /// method failed to allocate some memory
    OutOfMemory = 16,
    /// The library failed for some reason.
    /// Possibly you have passed invalid values like NaNs, which are not checked for.
    InternalError = 32,
    /// An unrecoverable error, execution should be halted and log output flushed
    Abort = 64,
    /// The SDK has determined that an operation may result in poor performance.
    PerfWarning = 128,
    /// A bit mask for including all errors
    MaskAll = -1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PxThreadPriority {
    /// High priority
    High = 0,
    /// Above Normal priority
    AboveNormal = 1,
    /// Normal/default priority
    Normal = 2,
    /// Below Normal priority
    BelowNormal = 3,
    /// Low priority.
    Low = 4,
    ForceDword = 4294967295,
}

/// Default color values used for debug rendering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PxDebugColor {
    ArgbBlack = 4278190080,
    ArgbRed = 4294901760,
    ArgbGreen = 4278255360,
    ArgbBlue = 4278190335,
    ArgbYellow = 4294967040,
    ArgbMagenta = 4294902015,
    ArgbCyan = 4278255615,
    ArgbWhite = 4294967295,
    ArgbGrey = 4286611584,
    ArgbDarkred = 4287102976,
    ArgbDarkgreen = 4278224896,
    ArgbDarkblue = 4278190216,
}

/// an enumeration of concrete classes inheriting from PxBase
///
/// Enumeration space is reserved for future PhysX core types, PhysXExtensions,
/// PhysXVehicle and Custom application types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConcreteType {
    Undefined = 0,
    Heightfield = 1,
    ConvexMesh = 2,
    TriangleMeshBvh33 = 3,
    TriangleMeshBvh34 = 4,
    TetrahedronMesh = 5,
    SoftbodyMesh = 6,
    RigidDynamic = 7,
    RigidStatic = 8,
    Shape = 9,
    Material = 10,
    SoftbodyMaterial = 11,
    ClothMaterial = 12,
    PbdMaterial = 13,
    FlipMaterial = 14,
    MpmMaterial = 15,
    CustomMaterial = 16,
    Constraint = 17,
    Aggregate = 18,
    ArticulationReducedCoordinate = 19,
    ArticulationLink = 20,
    ArticulationJointReducedCoordinate = 21,
    ArticulationSensor = 22,
    ArticulationSpatialTendon = 23,
    ArticulationFixedTendon = 24,
    ArticulationAttachment = 25,
    ArticulationTendonJoint = 26,
    PruningStructure = 27,
    Bvh = 28,
    SoftBody = 29,
    SoftBodyState = 30,
    PbdParticlesystem = 31,
    FlipParticlesystem = 32,
    MpmParticlesystem = 33,
    CustomParticlesystem = 34,
    FemCloth = 35,
    HairSystem = 36,
    ParticleBuffer = 37,
    ParticleDiffuseBuffer = 38,
    ParticleClothBuffer = 39,
    ParticleRigidBuffer = 40,
    PhysxCoreCount = 41,
    FirstPhysxExtension = 256,
    FirstVehicleExtension = 512,
    FirstUserExtension = 1024,
}

impl From<u16> for PxConcreteType {
    fn from(val: u16) -> Self {
        #[allow(clippy::match_same_arms)]
        match val {
            0 => Self::Undefined,
            1 => Self::Heightfield,
            2 => Self::ConvexMesh,
            3 => Self::TriangleMeshBvh33,
            4 => Self::TriangleMeshBvh34,
            5 => Self::TetrahedronMesh,
            6 => Self::SoftbodyMesh,
            7 => Self::RigidDynamic,
            8 => Self::RigidStatic,
            9 => Self::Shape,
            10 => Self::Material,
            11 => Self::SoftbodyMaterial,
            12 => Self::ClothMaterial,
            13 => Self::PbdMaterial,
            14 => Self::FlipMaterial,
            15 => Self::MpmMaterial,
            16 => Self::CustomMaterial,
            17 => Self::Constraint,
            18 => Self::Aggregate,
            19 => Self::ArticulationReducedCoordinate,
            20 => Self::ArticulationLink,
            21 => Self::ArticulationJointReducedCoordinate,
            22 => Self::ArticulationSensor,
            23 => Self::ArticulationSpatialTendon,
            24 => Self::ArticulationFixedTendon,
            25 => Self::ArticulationAttachment,
            26 => Self::ArticulationTendonJoint,
            27 => Self::PruningStructure,
            28 => Self::Bvh,
            29 => Self::SoftBody,
            30 => Self::SoftBodyState,
            31 => Self::PbdParticlesystem,
            32 => Self::FlipParticlesystem,
            33 => Self::MpmParticlesystem,
            34 => Self::CustomParticlesystem,
            35 => Self::FemCloth,
            36 => Self::HairSystem,
            37 => Self::ParticleBuffer,
            38 => Self::ParticleDiffuseBuffer,
            39 => Self::ParticleClothBuffer,
            40 => Self::ParticleRigidBuffer,
            41 => Self::PhysxCoreCount,
            256 => Self::FirstPhysxExtension,
            512 => Self::FirstVehicleExtension,
            1024 => Self::FirstUserExtension,
            _ => Self::Undefined,
        }
    }
}

/// Flags for PxBase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxBaseFlag {
    OwnsMemory = 1,
    IsReleasable = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxBaseFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxBaseFlags: u16 {
        const OwnsMemory = 1 << 0;
        const IsReleasable = 1 << 1;
    }
}

/// Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMetaDataFlag {
    /// declares a class
    Class = 1,
    /// declares class to be virtual
    Virtual = 2,
    /// declares a typedef
    Typedef = 4,
    /// declares a pointer
    Ptr = 8,
    /// declares a handle
    Handle = 16,
    /// declares extra data exported with PxSerializer::exportExtraData
    ExtraData = 32,
    /// specifies one element of extra data
    ExtraItem = 64,
    /// specifies an array of extra data
    ExtraItems = 128,
    /// specifies a name of extra data
    ExtraName = 256,
    /// declares a union
    Union = 512,
    /// declares explicit padding data
    Padding = 1024,
    /// declares aligned data
    Alignment = 2048,
    /// specifies that the count value's most significant bit needs to be masked out
    CountMaskMsb = 4096,
    /// specifies that the count value is treated as zero for a variable value of one - special case for single triangle meshes
    CountSkipIfOne = 8192,
    /// specifies that the control value is the negate of the variable value
    ControlFlip = 16384,
    /// specifies that the control value is masked - mask bits are assumed to be within eCONTROL_MASK_RANGE
    ControlMask = 32768,
    /// mask range allowed for eCONTROL_MASK
    ControlMaskRange = 255,
    ForceDword = 2147483647,
}

/// Identifies the type of each heavyweight PxTask object
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxTaskType {
    /// PxTask will be run on the CPU
    Cpu = 0,
    /// Return code when attempting to find a task that does not exist
    NotPresent = 1,
    /// PxTask execution has been completed
    Completed = 2,
}

/// A geometry type.
///
/// Used to distinguish the type of a ::PxGeometry object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxGeometryType {
    Sphere = 0,
    Plane = 1,
    Capsule = 2,
    Box = 3,
    Convexmesh = 4,
    Particlesystem = 5,
    Tetrahedronmesh = 6,
    Trianglemesh = 7,
    Heightfield = 8,
    Hairsystem = 9,
    Custom = 10,
    /// internal use only!
    GeometryCount = 11,
    /// internal use only!
    Invalid = -1,
}

/// Geometry-level query flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxGeometryQueryFlag {
    /// Saves/restores SIMD control word for each query (safer but slower). Omit this if you took care of it yourself in your app.
    SimdGuard = 1,
}

bitflags::bitflags! {
    /// Flags for [`PxGeometryQueryFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxGeometryQueryFlags: u32 {
        const SimdGuard = 1 << 0;
    }
}

/// Desired build strategy for bounding-volume hierarchies
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxBVHBuildStrategy {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
}

/// Flags controlling the simulated behavior of the convex mesh geometry.
///
/// Used in ::PxConvexMeshGeometryFlags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConvexMeshGeometryFlag {
    /// Use tighter (but more expensive to compute) bounds around the convex geometry.
    TightBounds = 1,
}

bitflags::bitflags! {
    /// Flags for [`PxConvexMeshGeometryFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxConvexMeshGeometryFlags: u8 {
        const TightBounds = 1 << 0;
    }
}

/// Flags controlling the simulated behavior of the triangle mesh geometry.
///
/// Used in ::PxMeshGeometryFlags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMeshGeometryFlag {
    /// Use tighter (but more expensive to compute) bounds around the triangle mesh geometry.
    TightBounds = 1,
    /// Meshes with this flag set are treated as double-sided.
    /// This flag is currently only used for raycasts and sweeps (it is ignored for overlap queries).
    /// For detailed specifications of this flag for meshes and heightfields please refer to the Geometry Query section of the user guide.
    DoubleSided = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxMeshGeometryFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxMeshGeometryFlags: u8 {
        const TightBounds = 1 << 0;
        const DoubleSided = 1 << 1;
    }
}

/// Identifies the solver to use for a particle system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxParticleSolverType {
    /// The position based dynamics solver that can handle fluid, granular material, cloth, inflatables etc. See [`PxPBDParticleSystem`].
    Pbd = 1,
    /// The FLIP fluid solver. See [`PxFLIPParticleSystem`].
    Flip = 2,
    /// The MPM (material point method) solver that can handle a variety of materials. See [`PxMPMParticleSystem`].
    Mpm = 4,
    /// Custom solver. The user needs to specify the interaction of the particle by providing appropriate functions. Can be used e.g. for molecular dynamics simulations. See [`PxCustomParticleSystem`].
    Custom = 8,
}

/// Scene query and geometry query behavior flags.
///
/// PxHitFlags are used for 3 different purposes:
///
/// 1) To request hit fields to be filled in by scene queries (such as hit position, normal, face index or UVs).
/// 2) Once query is completed, to indicate which fields are valid (note that a query may produce more valid fields than requested).
/// 3) To specify additional options for the narrow phase and mid-phase intersection routines.
///
/// All these flags apply to both scene queries and geometry queries (PxGeometryQuery).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHitFlag {
    /// "position" member of [`PxQueryHit`] is valid
    Position = 1,
    /// "normal" member of [`PxQueryHit`] is valid
    Normal = 2,
    /// "u" and "v" barycentric coordinates of [`PxQueryHit`] are valid. Not applicable to sweep queries.
    Uv = 8,
    /// Performance hint flag for sweeps when it is known upfront there's no initial overlap.
    /// NOTE: using this flag may cause undefined results if shapes are initially overlapping.
    AssumeNoInitialOverlap = 16,
    /// Report any first hit. Used for geometries that contain more than one primitive. For meshes,
    /// if neither eMESH_MULTIPLE nor eANY_HIT is specified, a single closest hit will be reported.
    AnyHit = 32,
    /// Report all hits for meshes rather than just the first. Not applicable to sweep queries.
    MeshMultiple = 64,
    /// Report hits with back faces of mesh triangles. Also report hits for raycast
    /// originating on mesh surface and facing away from the surface normal. Not applicable to sweep queries.
    /// Please refer to the user guide for heightfield-specific differences.
    MeshBothSides = 128,
    /// Use more accurate but slower narrow phase sweep tests.
    /// May provide better compatibility with PhysX 3.2 sweep behavior.
    PreciseSweep = 256,
    /// Report the minimum translation depth, normal and contact point.
    Mtd = 512,
    /// "face index" member of [`PxQueryHit`] is valid
    FaceIndex = 1024,
    Default = 1027,
    /// Only this subset of flags can be modified by pre-filter. Other modifications will be discarded.
    ModifiableFlags = 464,
}

bitflags::bitflags! {
    /// Flags for [`PxHitFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxHitFlags: u16 {
        const Position = 1 << 0;
        const Normal = 1 << 1;
        const Uv = 1 << 3;
        const AssumeNoInitialOverlap = 1 << 4;
        const AnyHit = 1 << 5;
        const MeshMultiple = 1 << 6;
        const MeshBothSides = 1 << 7;
        const PreciseSweep = 1 << 8;
        const Mtd = 1 << 9;
        const FaceIndex = 1 << 10;
        const Default = Self::Position.bits | Self::Normal.bits | Self::FaceIndex.bits;
        const ModifiableFlags = Self::AssumeNoInitialOverlap.bits | Self::MeshMultiple.bits | Self::MeshBothSides.bits | Self::PreciseSweep.bits;
    }
}

/// Describes the format of height field samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHeightFieldFormat {
    /// Height field height data is 16 bit signed integers, followed by triangle materials.
    ///
    /// Each sample is 32 bits wide arranged as follows:
    ///
    /// 1) First there is a 16 bit height value.
    /// 2) Next, two one byte material indices, with the high bit of each byte reserved for special use.
    /// (so the material index is only 7 bits).
    /// The high bit of material0 is the tess-flag.
    /// The high bit of material1 is reserved for future use.
    ///
    /// There are zero or more unused bytes before the next sample depending on PxHeightFieldDesc.sampleStride,
    /// where the application may eventually keep its own data.
    ///
    /// This is the only format supported at the moment.
    S16Tm = 1,
}

/// Determines the tessellation of height field cells.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHeightFieldTessFlag {
    /// This flag determines which way each quad cell is subdivided.
    ///
    /// The flag lowered indicates subdivision like this: (the 0th vertex is referenced by only one triangle)
    ///
    /// +--+--+--+---> column
    /// | /| /| /|
    /// |/ |/ |/ |
    /// +--+--+--+
    /// | /| /| /|
    /// |/ |/ |/ |
    /// +--+--+--+
    /// |
    /// |
    /// V row
    ///
    /// The flag raised indicates subdivision like this: (the 0th vertex is shared by two triangles)
    ///
    /// +--+--+--+---> column
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// +--+--+--+
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// |
    /// \
    /// |
    /// \
    /// |
    /// \
    /// |
    /// +--+--+--+
    /// |
    /// |
    /// V row
    E0ThVertexShared = 1,
}

/// Enum with flag values to be used in PxHeightFieldDesc.flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHeightFieldFlag {
    /// Disable collisions with height field with boundary edges.
    ///
    /// Raise this flag if several terrain patches are going to be placed adjacent to each other,
    /// to avoid a bump when sliding across.
    ///
    /// This flag is ignored in contact generation with sphere and capsule shapes.
    NoBoundaryEdges = 1,
}

bitflags::bitflags! {
    /// Flags for [`PxHeightFieldFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxHeightFieldFlags: u16 {
        const NoBoundaryEdges = 1 << 0;
    }
}

/// Special material index values for height field samples.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHeightFieldMaterial {
    /// A material indicating that the triangle should be treated as a hole in the mesh.
    Hole = 127,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMeshMeshQueryFlag {
    /// Report all overlaps
    Default = 0,
    /// Ignore coplanar triangle-triangle overlaps
    DiscardCoplanar = 1,
}

bitflags::bitflags! {
    /// Flags for [`PxMeshMeshQueryFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxMeshMeshQueryFlags: u32 {
        const DiscardCoplanar = 1 << 0;
    }
}

/// Enum with flag values to be used in PxSimpleTriangleMesh::flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMeshFlag {
    /// Specifies if the SDK should flip normals.
    ///
    /// The PhysX libraries assume that the face normal of a triangle with vertices [a,b,c] can be computed as:
    /// edge1 = b-a
    /// edge2 = c-a
    /// face_normal = edge1 x edge2.
    ///
    /// Note: This is the same as a counterclockwise winding in a right handed coordinate system or
    /// alternatively a clockwise winding order in a left handed coordinate system.
    ///
    /// If this does not match the winding order for your triangles, raise the below flag.
    Flipnormals = 1,
    /// Denotes the use of 16-bit vertex indices
    E16BitIndices = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxMeshFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxMeshFlags: u16 {
        const Flipnormals = 1 << 0;
        const E16BitIndices = 1 << 1;
    }
}

/// Mesh midphase structure. This enum is used to select the desired acceleration structure for midphase queries
/// (i.e. raycasts, overlaps, sweeps vs triangle meshes).
///
/// The PxMeshMidPhase::eBVH33 structure is the one used in recent PhysX versions (up to PhysX 3.3). It has great performance and is
/// supported on all platforms. It is deprecated since PhysX 5.x.
///
/// The PxMeshMidPhase::eBVH34 structure is a revisited implementation introduced in PhysX 3.4. It can be significantly faster both
/// in terms of cooking performance and runtime performance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMeshMidPhase {
    /// Default midphase mesh structure, as used up to PhysX 3.3 (deprecated)
    Bvh33 = 0,
    /// New midphase mesh structure, introduced in PhysX 3.4
    Bvh34 = 1,
    Last = 2,
}

/// Flags for the mesh geometry properties.
///
/// Used in ::PxTriangleMeshFlags.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxTriangleMeshFlag {
    /// The triangle mesh has 16bits vertex indices.
    E16BitIndices = 2,
    /// The triangle mesh has adjacency information build.
    AdjacencyInfo = 4,
    /// Indicates that this mesh would preferably not be the mesh projected for mesh-mesh collision. This can indicate that the mesh is not well tessellated.
    PreferNoSdfProj = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxTriangleMeshFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxTriangleMeshFlags: u8 {
        const E16BitIndices = 1 << 1;
        const AdjacencyInfo = 1 << 2;
        const PreferNoSdfProj = 1 << 3;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxTetrahedronMeshFlag {
    /// The tetrahedron mesh has 16bits vertex indices
    E16BitIndices = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxTetrahedronMeshFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxTetrahedronMeshFlags: u8 {
        const E16BitIndices = 1 << 1;
    }
}

/// Flags which control the behavior of an actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxActorFlag {
    /// Enable debug renderer for this actor
    Visualization = 1,
    /// Disables scene gravity for this actor
    DisableGravity = 2,
    /// Enables the sending of PxSimulationEventCallback::onWake() and PxSimulationEventCallback::onSleep() notify events
    SendSleepNotifies = 4,
    /// Disables simulation for the actor.
    ///
    /// This is only supported by PxRigidStatic and PxRigidDynamic actors and can be used to reduce the memory footprint when rigid actors are
    /// used for scene queries only.
    ///
    /// Setting this flag will remove all constraints attached to the actor from the scene.
    ///
    /// If this flag is set, the following calls are forbidden:
    ///
    /// PxRigidBody: setLinearVelocity(), setAngularVelocity(), addForce(), addTorque(), clearForce(), clearTorque(), setForceAndTorque()
    ///
    /// PxRigidDynamic: setKinematicTarget(), setWakeCounter(), wakeUp(), putToSleep()
    ///
    /// Sleeping:
    /// Raising this flag will set all velocities and the wake counter to 0, clear all forces, clear the kinematic target, put the actor
    /// to sleep and wake up all touching actors from the previous frame.
    DisableSimulation = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxActorFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxActorFlags: u8 {
        const Visualization = 1 << 0;
        const DisableGravity = 1 << 1;
        const SendSleepNotifies = 1 << 2;
        const DisableSimulation = 1 << 3;
    }
}

/// Identifies each type of actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxActorType {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation link
    ArticulationLink = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxAggregateType {
    /// Aggregate will contain various actors of unspecified types
    Generic = 0,
    /// Aggregate will only contain static actors
    Static = 1,
    /// Aggregate will only contain kinematic actors
    Kinematic = 2,
}

/// Constraint row flags
///
/// These flags configure the post-processing of constraint rows and the behavior of the solver while solving constraints
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum Px1DConstraintFlag {
    /// whether the constraint is a spring. Mutually exclusive with eRESTITUTION. If set, eKEEPBIAS is ignored.
    Spring = 1,
    /// whether the constraint is a force or acceleration spring. Only valid if eSPRING is set.
    AccelerationSpring = 2,
    /// whether the restitution model should be applied to generate the target velocity. Mutually exclusive with eSPRING. If restitution causes a bounces, eKEEPBIAS is ignored
    Restitution = 4,
    /// whether to keep the error term when solving for velocity. Ignored if restitution generates bounce, or eSPRING is set.
    Keepbias = 8,
    /// whether to accumulate the force value from this constraint in the force total that is reported for the constraint and tested for breakage
    OutputForce = 16,
    /// whether the constraint has a drive force limit (which will be scaled by dt unless PxConstraintFlag::eLIMITS_ARE_FORCES is set)
    HasDriveLimit = 32,
    /// whether this is an angular or linear constraint
    AngularConstraint = 64,
    /// whether the constraint's geometric error should drive the target velocity
    DriveRow = 128,
}

bitflags::bitflags! {
    /// Flags for [`Px1DConstraintFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct Px1DConstraintFlags: u16 {
        const Spring = 1 << 0;
        const AccelerationSpring = 1 << 1;
        const Restitution = 1 << 2;
        const Keepbias = 1 << 3;
        const OutputForce = 1 << 4;
        const HasDriveLimit = 1 << 5;
        const AngularConstraint = 1 << 6;
        const DriveRow = 1 << 7;
    }
}

/// Constraint type hints which the solver uses to optimize constraint handling
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConstraintSolveHint {
    /// no special properties
    None = 0,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration1 = 256,
    /// temporary special value to identify SLERP drive rows
    SlerpSpring = 258,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration2 = 512,
    /// a group of acceleration drive constraints with the same stiffness and drive parameters
    Acceleration3 = 768,
    /// rotational equality constraints with no force limit and no velocity target
    RotationalEquality = 1024,
    /// rotational inequality constraints with (0, PX_MAX_FLT) force limits
    RotationalInequality = 1025,
    /// equality constraints with no force limit and no velocity target
    Equality = 2048,
    /// inequality constraints with (0, PX_MAX_FLT) force limits
    Inequality = 2049,
}

/// Flags for determining which components of the constraint should be visualized.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConstraintVisualizationFlag {
    /// visualize constraint frames
    LocalFrames = 1,
    /// visualize constraint limits
    Limits = 2,
}

/// Flags for determining how PVD should serialize a constraint update
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPvdUpdateType {
    /// triggers createPvdInstance call, creates an instance of a constraint
    CreateInstance = 0,
    /// triggers releasePvdInstance call, releases an instance of a constraint
    ReleaseInstance = 1,
    /// triggers updatePvdProperties call, updates all properties of a constraint
    UpdateAllProperties = 2,
    /// triggers simUpdate call, updates all simulation properties of a constraint
    UpdateSimProperties = 3,
}

/// Constraint descriptor used inside the solver
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum ConstraintType {
    /// Defines this pair is a contact constraint
    ContactConstraint = 0,
    /// Defines this pair is a joint constraint
    JointConstraint = 1,
}

/// Data structure used for preparing constraints before solving them
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum BodyState {
    DynamicBody = 1,
    StaticBody = 2,
    KinematicBody = 4,
    Articulation = 8,
}

/// @
/// {
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationAxis {
    /// Rotational about eX
    Twist = 0,
    /// Rotational about eY
    Swing1 = 1,
    /// Rotational about eZ
    Swing2 = 2,
    /// Linear in eX
    X = 3,
    /// Linear in eY
    Y = 4,
    /// Linear in eZ
    Z = 5,
    Count = 6,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationMotion {
    /// Locked axis, i.e. degree of freedom (DOF)
    Locked = 0,
    /// Limited DOF - set limits of joint DOF together with this flag, see PxArticulationJointReducedCoordinate::setLimitParams
    Limited = 1,
    /// Free DOF
    Free = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxArticulationMotion`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxArticulationMotions: u8 {
        const Limited = 1 << 0;
        const Free = 1 << 1;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationJointType {
    /// All joint axes, i.e. degrees of freedom (DOFs) locked
    Fix = 0,
    /// Single linear DOF, e.g. cart on a rail
    Prismatic = 1,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position wrapped at 2pi radians
    Revolute = 2,
    /// Single rotational DOF, e.g. an elbow joint or a rotational motor, position not wrapped
    RevoluteUnwrapped = 3,
    /// Ball and socket joint with two or three DOFs
    Spherical = 4,
    Undefined = 5,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationFlag {
    /// Set articulation base to be fixed.
    FixBase = 1,
    /// Limits for drive effort are forces and torques rather than impulses, see PxArticulationDrive::maxForce.
    DriveLimitsAreForces = 2,
    /// Disable collisions between the articulation's links (note that parent/child collisions are disabled internally in either case).
    DisableSelfCollision = 4,
    /// Enable in order to be able to query joint solver (i.e. constraint) forces using PxArticulationCache::jointSolverForces.
    ComputeJointForces = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxArticulationFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxArticulationFlags: u8 {
        const FixBase = 1 << 0;
        const DriveLimitsAreForces = 1 << 1;
        const DisableSelfCollision = 1 << 2;
        const ComputeJointForces = 1 << 3;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationDriveType {
    /// The output of the implicit spring drive controller is a force/torque.
    Force = 0,
    /// The output of the implicit spring drive controller is a joint acceleration (use this to get (spatial)-inertia-invariant behavior of the drive).
    Acceleration = 1,
    /// Sets the drive gains internally to track a target position almost kinematically (i.e. with very high drive gains).
    Target = 2,
    /// Sets the drive gains internally to track a target velocity almost kinematically (i.e. with very high drive gains).
    Velocity = 3,
    None = 4,
}

/// A description of the types of articulation data that may be directly written to and read from the GPU using the functions
/// PxScene::copyArticulationData() and PxScene::applyArticulationData(). Types that are read-only may only be used in conjunction with
/// PxScene::copyArticulationData(). Types that are write-only may only be used in conjunction with PxScene::applyArticulationData().
/// A subset of data types may be used in conjunction with both PxScene::applyArticulationData() and PxScene::applyArticulationData().
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationGpuDataType {
    /// The joint positions, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    JointPosition = 0,
    /// The joint velocities, read and write,  see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    JointVelocity = 1,
    /// The joint accelerations, read only, see PxScene::copyArticulationData()
    JointAcceleration = 2,
    /// The applied joint forces, write only, see PxScene::applyArticulationData()
    JointForce = 3,
    /// The computed joint constraint solver forces, read only, see PxScene::copyArticulationData()()
    JointSolverForce = 4,
    /// The velocity targets for the joint drives, write only, see PxScene::applyArticulationData()
    JointTargetVelocity = 5,
    /// The position targets for the joint drives, write only, see PxScene::applyArticulationData()
    JointTargetPosition = 6,
    /// The spatial sensor forces, read only, see PxScene::copyArticulationData()
    SensorForce = 7,
    /// The root link transform, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    RootTransform = 8,
    /// The root link velocity, read and write, see PxScene::copyArticulationData(), PxScene::applyArticulationData()
    RootVelocity = 9,
    /// The link transforms including root link, read only, see PxScene::copyArticulationData()
    LinkTransform = 10,
    /// The link velocities including root link, read only, see PxScene::copyArticulationData()
    LinkVelocity = 11,
    /// The forces to apply to links, write only, see PxScene::applyArticulationData()
    LinkForce = 12,
    /// The torques to apply to links, write only, see PxScene::applyArticulationData()
    LinkTorque = 13,
    /// Fixed tendon data, write only, see PxScene::applyArticulationData()
    FixedTendon = 14,
    /// Fixed tendon joint data, write only, see PxScene::applyArticulationData()
    FixedTendonJoint = 15,
    /// Spatial tendon data, write only, see PxScene::applyArticulationData()
    SpatialTendon = 16,
    /// Spatial tendon attachment data, write only, see PxScene::applyArticulationData()
    SpatialTendonAttachment = 17,
}

/// These flags determine what data is read or written to the internal articulation data via cache.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationCacheFlag {
    /// The joint velocities, see PxArticulationCache::jointVelocity.
    Velocity = 1,
    /// The joint accelerations, see PxArticulationCache::jointAcceleration.
    Acceleration = 2,
    /// The joint positions, see PxArticulationCache::jointPosition.
    Position = 4,
    /// The joint forces, see PxArticulationCache::jointForce.
    Force = 8,
    /// The link velocities, see PxArticulationCache::linkVelocity.
    LinkVelocity = 16,
    /// The link accelerations, see PxArticulationCache::linkAcceleration.
    LinkAcceleration = 32,
    /// Root link transform, see PxArticulationCache::rootLinkData.
    RootTransform = 64,
    /// Root link velocities (read/write) and accelerations (read), see PxArticulationCache::rootLinkData.
    RootVelocities = 128,
    /// The spatial sensor forces, see PxArticulationCache::sensorForces.
    SensorForces = 256,
    /// Solver constraint joint forces, see PxArticulationCache::jointSolverForces.
    JointSolverForces = 512,
    All = 247,
}

bitflags::bitflags! {
    /// Flags for [`PxArticulationCacheFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxArticulationCacheFlags: u32 {
        const Velocity = 1 << 0;
        const Acceleration = 1 << 1;
        const Position = 1 << 2;
        const Force = 1 << 3;
        const LinkVelocity = 1 << 4;
        const LinkAcceleration = 1 << 5;
        const RootTransform = 1 << 6;
        const RootVelocities = 1 << 7;
        const SensorForces = 1 << 8;
        const JointSolverForces = 1 << 9;
        const All = Self::Velocity.bits | Self::Acceleration.bits | Self::Position.bits | Self::LinkVelocity.bits | Self::LinkAcceleration.bits | Self::RootTransform.bits | Self::RootVelocities.bits;
    }
}

/// Flags to configure the forces reported by articulation link sensors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationSensorFlag {
    /// Raise to receive forces from forward dynamics.
    ForwardDynamicsForces = 1,
    /// Raise to receive forces from constraint solver.
    ConstraintSolverForces = 2,
    /// Raise to receive forces in the world rotation frame, otherwise they will be reported in the sensor's local frame.
    WorldFrame = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxArticulationSensorFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxArticulationSensorFlags: u8 {
        const ForwardDynamicsForces = 1 << 0;
        const ConstraintSolverForces = 1 << 1;
        const WorldFrame = 1 << 2;
    }
}

/// Flag that configures articulation-state updates by PxArticulationReducedCoordinate::updateKinematic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxArticulationKinematicFlag {
    /// Raise after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    Position = 1,
    /// Raise after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities.
    Velocity = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxArticulationKinematicFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxArticulationKinematicFlags: u8 {
        const Position = 1 << 0;
        const Velocity = 1 << 1;
    }
}

/// Flags which affect the behavior of PxShapes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxShapeFlag {
    /// The shape will partake in collision in the physical simulation.
    ///
    /// It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
    /// In the event that one of these flags is already raised the sdk will reject any
    /// attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that
    /// eTRIGGER_SHAPE is already lowered.
    ///
    /// This flag has no effect if simulation is disabled for the corresponding actor (see [`PxActorFlag::eDISABLE_SIMULATION`]).
    SimulationShape = 1,
    /// The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
    SceneQueryShape = 2,
    /// The shape is a trigger which can send reports whenever other shapes enter/leave its volume.
    ///
    /// Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.
    ///
    /// Shapes marked as triggers do not collide with other objects. If an object should act both
    /// as a trigger shape and a collision shape then create a rigid body with two shapes, one being a
    /// trigger shape and the other a collision shape. It is illegal to raise the eTRIGGER_SHAPE and
    /// eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already
    /// raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first
    /// ensure that eSIMULATION_SHAPE flag is already lowered.
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    ///
    /// Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE flag is set.
    ///
    /// This flag has no effect if simulation is disabled for the corresponding actor (see [`PxActorFlag::eDISABLE_SIMULATION`]).
    TriggerShape = 4,
    /// Enable debug renderer for this shape
    Visualization = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxShapeFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxShapeFlags: u8 {
        const SimulationShape = 1 << 0;
        const SceneQueryShape = 1 << 1;
        const TriggerShape = 1 << 2;
        const Visualization = 1 << 3;
    }
}

/// Parameter to addForce() and addTorque() calls, determines the exact operation that is carried out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxForceMode {
    /// parameter has unit of mass * length / time^2, i.e., a force
    Force = 0,
    /// parameter has unit of mass * length / time, i.e., force * time
    Impulse = 1,
    /// parameter has unit of length / time, i.e., the effect is mass independent: a velocity change.
    VelocityChange = 2,
    /// parameter has unit of length/ time^2, i.e., an acceleration. It gets treated just like a force except the mass is not divided out before integration.
    Acceleration = 3,
}

/// Collection of flags describing the behavior of a rigid body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxRigidBodyFlag {
    /// Enable kinematic mode for the body.
    Kinematic = 1,
    /// Use the kinematic target transform for scene queries.
    ///
    /// If this flag is raised, then scene queries will treat the kinematic target transform as the current pose
    /// of the body (instead of using the actual pose). Without this flag, the kinematic target will only take
    /// effect with respect to scene queries after a simulation step.
    UseKinematicTargetForSceneQueries = 2,
    /// Enable CCD for the body.
    EnableCcd = 4,
    /// Enabled CCD in swept integration for the actor.
    ///
    /// If this flag is raised and CCD is enabled, CCD interactions will simulate friction. By default, friction is disabled in CCD interactions because
    /// CCD friction has been observed to introduce some simulation artifacts. CCD friction was enabled in previous versions of the SDK. Raising this flag will result in behavior
    /// that is a closer match for previous versions of the SDK.
    ///
    /// This flag requires PxRigidBodyFlag::eENABLE_CCD to be raised to have any effect.
    EnableCcdFriction = 8,
    /// Register a rigid body to dynamically adjust contact offset based on velocity. This can be used to achieve a CCD effect.
    ///
    /// If both eENABLE_CCD and eENABLE_SPECULATIVE_CCD are set on the same body, then angular motions are handled by speculative
    /// contacts (eENABLE_SPECULATIVE_CCD) while linear motions are handled by sweeps (eENABLE_CCD).
    EnableSpeculativeCcd = 16,
    /// Register a rigid body for reporting pose changes by the simulation at an early stage.
    ///
    /// Sometimes it might be advantageous to get access to the new pose of a rigid body as early as possible and
    /// not wait until the call to fetchResults() returns. Setting this flag will schedule the rigid body to get reported
    /// in [`PxSimulationEventCallback::onAdvance`](). Please refer to the documentation of that callback to understand
    /// the behavior and limitations of this functionality.
    EnablePoseIntegrationPreview = 32,
    /// Permit CCD to limit maxContactImpulse. This is useful for use-cases like a destruction system but can cause visual artefacts so is not enabled by default.
    EnableCcdMaxContactImpulse = 64,
    /// Carries over forces/accelerations between frames, rather than clearing them
    RetainAccelerations = 128,
    /// Forces kinematic-kinematic pairs notifications for this actor.
    ///
    /// This flag overrides the global scene-level PxPairFilteringMode setting for kinematic actors.
    /// This is equivalent to having PxPairFilteringMode::eKEEP for pairs involving this actor.
    ///
    /// A particular use case is when you have a large amount of kinematic actors, but you are only
    /// interested in interactions between a few of them. In this case it is best to use
    /// PxSceneDesc.kineKineFilteringMode = PxPairFilteringMode::eKILL, and then raise the
    /// eFORCE_KINE_KINE_NOTIFICATIONS flag on the small set of kinematic actors that need
    /// notifications.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    ForceKineKineNotifications = 256,
    /// Forces static-kinematic pairs notifications for this actor.
    ///
    /// Similar to eFORCE_KINE_KINE_NOTIFICATIONS, but for static-kinematic interactions.
    ///
    /// This has no effect if PxRigidBodyFlag::eKINEMATIC is not set.
    ///
    /// Changing this flag at runtime will not have an effect until you remove and re-add the actor to the scene.
    ForceStaticKineNotifications = 512,
    /// Enables computation of gyroscopic forces on the rigid body.
    EnableGyroscopicForces = 1024,
}

bitflags::bitflags! {
    /// Flags for [`PxRigidBodyFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxRigidBodyFlags: u16 {
        const Kinematic = 1 << 0;
        const UseKinematicTargetForSceneQueries = 1 << 1;
        const EnableCcd = 1 << 2;
        const EnableCcdFriction = 1 << 3;
        const EnableSpeculativeCcd = 1 << 4;
        const EnablePoseIntegrationPreview = 1 << 5;
        const EnableCcdMaxContactImpulse = 1 << 6;
        const RetainAccelerations = 1 << 7;
        const ForceKineKineNotifications = 1 << 8;
        const ForceStaticKineNotifications = 1 << 9;
        const EnableGyroscopicForces = 1 << 10;
    }
}

/// constraint flags
///
/// eBROKEN is a read only flag
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConstraintFlag {
    /// whether the constraint is broken
    Broken = 1,
    /// whether actor1 should get projected to actor0 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    ProjectToActor0 = 2,
    /// whether actor0 should get projected to actor1 for this constraint (note: projection of a static/kinematic actor to a dynamic actor will be ignored)
    ProjectToActor1 = 4,
    /// whether the actors should get projected for this constraint (the direction will be chosen by PhysX)
    Projection = 6,
    /// whether contacts should be generated between the objects this constraint constrains
    CollisionEnabled = 8,
    /// whether this constraint should be visualized, if constraint visualization is turned on
    Visualization = 16,
    /// limits for drive strength are forces rather than impulses
    DriveLimitsAreForces = 32,
    /// perform preprocessing for improved accuracy on D6 Slerp Drive (this flag will be removed in a future release when preprocessing is no longer required)
    ImprovedSlerp = 128,
    /// suppress constraint preprocessing, intended for use with rowResponseThreshold. May result in worse solver accuracy for ill-conditioned constraints.
    DisablePreprocessing = 256,
    /// enables extended limit ranges for angular limits (e.g., limit values > PxPi or
    /// <
    /// -PxPi)
    EnableExtendedLimits = 512,
    /// the constraint type is supported by gpu dynamics
    GpuCompatible = 1024,
    /// updates the constraint each frame
    AlwaysUpdate = 2048,
    /// disables the constraint. SolverPrep functions won't be called for this constraint.
    DisableConstraint = 4096,
}

bitflags::bitflags! {
    /// Flags for [`PxConstraintFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxConstraintFlags: u16 {
        const Broken = 1 << 0;
        const ProjectToActor0 = 1 << 1;
        const ProjectToActor1 = 1 << 2;
        const Projection = Self::ProjectToActor0.bits | Self::ProjectToActor1.bits;
        const CollisionEnabled = 1 << 3;
        const Visualization = 1 << 4;
        const DriveLimitsAreForces = 1 << 5;
        const ImprovedSlerp = 1 << 7;
        const DisablePreprocessing = 1 << 8;
        const EnableExtendedLimits = 1 << 9;
        const GpuCompatible = 1 << 10;
        const AlwaysUpdate = 1 << 11;
        const DisableConstraint = 1 << 12;
    }
}

/// Header for a contact patch where all points share same material and normal
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxContactPatchFlags {
    /// Indicates this contact stream has face indices.
    HasFaceIndices = 1,
    /// Indicates this contact stream is modifiable.
    Modifiable = 2,
    /// Indicates this contact stream is notify-only (no contact response).
    ForceNoResponse = 4,
    /// Indicates this contact stream has modified mass ratios
    HasModifiedMassRatios = 8,
    /// Indicates this contact stream has target velocities set
    HasTargetVelocity = 16,
    /// Indicates this contact stream has max impulses set
    HasMaxImpulse = 32,
    /// Indicates this contact stream needs patches re-generated. This is required if the application modified either the contact normal or the material properties
    RegeneratePatches = 64,
    CompressedModifiedContact = 128,
}

/// A class to iterate over a compressed contact stream. This supports read-only access to the various contact formats.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum StreamFormat {
    SimpleStream = 0,
    ModifiableStream = 1,
    CompressedModifiableStream = 2,
}

/// Flags specifying deletion event types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxDeletionEventFlag {
    /// The user has called release on an object.
    UserRelease = 1,
    /// The destructor of an object has been called and the memory has been released.
    MemoryRelease = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxDeletionEventFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxDeletionEventFlags: u8 {
        const UserRelease = 1 << 0;
        const MemoryRelease = 1 << 1;
    }
}

/// Collection of flags describing the actions to take for a collision pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPairFlag {
    /// Process the contacts of this collision pair in the dynamics solver.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    SolveContact = 1,
    /// Call contact modification callback for this collision pair
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ModifyContacts = 2,
    /// Call contact report callback or trigger callback when this collision pair starts to be in contact.
    ///
    /// If one of the two collision objects is a trigger shape (see [`PxShapeFlag::eTRIGGER_SHAPE`])
    /// then the trigger callback will get called as soon as the other object enters the trigger volume.
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called when the actors of this collision pair start to be in contact.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyTouchFound = 4,
    /// Call contact report callback while this collision pair is in contact
    ///
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called while the actors of this collision pair are in contact.
    ///
    /// Triggers do not support this event. Persistent trigger contacts need to be tracked separately by observing eNOTIFY_TOUCH_FOUND/eNOTIFY_TOUCH_LOST events.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// No report will get sent if the objects in contact are sleeping.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    ///
    /// If this flag gets enabled while a pair is in touch already, there will be no eNOTIFY_TOUCH_PERSISTS events until the pair loses and regains touch.
    NotifyTouchPersists = 8,
    /// Call contact report callback or trigger callback when this collision pair stops to be in contact
    ///
    /// If one of the two collision objects is a trigger shape (see [`PxShapeFlag::eTRIGGER_SHAPE`])
    /// then the trigger callback will get called as soon as the other object leaves the trigger volume.
    /// If none of the two collision objects is a trigger shape then the contact report callback will get
    /// called when the actors of this collision pair stop to be in contact.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// This event will also get triggered if one of the colliding objects gets deleted.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyTouchLost = 16,
    /// Call contact report callback when this collision pair is in contact during CCD passes.
    ///
    /// If CCD with multiple passes is enabled, then a fast moving object might bounce on and off the same
    /// object multiple times. Hence, the same pair might be in contact multiple times during a simulation step.
    /// This flag will make sure that all the detected collision during CCD will get reported. For performance
    /// reasons, the system can not always tell whether the contact pair lost touch in one of the previous CCD
    /// passes and thus can also not always tell whether the contact is new or has persisted. eNOTIFY_TOUCH_CCD
    /// just reports when the two collision objects were detected as being in contact during a CCD pass.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Trigger shapes are not supported.
    ///
    /// Only takes effect if eDETECT_CCD_CONTACT is raised
    NotifyTouchCcd = 32,
    /// Call contact report callback when the contact force between the actors of this collision pair exceeds one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForceFound = 64,
    /// Call contact report callback when the contact force between the actors of this collision pair continues to exceed one of the actor-defined force thresholds.
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForcePersists = 128,
    /// Call contact report callback when the contact force between the actors of this collision pair falls below one of the actor-defined force thresholds (includes the case where this collision pair stops being in contact).
    ///
    /// Only takes effect if the colliding actors are rigid bodies.
    ///
    /// If a pair gets re-filtered and this flag has previously been disabled, then the report will not get fired in the same frame even if the force threshold has been reached in the
    /// previous one (unless [`eNOTIFY_THRESHOLD_FORCE_FOUND`] or #eNOTIFY_THRESHOLD_FORCE_PERSISTS has been set in the previous frame).
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyThresholdForceLost = 256,
    /// Provide contact points in contact reports for this collision pair.
    ///
    /// Only takes effect if the colliding actors are rigid bodies and if used in combination with the flags eNOTIFY_TOUCH_... or eNOTIFY_THRESHOLD_FORCE_...
    ///
    /// Only takes effect if eDETECT_DISCRETE_CONTACT or eDETECT_CCD_CONTACT is raised
    NotifyContactPoints = 512,
    /// This flag is used to indicate whether this pair generates discrete collision detection contacts.
    ///
    /// Contacts are only responded to if eSOLVE_CONTACT is enabled.
    DetectDiscreteContact = 1024,
    /// This flag is used to indicate whether this pair generates CCD contacts.
    ///
    /// The contacts will only be responded to if eSOLVE_CONTACT is enabled on this pair.
    ///
    /// The scene must have PxSceneFlag::eENABLE_CCD enabled to use this feature.
    ///
    /// Non-static bodies of the pair should have PxRigidBodyFlag::eENABLE_CCD specified for this feature to work correctly.
    ///
    /// This flag is not supported with trigger shapes. However, CCD trigger events can be emulated using non-trigger shapes
    /// and requesting eNOTIFY_TOUCH_FOUND and eNOTIFY_TOUCH_LOST and not raising eSOLVE_CONTACT on the pair.
    DetectCcdContact = 2048,
    /// Provide pre solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies before contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these velocities as they will be available by querying the velocity from the provided
    /// PxRigidActor object directly. However, it might be the case that the velocity of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new velocity in the contact report callback and not the velocity the simulation used.
    PreSolverVelocity = 4096,
    /// Provide post solver velocities in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the velocities of the rigid bodies after contacts have been solved
    /// will be provided in the contact report callback unless the pair lost touch in which case no data will be provided.
    PostSolverVelocity = 8192,
    /// Provide rigid body poses in contact reports for this collision pair.
    ///
    /// If the collision pair has contact reports enabled, the rigid body poses at the contact event will be provided
    /// in the contact report callback unless the pair lost touch in which case no data will be provided.
    ///
    /// Usually it is not necessary to request these poses as they will be available by querying the pose from the provided
    /// PxRigidActor object directly. However, it might be the case that the pose of a rigid body gets set while the simulation is running
    /// in which case the PxRigidActor would return this new pose in the contact report callback and not the pose the simulation used.
    /// Another use case is related to CCD with multiple passes enabled, A fast moving object might bounce on and off the same
    /// object multiple times. This flag can be used to request the rigid body poses at the time of impact for each such collision event.
    ContactEventPose = 16384,
    /// For internal use only.
    NextFree = 32768,
    /// Provided default flag to do simple contact processing for this collision pair.
    ContactDefault = 1025,
    /// Provided default flag to get commonly used trigger behavior for this collision pair.
    TriggerDefault = 1044,
}

bitflags::bitflags! {
    /// Flags for [`PxPairFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxPairFlags: u16 {
        const SolveContact = 1 << 0;
        const ModifyContacts = 1 << 1;
        const NotifyTouchFound = 1 << 2;
        const NotifyTouchPersists = 1 << 3;
        const NotifyTouchLost = 1 << 4;
        const NotifyTouchCcd = 1 << 5;
        const NotifyThresholdForceFound = 1 << 6;
        const NotifyThresholdForcePersists = 1 << 7;
        const NotifyThresholdForceLost = 1 << 8;
        const NotifyContactPoints = 1 << 9;
        const DetectDiscreteContact = 1 << 10;
        const DetectCcdContact = 1 << 11;
        const PreSolverVelocity = 1 << 12;
        const PostSolverVelocity = 1 << 13;
        const ContactEventPose = 1 << 14;
        const NextFree = 1 << 15;
        const ContactDefault = Self::SolveContact.bits | Self::DetectDiscreteContact.bits;
        const TriggerDefault = Self::NotifyTouchFound.bits | Self::NotifyTouchLost.bits | Self::DetectDiscreteContact.bits;
    }
}

/// Collection of flags describing the filter actions to take for a collision pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxFilterFlag {
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap.
    ///
    /// Killed pairs will be ignored by the simulation and won't run through the filter again until one
    /// of the following occurs:
    ///
    /// The bounding volumes of the two objects overlap again (after being separated)
    ///
    /// The user enforces a re-filtering (see [`PxScene::resetFiltering`]())
    Kill = 1,
    /// Ignore the collision pair as long as the bounding volumes of the pair objects overlap or until filtering relevant data changes for one of the collision objects.
    ///
    /// Suppressed pairs will be ignored by the simulation and won't make another filter request until one
    /// of the following occurs:
    ///
    /// Same conditions as for killed pairs (see [`eKILL`])
    ///
    /// The filter data or the filter object attributes change for one of the collision objects
    Suppress = 2,
    /// Invoke the filter callback ([`PxSimulationFilterCallback::pairFound`]()) for this collision pair.
    Callback = 4,
    /// Track this collision pair with the filter callback mechanism.
    ///
    /// When the bounding volumes of the collision pair lose contact, the filter callback [`PxSimulationFilterCallback::pairLost`]()
    /// will be invoked. Furthermore, the filter status of the collision pair can be adjusted through [`PxSimulationFilterCallback::statusChange`]()
    /// once per frame (until a pairLost() notification occurs).
    Notify = 12,
    /// Provided default to get standard behavior:
    ///
    /// The application configure the pair's collision properties once when bounding volume overlap is found and
    /// doesn't get asked again about that pair until overlap status or filter properties changes, or re-filtering is requested.
    ///
    /// No notification is provided when bounding volume overlap is lost
    ///
    /// The pair will not be killed or suppressed, so collision detection will be processed
    Default = 0,
}

bitflags::bitflags! {
    /// Flags for [`PxFilterFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxFilterFlags: u16 {
        const Kill = 1 << 0;
        const Suppress = 1 << 1;
        const Callback = 1 << 2;
        const Notify = Self::Callback.bits;
    }
}

/// Identifies each type of filter object.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxFilterObjectType {
    /// A static rigid body
    RigidStatic = 0,
    /// A dynamic rigid body
    RigidDynamic = 1,
    /// An articulation
    Articulation = 2,
    /// A particle system
    Particlesystem = 3,
    /// A FEM-based soft body
    Softbody = 4,
    /// A FEM-based cloth
    ///
    /// In development
    Femcloth = 5,
    /// A hair system
    ///
    /// In development
    Hairsystem = 6,
    /// internal use only!
    MaxTypeCount = 16,
    /// internal use only!
    Undefined = 15,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxFilterObjectFlag {
    Kinematic = 16,
    Trigger = 32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPairFilteringMode {
    /// Output pair from BP, potentially send to user callbacks, create regular interaction object.
    ///
    /// Enable contact pair filtering between kinematic/static or kinematic/kinematic rigid bodies.
    ///
    /// By default contacts between these are suppressed (see [`PxFilterFlag::eSUPPRESS`]) and don't get reported to the filter mechanism.
    /// Use this mode if these pairs should go through the filtering pipeline nonetheless.
    ///
    /// This mode is not mutable, and must be set in PxSceneDesc at scene creation.
    Keep = 0,
    /// Output pair from BP, create interaction marker. Can be later switched to regular interaction.
    Suppress = 1,
    /// Don't output pair from BP. Cannot be later switched to regular interaction, needs "resetFiltering" call.
    Kill = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxDataAccessFlag {
    Readable = 1,
    Writable = 2,
    Device = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxDataAccessFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxDataAccessFlags: u8 {
        const Readable = 1 << 0;
        const Writable = 1 << 1;
        const Device = 1 << 2;
    }
}

/// Flags which control the behavior of a material.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMaterialFlag {
    /// If this flag is set, friction computations are always skipped between shapes with this material and any other shape.
    DisableFriction = 1,
    /// Whether to use strong friction.
    /// The difference between "normal" and "strong" friction is that the strong friction feature
    /// remembers the "friction error" between simulation steps. The friction is a force trying to
    /// hold objects in place (or slow them down) and this is handled in the solver. But since the
    /// solver is only an approximation, the result of the friction calculation can include a small
    /// "error" - e.g. a box resting on a slope should not move at all if the static friction is in
    /// action, but could slowly glide down the slope because of a small friction error in each
    /// simulation step. The strong friction counter-acts this by remembering the small error and
    /// taking it to account during the next simulation step.
    ///
    /// However, in some cases the strong friction could cause problems, and this is why it is
    /// possible to disable the strong friction feature by setting this flag. One example is
    /// raycast vehicles that are sliding fast across the surface, but still need a precise
    /// steering behavior. It may be a good idea to reenable the strong friction when objects
    /// are coming to a rest, to prevent them from slowly creeping down inclines.
    ///
    /// Note: This flag only has an effect if the PxMaterialFlag::eDISABLE_FRICTION bit is 0.
    DisableStrongFriction = 2,
    /// Whether to use the patch friction model.
    /// This flag only has an effect if PxFrictionType::ePATCH friction model is used.
    ///
    /// When using the patch friction model, up to 2 friction anchors are generated per patch. As the number of friction anchors
    /// can be smaller than the number of contacts, the normal force is accumulated over all contacts and used to compute friction
    /// for all anchors. Where there are more than 2 anchors, this can produce frictional behavior that is too strong (approximately 2x as strong
    /// as analytical models suggest).
    ///
    /// This flag causes the normal force to be distributed between the friction anchors such that the total amount of friction applied does not
    /// exceed the analytical results.
    ImprovedPatchFriction = 4,
    /// This flag has the effect of enabling an implicit spring model for the normal force computation.
    CompliantContact = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxMaterialFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxMaterialFlags: u16 {
        const DisableFriction = 1 << 0;
        const DisableStrongFriction = 1 << 1;
        const ImprovedPatchFriction = 1 << 2;
        const CompliantContact = 1 << 3;
    }
}

/// Enumeration that determines the way in which two material properties will be combined to yield a friction or restitution coefficient for a collision.
///
/// When two actors come in contact with each other, they each have materials with various coefficients, but we only need a single set of coefficients for the pair.
///
/// Physics doesn't have any inherent combinations because the coefficients are determined empirically on a case by case
/// basis. However, simulating this with a pairwise lookup table is often impractical.
///
/// For this reason the following combine behaviors are available:
///
/// eAVERAGE
/// eMIN
/// eMULTIPLY
/// eMAX
///
/// The effective combine mode for the pair is maximum(material0.combineMode, material1.combineMode).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxCombineMode {
    /// Average: (a + b)/2
    Average = 0,
    /// Minimum: minimum(a,b)
    Min = 1,
    /// Multiply: a*b
    Multiply = 2,
    /// Maximum: maximum(a,b)
    Max = 3,
    /// This is not a valid combine mode, it is a sentinel to denote the number of possible values. We assert that the variable's value is smaller than this.
    NValues = 4,
    /// This is not a valid combine mode, it is to assure that the size of the enum type is big enough.
    Pad32 = 2147483647,
}

/// Identifies dirty particle buffers that need to be updated in the particle system.
///
/// This flag can be used mark the device user buffers that are dirty and need to be written to the particle system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxParticleBufferFlag {
    /// No data specified
    None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * number of particles)
    UpdatePosition = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * number of particles)
    UpdateVelocity = 2,
    /// Specifies the per-particle phase flag data (array of PxU32 * number of particles)
    UpdatePhase = 4,
    /// Specifies the rest position (first 3 floats) data for cloth buffers
    UpdateRestposition = 8,
    /// Specifies the cloth buffer (see PxParticleClothBuffer)
    UpdateCloth = 32,
    /// Specifies the rigid buffer (see PxParticleRigidBuffer)
    UpdateRigid = 64,
    /// Specifies the diffuse particle parameter buffer (see PxDiffuseParticleParams)
    UpdateDiffuseParam = 128,
    /// Specifies the attachments.
    UpdateAttachments = 256,
    All = 495,
}

bitflags::bitflags! {
    /// Flags for [`PxParticleBufferFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxParticleBufferFlags: u32 {
        const UpdatePosition = 1 << 0;
        const UpdateVelocity = 1 << 1;
        const UpdatePhase = 1 << 2;
        const UpdateRestposition = 1 << 3;
        const UpdateCloth = 1 << 5;
        const UpdateRigid = 1 << 6;
        const UpdateDiffuseParam = 1 << 7;
        const UpdateAttachments = 1 << 8;
        const All = Self::UpdatePosition.bits | Self::UpdateVelocity.bits | Self::UpdatePhase.bits | Self::UpdateRestposition.bits | Self::UpdateCloth.bits | Self::UpdateRigid.bits | Self::UpdateDiffuseParam.bits | Self::UpdateAttachments.bits;
    }
}

/// Identifies per-particle behavior for a PxParticleSystem.
///
/// See [`PxParticleSystem::createPhase`]().
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PxParticlePhaseFlag {
    /// Bits [ 0, 19] represent the particle group for controlling collisions
    ParticlePhaseGroupMask = 1048575,
    /// Bits [20, 23] hold flags about how the particle behave
    ParticlePhaseFlagsMask = 4293918720,
    /// If set this particle will interact with particles of the same group
    ParticlePhaseSelfCollide = 1048576,
    /// If set this particle will ignore collisions with particles closer than the radius in the rest pose, this flag should not be specified unless valid rest positions have been specified using setRestParticles()
    ParticlePhaseSelfCollideFilter = 2097152,
    /// If set this particle will generate fluid density constraints for its overlapping neighbors
    ParticlePhaseFluid = 4194304,
}

bitflags::bitflags! {
    /// Flags for [`PxParticlePhaseFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxParticlePhaseFlags: u32 {
        const ParticlePhaseGroupMask = 0x000fffff;
        const ParticlePhaseFlagsMask = Self::ParticlePhaseSelfCollide.bits | Self::ParticlePhaseSelfCollideFilter.bits | Self::ParticlePhaseFluid.bits;
        const ParticlePhaseSelfCollide = 1 << 20;
        const ParticlePhaseSelfCollideFilter = 1 << 21;
        const ParticlePhaseFluid = 1 << 22;
    }
}

/// Specifies memory space for a PxBuffer instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxBufferType {
    Host = 0,
    Device = 1,
}

/// Filtering flags for scene queries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxQueryFlag {
    /// Traverse static shapes
    Static = 1,
    /// Traverse dynamic shapes
    Dynamic = 2,
    /// Run the pre-intersection-test filter (see [`PxQueryFilterCallback::preFilter`]())
    Prefilter = 4,
    /// Run the post-intersection-test filter (see [`PxQueryFilterCallback::postFilter`]())
    Postfilter = 8,
    /// Abort traversal as soon as any hit is found and return it via callback.block.
    /// Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.
    AnyHit = 16,
    /// All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
    /// This is also an optimization hint that may improve query performance.
    NoBlock = 32,
    /// Same as eBATCH_QUERY_LEGACY_BEHAVIOUR, more explicit name making it clearer that this can also be used
    /// with regular/non-batched queries if needed.
    DisableHardcodedFilter = 64,
    /// Reserved for internal use
    Reserved = 32768,
}

bitflags::bitflags! {
    /// Flags for [`PxQueryFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxQueryFlags: u16 {
        const Static = 1 << 0;
        const Dynamic = 1 << 1;
        const Prefilter = 1 << 2;
        const Postfilter = 1 << 3;
        const AnyHit = 1 << 4;
        const NoBlock = 1 << 5;
        const DisableHardcodedFilter = 1 << 6;
        const Reserved = 1 << 15;
    }
}

/// Classification of scene query hits (intersections).
///
/// - eNONE: Returning this hit type means that the hit should not be reported.
/// - eBLOCK: For all raycast, sweep and overlap queries the nearest eBLOCK type hit will always be returned in PxHitCallback::block member.
/// - eTOUCH: Whenever a raycast, sweep or overlap query was called with non-zero PxHitCallback::nbTouches and PxHitCallback::touches
/// parameters, eTOUCH type hits that are closer or same distance (touchDistance
/// <
/// = blockDistance condition)
/// as the globally nearest eBLOCK type hit, will be reported.
/// - For example, to record all hits from a raycast query, always return eTOUCH.
///
/// All hits in overlap() queries are treated as if the intersection distance were zero.
/// This means the hits are unsorted and all eTOUCH hits are recorded by the callback even if an eBLOCK overlap hit was encountered.
/// Even though all overlap() blocking hits have zero length, only one (arbitrary) eBLOCK overlap hit is recorded in PxHitCallback::block.
/// All overlap() eTOUCH type hits are reported (zero touchDistance
/// <
/// = zero blockDistance condition).
///
/// For raycast/sweep/overlap calls with zero touch buffer or PxHitCallback::nbTouches member,
/// only the closest hit of type eBLOCK is returned. All eTOUCH hits are discarded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxQueryHitType {
    /// the query should ignore this shape
    None = 0,
    /// a hit on the shape touches the intersection geometry of the query but does not block it
    Touch = 1,
    /// a hit on the shape blocks the query (does not block overlap queries)
    Block = 2,
}

/// Collection of flags providing a mechanism to lock motion along/around a specific axis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxRigidDynamicLockFlag {
    LockLinearX = 1,
    LockLinearY = 2,
    LockLinearZ = 4,
    LockAngularX = 8,
    LockAngularY = 16,
    LockAngularZ = 32,
}

bitflags::bitflags! {
    /// Flags for [`PxRigidDynamicLockFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxRigidDynamicLockFlags: u8 {
        const LockLinearX = 1 << 0;
        const LockLinearY = 1 << 1;
        const LockLinearZ = 1 << 2;
        const LockAngularX = 1 << 3;
        const LockAngularY = 1 << 4;
        const LockAngularZ = 1 << 5;
    }
}

/// Pruning structure used to accelerate scene queries.
///
/// eNONE uses a simple data structure that consumes less memory than the alternatives,
/// but generally has slower query performance.
///
/// eDYNAMIC_AABB_TREE usually provides the fastest queries. However there is a
/// constant per-frame management cost associated with this structure. How much work should
/// be done per frame can be tuned via the [`PxSceneQueryDesc::dynamicTreeRebuildRateHint`]
/// parameter.
///
/// eSTATIC_AABB_TREE is typically used for static objects. It is the same as the
/// dynamic AABB tree, without the per-frame overhead. This can be a good choice for static
/// objects, if no static objects are added, moved or removed after the scene has been
/// created. If there is no such guarantee (e.g. when streaming parts of the world in and out),
/// then the dynamic version is a better choice even for static objects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPruningStructureType {
    /// Using a simple data structure
    None = 0,
    /// Using a dynamic AABB tree
    DynamicAabbTree = 1,
    /// Using a static AABB tree
    StaticAabbTree = 2,
    Last = 3,
}

/// Secondary pruning structure used for newly added objects in dynamic trees.
///
/// Dynamic trees (PxPruningStructureType::eDYNAMIC_AABB_TREE) are slowly rebuilt
/// over several frames. A secondary pruning structure holds and manages objects
/// added to the scene while this rebuild is in progress.
///
/// eNONE ignores newly added objects. This means that for a number of frames (roughly
/// defined by PxSceneQueryDesc::dynamicTreeRebuildRateHint) newly added objects will
/// be ignored by scene queries. This can be acceptable when streaming large worlds, e.g.
/// when the objects added at the boundaries of the game world don't immediately need to be
/// visible from scene queries (it would be equivalent to streaming that data in a few frames
/// later). The advantage of this approach is that there is no CPU cost associated with
/// inserting the new objects in the scene query data structures, and no extra runtime cost
/// when performing queries.
///
/// eBUCKET uses a structure similar to PxPruningStructureType::eNONE. Insertion is fast but
/// query cost can be high.
///
/// eINCREMENTAL uses an incremental AABB-tree, with no direct PxPruningStructureType equivalent.
/// Query time is fast but insertion cost can be high.
///
/// eBVH uses a PxBVH structure. This usually offers the best overall performance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxDynamicTreeSecondaryPruner {
    /// no secondary pruner, new objects aren't visible to SQ for a few frames
    None = 0,
    /// bucket-based secondary pruner, faster updates, slower query time
    Bucket = 1,
    /// incremental-BVH secondary pruner, faster query time, slower updates
    Incremental = 2,
    /// PxBVH-based secondary pruner, good overall performance
    Bvh = 3,
    Last = 4,
}

/// Scene query update mode
///
/// This enum controls what work is done when the scene query system is updated. The updates traditionally happen when PxScene::fetchResults
/// is called. This function then calls PxSceneQuerySystem::finalizeUpdates, where the update mode is used.
///
/// fetchResults/finalizeUpdates will sync changed bounds during simulation and update the scene query bounds in pruners, this work is mandatory.
///
/// eBUILD_ENABLED_COMMIT_ENABLED does allow to execute the new AABB tree build step during fetchResults/finalizeUpdates, additionally
/// the pruner commit is called where any changes are applied. During commit PhysX refits the dynamic scene query tree and if a new tree
/// was built and the build finished the tree is swapped with current AABB tree.
///
/// eBUILD_ENABLED_COMMIT_DISABLED does allow to execute the new AABB tree build step during fetchResults/finalizeUpdates. Pruner commit
/// is not called, this means that refit will then occur during the first scene query following fetchResults/finalizeUpdates, or may be forced
/// by the method PxScene::flushQueryUpdates() / PxSceneQuerySystemBase::flushUpdates().
///
/// eBUILD_DISABLED_COMMIT_DISABLED no further scene query work is executed. The scene queries update needs to be called manually, see
/// PxScene::sceneQueriesUpdate (see that function's doc for the equivalent PxSceneQuerySystem sequence). It is recommended to call
/// PxScene::sceneQueriesUpdate right after fetchResults/finalizeUpdates as the pruning structures are not updated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSceneQueryUpdateMode {
    /// Both scene query build and commit are executed.
    BuildEnabledCommitEnabled = 0,
    /// Scene query build only is executed.
    BuildEnabledCommitDisabled = 1,
    /// No work is done, no update of scene queries
    BuildDisabledCommitDisabled = 2,
}

/// Built-in enum for default PxScene pruners
///
/// This is passed as a pruner index to various functions in the following APIs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PxScenePrunerIndex {
    PxScenePrunerStatic = 0,
    PxScenePrunerDynamic = 1,
    PxSceneCompoundPruner = 4294967295,
}

/// Broad phase algorithm used in the simulation
///
/// eSAP is a good generic choice with great performance when many objects are sleeping. Performance
/// can degrade significantly though, when all objects are moving, or when large numbers of objects
/// are added to or removed from the broad phase. This algorithm does not need world bounds to be
/// defined in order to work.
///
/// eMBP is an alternative broad phase algorithm that does not suffer from the same performance
/// issues as eSAP when all objects are moving or when inserting large numbers of objects. However
/// its generic performance when many objects are sleeping might be inferior to eSAP, and it requires
/// users to define world bounds in order to work.
///
/// eABP is a revisited implementation of MBP, which automatically manages broad-phase regions.
/// It offers the convenience of eSAP (no need to define world bounds or regions) and the performance
/// of eMBP when a lot of objects are moving. While eSAP can remain faster when most objects are
/// sleeping and eMBP can remain faster when it uses a large number of properly-defined regions,
/// eABP often gives the best performance on average and the best memory usage.
///
/// ePABP is a parallel implementation of ABP. It can often be the fastest (CPU) broadphase, but it
/// can use more memory than ABP.
///
/// eGPU is a GPU implementation of the incremental sweep and prune approach. Additionally, it uses a ABP-style
/// initial pair generation approach to avoid large spikes when inserting shapes. It not only has the advantage
/// of traditional SAP approch which is good for when many objects are sleeping, but due to being fully parallel,
/// it also is great when lots of shapes are moving or for runtime pair insertion and removal. It can become a
/// performance bottleneck if there are a very large number of shapes roughly projecting to the same values
/// on a given axis. If the scene has a very large number of shapes in an actor, e.g. a humanoid, it is recommended
/// to use an aggregate to represent multi-shape or multi-body actors to minimize stress placed on the broad phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxBroadPhaseType {
    /// 3-axes sweep-and-prune
    Sap = 0,
    /// Multi box pruning
    Mbp = 1,
    /// Automatic box pruning
    Abp = 2,
    /// Parallel automatic box pruning
    Pabp = 3,
    /// GPU broad phase
    Gpu = 4,
    Last = 5,
}

/// Enum for selecting the friction algorithm used for simulation.
///
/// [`PxFrictionType::ePATCH`] selects the patch friction model which typically leads to the most stable results at low solver iteration counts and is also quite inexpensive, as it uses only
/// up to four scalar solver constraints per pair of touching objects.  The patch friction model is the same basic strong friction algorithm as PhysX 3.2 and before.
///
/// [`PxFrictionType::eONE_DIRECTIONAL`] is a simplification of the Coulomb friction model, in which the friction for a given point of contact is applied in the alternating tangent directions of
/// the contact's normal.  This simplification allows us to reduce the number of iterations required for convergence but is not as accurate as the two directional model.
///
/// [`PxFrictionType::eTWO_DIRECTIONAL`] is identical to the one directional model, but it applies friction in both tangent directions simultaneously.  This hurts convergence a bit so it
/// requires more solver iterations, but is more accurate.  Like the one directional model, it is applied at every contact point, which makes it potentially more expensive
/// than patch friction for scenarios with many contact points.
///
/// [`PxFrictionType::eFRICTION_COUNT`] is the total numer of friction models supported by the SDK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxFrictionType {
    /// Select default patch-friction model.
    Patch = 0,
    /// Select one directional per-contact friction model.
    OneDirectional = 1,
    /// Select two directional per-contact friction model.
    TwoDirectional = 2,
    /// The total number of friction models supported by the SDK.
    FrictionCount = 3,
}

/// Enum for selecting the type of solver used for the simulation.
///
/// [`PxSolverType::ePGS`] selects the iterative sequential impulse solver. This is the same kind of solver used in PhysX 3.4 and earlier releases.
///
/// [`PxSolverType::eTGS`] selects a non linear iterative solver. This kind of solver can lead to improved convergence and handle large mass ratios, long chains and jointed systems better. It is slightly more expensive than the default solver and can introduce more energy to correct joint and contact errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSolverType {
    /// Projected Gauss-Seidel iterative solver
    Pgs = 0,
    /// Default Temporal Gauss-Seidel solver
    Tgs = 1,
}

/// flags for configuring properties of the scene
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSceneFlag {
    /// Enable Active Actors Notification.
    ///
    /// This flag enables the Active Actor Notification feature for a scene.  This
    /// feature defaults to disabled.  When disabled, the function
    /// PxScene::getActiveActors() will always return a NULL list.
    ///
    /// There may be a performance penalty for enabling the Active Actor Notification, hence this flag should
    /// only be enabled if the application intends to use the feature.
    ///
    /// Default:
    /// False
    EnableActiveActors = 1,
    /// Enables a second broad phase check after integration that makes it possible to prevent objects from tunneling through eachother.
    ///
    /// PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.
    ///
    /// For this feature to be effective for bodies that can move at a significant velocity, the user should raise the flag PxRigidBodyFlag::eENABLE_CCD for them.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// False
    EnableCcd = 2,
    /// Enables a simplified swept integration strategy, which sacrifices some accuracy for improved performance.
    ///
    /// This simplified swept integration approach makes certain assumptions about the motion of objects that are not made when using a full swept integration.
    /// These assumptions usually hold but there are cases where they could result in incorrect behavior between a set of fast-moving rigid bodies. A key issue is that
    /// fast-moving dynamic objects may tunnel through each-other after a rebound. This will not happen if this mode is disabled. However, this approach will be potentially
    /// faster than a full swept integration because it will perform significantly fewer sweeps in non-trivial scenes involving many fast-moving objects. This approach
    /// should successfully resist objects passing through the static environment.
    ///
    /// PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.
    ///
    /// This scene flag requires eENABLE_CCD to be enabled as well. If it is not, this scene flag will do nothing.
    ///
    /// For this feature to be effective for bodies that can move at a significant velocity, the user should raise the flag PxRigidBodyFlag::eENABLE_CCD for them.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// False
    DisableCcdResweep = 4,
    /// Enable GJK-based distance collision detection system.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// true
    EnablePcm = 64,
    /// Disable contact report buffer resize. Once the contact buffer is full, the rest of the contact reports will
    /// not be buffered and sent.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    DisableContactReportBufferResize = 128,
    /// Disable contact cache.
    ///
    /// Contact caches are used internally to provide faster contact generation. You can disable all contact caches
    /// if memory usage for this feature becomes too high.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    DisableContactCache = 256,
    /// Require scene-level locking
    ///
    /// When set to true this requires that threads accessing the PxScene use the
    /// multi-threaded lock methods.
    ///
    /// This flag is not mutable, and must be set in PxSceneDesc at scene creation.
    ///
    /// Default:
    /// false
    RequireRwLock = 512,
    /// Enables additional stabilization pass in solver
    ///
    /// When set to true, this enables additional stabilization processing to improve that stability of complex interactions between large numbers of bodies.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation. Also, this is an experimental feature which does result in some loss of momentum.
    EnableStabilization = 1024,
    /// Enables average points in contact manifolds
    ///
    /// When set to true, this enables additional contacts to be generated per manifold to represent the average point in a manifold. This can stabilize stacking when only a small
    /// number of solver iterations is used.
    ///
    /// Note that this flag is not mutable and must be set in PxSceneDesc at scene creation.
    EnableAveragePoint = 2048,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    ExcludeKinematicsFromActiveActors = 4096,
    /// Do not report kinematics in list of active actors.
    ///
    /// Since the target pose for kinematics is set by the user, an application can track the activity state directly and use
    /// this flag to avoid that kinematics get added to the list of active actors.
    ///
    /// This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.
    ///
    /// Default:
    /// false
    EnableGpuDynamics = 8192,
    /// Provides improved determinism at the expense of performance.
    ///
    /// By default, PhysX provides limited determinism guarantees. Specifically, PhysX guarantees that the exact scene (same actors created in the same order) and simulated using the same
    /// time-stepping scheme should provide the exact same behaviour.
    ///
    /// However, if additional actors are added to the simulation, this can affect the behaviour of the existing actors in the simulation, even if the set of new actors do not interact with
    /// the existing actors.
    ///
    /// This flag provides an additional level of determinism that guarantees that the simulation will not change if additional actors are added to the simulation, provided those actors do not interfere
    /// with the existing actors in the scene. Determinism is only guaranteed if the actors are inserted in a consistent order each run in a newly-created scene and simulated using a consistent time-stepping
    /// scheme.
    ///
    /// Note that this flag is not mutable and must be set at scene creation.
    ///
    /// Note that enabling this flag can have a negative impact on performance.
    ///
    /// Note that this feature is not currently supported on GPU.
    ///
    /// Default
    /// false
    EnableEnhancedDeterminism = 16384,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    EnableFrictionEveryIteration = 32768,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    SuppressReadback = 65536,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    ForceReadback = 131072,
    /// Controls processing friction in all solver iterations
    ///
    /// By default, PhysX processes friction only in the final 3 position iterations, and all velocity
    /// iterations. This flag enables friction processing in all position and velocity iterations.
    ///
    /// The default behaviour provides a good trade-off between performance and stability and is aimed
    /// primarily at game development.
    ///
    /// When simulating more complex frictional behaviour, such as grasping of complex geometries with
    /// a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.
    ///
    /// This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
    MutableFlags = 69633,
}

bitflags::bitflags! {
    /// Flags for [`PxSceneFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxSceneFlags: u32 {
        const EnableActiveActors = 1 << 0;
        const EnableCcd = 1 << 1;
        const DisableCcdResweep = 1 << 2;
        const EnablePcm = 1 << 6;
        const DisableContactReportBufferResize = 1 << 7;
        const DisableContactCache = 1 << 8;
        const RequireRwLock = 1 << 9;
        const EnableStabilization = 1 << 10;
        const EnableAveragePoint = 1 << 11;
        const ExcludeKinematicsFromActiveActors = 1 << 12;
        const EnableGpuDynamics = 1 << 13;
        const EnableEnhancedDeterminism = 1 << 14;
        const EnableFrictionEveryIteration = 1 << 15;
        const SuppressReadback = 1 << 16;
        const ForceReadback = 1 << 17;
        const MutableFlags = Self::EnableActiveActors.bits | Self::ExcludeKinematicsFromActiveActors.bits | Self::SuppressReadback.bits;
    }
}

/// Debug visualization parameters.
///
/// [`PxVisualizationParameter::eSCALE`] is the master switch for enabling visualization, please read the corresponding documentation
/// for further details.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxVisualizationParameter {
    /// This overall visualization scale gets multiplied with the individual scales. Setting to zero ignores all visualizations. Default is 0.
    ///
    /// The below settings permit the debug visualization of various simulation properties.
    /// The setting is either zero, in which case the property is not drawn. Otherwise it is a scaling factor
    /// that determines the size of the visualization widgets.
    ///
    /// Only objects for which visualization is turned on using setFlag(eVISUALIZATION) are visualized (see [`PxActorFlag::eVISUALIZATION`], #PxShapeFlag::eVISUALIZATION, ...).
    /// Contacts are visualized if they involve a body which is being visualized.
    /// Default is 0.
    ///
    /// Notes:
    /// - to see any visualization, you have to set PxVisualizationParameter::eSCALE to nonzero first.
    /// - the scale factor has been introduced because it's difficult (if not impossible) to come up with a
    /// good scale for 3D vectors. Normals are normalized and their length is always 1. But it doesn't mean
    /// we should render a line of length 1. Depending on your objects/scene, this might be completely invisible
    /// or extremely huge. That's why the scale factor is here, to let you tune the length until it's ok in
    /// your scene.
    /// - however, things like collision shapes aren't ambiguous. They are clearly defined for example by the
    /// triangles
    /// &
    /// polygons themselves, and there's no point in scaling that. So the visualization widgets
    /// are only scaled when it makes sense.
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 0
    Scale = 0,
    /// Visualize the world axes.
    WorldAxes = 1,
    /// Visualize a bodies axes.
    BodyAxes = 2,
    /// Visualize a body's mass axes.
    ///
    /// This visualization is also useful for visualizing the sleep state of bodies. Sleeping bodies are drawn in
    /// black, while awake bodies are drawn in white. If the body is sleeping and part of a sleeping group, it is
    /// drawn in red.
    BodyMassAxes = 3,
    /// Visualize the bodies linear velocity.
    BodyLinVelocity = 4,
    /// Visualize the bodies angular velocity.
    BodyAngVelocity = 5,
    /// Visualize contact points. Will enable contact information.
    ContactPoint = 6,
    /// Visualize contact normals. Will enable contact information.
    ContactNormal = 7,
    /// Visualize contact errors. Will enable contact information.
    ContactError = 8,
    /// Visualize Contact forces. Will enable contact information.
    ContactForce = 9,
    /// Visualize actor axes.
    ActorAxes = 10,
    /// Visualize bounds (AABBs in world space)
    CollisionAabbs = 11,
    /// Shape visualization
    CollisionShapes = 12,
    /// Shape axis visualization
    CollisionAxes = 13,
    /// Compound visualization (compound AABBs in world space)
    CollisionCompounds = 14,
    /// Mesh
    /// &
    /// convex face normals
    CollisionFnormals = 15,
    /// Active edges for meshes
    CollisionEdges = 16,
    /// Static pruning structures
    CollisionStatic = 17,
    /// Dynamic pruning structures
    CollisionDynamic = 18,
    /// Joint local axes
    JointLocalFrames = 19,
    /// Joint limits
    JointLimits = 20,
    /// Visualize culling box
    CullBox = 21,
    /// MBP regions
    MbpRegions = 22,
    /// Renders the simulation mesh instead of the collision mesh (only available for tetmeshes)
    SimulationMesh = 23,
    /// Renders the SDF of a mesh instead of the collision mesh (only available for triangle meshes with SDFs)
    Sdf = 24,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    NumValues = 25,
    /// This is not a parameter, it just records the current number of parameters (as maximum(PxVisualizationParameter)+1) for use in loops.
    ForceDword = 2147483647,
}

/// Different types of rigid body collision pair statistics.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum RbPairStatsType {
    /// Shape pairs processed as discrete contact pairs for the current simulation step.
    DiscreteContactPairs = 0,
    /// Shape pairs processed as swept integration pairs for the current simulation step.
    ///
    /// Counts the pairs for which special CCD (continuous collision detection) work was actually done and NOT the number of pairs which were configured for CCD.
    /// Furthermore, there can be multiple CCD passes and all processed pairs of all passes are summed up, hence the number can be larger than the amount of pairs which have been configured for CCD.
    CcdPairs = 1,
    /// Shape pairs processed with user contact modification enabled for the current simulation step.
    ModifiedContactPairs = 2,
    /// Trigger shape pairs processed for the current simulation step.
    TriggerPairs = 3,
}

/// These flags determine what data is read or written to the gpu softbody.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSoftBodyDataFlag {
    /// The collision mesh tetrahedron indices (quadruples of int32)
    TetIndices = 0,
    /// The collision mesh cauchy stress tensors (float 3x3 matrices)
    TetStress = 1,
    /// The collision mesh tetrahedron von Mises stress (float scalar)
    TetStresscoeff = 2,
    /// The collision mesh tetrahedron rest poses (float 3x3 matrices)
    TetRestPoses = 3,
    /// The collision mesh tetrahedron orientations (quaternions, quadruples of float)
    TetRotations = 4,
    /// The collision mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    TetPositionInvMass = 5,
    /// The simulation mesh tetrahedron indices (quadruples of int32)
    SimTetIndices = 6,
    /// The simulation mesh vertex velocities and their inverted mass in the 4th component (quadruples of float)
    SimVelocityInvMass = 7,
    /// The simulation mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
    SimPositionInvMass = 8,
    /// The simulation mesh kinematic target positions
    SimKinematicTarget = 9,
}

/// Identifies input and output buffers for PxHairSystem
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHairSystemData {
    /// No data specified
    None = 0,
    /// Specifies the position (first 3 floats) and inverse mass (last float) data (array of PxVec4 * max number of vertices)
    PositionInvmass = 1,
    /// Specifies the velocity (first 3 floats) data (array of PxVec4 * max number of vertices)
    Velocity = 2,
    /// Specifies everything
    All = 3,
}

bitflags::bitflags! {
    /// Flags for [`PxHairSystemData`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxHairSystemDataFlags: u32 {
        const PositionInvmass = 1 << 0;
        const Velocity = 1 << 1;
        const All = Self::PositionInvmass.bits | Self::Velocity.bits;
    }
}

/// Binary settings for hair system simulation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxHairSystemFlag {
    /// Determines if self-collision between hair vertices is ignored
    DisableSelfCollision = 1,
    /// Determines if collision between hair and external bodies is ignored
    DisableExternalCollision = 2,
    /// Determines if attachment constraint is also felt by body to which the hair is attached
    DisableTwosidedAttachment = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxHairSystemFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxHairSystemFlags: u32 {
        const DisableSelfCollision = 1 << 0;
        const DisableExternalCollision = 1 << 1;
        const DisableTwosidedAttachment = 1 << 2;
    }
}

/// Identifies each type of information for retrieving from actor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxActorCacheFlag {
    ActorData = 1,
    Force = 4,
    Torque = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxActorCacheFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxActorCacheFlags: u16 {
        const ActorData = 1 << 0;
        const Force = 1 << 2;
        const Torque = 1 << 3;
    }
}

/// PVD scene Flags. They are disabled by default, and only works if PxPvdInstrumentationFlag::eDEBUG is set.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPvdSceneFlag {
    TransmitContacts = 1,
    /// Transmits contact stream to PVD.
    TransmitScenequeries = 2,
    /// Transmits scene query stream to PVD.
    TransmitConstraints = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxPvdSceneFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxPvdSceneFlags: u8 {
        const TransmitContacts = 1 << 0;
        const TransmitScenequeries = 1 << 1;
        const TransmitConstraints = 1 << 2;
    }
}

/// Identifies each type of actor for retrieving actors from a scene.
///
/// [`PxArticulationLink`] objects are not supported. Use the #PxArticulationReducedCoordinate object to retrieve all its links.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxActorTypeFlag {
    /// A static rigid body
    RigidStatic = 1,
    /// A dynamic rigid body
    RigidDynamic = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxActorTypeFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxActorTypeFlags: u16 {
        const RigidStatic = 1 << 0;
        const RigidDynamic = 1 << 1;
    }
}

/// Extra data item types for contact pairs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxContactPairExtraDataType {
    /// see [`PxContactPairVelocity`]
    PreSolverVelocity = 0,
    /// see [`PxContactPairVelocity`]
    PostSolverVelocity = 1,
    /// see [`PxContactPairPose`]
    ContactEventPose = 2,
    /// see [`PxContactPairIndex`]
    ContactPairIndex = 3,
}

/// Collection of flags providing information on contact report pairs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxContactPairHeaderFlag {
    /// The actor with index 0 has been removed from the scene.
    RemovedActor0 = 1,
    /// The actor with index 1 has been removed from the scene.
    RemovedActor1 = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxContactPairHeaderFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxContactPairHeaderFlags: u16 {
        const RemovedActor0 = 1 << 0;
        const RemovedActor1 = 1 << 1;
    }
}

/// Collection of flags providing information on contact report pairs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxContactPairFlag {
    /// The shape with index 0 has been removed from the actor/scene.
    RemovedShape0 = 1,
    /// The shape with index 1 has been removed from the actor/scene.
    RemovedShape1 = 2,
    /// First actor pair contact.
    ///
    /// The provided shape pair marks the first contact between the two actors, no other shape pair has been touching prior to the current simulation frame.
    ///
    /// : This info is only available if [`PxPairFlag::eNOTIFY_TOUCH_FOUND`] has been declared for the pair.
    ActorPairHasFirstTouch = 4,
    /// All contact between the actor pair was lost.
    ///
    /// All contact between the two actors has been lost, no shape pairs remain touching after the current simulation frame.
    ActorPairLostTouch = 8,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The applied contact impulses are provided for every contact point.
    /// This is the case if [`PxPairFlag::eSOLVE_CONTACT`] has been set for the pair.
    InternalHasImpulses = 16,
    /// Internal flag, used by [`PxContactPair`].extractContacts()
    ///
    /// The provided contact point information is flipped with regards to the shapes of the contact pair. This mainly concerns the order of the internal triangle indices.
    InternalContactsAreFlipped = 32,
}

bitflags::bitflags! {
    /// Flags for [`PxContactPairFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxContactPairFlags: u16 {
        const RemovedShape0 = 1 << 0;
        const RemovedShape1 = 1 << 1;
        const ActorPairHasFirstTouch = 1 << 2;
        const ActorPairLostTouch = 1 << 3;
        const InternalHasImpulses = 1 << 4;
        const InternalContactsAreFlipped = 1 << 5;
    }
}

/// Collection of flags providing information on trigger report pairs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxTriggerPairFlag {
    /// The trigger shape has been removed from the actor/scene.
    RemovedShapeTrigger = 1,
    /// The shape causing the trigger event has been removed from the actor/scene.
    RemovedShapeOther = 2,
    /// For internal use only.
    NextFree = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxTriggerPairFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxTriggerPairFlags: u8 {
        const RemovedShapeTrigger = 1 << 0;
        const RemovedShapeOther = 1 << 1;
        const NextFree = 1 << 2;
    }
}

/// Identifies input and output buffers for PxSoftBody.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSoftBodyData {
    None = 0,
    /// Flag to request access to the collision mesh's positions; read only
    PositionInvmass = 1,
    /// Flag to request access to the simulation mesh's positions and inverse masses
    SimPositionInvmass = 4,
    /// Flag to request access to the simulation mesh's velocities and inverse masses
    SimVelocity = 8,
    /// Flag to request access to the simulation mesh's kinematic target position
    SimKinematicTarget = 16,
    All = 29,
}

bitflags::bitflags! {
    /// Flags for [`PxSoftBodyData`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxSoftBodyDataFlags: u32 {
        const PositionInvmass = 1 << 0;
        const SimPositionInvmass = 1 << 2;
        const SimVelocity = 1 << 3;
        const SimKinematicTarget = 1 << 4;
        const All = Self::PositionInvmass.bits | Self::SimPositionInvmass.bits | Self::SimVelocity.bits | Self::SimKinematicTarget.bits;
    }
}

/// Flags to enable or disable special modes of a SoftBody
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSoftBodyFlag {
    /// Determines if self collision will be detected and resolved
    DisableSelfCollision = 1,
    /// Enables computation of a Cauchy stress tensor for every tetrahedron in the simulation mesh. The tensors can be accessed through the softbody direct API
    ComputeStressTensor = 2,
    /// Enables support for continuous collision detection
    EnableCcd = 4,
    /// Enable debug rendering to display the simulation mesh
    DisplaySimMesh = 8,
    /// Enables support for kinematic motion of the collision and simulation mesh.
    Kinematic = 16,
    /// Enables partially kinematic motion of the collisios and simulation mesh.
    PartiallyKinematic = 32,
}

bitflags::bitflags! {
    /// Flags for [`PxSoftBodyFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxSoftBodyFlags: u32 {
        const DisableSelfCollision = 1 << 0;
        const ComputeStressTensor = 1 << 1;
        const EnableCcd = 1 << 2;
        const DisplaySimMesh = 1 << 3;
        const Kinematic = 1 << 4;
        const PartiallyKinematic = 1 << 5;
    }
}

/// The type of controller, eg box, sphere or capsule.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxControllerShapeType {
    /// A box controller.
    Box = 0,
    /// A capsule controller
    Capsule = 1,
    /// A capsule controller
    ForceDword = 2147483647,
}

/// specifies how a CCT interacts with non-walkable parts.
///
/// This is only used when slopeLimit is non zero. It is currently enabled for static actors only, and not supported for spheres or capsules.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxControllerNonWalkableMode {
    /// Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
    PreventClimbing = 0,
    /// Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
    PreventClimbingAndForceSliding = 1,
}

/// specifies which sides a character is colliding with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxControllerCollisionFlag {
    /// Character is colliding to the sides.
    CollisionSides = 1,
    /// Character has collision above.
    CollisionUp = 2,
    /// Character has collision below.
    CollisionDown = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxControllerCollisionFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxControllerCollisionFlags: u8 {
        const CollisionSides = 1 << 0;
        const CollisionUp = 1 << 1;
        const CollisionDown = 1 << 2;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxCapsuleClimbingMode {
    /// Standard mode, let the capsule climb over surfaces according to impact normal
    Easy = 0,
    /// Constrained mode, try to limit climbing according to the step offset
    Constrained = 1,
    Last = 2,
}

/// specifies controller behavior
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxControllerBehaviorFlag {
    /// Controller can ride on touched object (i.e. when this touched object is moving horizontally).
    ///
    /// The CCT vs. CCT case is not supported.
    CctCanRideOnObject = 1,
    /// Controller should slide on touched object
    CctSlide = 2,
    /// Disable all code dealing with controllers riding on objects, let users define it outside of the SDK.
    CctUserDefinedRide = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxControllerBehaviorFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxControllerBehaviorFlags: u8 {
        const CctCanRideOnObject = 1 << 0;
        const CctSlide = 1 << 1;
        const CctUserDefinedRide = 1 << 2;
    }
}

/// specifies debug-rendering flags
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum PxControllerDebugRenderFlag {
    /// Temporal bounding volume around controllers
    TemporalBv = 1,
    /// Cached bounding volume around controllers
    CachedBv = 2,
    /// User-defined obstacles
    Obstacles = 4,
    None = 0,
    All = 4294967295,
}

bitflags::bitflags! {
    /// Flags for [`PxControllerDebugRenderFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxControllerDebugRenderFlags: u32 {
        const TemporalBv = 1 << 0;
        const CachedBv = 1 << 1;
        const Obstacles = 1 << 2;
        const All = Self::TemporalBv.bits | Self::CachedBv.bits | Self::Obstacles.bits;
    }
}

/// Defines the number of bits per subgrid pixel
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSdfBitsPerSubgridPixel {
    /// 8 bit per subgrid pixel (values will be stored as normalized integers)
    E8BitPerPixel = 1,
    /// 16 bit per subgrid pixel (values will be stored as normalized integers)
    E16BitPerPixel = 2,
    /// 32 bit per subgrid pixel (values will be stored as floats in world scale units)
    E32BitPerPixel = 4,
}

/// Flags which describe the format and behavior of a convex mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConvexFlag {
    /// Denotes the use of 16-bit vertex indices in PxConvexMeshDesc::triangles or PxConvexMeshDesc::polygons.
    /// (otherwise, 32-bit indices are assumed)
    E16BitIndices = 1,
    /// Automatically recomputes the hull from the vertices. If this flag is not set, you must provide the entire geometry manually.
    ///
    /// There are two different algorithms for hull computation, please see PxConvexMeshCookingType.
    ComputeConvex = 2,
    /// Checks and removes almost zero-area triangles during convex hull computation.
    /// The rejected area size is specified in PxCookingParams::areaTestEpsilon
    ///
    /// This flag is only used in combination with eCOMPUTE_CONVEX.
    CheckZeroAreaTriangles = 4,
    /// Quantizes the input vertices using the k-means clustering
    ///
    /// The input vertices are quantized to PxConvexMeshDesc::quantizedCount
    /// see http://en.wikipedia.org/wiki/K-means_clustering
    QuantizeInput = 8,
    /// Disables the convex mesh validation to speed-up hull creation. Please use separate validation
    /// function in checked/debug builds. Creating a convex mesh with invalid input data without prior validation
    /// may result in undefined behavior.
    DisableMeshValidation = 16,
    /// Enables plane shifting vertex limit algorithm.
    ///
    /// Plane shifting is an alternative algorithm for the case when the computed hull has more vertices
    /// than the specified vertex limit.
    ///
    /// The default algorithm computes the full hull, and an OBB around the input vertices. This OBB is then sliced
    /// with the hull planes until the vertex limit is reached.The default algorithm requires the vertex limit
    /// to be set to at least 8, and typically produces results that are much better quality than are produced
    /// by plane shifting.
    ///
    /// When plane shifting is enabled, the hull computation stops when vertex limit is reached. The hull planes
    /// are then shifted to contain all input vertices, and the new plane intersection points are then used to
    /// generate the final hull with the given vertex limit.Plane shifting may produce sharp edges to vertices
    /// very far away from the input cloud, and does not guarantee that all input vertices are inside the resulting
    /// hull.However, it can be used with a vertex limit as low as 4.
    PlaneShifting = 32,
    /// Inertia tensor computation is faster using SIMD code, but the precision is lower, which may result
    /// in incorrect inertia for very thin hulls.
    FastInertiaComputation = 64,
    /// Convex hulls are created with respect to GPU simulation limitations. Vertex limit and polygon limit
    /// is set to 64 and vertex limit per face is internally set to 32.
    ///
    /// Can be used only with eCOMPUTE_CONVEX flag.
    GpuCompatible = 128,
    /// Convex hull input vertices are shifted to be around origin to provide better computation stability.
    /// It is recommended to provide input vertices around the origin, otherwise use this flag to improve
    /// numerical stability.
    ///
    /// Is used only with eCOMPUTE_CONVEX flag.
    ShiftVertices = 256,
}

bitflags::bitflags! {
    /// Flags for [`PxConvexFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxConvexFlags: u16 {
        const E16BitIndices = 1 << 0;
        const ComputeConvex = 1 << 1;
        const CheckZeroAreaTriangles = 1 << 2;
        const QuantizeInput = 1 << 3;
        const DisableMeshValidation = 1 << 4;
        const PlaneShifting = 1 << 5;
        const FastInertiaComputation = 1 << 6;
        const GpuCompatible = 1 << 7;
        const ShiftVertices = 1 << 8;
    }
}

/// Defines the tetrahedron structure of a mesh.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMeshFormat {
    /// Normal tetmesh with arbitrary tetrahedra
    TetMesh = 0,
    /// 6 tetrahedra in a row will form a hexahedron
    HexMesh = 1,
}

/// Desired build strategy for PxMeshMidPhase::eBVH34
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxBVH34BuildStrategy {
    /// Fast build strategy. Fast build speed, good runtime performance in most cases. Recommended for runtime mesh cooking.
    Fast = 0,
    /// Default build strategy. Medium build speed, good runtime performance in all cases.
    Default = 1,
    /// SAH build strategy. Slower builds, slightly improved runtime performance in some cases.
    Sah = 2,
    Last = 3,
}

/// Result from convex cooking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConvexMeshCookingResult {
    /// Convex mesh cooking succeeded.
    Success = 0,
    /// Convex mesh cooking failed, algorithm couldn't find 4 initial vertices without a small triangle.
    ZeroAreaTestFailed = 1,
    /// Convex mesh cooking succeeded, but the algorithm has reached the 255 polygons limit.
    /// The produced hull does not contain all input vertices. Try to simplify the input vertices
    /// or try to use the eINFLATE_CONVEX or the eQUANTIZE_INPUT flags.
    PolygonsLimitReached = 2,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 3,
}

/// Enumeration for convex mesh cooking algorithms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConvexMeshCookingType {
    /// The Quickhull algorithm constructs the hull from the given input points. The resulting hull
    /// will only contain a subset of the input points.
    Quickhull = 0,
}

/// Result from triangle mesh cooking
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxTriangleMeshCookingResult {
    /// Everything is A-OK.
    Success = 0,
    /// a triangle is too large for well-conditioned results. Tessellate the mesh for better behavior, see the user guide section on cooking for more details.
    LargeTriangle = 1,
    /// Something unrecoverable happened. Check the error stream to find out what.
    Failure = 2,
}

/// Enum for the set of mesh pre-processing parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxMeshPreprocessingFlag {
    /// When set, mesh welding is performed. See PxCookingParams::meshWeldTolerance. Clean mesh must be enabled.
    WeldVertices = 1,
    /// When set, mesh cleaning is disabled. This makes cooking faster.
    ///
    /// When clean mesh is not performed, mesh welding is also not performed.
    ///
    /// It is recommended to use only meshes that passed during validateTriangleMesh.
    DisableCleanMesh = 2,
    /// When set, active edges are set for each triangle edge. This makes cooking faster but slow up contact generation.
    DisableActiveEdgesPrecompute = 4,
    /// When set, 32-bit indices will always be created regardless of triangle count.
    ///
    /// By default mesh will be created with 16-bit indices for triangle count
    /// <
    /// = 0xFFFF and 32-bit otherwise.
    Force32bitIndices = 8,
    /// When set, a list of triangles will be created for each associated vertex in the mesh
    EnableVertMapping = 16,
    /// When set, inertia tensor is calculated for the mesh
    EnableInertia = 32,
}

bitflags::bitflags! {
    /// Flags for [`PxMeshPreprocessingFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxMeshPreprocessingFlags: u32 {
        const WeldVertices = 1 << 0;
        const DisableCleanMesh = 1 << 1;
        const DisableActiveEdgesPrecompute = 1 << 2;
        const Force32bitIndices = 1 << 3;
        const EnableVertMapping = 1 << 4;
        const EnableInertia = 1 << 5;
    }
}

/// Unique identifiers for extensions classes which implement a constraint based on PxConstraint.
///
/// Users which want to create their own custom constraint types should choose an ID larger or equal to eNEXT_FREE_ID
/// and not eINVALID_ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxConstraintExtIDs {
    Joint = 0,
    VehicleSuspLimitDeprecated = 1,
    VehicleStickyTyreDeprecated = 2,
    VehicleJoint = 3,
    NextFreeId = 4,
    InvalidId = 2147483647,
}

/// an enumeration of PhysX' built-in joint types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxJointConcreteType {
    Spherical = 256,
    Revolute = 257,
    Prismatic = 258,
    Fixed = 259,
    Distance = 260,
    D6 = 261,
    Contact = 262,
    Gear = 263,
    RackAndPinion = 264,
    Last = 265,
}

/// an enumeration for specifying one or other of the actors referenced by a joint
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxJointActorIndex {
    Actor0 = 0,
    Actor1 = 1,
    Count = 2,
}

/// flags for configuring the drive of a PxDistanceJoint
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxDistanceJointFlag {
    MaxDistanceEnabled = 2,
    MinDistanceEnabled = 4,
    SpringEnabled = 8,
}

bitflags::bitflags! {
    /// Flags for [`PxDistanceJointFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxDistanceJointFlags: u16 {
        const MaxDistanceEnabled = 1 << 1;
        const MinDistanceEnabled = 1 << 2;
        const SpringEnabled = 1 << 3;
    }
}

/// Flags specific to the prismatic joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPrismaticJointFlag {
    LimitEnabled = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxPrismaticJointFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxPrismaticJointFlags: u16 {
        const LimitEnabled = 1 << 1;
    }
}

/// Flags specific to the Revolute Joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxRevoluteJointFlag {
    /// enable the limit
    LimitEnabled = 1,
    /// enable the drive
    DriveEnabled = 2,
    /// if the existing velocity is beyond the drive velocity, do not add force
    DriveFreespin = 4,
}

bitflags::bitflags! {
    /// Flags for [`PxRevoluteJointFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxRevoluteJointFlags: u16 {
        const LimitEnabled = 1 << 0;
        const DriveEnabled = 1 << 1;
        const DriveFreespin = 1 << 2;
    }
}

/// Flags specific to the spherical joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxSphericalJointFlag {
    /// the cone limit for the spherical joint is enabled
    LimitEnabled = 2,
}

bitflags::bitflags! {
    /// Flags for [`PxSphericalJointFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxSphericalJointFlags: u16 {
        const LimitEnabled = 1 << 1;
    }
}

/// Used to specify one of the degrees of freedom of  a D6 joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxD6Axis {
    /// motion along the X axis
    X = 0,
    /// motion along the Y axis
    Y = 1,
    /// motion along the Z axis
    Z = 2,
    /// motion around the X axis
    Twist = 3,
    /// motion around the Y axis
    Swing1 = 4,
    /// motion around the Z axis
    Swing2 = 5,
    Count = 6,
}

/// Used to specify the range of motions allowed for a degree of freedom in a D6 joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxD6Motion {
    /// The DOF is locked, it does not allow relative motion.
    Locked = 0,
    /// The DOF is limited, it only allows motion within a specific range.
    Limited = 1,
    /// The DOF is free and has its full range of motion.
    Free = 2,
}

/// Used to specify which axes of a D6 joint are driven.
///
/// Each drive is an implicit force-limited damped spring:
///
/// force = spring * (target position - position) + damping * (targetVelocity - velocity)
///
/// Alternatively, the spring may be configured to generate a specified acceleration instead of a force.
///
/// A linear axis is affected by drive only if the corresponding drive flag is set. There are two possible models
/// for angular drive: swing/twist, which may be used to drive one or more angular degrees of freedom, or slerp,
/// which may only be used to drive all three angular degrees simultaneously.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxD6Drive {
    /// drive along the X-axis
    X = 0,
    /// drive along the Y-axis
    Y = 1,
    /// drive along the Z-axis
    Z = 2,
    /// drive of displacement from the X-axis
    Swing = 3,
    /// drive of the displacement around the X-axis
    Twist = 4,
    /// drive of all three angular degrees along a SLERP-path
    Slerp = 5,
    Count = 6,
}

impl From<usize> for PxD6Drive {
    fn from(val: usize) -> Self {
        #[allow(clippy::match_same_arms)]
        match val {
            0 => Self::X,
            1 => Self::Y,
            2 => Self::Z,
            3 => Self::Swing,
            4 => Self::Twist,
            5 => Self::Slerp,
            6 => Self::Count,
            _ => Self::Count,
        }
    }
}

/// flags for configuring the drive model of a PxD6Joint
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxD6JointDriveFlag {
    /// drive spring is for the acceleration at the joint (rather than the force)
    Acceleration = 1,
}

bitflags::bitflags! {
    /// Flags for [`PxD6JointDriveFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxD6JointDriveFlags: u32 {
        const Acceleration = 1 << 0;
    }
}

/// Collision filtering operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxFilterOp {
    PxFilteropAnd = 0,
    PxFilteropOr = 1,
    PxFilteropXor = 2,
    PxFilteropNand = 3,
    PxFilteropNor = 4,
    PxFilteropNxor = 5,
    PxFilteropSwapAnd = 6,
}

/// If a thread ends up waiting for work it will find itself in a spin-wait loop until work becomes available.
/// Three strategies are available to limit wasted cycles.
/// The strategies are as follows:
/// a) wait until a work task signals the end of the spin-wait period.
/// b) yield the thread by providing a hint to reschedule thread execution, thereby allowing other threads to run.
/// c) yield the processor by informing it that it is waiting for work and requesting it to more efficiently use compute resources.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxDefaultCpuDispatcherWaitForWorkMode {
    WaitForWork = 0,
    YieldThread = 1,
    YieldProcessor = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxBatchQueryStatus {
    /// This is the initial state before a query starts.
    Pending = 0,
    /// The query is finished; results have been written into the result and hit buffers.
    Success = 1,
    /// The query results were incomplete due to touch hit buffer overflow. Blocking hit is still correct.
    Overflow = 2,
}

/// types of instrumentation that PVD can do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub enum PxPvdInstrumentationFlag {
    /// Send debugging information to PVD.
    ///
    /// This information is the actual object data of the rigid statics, shapes,
    /// articulations, etc.  Sending this information has a noticeable impact on
    /// performance and thus this flag should not be set if you want an accurate
    /// performance profile.
    Debug = 1,
    /// Send profile information to PVD.
    ///
    /// This information populates PVD's profile view.  It has (at this time) negligible
    /// cost compared to Debug information and makes PVD *much* more useful so it is quite
    /// highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter.
    /// Using it allows the SDK to send profile events to PVD.
    Profile = 2,
    /// Send memory information to PVD.
    ///
    /// The PVD sdk side hooks into the Foundation memory controller and listens to
    /// allocation/deallocation events.  This has a noticable hit on the first frame,
    /// however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
    /// once it hits a steady state.  This information also has a fairly negligible
    /// impact and thus is also highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter,
    /// trackOutstandingAllocations.  Using both of them together allows users to have
    /// an accurate view of the overall memory usage of the simulation at the cost of
    /// a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
    /// attempt not to allocate or deallocate during simulation so this hashtable lookup
    /// tends to have no effect past the first frame.
    ///
    /// Sending memory information without tracking outstanding allocations means that
    /// PVD will accurate information about the state of the memory system before the
    /// actual connection happened.
    Memory = 4,
    /// Send memory information to PVD.
    ///
    /// The PVD sdk side hooks into the Foundation memory controller and listens to
    /// allocation/deallocation events.  This has a noticable hit on the first frame,
    /// however, this data is somewhat compressed and the PhysX SDK doesn't allocate much
    /// once it hits a steady state.  This information also has a fairly negligible
    /// impact and thus is also highly recommended.
    ///
    /// This flag works together with a PxCreatePhysics parameter,
    /// trackOutstandingAllocations.  Using both of them together allows users to have
    /// an accurate view of the overall memory usage of the simulation at the cost of
    /// a hashtable lookup per allocation/deallocation.  Again, PhysX makes a best effort
    /// attempt not to allocate or deallocate during simulation so this hashtable lookup
    /// tends to have no effect past the first frame.
    ///
    /// Sending memory information without tracking outstanding allocations means that
    /// PVD will accurate information about the state of the memory system before the
    /// actual connection happened.
    All = 7,
}

bitflags::bitflags! {
    /// Flags for [`PxPvdInstrumentationFlag`]
    #[derive(Default)]
    #[repr(transparent)]
    pub struct PxPvdInstrumentationFlags: u8 {
        const Debug = 1 << 0;
        const Profile = 1 << 1;
        const Memory = 1 << 2;
        const All = Self::Debug.bits | Self::Profile.bits | Self::Memory.bits;
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxMat34 {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxAllocatorCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxAssertHandler {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxFoundation {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxVirtualAllocatorCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub union PxTempAllocatorChunk {
    pub mNext: *mut PxTempAllocatorChunk,
    pub mIndex: u32,
    pub mPad: [u8; 16],
}
#[cfg(feature = "debug-structs")]
impl std::fmt::Debug for PxTempAllocatorChunk {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("PxTempAllocatorChunk")
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxLogTwo {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxUnConst {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxErrorCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxAllocationListener {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxHash {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxInputStream {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxInputData {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxOutputStream {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxProfilerCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRunnable {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRenderBuffer {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxProcessPxBaseCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSerializationContext {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSerializationRegistry {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCollection {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxTypeInfo {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxFEMSoftBodyMaterial {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxFEMClothMaterial {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxPBDMaterial {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxFLIPMaterial {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxMPMMaterial {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxCustomMaterial {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxBVH33TriangleMesh {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxParticleSystem {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxPBDParticleSystem {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxFLIPParticleSystem {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxMPMParticleSystem {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxCustomParticleSystem {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxSoftBody {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxFEMCloth {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxHairSystem {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxParticleBuffer {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxParticleAndDiffuseBuffer {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxParticleClothBuffer {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxParticleRigidBuffer {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxStringTable {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSerializer {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxInsertionCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTaskManager {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCpuDispatcher {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVHRaycastCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVHOverlapCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVHTraversalCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxContactBuffer {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxRenderOutput {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCustomGeometryCallbacks {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub union Px1DConstraintMods {
    pub spring: PxSpringModifiers,
    pub bounce: PxRestitutionModifiers,
}
#[cfg(feature = "debug-structs")]
impl std::fmt::Debug for Px1DConstraintMods {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Px1DConstraintMods")
    }
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintVisualizer {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintConnector {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintAllocator {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactModifyCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCCDContactModifyCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDeletionListener {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSimulationFilterCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxLockedData {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxCudaContextManager {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxParticleRigidAttachment {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxOmniPvd {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPhysics {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxQueryFilterCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneQuerySystemBase {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneSQSystem {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneQuerySystem {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseRegions {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhase {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxAABBManager {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPvdSceneClient {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSimulationEventCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxObstacleContext {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxUserControllerHitReport {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerFilterCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxController {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBoxController {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCapsuleController {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerBehaviorCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerManager {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultAllocator {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultErrorCallback {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxBinaryConverter {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultCpuDispatcher {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBatchQueryExt {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCustomSceneQuerySystem {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCustomSceneQuerySystemAdapter {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxCooking {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct XmlMemoryAllocator {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct XmlWriter {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct XmlReader {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct MemoryBuffer {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRepXSerializer {
    vtable_: *const std::ffi::c_void,
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxVehicleWheels4SimData {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxVehicleWheels4DynData {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxVehicleTireForceCalculator {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxVehicleDrivableSurfaceToTireFrictionPairs {
    _unused: [u8; 0],
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct PxVehicleTelemetryData {
    _unused: [u8; 0],
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPvd {
    vtable_: *const std::ffi::c_void,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPvdTransport {
    vtable_: *const std::ffi::c_void,
}
extern "C" {
    pub fn PxAllocatorCallback_delete(self_: *mut PxAllocatorCallback);

    /// Allocates size bytes of memory, which must be 16-byte aligned.
    ///
    /// This method should never return NULL.  If you run out of memory, then
    /// you should terminate the app or take some other appropriate action.
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    ///
    /// The allocated block of memory.
    pub fn PxAllocatorCallback_allocate_mut(self_: *mut PxAllocatorCallback, size: usize, typeName: *const std::ffi::c_char, filename: *const std::ffi::c_char, line: i32) -> *mut std::ffi::c_void;

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    pub fn PxAllocatorCallback_deallocate_mut(self_: *mut PxAllocatorCallback, ptr: *mut std::ffi::c_void);

    pub fn PxAssertHandler_delete(self_: *mut PxAssertHandler);

    pub fn phys_PxGetAssertHandler() -> *mut PxAssertHandler;

    pub fn phys_PxSetAssertHandler(handler: *mut PxAssertHandler);

    /// Destroys the instance it is called on.
    ///
    /// The operation will fail, if there are still modules referencing the foundation object. Release all dependent modules
    /// prior to calling this method.
    pub fn PxFoundation_release_mut(self_: *mut PxFoundation);

    /// retrieves error callback
    pub fn PxFoundation_getErrorCallback_mut(self_: *mut PxFoundation) -> *mut PxErrorCallback;

    /// Sets mask of errors to report.
    pub fn PxFoundation_setErrorLevel_mut(self_: *mut PxFoundation, mask: u32);

    /// Retrieves mask of errors to be reported.
    pub fn PxFoundation_getErrorLevel(self_: *const PxFoundation) -> u32;

    /// Retrieves the allocator this object was created with.
    pub fn PxFoundation_getAllocatorCallback_mut(self_: *mut PxFoundation) -> *mut PxAllocatorCallback;

    /// Retrieves if allocation names are being passed to allocator callback.
    pub fn PxFoundation_getReportAllocationNames(self_: *const PxFoundation) -> bool;

    /// Set if allocation names are being passed to allocator callback.
    ///
    /// Enabled by default in debug and checked build, disabled by default in profile and release build.
    pub fn PxFoundation_setReportAllocationNames_mut(self_: *mut PxFoundation, value: bool);

    pub fn PxFoundation_registerAllocationListener_mut(self_: *mut PxFoundation, listener: *mut PxAllocationListener);

    pub fn PxFoundation_deregisterAllocationListener_mut(self_: *mut PxFoundation, listener: *mut PxAllocationListener);

    pub fn PxFoundation_registerErrorCallback_mut(self_: *mut PxFoundation, callback: *mut PxErrorCallback);

    pub fn PxFoundation_deregisterErrorCallback_mut(self_: *mut PxFoundation, callback: *mut PxErrorCallback);

    /// Creates an instance of the foundation class
    ///
    /// The foundation class is needed to initialize higher level SDKs. There may be only one instance per process.
    /// Calling this method after an instance has been created already will result in an error message and NULL will be
    /// returned.
    ///
    /// Foundation instance on success, NULL if operation failed
    pub fn phys_PxCreateFoundation(version: u32, allocator: *mut PxAllocatorCallback, errorCallback: *mut PxErrorCallback) -> *mut PxFoundation;

    pub fn phys_PxSetFoundationInstance(foundation: *mut PxFoundation);

    pub fn phys_PxGetFoundation() -> *mut PxFoundation;

    /// Get the callback that will be used for all profiling.
    pub fn phys_PxGetProfilerCallback() -> *mut PxProfilerCallback;

    /// Set the callback that will be used for all profiling.
    pub fn phys_PxSetProfilerCallback(profiler: *mut PxProfilerCallback);

    /// Get the allocator callback
    pub fn phys_PxGetAllocatorCallback() -> *mut PxAllocatorCallback;

    /// Get the broadcasting allocator callback
    pub fn phys_PxGetBroadcastAllocator() -> *mut PxAllocatorCallback;

    /// Get the error callback
    pub fn phys_PxGetErrorCallback() -> *mut PxErrorCallback;

    /// Get the broadcasting error callback
    pub fn phys_PxGetBroadcastError() -> *mut PxErrorCallback;

    /// Get the warn once timestamp
    pub fn phys_PxGetWarnOnceTimeStamp() -> u32;

    /// Decrement the ref count of PxFoundation
    pub fn phys_PxDecFoundationRefCount();

    /// Increment the ref count of PxFoundation
    pub fn phys_PxIncFoundationRefCount();

    pub fn PxAllocator_new(anon_param0: *const std::ffi::c_char) -> PxAllocator;

    pub fn PxAllocator_allocate_mut(self_: *mut PxAllocator, size: usize, file: *const std::ffi::c_char, line: i32) -> *mut std::ffi::c_void;

    pub fn PxAllocator_deallocate_mut(self_: *mut PxAllocator, ptr: *mut std::ffi::c_void);

    pub fn PxRawAllocator_new(anon_param0: *const std::ffi::c_char) -> PxRawAllocator;

    pub fn PxRawAllocator_allocate_mut(self_: *mut PxRawAllocator, size: usize, anon_param1: *const std::ffi::c_char, anon_param2: i32) -> *mut std::ffi::c_void;

    pub fn PxRawAllocator_deallocate_mut(self_: *mut PxRawAllocator, ptr: *mut std::ffi::c_void);

    pub fn PxVirtualAllocatorCallback_delete(self_: *mut PxVirtualAllocatorCallback);

    pub fn PxVirtualAllocatorCallback_allocate_mut(self_: *mut PxVirtualAllocatorCallback, size: usize, group: i32, file: *const std::ffi::c_char, line: i32) -> *mut std::ffi::c_void;

    pub fn PxVirtualAllocatorCallback_deallocate_mut(self_: *mut PxVirtualAllocatorCallback, ptr: *mut std::ffi::c_void);

    pub fn PxVirtualAllocator_new(callback: *mut PxVirtualAllocatorCallback, group: i32) -> PxVirtualAllocator;

    pub fn PxVirtualAllocator_allocate_mut(self_: *mut PxVirtualAllocator, size: usize, file: *const std::ffi::c_char, line: i32) -> *mut std::ffi::c_void;

    pub fn PxVirtualAllocator_deallocate_mut(self_: *mut PxVirtualAllocator, ptr: *mut std::ffi::c_void);

    pub fn PxTempAllocatorChunk_new() -> PxTempAllocatorChunk;

    pub fn PxTempAllocator_new(anon_param0: *const std::ffi::c_char) -> PxTempAllocator;

    pub fn PxTempAllocator_allocate_mut(self_: *mut PxTempAllocator, size: usize, file: *const std::ffi::c_char, line: i32) -> *mut std::ffi::c_void;

    pub fn PxTempAllocator_deallocate_mut(self_: *mut PxTempAllocator, ptr: *mut std::ffi::c_void);

    /// Sets the bytes of the provided buffer to zero.
    ///
    /// Pointer to memory block (same as input)
    pub fn phys_PxMemZero(dest: *mut std::ffi::c_void, count: u32) -> *mut std::ffi::c_void;

    /// Sets the bytes of the provided buffer to the specified value.
    ///
    /// Pointer to memory block (same as input)
    pub fn phys_PxMemSet(dest: *mut std::ffi::c_void, c: i32, count: u32) -> *mut std::ffi::c_void;

    /// Copies the bytes of one memory block to another. The memory blocks must not overlap.
    ///
    /// Use [`PxMemMove`] if memory blocks overlap.
    ///
    /// Pointer to destination memory block
    pub fn phys_PxMemCopy(dest: *mut std::ffi::c_void, src: *const std::ffi::c_void, count: u32) -> *mut std::ffi::c_void;

    /// Copies the bytes of one memory block to another. The memory blocks can overlap.
    ///
    /// Use [`PxMemCopy`] if memory blocks do not overlap.
    ///
    /// Pointer to destination memory block
    pub fn phys_PxMemMove(dest: *mut std::ffi::c_void, src: *const std::ffi::c_void, count: u32) -> *mut std::ffi::c_void;

    /// Mark a specified amount of memory with 0xcd pattern. This is used to check that the meta data
    /// definition for serialized classes is complete in checked builds.
    pub fn phys_PxMarkSerializedMemory(ptr: *mut std::ffi::c_void, byteSize: u32);

    pub fn phys_PxMemoryBarrier();

    /// Return the index of the highest set bit. Undefined for zero arg.
    pub fn phys_PxHighestSetBitUnsafe(v: u32) -> u32;

    /// Return the index of the highest set bit. Undefined for zero arg.
    pub fn phys_PxLowestSetBitUnsafe(v: u32) -> u32;

    /// Returns the index of the highest set bit. Returns 32 for v=0.
    pub fn phys_PxCountLeadingZeros(v: u32) -> u32;

    /// Prefetch aligned 64B x86, 32b ARM around
    pub fn phys_PxPrefetchLine(ptr: *const std::ffi::c_void, offset: u32);

    /// Prefetch
    /// bytes starting at
    pub fn phys_PxPrefetch(ptr: *const std::ffi::c_void, count: u32);

    pub fn phys_PxBitCount(v: u32) -> u32;

    pub fn phys_PxIsPowerOfTwo(x: u32) -> bool;

    pub fn phys_PxNextPowerOfTwo(x: u32) -> u32;

    /// Return the index of the highest set bit. Not valid for zero arg.
    pub fn phys_PxLowestSetBit(x: u32) -> u32;

    /// Return the index of the highest set bit. Not valid for zero arg.
    pub fn phys_PxHighestSetBit(x: u32) -> u32;

    pub fn phys_PxILog2(num: u32) -> u32;

    /// default constructor leaves data uninitialized.
    pub fn PxVec3_new() -> PxVec3;

    /// zero constructor.
    pub fn PxVec3_new_1(anon_param0: PxZERO) -> PxVec3;

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    pub fn PxVec3_new_2(a: f32) -> PxVec3;

    /// Initializes from 3 scalar parameters.
    pub fn PxVec3_new_3(nx: f32, ny: f32, nz: f32) -> PxVec3;

    /// tests for exact zero vector
    pub fn PxVec3_isZero(self_: *const PxVec3) -> bool;

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    pub fn PxVec3_isFinite(self_: *const PxVec3) -> bool;

    /// is normalized - used by API parameter validation
    pub fn PxVec3_isNormalized(self_: *const PxVec3) -> bool;

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    pub fn PxVec3_magnitudeSquared(self_: *const PxVec3) -> f32;

    /// returns the magnitude
    pub fn PxVec3_magnitude(self_: *const PxVec3) -> f32;

    /// returns the scalar product of this and other.
    pub fn PxVec3_dot(self_: *const PxVec3, v: *const PxVec3) -> f32;

    /// cross product
    pub fn PxVec3_cross(self_: *const PxVec3, v: *const PxVec3) -> PxVec3;

    /// returns a unit vector
    pub fn PxVec3_getNormalized(self_: *const PxVec3) -> PxVec3;

    /// normalizes the vector in place
    pub fn PxVec3_normalize_mut(self_: *mut PxVec3) -> f32;

    /// normalizes the vector in place. Does nothing if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// Returns vector magnitude if >= PX_NORMALIZATION_EPSILON and 0.0f otherwise.
    pub fn PxVec3_normalizeSafe_mut(self_: *mut PxVec3) -> f32;

    /// normalizes the vector in place. Asserts if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// returns vector magnitude.
    pub fn PxVec3_normalizeFast_mut(self_: *mut PxVec3) -> f32;

    /// a[i] * b[i], for all i.
    pub fn PxVec3_multiply(self_: *const PxVec3, a: *const PxVec3) -> PxVec3;

    /// element-wise minimum
    pub fn PxVec3_minimum(self_: *const PxVec3, v: *const PxVec3) -> PxVec3;

    /// returns MIN(x, y, z);
    pub fn PxVec3_minElement(self_: *const PxVec3) -> f32;

    /// element-wise maximum
    pub fn PxVec3_maximum(self_: *const PxVec3, v: *const PxVec3) -> PxVec3;

    /// returns MAX(x, y, z);
    pub fn PxVec3_maxElement(self_: *const PxVec3) -> f32;

    /// returns absolute values of components;
    pub fn PxVec3_abs(self_: *const PxVec3) -> PxVec3;

    pub fn PxVec3Padded_new_alloc() -> *mut PxVec3Padded;

    pub fn PxVec3Padded_delete(self_: *mut PxVec3Padded);

    pub fn PxVec3Padded_new_alloc_1(p: *const PxVec3) -> *mut PxVec3Padded;

    pub fn PxVec3Padded_new_alloc_2(f: f32) -> *mut PxVec3Padded;

    /// Default constructor, does not do any initialization.
    pub fn PxQuat_new() -> PxQuat;

    /// identity constructor
    pub fn PxQuat_new_1(anon_param0: PxIDENTITY) -> PxQuat;

    /// Constructor from a scalar: sets the real part w to the scalar value, and the imaginary parts (x,y,z) to zero
    pub fn PxQuat_new_2(r: f32) -> PxQuat;

    /// Constructor. Take note of the order of the elements!
    pub fn PxQuat_new_3(nx: f32, ny: f32, nz: f32, nw: f32) -> PxQuat;

    /// Creates from angle-axis representation.
    ///
    /// Axis must be normalized!
    ///
    /// Angle is in radians!
    ///
    /// Unit:
    /// Radians
    pub fn PxQuat_new_4(angleRadians: f32, unitAxis: *const PxVec3) -> PxQuat;

    /// Creates from orientation matrix.
    pub fn PxQuat_new_5(m: *const PxMat33) -> PxQuat;

    /// returns true if quat is identity
    pub fn PxQuat_isIdentity(self_: *const PxQuat) -> bool;

    /// returns true if all elements are finite (not NAN or INF, etc.)
    pub fn PxQuat_isFinite(self_: *const PxQuat) -> bool;

    /// returns true if finite and magnitude is close to unit
    pub fn PxQuat_isUnit(self_: *const PxQuat) -> bool;

    /// returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
    /// isValid
    pub fn PxQuat_isSane(self_: *const PxQuat) -> bool;

    /// converts this quaternion to angle-axis representation
    pub fn PxQuat_toRadiansAndUnitAxis(self_: *const PxQuat, angle: *mut f32, axis: *mut PxVec3);

    /// Gets the angle between this quat and the identity quaternion.
    ///
    /// Unit:
    /// Radians
    pub fn PxQuat_getAngle(self_: *const PxQuat) -> f32;

    /// Gets the angle between this quat and the argument
    ///
    /// Unit:
    /// Radians
    pub fn PxQuat_getAngle_1(self_: *const PxQuat, q: *const PxQuat) -> f32;

    /// This is the squared 4D vector length, should be 1 for unit quaternions.
    pub fn PxQuat_magnitudeSquared(self_: *const PxQuat) -> f32;

    /// returns the scalar product of this and other.
    pub fn PxQuat_dot(self_: *const PxQuat, v: *const PxQuat) -> f32;

    pub fn PxQuat_getNormalized(self_: *const PxQuat) -> PxQuat;

    pub fn PxQuat_magnitude(self_: *const PxQuat) -> f32;

    /// maps to the closest unit quaternion.
    pub fn PxQuat_normalize_mut(self_: *mut PxQuat) -> f32;

    pub fn PxQuat_getConjugate(self_: *const PxQuat) -> PxQuat;

    pub fn PxQuat_getImaginaryPart(self_: *const PxQuat) -> PxVec3;

    /// brief computes rotation of x-axis
    pub fn PxQuat_getBasisVector0(self_: *const PxQuat) -> PxVec3;

    /// brief computes rotation of y-axis
    pub fn PxQuat_getBasisVector1(self_: *const PxQuat) -> PxVec3;

    /// brief computes rotation of z-axis
    pub fn PxQuat_getBasisVector2(self_: *const PxQuat) -> PxVec3;

    /// rotates passed vec by this (assumed unitary)
    pub fn PxQuat_rotate(self_: *const PxQuat, v: *const PxVec3) -> PxVec3;

    /// inverse rotates passed vec by this (assumed unitary)
    pub fn PxQuat_rotateInv(self_: *const PxQuat, v: *const PxVec3) -> PxVec3;

    pub fn PxTransform_new() -> PxTransform;

    pub fn PxTransform_new_1(position: *const PxVec3) -> PxTransform;

    pub fn PxTransform_new_2(anon_param0: PxIDENTITY) -> PxTransform;

    pub fn PxTransform_new_3(orientation: *const PxQuat) -> PxTransform;

    pub fn PxTransform_new_4(x: f32, y: f32, z: f32, aQ: PxQuat) -> PxTransform;

    pub fn PxTransform_new_5(p0: *const PxVec3, q0: *const PxQuat) -> PxTransform;

    pub fn PxTransform_new_6(m: *const PxMat44) -> PxTransform;

    pub fn PxTransform_getInverse(self_: *const PxTransform) -> PxTransform;

    pub fn PxTransform_transform(self_: *const PxTransform, input: *const PxVec3) -> PxVec3;

    pub fn PxTransform_transformInv(self_: *const PxTransform, input: *const PxVec3) -> PxVec3;

    pub fn PxTransform_rotate(self_: *const PxTransform, input: *const PxVec3) -> PxVec3;

    pub fn PxTransform_rotateInv(self_: *const PxTransform, input: *const PxVec3) -> PxVec3;

    /// Transform transform to parent (returns compound transform: first src, then *this)
    pub fn PxTransform_transform_1(self_: *const PxTransform, src: *const PxTransform) -> PxTransform;

    /// returns true if finite and q is a unit quaternion
    pub fn PxTransform_isValid(self_: *const PxTransform) -> bool;

    /// returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error
    /// vs isValid
    pub fn PxTransform_isSane(self_: *const PxTransform) -> bool;

    /// returns true if all elems are finite (not NAN or INF, etc.)
    pub fn PxTransform_isFinite(self_: *const PxTransform) -> bool;

    /// Transform transform from parent (returns compound transform: first src, then this->inverse)
    pub fn PxTransform_transformInv_1(self_: *const PxTransform, src: *const PxTransform) -> PxTransform;

    /// return a normalized transform (i.e. one in which the quaternion has unit magnitude)
    pub fn PxTransform_getNormalized(self_: *const PxTransform) -> PxTransform;

    /// Default constructor
    pub fn PxMat33_new() -> PxMat33;

    /// identity constructor
    pub fn PxMat33_new_1(anon_param0: PxIDENTITY) -> PxMat33;

    /// zero constructor
    pub fn PxMat33_new_2(anon_param0: PxZERO) -> PxMat33;

    /// Construct from three base vectors
    pub fn PxMat33_new_3(col0: *const PxVec3, col1: *const PxVec3, col2: *const PxVec3) -> PxMat33;

    /// constructor from a scalar, which generates a multiple of the identity matrix
    pub fn PxMat33_new_4(r: f32) -> PxMat33;

    /// Construct from float[9]
    pub fn PxMat33_new_5(values: *mut f32) -> PxMat33;

    /// Construct from a quaternion
    pub fn PxMat33_new_6(q: *const PxQuat) -> PxMat33;

    /// Construct from diagonal, off-diagonals are zero.
    pub fn PxMat33_createDiagonal(d: *const PxVec3) -> PxMat33;

    /// Computes the outer product of two vectors
    pub fn PxMat33_outer(a: *const PxVec3, b: *const PxVec3) -> PxMat33;

    /// Get transposed matrix
    pub fn PxMat33_getTranspose(self_: *const PxMat33) -> PxMat33;

    /// Get the real inverse
    pub fn PxMat33_getInverse(self_: *const PxMat33) -> PxMat33;

    /// Get determinant
    pub fn PxMat33_getDeterminant(self_: *const PxMat33) -> f32;

    /// Transform vector by matrix, equal to v' = M*v
    pub fn PxMat33_transform(self_: *const PxMat33, other: *const PxVec3) -> PxVec3;

    /// Transform vector by matrix transpose, v' = M^t*v
    pub fn PxMat33_transformTranspose(self_: *const PxMat33, other: *const PxVec3) -> PxVec3;

    pub fn PxMat33_front(self_: *const PxMat33) -> *const f32;

    /// Default constructor, not performing any initialization for performance reason.
    ///
    /// Use empty() function below to construct empty bounds.
    pub fn PxBounds3_new() -> PxBounds3;

    /// Construct from two bounding points
    pub fn PxBounds3_new_1(minimum: *const PxVec3, maximum: *const PxVec3) -> PxBounds3;

    /// Return empty bounds.
    pub fn PxBounds3_empty() -> PxBounds3;

    /// returns the AABB containing v0 and v1.
    pub fn PxBounds3_boundsOfPoints(v0: *const PxVec3, v1: *const PxVec3) -> PxBounds3;

    /// returns the AABB from center and extents vectors.
    pub fn PxBounds3_centerExtents(center: *const PxVec3, extent: *const PxVec3) -> PxBounds3;

    /// Construct from center, extent, and (not necessarily orthogonal) basis
    pub fn PxBounds3_basisExtent(center: *const PxVec3, basis: *const PxMat33, extent: *const PxVec3) -> PxBounds3;

    /// Construct from pose and extent
    pub fn PxBounds3_poseExtent(pose: *const PxTransform, extent: *const PxVec3) -> PxBounds3;

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    pub fn PxBounds3_transformSafe(matrix: *const PxMat33, bounds: *const PxBounds3) -> PxBounds3;

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    pub fn PxBounds3_transformFast(matrix: *const PxMat33, bounds: *const PxBounds3) -> PxBounds3;

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
    pub fn PxBounds3_transformSafe_1(transform: *const PxTransform, bounds: *const PxBounds3) -> PxBounds3;

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
    pub fn PxBounds3_transformFast_1(transform: *const PxTransform, bounds: *const PxBounds3) -> PxBounds3;

    /// Sets empty to true
    pub fn PxBounds3_setEmpty_mut(self_: *mut PxBounds3);

    /// Sets the bounds to maximum size [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS].
    pub fn PxBounds3_setMaximal_mut(self_: *mut PxBounds3);

    /// expands the volume to include v
    pub fn PxBounds3_include_mut(self_: *mut PxBounds3, v: *const PxVec3);

    /// expands the volume to include b.
    pub fn PxBounds3_include_mut_1(self_: *mut PxBounds3, b: *const PxBounds3);

    pub fn PxBounds3_isEmpty(self_: *const PxBounds3) -> bool;

    /// indicates whether the intersection of this and b is empty or not.
    pub fn PxBounds3_intersects(self_: *const PxBounds3, b: *const PxBounds3) -> bool;

    /// computes the 1D-intersection between two AABBs, on a given axis.
    pub fn PxBounds3_intersects1D(self_: *const PxBounds3, a: *const PxBounds3, axis: u32) -> bool;

    /// indicates if these bounds contain v.
    pub fn PxBounds3_contains(self_: *const PxBounds3, v: *const PxVec3) -> bool;

    /// checks a box is inside another box.
    pub fn PxBounds3_isInside(self_: *const PxBounds3, box_: *const PxBounds3) -> bool;

    /// returns the center of this axis aligned box.
    pub fn PxBounds3_getCenter(self_: *const PxBounds3) -> PxVec3;

    /// get component of the box's center along a given axis
    pub fn PxBounds3_getCenter_1(self_: *const PxBounds3, axis: u32) -> f32;

    /// get component of the box's extents along a given axis
    pub fn PxBounds3_getExtents(self_: *const PxBounds3, axis: u32) -> f32;

    /// returns the dimensions (width/height/depth) of this axis aligned box.
    pub fn PxBounds3_getDimensions(self_: *const PxBounds3) -> PxVec3;

    /// returns the extents, which are half of the width/height/depth.
    pub fn PxBounds3_getExtents_1(self_: *const PxBounds3) -> PxVec3;

    /// scales the AABB.
    ///
    /// This version is safe to call for empty bounds.
    pub fn PxBounds3_scaleSafe_mut(self_: *mut PxBounds3, scale: f32);

    /// scales the AABB.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`scaleSafe`]() instead.
    pub fn PxBounds3_scaleFast_mut(self_: *mut PxBounds3, scale: f32);

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// This version is safe to call for empty bounds.
    pub fn PxBounds3_fattenSafe_mut(self_: *mut PxBounds3, distance: f32);

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`fattenSafe`]() instead.
    pub fn PxBounds3_fattenFast_mut(self_: *mut PxBounds3, distance: f32);

    /// checks that the AABB values are not NaN
    pub fn PxBounds3_isFinite(self_: *const PxBounds3) -> bool;

    /// checks that the AABB values describe a valid configuration.
    pub fn PxBounds3_isValid(self_: *const PxBounds3) -> bool;

    /// Finds the closest point in the box to the point p. If p is contained, this will be p, otherwise it
    /// will be the closest point on the surface of the box.
    pub fn PxBounds3_closestPoint(self_: *const PxBounds3, p: *const PxVec3) -> PxVec3;

    pub fn PxErrorCallback_delete(self_: *mut PxErrorCallback);

    /// Reports an error code.
    pub fn PxErrorCallback_reportError_mut(self_: *mut PxErrorCallback, code: PxErrorCode, message: *const std::ffi::c_char, file: *const std::ffi::c_char, line: i32);

    /// callback when memory is allocated.
    pub fn PxAllocationListener_onAllocation_mut(self_: *mut PxAllocationListener, size: usize, typeName: *const std::ffi::c_char, filename: *const std::ffi::c_char, line: i32, allocatedMemory: *mut std::ffi::c_void);

    /// callback when memory is deallocated.
    pub fn PxAllocationListener_onDeallocation_mut(self_: *mut PxAllocationListener, allocatedMemory: *mut std::ffi::c_void);

    /// The default constructor.
    pub fn PxBroadcastingAllocator_new_alloc(allocator: *mut PxAllocatorCallback, error: *mut PxErrorCallback) -> *mut PxBroadcastingAllocator;

    /// The default constructor.
    pub fn PxBroadcastingAllocator_delete(self_: *mut PxBroadcastingAllocator);

    /// Allocates size bytes of memory, which must be 16-byte aligned.
    ///
    /// This method should never return NULL.  If you run out of memory, then
    /// you should terminate the app or take some other appropriate action.
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    ///
    /// The allocated block of memory.
    pub fn PxBroadcastingAllocator_allocate_mut(self_: *mut PxBroadcastingAllocator, size: usize, typeName: *const std::ffi::c_char, filename: *const std::ffi::c_char, line: i32) -> *mut std::ffi::c_void;

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    pub fn PxBroadcastingAllocator_deallocate_mut(self_: *mut PxBroadcastingAllocator, ptr: *mut std::ffi::c_void);

    /// The default constructor.
    pub fn PxBroadcastingErrorCallback_new_alloc(errorCallback: *mut PxErrorCallback) -> *mut PxBroadcastingErrorCallback;

    /// The default destructor.
    pub fn PxBroadcastingErrorCallback_delete(self_: *mut PxBroadcastingErrorCallback);

    /// Reports an error code.
    pub fn PxBroadcastingErrorCallback_reportError_mut(self_: *mut PxBroadcastingErrorCallback, code: PxErrorCode, message: *const std::ffi::c_char, file: *const std::ffi::c_char, line: i32);

    /// Enables floating point exceptions for the scalar and SIMD unit
    pub fn phys_PxEnableFPExceptions();

    /// Disables floating point exceptions for the scalar and SIMD unit
    pub fn phys_PxDisableFPExceptions();

    /// read from the stream. The number of bytes read may be less than the number requested.
    ///
    /// the number of bytes read from the stream.
    pub fn PxInputStream_read_mut(self_: *mut PxInputStream, dest: *mut std::ffi::c_void, count: u32) -> u32;

    pub fn PxInputStream_delete(self_: *mut PxInputStream);

    /// return the length of the input data
    ///
    /// size in bytes of the input data
    pub fn PxInputData_getLength(self_: *const PxInputData) -> u32;

    /// seek to the given offset from the start of the data.
    pub fn PxInputData_seek_mut(self_: *mut PxInputData, offset: u32);

    /// return the current offset from the start of the data
    ///
    /// the offset to seek to.
    pub fn PxInputData_tell(self_: *const PxInputData) -> u32;

    pub fn PxInputData_delete(self_: *mut PxInputData);

    /// write to the stream. The number of bytes written may be less than the number sent.
    ///
    /// the number of bytes written to the stream by this call.
    pub fn PxOutputStream_write_mut(self_: *mut PxOutputStream, src: *const std::ffi::c_void, count: u32) -> u32;

    pub fn PxOutputStream_delete(self_: *mut PxOutputStream);

    /// default constructor leaves data uninitialized.
    pub fn PxVec4_new() -> PxVec4;

    /// zero constructor.
    pub fn PxVec4_new_1(anon_param0: PxZERO) -> PxVec4;

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    pub fn PxVec4_new_2(a: f32) -> PxVec4;

    /// Initializes from 3 scalar parameters.
    pub fn PxVec4_new_3(nx: f32, ny: f32, nz: f32, nw: f32) -> PxVec4;

    /// Initializes from 3 scalar parameters.
    pub fn PxVec4_new_4(v: *const PxVec3, nw: f32) -> PxVec4;

    /// Initializes from an array of scalar parameters.
    pub fn PxVec4_new_5(v: *const f32) -> PxVec4;

    /// tests for exact zero vector
    pub fn PxVec4_isZero(self_: *const PxVec4) -> bool;

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
    pub fn PxVec4_isFinite(self_: *const PxVec4) -> bool;

    /// is normalized - used by API parameter validation
    pub fn PxVec4_isNormalized(self_: *const PxVec4) -> bool;

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    pub fn PxVec4_magnitudeSquared(self_: *const PxVec4) -> f32;

    /// returns the magnitude
    pub fn PxVec4_magnitude(self_: *const PxVec4) -> f32;

    /// returns the scalar product of this and other.
    pub fn PxVec4_dot(self_: *const PxVec4, v: *const PxVec4) -> f32;

    /// returns a unit vector
    pub fn PxVec4_getNormalized(self_: *const PxVec4) -> PxVec4;

    /// normalizes the vector in place
    pub fn PxVec4_normalize_mut(self_: *mut PxVec4) -> f32;

    /// a[i] * b[i], for all i.
    pub fn PxVec4_multiply(self_: *const PxVec4, a: *const PxVec4) -> PxVec4;

    /// element-wise minimum
    pub fn PxVec4_minimum(self_: *const PxVec4, v: *const PxVec4) -> PxVec4;

    /// element-wise maximum
    pub fn PxVec4_maximum(self_: *const PxVec4, v: *const PxVec4) -> PxVec4;

    pub fn PxVec4_getXYZ(self_: *const PxVec4) -> PxVec3;

    /// Default constructor
    pub fn PxMat44_new() -> PxMat44;

    /// identity constructor
    pub fn PxMat44_new_1(anon_param0: PxIDENTITY) -> PxMat44;

    /// zero constructor
    pub fn PxMat44_new_2(anon_param0: PxZERO) -> PxMat44;

    /// Construct from four 4-vectors
    pub fn PxMat44_new_3(col0: *const PxVec4, col1: *const PxVec4, col2: *const PxVec4, col3: *const PxVec4) -> PxMat44;

    /// constructor that generates a multiple of the identity matrix
    pub fn PxMat44_new_4(r: f32) -> PxMat44;

    /// Construct from three base vectors and a translation
    pub fn PxMat44_new_5(col0: *const PxVec3, col1: *const PxVec3, col2: *const PxVec3, col3: *const PxVec3) -> PxMat44;

    /// Construct from float[16]
    pub fn PxMat44_new_6(values: *mut f32) -> PxMat44;

    /// Construct from a quaternion
    pub fn PxMat44_new_7(q: *const PxQuat) -> PxMat44;

    /// Construct from a diagonal vector
    pub fn PxMat44_new_8(diagonal: *const PxVec4) -> PxMat44;

    /// Construct from Mat33 and a translation
    pub fn PxMat44_new_9(axes: *const PxMat33, position: *const PxVec3) -> PxMat44;

    pub fn PxMat44_new_10(t: *const PxTransform) -> PxMat44;

    /// Get transposed matrix
    pub fn PxMat44_getTranspose(self_: *const PxMat44) -> PxMat44;

    /// Transform vector by matrix, equal to v' = M*v
    pub fn PxMat44_transform(self_: *const PxMat44, other: *const PxVec4) -> PxVec4;

    /// Transform vector by matrix, equal to v' = M*v
    pub fn PxMat44_transform_1(self_: *const PxMat44, other: *const PxVec3) -> PxVec3;

    /// Rotate vector by matrix, equal to v' = M*v
    pub fn PxMat44_rotate(self_: *const PxMat44, other: *const PxVec4) -> PxVec4;

    /// Rotate vector by matrix, equal to v' = M*v
    pub fn PxMat44_rotate_1(self_: *const PxMat44, other: *const PxVec3) -> PxVec3;

    pub fn PxMat44_getBasis(self_: *const PxMat44, num: u32) -> PxVec3;

    pub fn PxMat44_getPosition(self_: *const PxMat44) -> PxVec3;

    pub fn PxMat44_setPosition_mut(self_: *mut PxMat44, position: *const PxVec3);

    pub fn PxMat44_front(self_: *const PxMat44) -> *const f32;

    pub fn PxMat44_scale_mut(self_: *mut PxMat44, p: *const PxVec4);

    pub fn PxMat44_inverseRT(self_: *const PxMat44) -> PxMat44;

    pub fn PxMat44_isFinite(self_: *const PxMat44) -> bool;

    /// Constructor
    pub fn PxPlane_new() -> PxPlane;

    /// Constructor from a normal and a distance
    pub fn PxPlane_new_1(nx: f32, ny: f32, nz: f32, distance: f32) -> PxPlane;

    /// Constructor from a normal and a distance
    pub fn PxPlane_new_2(normal: *const PxVec3, distance: f32) -> PxPlane;

    /// Constructor from a point on the plane and a normal
    pub fn PxPlane_new_3(point: *const PxVec3, normal: *const PxVec3) -> PxPlane;

    /// Constructor from three points
    pub fn PxPlane_new_4(p0: *const PxVec3, p1: *const PxVec3, p2: *const PxVec3) -> PxPlane;

    pub fn PxPlane_distance(self_: *const PxPlane, p: *const PxVec3) -> f32;

    pub fn PxPlane_contains(self_: *const PxPlane, p: *const PxVec3) -> bool;

    /// projects p into the plane
    pub fn PxPlane_project(self_: *const PxPlane, p: *const PxVec3) -> PxVec3;

    /// find an arbitrary point in the plane
    pub fn PxPlane_pointInPlane(self_: *const PxPlane) -> PxVec3;

    /// equivalent plane with unit normal
    pub fn PxPlane_normalize_mut(self_: *mut PxPlane);

    /// transform plane
    pub fn PxPlane_transform(self_: *const PxPlane, pose: *const PxTransform) -> PxPlane;

    /// inverse-transform plane
    pub fn PxPlane_inverseTransform(self_: *const PxPlane, pose: *const PxTransform) -> PxPlane;

    /// finds the shortest rotation between two vectors.
    ///
    /// a rotation about an axis normal to the two vectors which takes one to the other via the shortest path
    pub fn phys_PxShortestRotation(from: *const PxVec3, target: *const PxVec3) -> PxQuat;

    pub fn phys_PxDiagonalize(m: *const PxMat33, axes: *mut PxQuat) -> PxVec3;

    /// creates a transform from the endpoints of a segment, suitable for an actor transform for a PxCapsuleGeometry
    ///
    /// A PxTransform which will transform the vector (1,0,0) to the capsule axis shrunk by the halfHeight
    pub fn phys_PxTransformFromSegment(p0: *const PxVec3, p1: *const PxVec3, halfHeight: *mut f32) -> PxTransform;

    /// creates a transform from a plane equation, suitable for an actor transform for a PxPlaneGeometry
    ///
    /// a PxTransform which will transform the plane PxPlane(1,0,0,0) to the specified plane
    pub fn phys_PxTransformFromPlaneEquation(plane: *const PxPlane) -> PxTransform;

    /// creates a plane equation from a transform, such as the actor transform for a PxPlaneGeometry
    ///
    /// the plane
    pub fn phys_PxPlaneEquationFromTransform(pose: *const PxTransform) -> PxPlane;

    /// Spherical linear interpolation of two quaternions.
    ///
    /// Returns left when t=0, right when t=1 and a linear interpolation of left and right when 0
    /// <
    /// t
    /// <
    /// 1.
    /// Returns angle between -PI and PI in radians
    pub fn phys_PxSlerp(t: f32, left: *const PxQuat, right: *const PxQuat) -> PxQuat;

    /// integrate transform.
    pub fn phys_PxIntegrateTransform(curTrans: *const PxTransform, linvel: *const PxVec3, angvel: *const PxVec3, timeStep: f32, result: *mut PxTransform);

    /// Compute the exponent of a PxVec3
    pub fn phys_PxExp(v: *const PxVec3) -> PxQuat;

    /// computes a oriented bounding box around the scaled basis.
    ///
    /// Bounding box extent.
    pub fn phys_PxOptimizeBoundingBox(basis: *mut PxMat33) -> PxVec3;

    /// return Returns the log of a PxQuat
    pub fn phys_PxLog(q: *const PxQuat) -> PxVec3;

    /// return Returns 0 if v.x is largest element of v, 1 if v.y is largest element, 2 if v.z is largest element.
    pub fn phys_PxLargestAxis(v: *const PxVec3) -> u32;

    /// Compute tan(theta/2) given sin(theta) and cos(theta) as inputs.
    ///
    /// Returns tan(theta/2)
    pub fn phys_PxTanHalf(sin: f32, cos: f32) -> f32;

    /// Compute the closest point on an 2d ellipse to a given 2d point.
    ///
    /// Returns the 2d position on the surface of the ellipse that is closest to point.
    pub fn phys_PxEllipseClamp(point: *const PxVec3, radii: *const PxVec3) -> PxVec3;

    /// Compute from an input quaternion q a pair of quaternions (swing, twist) such that
    /// q = swing * twist
    /// with the caveats that swing.x = twist.y = twist.z = 0.
    pub fn phys_PxSeparateSwingTwist(q: *const PxQuat, swing: *mut PxQuat, twist: *mut PxQuat);

    /// Compute the angle between two non-unit vectors
    ///
    /// Returns the angle (in radians) between the two vector v0 and v1.
    pub fn phys_PxComputeAngle(v0: *const PxVec3, v1: *const PxVec3) -> f32;

    /// Compute two normalized vectors (right and up) that are perpendicular to an input normalized vector (dir).
    pub fn phys_PxComputeBasisVectors(dir: *const PxVec3, right: *mut PxVec3, up: *mut PxVec3);

    /// Compute three normalized vectors (dir, right and up) that are parallel to (dir) and perpendicular to (right, up) the
    /// normalized direction vector (p1 - p0)/||p1 - p0||.
    pub fn phys_PxComputeBasisVectors_1(p0: *const PxVec3, p1: *const PxVec3, dir: *mut PxVec3, right: *mut PxVec3, up: *mut PxVec3);

    /// Compute (i+1)%3
    pub fn phys_PxGetNextIndex3(i: u32) -> u32;

    pub fn phys_computeBarycentric(a: *const PxVec3, b: *const PxVec3, c: *const PxVec3, d: *const PxVec3, p: *const PxVec3, bary: *mut PxVec4);

    pub fn phys_computeBarycentric_1(a: *const PxVec3, b: *const PxVec3, c: *const PxVec3, p: *const PxVec3, bary: *mut PxVec4);

    pub fn Interpolation_PxLerp(a: f32, b: f32, t: f32) -> f32;

    pub fn Interpolation_PxBiLerp(f00: f32, f10: f32, f01: f32, f11: f32, tx: f32, ty: f32) -> f32;

    pub fn Interpolation_PxTriLerp(f000: f32, f100: f32, f010: f32, f110: f32, f001: f32, f101: f32, f011: f32, f111: f32, tx: f32, ty: f32, tz: f32) -> f32;

    pub fn Interpolation_PxSDFIdx(i: u32, j: u32, k: u32, nbX: u32, nbY: u32) -> u32;

    pub fn Interpolation_PxSDFSampleImpl(sdf: *const f32, localPos: *const PxVec3, sdfBoxLower: *const PxVec3, sdfBoxHigher: *const PxVec3, sdfDx: f32, invSdfDx: f32, dimX: u32, dimY: u32, dimZ: u32, tolerance: f32) -> f32;

    pub fn phys_PxSdfSample(sdf: *const f32, localPos: *const PxVec3, sdfBoxLower: *const PxVec3, sdfBoxHigher: *const PxVec3, sdfDx: f32, invSdfDx: f32, dimX: u32, dimY: u32, dimZ: u32, gradient: *mut PxVec3, tolerance: f32) -> f32;

    /// The constructor for Mutex creates a mutex. It is initially unlocked.
    pub fn PxMutexImpl_new_alloc() -> *mut PxMutexImpl;

    /// The destructor for Mutex deletes the mutex.
    pub fn PxMutexImpl_delete(self_: *mut PxMutexImpl);

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method blocks until the mutex is
    /// unlocked.
    pub fn PxMutexImpl_lock_mut(self_: *mut PxMutexImpl);

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method returns false without blocking.
    pub fn PxMutexImpl_trylock_mut(self_: *mut PxMutexImpl) -> bool;

    /// Release (unlock) the mutex.
    pub fn PxMutexImpl_unlock_mut(self_: *mut PxMutexImpl);

    /// Size of this class.
    pub fn PxMutexImpl_getSize() -> u32;

    pub fn PxReadWriteLock_new_alloc() -> *mut PxReadWriteLock;

    pub fn PxReadWriteLock_delete(self_: *mut PxReadWriteLock);

    pub fn PxReadWriteLock_lockReader_mut(self_: *mut PxReadWriteLock, takeLock: bool);

    pub fn PxReadWriteLock_lockWriter_mut(self_: *mut PxReadWriteLock);

    pub fn PxReadWriteLock_unlockReader_mut(self_: *mut PxReadWriteLock);

    pub fn PxReadWriteLock_unlockWriter_mut(self_: *mut PxReadWriteLock);

    /// Mark the beginning of a nested profile block
    ///
    /// Returns implementation-specific profiler data for this event
    pub fn PxProfilerCallback_zoneStart_mut(self_: *mut PxProfilerCallback, eventName: *const std::ffi::c_char, detached: bool, contextId: u64) -> *mut std::ffi::c_void;

    /// Mark the end of a nested profile block
    ///
    /// eventName plus contextId can be used to uniquely match up start and end of a zone.
    pub fn PxProfilerCallback_zoneEnd_mut(self_: *mut PxProfilerCallback, profilerData: *mut std::ffi::c_void, eventName: *const std::ffi::c_char, detached: bool, contextId: u64);

    pub fn PxProfileScoped_new_alloc(callback: *mut PxProfilerCallback, eventName: *const std::ffi::c_char, detached: bool, contextId: u64) -> *mut PxProfileScoped;

    pub fn PxProfileScoped_delete(self_: *mut PxProfileScoped);

    pub fn PxSListEntry_new() -> PxSListEntry;

    pub fn PxSListEntry_next_mut(self_: *mut PxSListEntry) -> *mut PxSListEntry;

    pub fn PxSListImpl_new_alloc() -> *mut PxSListImpl;

    pub fn PxSListImpl_delete(self_: *mut PxSListImpl);

    pub fn PxSListImpl_push_mut(self_: *mut PxSListImpl, entry: *mut PxSListEntry);

    pub fn PxSListImpl_pop_mut(self_: *mut PxSListImpl) -> *mut PxSListEntry;

    pub fn PxSListImpl_flush_mut(self_: *mut PxSListImpl) -> *mut PxSListEntry;

    pub fn PxSListImpl_getSize() -> u32;

    pub fn PxSyncImpl_new_alloc() -> *mut PxSyncImpl;

    pub fn PxSyncImpl_delete(self_: *mut PxSyncImpl);

    /// Wait on the object for at most the given number of ms. Returns
    /// true if the object is signaled. Sync::waitForever will block forever
    /// or until the object is signaled.
    pub fn PxSyncImpl_wait_mut(self_: *mut PxSyncImpl, milliseconds: u32) -> bool;

    /// Signal the synchronization object, waking all threads waiting on it
    pub fn PxSyncImpl_set_mut(self_: *mut PxSyncImpl);

    /// Reset the synchronization object
    pub fn PxSyncImpl_reset_mut(self_: *mut PxSyncImpl);

    /// Size of this class.
    pub fn PxSyncImpl_getSize() -> u32;

    pub fn PxRunnable_new_alloc() -> *mut PxRunnable;

    pub fn PxRunnable_delete(self_: *mut PxRunnable);

    pub fn PxRunnable_execute_mut(self_: *mut PxRunnable);

    pub fn phys_PxTlsAlloc() -> u32;

    pub fn phys_PxTlsFree(index: u32);

    pub fn phys_PxTlsGet(index: u32) -> *mut std::ffi::c_void;

    pub fn phys_PxTlsGetValue(index: u32) -> usize;

    pub fn phys_PxTlsSet(index: u32, value: *mut std::ffi::c_void) -> u32;

    pub fn phys_PxTlsSetValue(index: u32, value: usize) -> u32;

    pub fn PxCounterFrequencyToTensOfNanos_new(inNum: u64, inDenom: u64) -> PxCounterFrequencyToTensOfNanos;

    pub fn PxCounterFrequencyToTensOfNanos_toTensOfNanos(self_: *const PxCounterFrequencyToTensOfNanos, inCounter: u64) -> u64;

    pub fn PxTime_getBootCounterFrequency() -> *const PxCounterFrequencyToTensOfNanos;

    pub fn PxTime_getCounterFrequency() -> PxCounterFrequencyToTensOfNanos;

    pub fn PxTime_getCurrentCounterValue() -> u64;

    pub fn PxTime_getCurrentTimeInTensOfNanoSeconds() -> u64;

    pub fn PxTime_new() -> PxTime;

    pub fn PxTime_getElapsedSeconds_mut(self_: *mut PxTime) -> f64;

    pub fn PxTime_peekElapsedSeconds_mut(self_: *mut PxTime) -> f64;

    pub fn PxTime_getLastTime(self_: *const PxTime) -> f64;

    /// default constructor leaves data uninitialized.
    pub fn PxVec2_new() -> PxVec2;

    /// zero constructor.
    pub fn PxVec2_new_1(anon_param0: PxZERO) -> PxVec2;

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
    pub fn PxVec2_new_2(a: f32) -> PxVec2;

    /// Initializes from 2 scalar parameters.
    pub fn PxVec2_new_3(nx: f32, ny: f32) -> PxVec2;

    /// tests for exact zero vector
    pub fn PxVec2_isZero(self_: *const PxVec2) -> bool;

    /// returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
    pub fn PxVec2_isFinite(self_: *const PxVec2) -> bool;

    /// is normalized - used by API parameter validation
    pub fn PxVec2_isNormalized(self_: *const PxVec2) -> bool;

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
    pub fn PxVec2_magnitudeSquared(self_: *const PxVec2) -> f32;

    /// returns the magnitude
    pub fn PxVec2_magnitude(self_: *const PxVec2) -> f32;

    /// returns the scalar product of this and other.
    pub fn PxVec2_dot(self_: *const PxVec2, v: *const PxVec2) -> f32;

    /// returns a unit vector
    pub fn PxVec2_getNormalized(self_: *const PxVec2) -> PxVec2;

    /// normalizes the vector in place
    pub fn PxVec2_normalize_mut(self_: *mut PxVec2) -> f32;

    /// a[i] * b[i], for all i.
    pub fn PxVec2_multiply(self_: *const PxVec2, a: *const PxVec2) -> PxVec2;

    /// element-wise minimum
    pub fn PxVec2_minimum(self_: *const PxVec2, v: *const PxVec2) -> PxVec2;

    /// returns MIN(x, y);
    pub fn PxVec2_minElement(self_: *const PxVec2) -> f32;

    /// element-wise maximum
    pub fn PxVec2_maximum(self_: *const PxVec2, v: *const PxVec2) -> PxVec2;

    /// returns MAX(x, y);
    pub fn PxVec2_maxElement(self_: *const PxVec2) -> f32;

    pub fn PxStridedData_new() -> PxStridedData;

    pub fn PxBoundedData_new() -> PxBoundedData;

    pub fn PxDebugPoint_new(p: *const PxVec3, c: *const u32) -> PxDebugPoint;

    pub fn PxDebugLine_new(p0: *const PxVec3, p1: *const PxVec3, c: *const u32) -> PxDebugLine;

    pub fn PxDebugTriangle_new(p0: *const PxVec3, p1: *const PxVec3, p2: *const PxVec3, c: *const u32) -> PxDebugTriangle;

    pub fn PxDebugText_new() -> PxDebugText;

    pub fn PxDebugText_new_1(pos: *const PxVec3, sz: *const f32, clr: *const u32, str: *const std::ffi::c_char) -> PxDebugText;

    pub fn PxRenderBuffer_delete(self_: *mut PxRenderBuffer);

    pub fn PxRenderBuffer_getNbPoints(self_: *const PxRenderBuffer) -> u32;

    pub fn PxRenderBuffer_getPoints(self_: *const PxRenderBuffer) -> *const PxDebugPoint;

    pub fn PxRenderBuffer_addPoint_mut(self_: *mut PxRenderBuffer, point: *const PxDebugPoint);

    pub fn PxRenderBuffer_getNbLines(self_: *const PxRenderBuffer) -> u32;

    pub fn PxRenderBuffer_getLines(self_: *const PxRenderBuffer) -> *const PxDebugLine;

    pub fn PxRenderBuffer_addLine_mut(self_: *mut PxRenderBuffer, line: *const PxDebugLine);

    pub fn PxRenderBuffer_reserveLines_mut(self_: *mut PxRenderBuffer, nbLines: u32) -> *mut PxDebugLine;

    pub fn PxRenderBuffer_reservePoints_mut(self_: *mut PxRenderBuffer, nbLines: u32) -> *mut PxDebugPoint;

    pub fn PxRenderBuffer_getNbTriangles(self_: *const PxRenderBuffer) -> u32;

    pub fn PxRenderBuffer_getTriangles(self_: *const PxRenderBuffer) -> *const PxDebugTriangle;

    pub fn PxRenderBuffer_addTriangle_mut(self_: *mut PxRenderBuffer, triangle: *const PxDebugTriangle);

    pub fn PxRenderBuffer_append_mut(self_: *mut PxRenderBuffer, other: *const PxRenderBuffer);

    pub fn PxRenderBuffer_clear_mut(self_: *mut PxRenderBuffer);

    pub fn PxRenderBuffer_shift_mut(self_: *mut PxRenderBuffer, delta: *const PxVec3);

    pub fn PxRenderBuffer_empty(self_: *const PxRenderBuffer) -> bool;

    pub fn PxProcessPxBaseCallback_delete(self_: *mut PxProcessPxBaseCallback);

    pub fn PxProcessPxBaseCallback_process_mut(self_: *mut PxProcessPxBaseCallback, anon_param0: *mut PxBase);

    /// Registers a reference value corresponding to a PxBase object.
    ///
    /// This method is assumed to be called in the implementation of PxSerializer::registerReferences for serialized
    /// references that need to be resolved on deserialization.
    ///
    /// A reference needs to be associated with exactly one PxBase object in either the collection or the
    /// external references collection.
    ///
    /// Different kinds of references are supported and need to be specified. In the most common case
    /// (PX_SERIAL_REF_KIND_PXBASE) the PxBase object matches the reference value (which is the pointer
    /// to the PxBase object). Integer references maybe registered as well (used for internal material
    /// indices with PX_SERIAL_REF_KIND_MATERIAL_IDX). Other kinds could be added with the restriction that
    /// for pointer types the kind value needs to be marked with the PX_SERIAL_REF_KIND_PTR_TYPE_BIT.
    pub fn PxSerializationContext_registerReference_mut(self_: *mut PxSerializationContext, base: *mut PxBase, kind: u32, reference: usize);

    /// Returns the collection that is being serialized.
    pub fn PxSerializationContext_getCollection(self_: *const PxSerializationContext) -> *const PxCollection;

    /// Serializes object data and object extra data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    pub fn PxSerializationContext_writeData_mut(self_: *mut PxSerializationContext, data: *const std::ffi::c_void, size: u32);

    /// Aligns the serialized data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
    pub fn PxSerializationContext_alignData_mut(self_: *mut PxSerializationContext, alignment: u32);

    /// Helper function to write a name to the extraData if serialization is configured to save names.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportExtraData.
    pub fn PxSerializationContext_writeName_mut(self_: *mut PxSerializationContext, name: *const std::ffi::c_char);

    /// Retrieves a pointer to a deserialized PxBase object given a corresponding deserialized reference value
    ///
    /// This method is assumed to be called in the implementation of PxSerializer::createObject in order
    /// to update reference values on deserialization.
    ///
    /// To update a PxBase reference the corresponding deserialized pointer value needs to be provided in order to retrieve
    /// the location of the corresponding deserialized PxBase object. (PxDeserializationContext::translatePxBase simplifies
    /// this common case).
    ///
    /// For other kinds of references the reverence values need to be updated by deduction given the corresponding PxBase instance.
    ///
    /// PxBase object associated with the reference value
    pub fn PxDeserializationContext_resolveReference(self_: *const PxDeserializationContext, kind: u32, reference: usize) -> *mut PxBase;

    /// Helper function to read a name from the extra data during deserialization.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    pub fn PxDeserializationContext_readName_mut(self_: *mut PxDeserializationContext, name: *mut *const std::ffi::c_char);

    /// Function to align the extra data stream to a power of 2 alignment
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
    pub fn PxDeserializationContext_alignExtraData_mut(self_: *mut PxDeserializationContext, alignment: u32);

    /// Register a serializer for a concrete type
    pub fn PxSerializationRegistry_registerSerializer_mut(self_: *mut PxSerializationRegistry, type_: u16, serializer: *mut PxSerializer);

    /// Unregister a serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered serializer corresponding to type, NULL for types for which no serializer has been registered.
    pub fn PxSerializationRegistry_unregisterSerializer_mut(self_: *mut PxSerializationRegistry, type_: u16) -> *mut PxSerializer;

    /// Returns PxSerializer corresponding to type
    ///
    /// Registered PxSerializer object corresponding to type
    pub fn PxSerializationRegistry_getSerializer(self_: *const PxSerializationRegistry, type_: u16) -> *const PxSerializer;

    /// Register a RepX serializer for a concrete type
    pub fn PxSerializationRegistry_registerRepXSerializer_mut(self_: *mut PxSerializationRegistry, type_: u16, serializer: *mut PxRepXSerializer);

    /// Unregister a RepX serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered PxRepxSerializer corresponding to type, NULL for types for which no RepX serializer has been registered.
    pub fn PxSerializationRegistry_unregisterRepXSerializer_mut(self_: *mut PxSerializationRegistry, type_: u16) -> *mut PxRepXSerializer;

    /// Returns RepX serializer given the corresponding type name
    ///
    /// Registered PxRepXSerializer object corresponding to type name
    pub fn PxSerializationRegistry_getRepXSerializer(self_: *const PxSerializationRegistry, typeName: *const std::ffi::c_char) -> *mut PxRepXSerializer;

    /// Releases PxSerializationRegistry instance.
    ///
    /// This unregisters all PhysX and PhysXExtension serializers. Make sure to unregister all custom type
    /// serializers before releasing the PxSerializationRegistry.
    pub fn PxSerializationRegistry_release_mut(self_: *mut PxSerializationRegistry);

    /// Adds a PxBase object to the collection.
    ///
    /// Adds a PxBase object to the collection. Optionally a PxSerialObjectId can be provided
    /// in order to resolve dependencies between collections. A PxSerialObjectId value of PX_SERIAL_OBJECT_ID_INVALID
    /// means the object remains without id. Objects can be added regardless of other objects they require. If the object
    /// is already in the collection, the ID will be set if it was PX_SERIAL_OBJECT_ID_INVALID previously, otherwise the
    /// operation fails.
    pub fn PxCollection_add_mut(self_: *mut PxCollection, object: *mut PxBase, id: u64);

    /// Removes a PxBase member object from the collection.
    ///
    /// Object needs to be contained by the collection.
    pub fn PxCollection_remove_mut(self_: *mut PxCollection, object: *mut PxBase);

    /// Returns whether the collection contains a certain PxBase object.
    ///
    /// Whether object is contained.
    pub fn PxCollection_contains(self_: *const PxCollection, object: *mut PxBase) -> bool;

    /// Adds an id to a member PxBase object.
    ///
    /// If the object is already associated with an id within the collection, the id is replaced.
    /// May only be called for objects that are members of the collection. The id needs to be unique
    /// within the collection.
    pub fn PxCollection_addId_mut(self_: *mut PxCollection, object: *mut PxBase, id: u64);

    /// Removes id from a contained PxBase object.
    ///
    /// May only be called for ids that are associated with an object in the collection.
    pub fn PxCollection_removeId_mut(self_: *mut PxCollection, id: u64);

    /// Adds all PxBase objects and their ids of collection to this collection.
    ///
    /// PxBase objects already in this collection are ignored. Object ids need to be conflict
    /// free, i.e. the same object may not have two different ids within the two collections.
    pub fn PxCollection_add_mut_1(self_: *mut PxCollection, collection: *mut PxCollection);

    /// Removes all PxBase objects of collection from this collection.
    ///
    /// PxBase objects not present in this collection are ignored. Ids of objects
    /// which are removed are also removed.
    pub fn PxCollection_remove_mut_1(self_: *mut PxCollection, collection: *mut PxCollection);

    /// Gets number of PxBase objects in this collection.
    ///
    /// Number of objects in this collection
    pub fn PxCollection_getNbObjects(self_: *const PxCollection) -> u32;

    /// Gets the PxBase object of this collection given its index.
    ///
    /// PxBase object at index index
    pub fn PxCollection_getObject(self_: *const PxCollection, index: u32) -> *mut PxBase;

    /// Copies member PxBase pointers to a user specified buffer.
    ///
    /// number of members PxBase objects that have been written to the userBuffer
    pub fn PxCollection_getObjects(self_: *const PxCollection, userBuffer: *mut *mut PxBase, bufferSize: u32, startIndex: u32) -> u32;

    /// Looks for a PxBase object given a PxSerialObjectId value.
    ///
    /// If there is no PxBase object in the collection with the given id, NULL is returned.
    ///
    /// PxBase object with the given id value or NULL
    pub fn PxCollection_find(self_: *const PxCollection, id: u64) -> *mut PxBase;

    /// Gets number of PxSerialObjectId names in this collection.
    ///
    /// Number of PxSerialObjectId names in this collection
    pub fn PxCollection_getNbIds(self_: *const PxCollection) -> u32;

    /// Copies member PxSerialObjectId values to a user specified buffer.
    ///
    /// number of members PxSerialObjectId values that have been written to the userBuffer
    pub fn PxCollection_getIds(self_: *const PxCollection, userBuffer: *mut u64, bufferSize: u32, startIndex: u32) -> u32;

    /// Gets the PxSerialObjectId name of a PxBase object within the collection.
    ///
    /// The PxBase object needs to be a member of the collection.
    ///
    /// PxSerialObjectId name of the object or PX_SERIAL_OBJECT_ID_INVALID if the object is unnamed
    pub fn PxCollection_getId(self_: *const PxCollection, object: *const PxBase) -> u64;

    /// Deletes a collection object.
    ///
    /// This function only deletes the collection object, i.e. the container class. It doesn't delete objects
    /// that are part of the collection.
    pub fn PxCollection_release_mut(self_: *mut PxCollection);

    /// Creates a collection object.
    ///
    /// Objects can only be serialized or deserialized through a collection.
    /// For serialization, users must add objects to the collection and serialize the collection as a whole.
    /// For deserialization, the system gives back a collection of deserialized objects to users.
    ///
    /// The new collection object.
    pub fn phys_PxCreateCollection() -> *mut PxCollection;

    /// Releases the PxBase instance, please check documentation of release in derived class.
    pub fn PxBase_release_mut(self_: *mut PxBase);

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    pub fn PxBase_getConcreteTypeName(self_: *const PxBase) -> *const std::ffi::c_char;

    /// Returns concrete type of object.
    ///
    /// PxConcreteType::Enum of serialized object
    pub fn PxBase_getConcreteType(self_: *const PxBase) -> u16;

    /// Set PxBaseFlag
    pub fn PxBase_setBaseFlag_mut(self_: *mut PxBase, flag: PxBaseFlag, value: bool);

    /// Set PxBaseFlags
    pub fn PxBase_setBaseFlags_mut(self_: *mut PxBase, inFlags: PxBaseFlags);

    /// Returns PxBaseFlags
    ///
    /// PxBaseFlags
    pub fn PxBase_getBaseFlags(self_: *const PxBase) -> PxBaseFlags;

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    pub fn PxBase_isReleasable(self_: *const PxBase) -> bool;

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    pub fn PxRefCounted_release_mut(self_: *mut PxRefCounted);

    /// Returns the reference count of the object.
    ///
    /// At creation, the reference count of the object is 1. Every other object referencing this object increments the
    /// count by 1. When the reference count reaches 0, and only then, the object gets destroyed automatically.
    ///
    /// the current reference count.
    pub fn PxRefCounted_getReferenceCount(self_: *const PxRefCounted) -> u32;

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    pub fn PxRefCounted_acquireReference_mut(self_: *mut PxRefCounted);

    /// constructor sets to default
    pub fn PxTolerancesScale_new(defaultLength: f32, defaultSpeed: f32) -> PxTolerancesScale;

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid (returns always true).
    pub fn PxTolerancesScale_isValid(self_: *const PxTolerancesScale) -> bool;

    /// Allocate a new string.
    ///
    /// *Always* a valid null terminated string.  "" is returned if "" or null is passed in.
    pub fn PxStringTable_allocateStr_mut(self_: *mut PxStringTable, inSrc: *const std::ffi::c_char) -> *const std::ffi::c_char;

    /// Release the string table and all the strings associated with it.
    pub fn PxStringTable_release_mut(self_: *mut PxStringTable);

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
    pub fn PxSerializer_getConcreteTypeName(self_: *const PxSerializer) -> *const std::ffi::c_char;

    /// Adds required objects to the collection.
    ///
    /// This method does not add the required objects recursively, e.g. objects required by required objects.
    pub fn PxSerializer_requiresObjects(self_: *const PxSerializer, anon_param0: *mut PxBase, anon_param1: *mut PxProcessPxBaseCallback);

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
    pub fn PxSerializer_isSubordinate(self_: *const PxSerializer) -> bool;

    /// Exports object's extra data to stream.
    pub fn PxSerializer_exportExtraData(self_: *const PxSerializer, anon_param0: *mut PxBase, anon_param1: *mut PxSerializationContext);

    /// Exports object's data to stream.
    pub fn PxSerializer_exportData(self_: *const PxSerializer, anon_param0: *mut PxBase, anon_param1: *mut PxSerializationContext);

    /// Register references that the object maintains to other objects.
    pub fn PxSerializer_registerReferences(self_: *const PxSerializer, obj: *mut PxBase, s: *mut PxSerializationContext);

    /// Returns size needed to create the class instance.
    ///
    /// sizeof class instance.
    pub fn PxSerializer_getClassSize(self_: *const PxSerializer) -> usize;

    /// Create object at a given address, resolve references and import extra data.
    ///
    /// Created PxBase pointer (needs to be identical to address before increment).
    pub fn PxSerializer_createObject(self_: *const PxSerializer, address: *mut *mut u8, context: *mut PxDeserializationContext) -> *mut PxBase;

    /// *******************************************************************************************************************
    pub fn PxSerializer_delete(self_: *mut PxSerializer);

    /// Builds object (TriangleMesh, Heightfield, ConvexMesh or BVH) from given data in PxPhysics.
    ///
    /// PxBase Created object in PxPhysics.
    pub fn PxInsertionCallback_buildObjectFromData_mut(self_: *mut PxInsertionCallback, type_: PxConcreteType, data: *mut std::ffi::c_void) -> *mut PxBase;

    /// Set the user-provided dispatcher object for CPU tasks
    pub fn PxTaskManager_setCpuDispatcher_mut(self_: *mut PxTaskManager, ref_: *mut PxCpuDispatcher);

    /// Get the user-provided dispatcher object for CPU tasks
    ///
    /// The CPU dispatcher object.
    pub fn PxTaskManager_getCpuDispatcher(self_: *const PxTaskManager) -> *mut PxCpuDispatcher;

    /// Reset any dependencies between Tasks
    ///
    /// Will be called at the start of every frame before tasks are submitted.
    pub fn PxTaskManager_resetDependencies_mut(self_: *mut PxTaskManager);

    /// Called by the owning scene to start the task graph.
    ///
    /// All tasks with ref count of 1 will be dispatched.
    pub fn PxTaskManager_startSimulation_mut(self_: *mut PxTaskManager);

    /// Called by the owning scene at the end of a simulation step.
    pub fn PxTaskManager_stopSimulation_mut(self_: *mut PxTaskManager);

    /// Called by the worker threads to inform the PxTaskManager that a task has completed processing.
    pub fn PxTaskManager_taskCompleted_mut(self_: *mut PxTaskManager, task: *mut PxTask);

    /// Retrieve a task by name
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    pub fn PxTaskManager_getNamedTask_mut(self_: *mut PxTaskManager, name: *const std::ffi::c_char) -> u32;

    /// Submit a task with a unique name.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    pub fn PxTaskManager_submitNamedTask_mut(self_: *mut PxTaskManager, task: *mut PxTask, name: *const std::ffi::c_char, type_: PxTaskType) -> u32;

    /// Submit an unnamed task.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
    pub fn PxTaskManager_submitUnnamedTask_mut(self_: *mut PxTaskManager, task: *mut PxTask, type_: PxTaskType) -> u32;

    /// Retrieve a task given a task ID
    ///
    /// The task associated with the ID
    pub fn PxTaskManager_getTaskFromID_mut(self_: *mut PxTaskManager, id: u32) -> *mut PxTask;

    /// Release the PxTaskManager object, referenced dispatchers will not be released
    pub fn PxTaskManager_release_mut(self_: *mut PxTaskManager);

    /// Construct a new PxTaskManager instance with the given [optional] dispatchers
    pub fn PxTaskManager_createTaskManager(errorCallback: *mut PxErrorCallback, anon_param1: *mut PxCpuDispatcher) -> *mut PxTaskManager;

    /// Called by the TaskManager when a task is to be queued for execution.
    ///
    /// Upon receiving a task, the dispatcher should schedule the task to run.
    /// After the task has been run, it should call the release() method and
    /// discard its pointer.
    pub fn PxCpuDispatcher_submitTask_mut(self_: *mut PxCpuDispatcher, task: *mut PxBaseTask);

    /// Returns the number of available worker threads for this dispatcher.
    ///
    /// The SDK will use this count to control how many tasks are submitted. By
    /// matching the number of tasks with the number of execution units task
    /// overhead can be reduced.
    pub fn PxCpuDispatcher_getWorkerCount(self_: *const PxCpuDispatcher) -> u32;

    pub fn PxCpuDispatcher_delete(self_: *mut PxCpuDispatcher);

    /// The user-implemented run method where the task's work should be performed
    ///
    /// run() methods must be thread safe, stack friendly (no alloca, etc), and
    /// must never block.
    pub fn PxBaseTask_run_mut(self_: *mut PxBaseTask);

    /// Return a user-provided task name for profiling purposes.
    ///
    /// It does not have to be unique, but unique names are helpful.
    ///
    /// The name of this task
    pub fn PxBaseTask_getName(self_: *const PxBaseTask) -> *const std::ffi::c_char;

    /// Implemented by derived implementation classes
    pub fn PxBaseTask_addReference_mut(self_: *mut PxBaseTask);

    /// Implemented by derived implementation classes
    pub fn PxBaseTask_removeReference_mut(self_: *mut PxBaseTask);

    /// Implemented by derived implementation classes
    pub fn PxBaseTask_getReference(self_: *const PxBaseTask) -> i32;

    /// Implemented by derived implementation classes
    ///
    /// A task may assume in its release() method that the task system no longer holds
    /// references to it - so it may safely run its destructor, recycle itself, etc.
    /// provided no additional user references to the task exist
    pub fn PxBaseTask_release_mut(self_: *mut PxBaseTask);

    /// Return PxTaskManager to which this task was submitted
    ///
    /// Note, can return NULL if task was not submitted, or has been
    /// completed.
    pub fn PxBaseTask_getTaskManager(self_: *const PxBaseTask) -> *mut PxTaskManager;

    pub fn PxBaseTask_setContextId_mut(self_: *mut PxBaseTask, id: u64);

    pub fn PxBaseTask_getContextId(self_: *const PxBaseTask) -> u64;

    /// Release method implementation
    pub fn PxTask_release_mut(self_: *mut PxTask);

    /// Inform the PxTaskManager this task must finish before the given
    pub fn PxTask_finishBefore_mut(self_: *mut PxTask, taskID: u32);

    /// Inform the PxTaskManager this task cannot start until the given
    pub fn PxTask_startAfter_mut(self_: *mut PxTask, taskID: u32);

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    pub fn PxTask_addReference_mut(self_: *mut PxTask);

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    pub fn PxTask_removeReference_mut(self_: *mut PxTask);

    /// Return the ref-count for this task
    pub fn PxTask_getReference(self_: *const PxTask) -> i32;

    /// Return the unique ID for this task
    pub fn PxTask_getTaskID(self_: *const PxTask) -> u32;

    /// Called by PxTaskManager at submission time for initialization
    ///
    /// Perform simulation step initialization here.
    pub fn PxTask_submitted_mut(self_: *mut PxTask);

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// Submission is deferred until the task's mRefCount is decremented to zero.
    /// Note that we only use the PxTaskManager to query the appropriate dispatcher.
    pub fn PxLightCpuTask_setContinuation_mut(self_: *mut PxLightCpuTask, tm: *mut PxTaskManager, c: *mut PxBaseTask);

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// This overload of setContinuation() queries the PxTaskManager from the continuation
    /// task, which cannot be NULL.
    pub fn PxLightCpuTask_setContinuation_mut_1(self_: *mut PxLightCpuTask, c: *mut PxBaseTask);

    /// Retrieves continuation task
    pub fn PxLightCpuTask_getContinuation(self_: *const PxLightCpuTask) -> *mut PxBaseTask;

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
    pub fn PxLightCpuTask_removeReference_mut(self_: *mut PxLightCpuTask);

    /// Return the ref-count for this task
    pub fn PxLightCpuTask_getReference(self_: *const PxLightCpuTask) -> i32;

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
    pub fn PxLightCpuTask_addReference_mut(self_: *mut PxLightCpuTask);

    /// called by CpuDispatcher after run method has completed
    ///
    /// Decrements the continuation task's reference count, if specified.
    pub fn PxLightCpuTask_release_mut(self_: *mut PxLightCpuTask);

    /// Returns the type of the geometry.
    ///
    /// The type of the object.
    pub fn PxGeometry_getType(self_: *const PxGeometry) -> PxGeometryType;

    /// Constructor to initialize half extents from scalar parameters.
    pub fn PxBoxGeometry_new(hx: f32, hy: f32, hz: f32) -> PxBoxGeometry;

    /// Constructor to initialize half extents from vector parameter.
    pub fn PxBoxGeometry_new_1(halfExtents_: PxVec3) -> PxBoxGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid box has a positive extent in each direction (halfExtents.x > 0, halfExtents.y > 0, halfExtents.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a box that has zero extent in any direction.
    pub fn PxBoxGeometry_isValid(self_: *const PxBoxGeometry) -> bool;

    pub fn PxBVHRaycastCallback_delete(self_: *mut PxBVHRaycastCallback);

    pub fn PxBVHRaycastCallback_reportHit_mut(self_: *mut PxBVHRaycastCallback, boundsIndex: u32, distance: *mut f32) -> bool;

    pub fn PxBVHOverlapCallback_delete(self_: *mut PxBVHOverlapCallback);

    pub fn PxBVHOverlapCallback_reportHit_mut(self_: *mut PxBVHOverlapCallback, boundsIndex: u32) -> bool;

    pub fn PxBVHTraversalCallback_delete(self_: *mut PxBVHTraversalCallback);

    pub fn PxBVHTraversalCallback_visitNode_mut(self_: *mut PxBVHTraversalCallback, bounds: *const PxBounds3) -> bool;

    pub fn PxBVHTraversalCallback_reportLeaf_mut(self_: *mut PxBVHTraversalCallback, nbPrims: u32, prims: *const u32) -> bool;

    /// Raycast test against a BVH.
    ///
    /// false if query has been aborted
    pub fn PxBVH_raycast(self_: *const PxBVH, origin: *const PxVec3, unitDir: *const PxVec3, maxDist: f32, cb: *mut PxBVHRaycastCallback, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Sweep test against a BVH.
    ///
    /// false if query has been aborted
    pub fn PxBVH_sweep(self_: *const PxBVH, geom: *const PxGeometry, pose: *const PxTransform, unitDir: *const PxVec3, maxDist: f32, cb: *mut PxBVHRaycastCallback, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Overlap test against a BVH.
    ///
    /// false if query has been aborted
    pub fn PxBVH_overlap(self_: *const PxBVH, geom: *const PxGeometry, pose: *const PxTransform, cb: *mut PxBVHOverlapCallback, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Frustum culling test against a BVH.
    ///
    /// This is similar in spirit to an overlap query using a convex object around the frustum.
    /// However this specialized query has better performance, and can support more than the 6 planes
    /// of a frustum, which can be useful in portal-based engines.
    ///
    /// On the other hand this test only returns a conservative number of bounds, i.e. some of the returned
    /// bounds may actually be outside the frustum volume, close to it but not touching it. This is usually
    /// an ok performance trade-off when the function is used for view-frustum culling.
    ///
    /// false if query has been aborted
    pub fn PxBVH_cull(self_: *const PxBVH, nbPlanes: u32, planes: *const PxPlane, cb: *mut PxBVHOverlapCallback, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Returns the number of bounds in the BVH.
    ///
    /// You can use [`getBounds`]() to retrieve the bounds.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    ///
    /// Number of bounds in the BVH.
    pub fn PxBVH_getNbBounds(self_: *const PxBVH) -> u32;

    /// Retrieve the read-only bounds in the BVH.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    pub fn PxBVH_getBounds(self_: *const PxBVH) -> *const PxBounds3;

    /// Retrieve the bounds in the BVH.
    ///
    /// These bounds can be modified. Call refit() after modifications are done.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    pub fn PxBVH_getBoundsForModification_mut(self_: *mut PxBVH) -> *mut PxBounds3;

    /// Refit the BVH.
    ///
    /// This function "refits" the tree, i.e. takes the new (leaf) bounding boxes into account and
    /// recomputes all the BVH bounds accordingly. This is an O(n) operation with n = number of bounds in the BVH.
    ///
    /// This works best with minor bounds modifications, i.e. when the bounds remain close to their initial values.
    /// With large modifications the tree quality degrades more and more, and subsequent query performance suffers.
    /// It might be a better strategy to create a brand new BVH if bounds change drastically.
    ///
    /// This function refits the whole tree after an arbitrary number of bounds have potentially been modified by
    /// users (via getBoundsForModification()). If you only have a small number of bounds to update, it might be
    /// more efficient to use setBounds() and partialRefit() instead.
    pub fn PxBVH_refit_mut(self_: *mut PxBVH);

    /// Update single bounds.
    ///
    /// This is an alternative to getBoundsForModification() / refit(). If you only have a small set of bounds to
    /// update, it can be inefficient to call the refit() function, because it refits the whole BVH.
    ///
    /// Instead, one can update individual bounds with this updateBounds() function. It sets the new bounds and
    /// marks the corresponding BVH nodes for partial refit. Once all the individual bounds have been updated,
    /// call partialRefit() to only refit the subset of marked nodes.
    ///
    /// true if success
    pub fn PxBVH_updateBounds_mut(self_: *mut PxBVH, boundsIndex: u32, newBounds: *const PxBounds3) -> bool;

    /// Refits subset of marked nodes.
    ///
    /// This is an alternative to the refit() function, to be called after updateBounds() calls.
    /// See updateBounds() for details.
    pub fn PxBVH_partialRefit_mut(self_: *mut PxBVH);

    /// Generic BVH traversal function.
    ///
    /// This can be used to implement custom BVH traversal functions if provided ones are not enough.
    /// In particular this can be used to visualize the tree's bounds.
    ///
    /// false if query has been aborted
    pub fn PxBVH_traverse(self_: *const PxBVH, cb: *mut PxBVHTraversalCallback) -> bool;

    pub fn PxBVH_getConcreteTypeName(self_: *const PxBVH) -> *const std::ffi::c_char;

    /// Constructor, initializes to a capsule with passed radius and half height.
    pub fn PxCapsuleGeometry_new(radius_: f32, halfHeight_: f32) -> PxCapsuleGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid.
    ///
    /// A valid capsule has radius > 0, halfHeight >= 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a capsule that has zero radius or height.
    pub fn PxCapsuleGeometry_isValid(self_: *const PxCapsuleGeometry) -> bool;

    /// Returns the number of vertices.
    ///
    /// Number of vertices.
    pub fn PxConvexMesh_getNbVertices(self_: *const PxConvexMesh) -> u32;

    /// Returns the vertices.
    ///
    /// Array of vertices.
    pub fn PxConvexMesh_getVertices(self_: *const PxConvexMesh) -> *const PxVec3;

    /// Returns the index buffer.
    ///
    /// Index buffer.
    pub fn PxConvexMesh_getIndexBuffer(self_: *const PxConvexMesh) -> *const u8;

    /// Returns the number of polygons.
    ///
    /// Number of polygons.
    pub fn PxConvexMesh_getNbPolygons(self_: *const PxConvexMesh) -> u32;

    /// Returns the polygon data.
    ///
    /// True if success.
    pub fn PxConvexMesh_getPolygonData(self_: *const PxConvexMesh, index: u32, data: *mut PxHullPolygon) -> bool;

    /// Decrements the reference count of a convex mesh and releases it if the new reference count is zero.
    pub fn PxConvexMesh_release_mut(self_: *mut PxConvexMesh);

    /// Returns the mass properties of the mesh assuming unit density.
    ///
    /// The following relationship holds between mass and volume:
    ///
    /// mass = volume * density
    ///
    /// The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.
    ///
    /// Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
    /// localInertia of the unit density mesh by d.
    pub fn PxConvexMesh_getMassInformation(self_: *const PxConvexMesh, mass: *mut f32, localInertia: *mut PxMat33, localCenterOfMass: *mut PxVec3);

    /// Returns the local-space (vertex space) AABB from the convex mesh.
    ///
    /// local-space bounds
    pub fn PxConvexMesh_getLocalBounds(self_: *const PxConvexMesh) -> PxBounds3;

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    pub fn PxConvexMesh_getSDF(self_: *const PxConvexMesh) -> *const f32;

    pub fn PxConvexMesh_getConcreteTypeName(self_: *const PxConvexMesh) -> *const std::ffi::c_char;

    /// This method decides whether a convex mesh is gpu compatible. If the total number of vertices are more than 64 or any number of vertices in a polygon is more than 32, or
    /// convex hull data was not cooked with GPU data enabled during cooking or was loaded from a serialized collection, the convex hull is incompatible with GPU collision detection. Otherwise
    /// it is compatible.
    ///
    /// True if the convex hull is gpu compatible
    pub fn PxConvexMesh_isGpuCompatible(self_: *const PxConvexMesh) -> bool;

    /// Constructor initializes to identity scale.
    pub fn PxMeshScale_new() -> PxMeshScale;

    /// Constructor from scalar.
    pub fn PxMeshScale_new_1(r: f32) -> PxMeshScale;

    /// Constructor to initialize to arbitrary scale and identity scale rotation.
    pub fn PxMeshScale_new_2(s: *const PxVec3) -> PxMeshScale;

    /// Constructor to initialize to arbitrary scaling.
    pub fn PxMeshScale_new_3(s: *const PxVec3, r: *const PxQuat) -> PxMeshScale;

    /// Returns true if the scaling is an identity transformation.
    pub fn PxMeshScale_isIdentity(self_: *const PxMeshScale) -> bool;

    /// Returns the inverse of this scaling transformation.
    pub fn PxMeshScale_getInverse(self_: *const PxMeshScale) -> PxMeshScale;

    /// Converts this transformation to a 3x3 matrix representation.
    pub fn PxMeshScale_toMat33(self_: *const PxMeshScale) -> PxMat33;

    /// Returns true if combination of negative scale components will cause the triangle normal to flip. The SDK will flip the normals internally.
    pub fn PxMeshScale_hasNegativeDeterminant(self_: *const PxMeshScale) -> bool;

    pub fn PxMeshScale_transform(self_: *const PxMeshScale, v: *const PxVec3) -> PxVec3;

    pub fn PxMeshScale_isValidForTriangleMesh(self_: *const PxMeshScale) -> bool;

    pub fn PxMeshScale_isValidForConvexMesh(self_: *const PxMeshScale) -> bool;

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    pub fn PxConvexMeshGeometry_new(mesh: *mut PxConvexMesh, scaling: *const PxMeshScale, flags: PxConvexMeshGeometryFlags) -> PxConvexMeshGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid convex mesh has a positive scale value in each direction (scale.x > 0, scale.y > 0, scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a convex that has zero extent in any direction.
    pub fn PxConvexMeshGeometry_isValid(self_: *const PxConvexMeshGeometry) -> bool;

    /// Constructor.
    pub fn PxSphereGeometry_new(ir: f32) -> PxSphereGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid sphere has radius > 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a sphere that has zero radius.
    pub fn PxSphereGeometry_isValid(self_: *const PxSphereGeometry) -> bool;

    /// Constructor.
    pub fn PxPlaneGeometry_new() -> PxPlaneGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    pub fn PxPlaneGeometry_isValid(self_: *const PxPlaneGeometry) -> bool;

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    pub fn PxTriangleMeshGeometry_new(mesh: *mut PxTriangleMesh, scaling: *const PxMeshScale, flags: PxMeshGeometryFlags) -> PxTriangleMeshGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid triangle mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a triangle mesh that has zero extents in any direction.
    pub fn PxTriangleMeshGeometry_isValid(self_: *const PxTriangleMeshGeometry) -> bool;

    /// Constructor.
    pub fn PxHeightFieldGeometry_new(hf: *mut PxHeightField, flags: PxMeshGeometryFlags, heightScale_: f32, rowScale_: f32, columnScale_: f32) -> PxHeightFieldGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid height field has a positive scale value in each direction (heightScale > 0, rowScale > 0, columnScale > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a height field that has zero extents in any direction.
    pub fn PxHeightFieldGeometry_isValid(self_: *const PxHeightFieldGeometry) -> bool;

    /// Default constructor.
    ///
    /// Creates an empty object with no particles.
    pub fn PxParticleSystemGeometry_new() -> PxParticleSystemGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    pub fn PxParticleSystemGeometry_isValid(self_: *const PxParticleSystemGeometry) -> bool;

    /// Default constructor.
    pub fn PxHairSystemGeometry_new() -> PxHairSystemGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    pub fn PxHairSystemGeometry_isValid(self_: *const PxHairSystemGeometry) -> bool;

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
    pub fn PxTetrahedronMeshGeometry_new(mesh: *mut PxTetrahedronMesh) -> PxTetrahedronMeshGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid tetrahedron mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a tetrahedron mesh that has zero extents in any direction.
    pub fn PxTetrahedronMeshGeometry_isValid(self_: *const PxTetrahedronMeshGeometry) -> bool;

    pub fn PxQueryHit_new() -> PxQueryHit;

    pub fn PxLocationHit_new() -> PxLocationHit;

    /// For raycast hits: true for shapes overlapping with raycast origin.
    ///
    /// For sweep hits: true for shapes overlapping at zero sweep distance.
    pub fn PxLocationHit_hadInitialOverlap(self_: *const PxLocationHit) -> bool;

    pub fn PxGeomRaycastHit_new() -> PxGeomRaycastHit;

    pub fn PxGeomOverlapHit_new() -> PxGeomOverlapHit;

    pub fn PxGeomSweepHit_new() -> PxGeomSweepHit;

    pub fn PxGeomIndexPair_new() -> PxGeomIndexPair;

    pub fn PxGeomIndexPair_new_1(_id0: u32, _id1: u32) -> PxGeomIndexPair;

    /// For internal use
    pub fn phys_PxCustomGeometry_getUniqueID() -> u32;

    /// Default constructor
    pub fn PxCustomGeometryType_new() -> PxCustomGeometryType;

    /// Invalid type
    pub fn PxCustomGeometryType_INVALID() -> PxCustomGeometryType;

    /// Return custom type. The type purpose is for user to differentiate custom geometries. Not used by PhysX.
    ///
    /// Unique ID of a custom geometry type.
    ///
    /// User should use DECLARE_CUSTOM_GEOMETRY_TYPE and IMPLEMENT_CUSTOM_GEOMETRY_TYPE intead of overwriting this function.
    pub fn PxCustomGeometryCallbacks_getCustomType(self_: *const PxCustomGeometryCallbacks) -> PxCustomGeometryType;

    /// Return local bounds.
    ///
    /// Bounding box in the geometry local space.
    pub fn PxCustomGeometryCallbacks_getLocalBounds(self_: *const PxCustomGeometryCallbacks, geometry: *const PxGeometry) -> PxBounds3;

    /// Raycast. Cast a ray against the geometry in given pose.
    ///
    /// Number of hits.
    pub fn PxCustomGeometryCallbacks_raycast(self_: *const PxCustomGeometryCallbacks, origin: *const PxVec3, unitDir: *const PxVec3, geom: *const PxGeometry, pose: *const PxTransform, maxDist: f32, hitFlags: PxHitFlags, maxHits: u32, rayHits: *mut PxGeomRaycastHit, stride: u32, threadContext: *mut PxQueryThreadContext) -> u32;

    /// Overlap. Test if geometries overlap.
    ///
    /// True if there is overlap. False otherwise.
    pub fn PxCustomGeometryCallbacks_overlap(self_: *const PxCustomGeometryCallbacks, geom0: *const PxGeometry, pose0: *const PxTransform, geom1: *const PxGeometry, pose1: *const PxTransform, threadContext: *mut PxQueryThreadContext) -> bool;

    /// Sweep. Sweep one geometry against the other.
    ///
    /// True if there is hit. False otherwise.
    pub fn PxCustomGeometryCallbacks_sweep(self_: *const PxCustomGeometryCallbacks, unitDir: *const PxVec3, maxDist: f32, geom0: *const PxGeometry, pose0: *const PxTransform, geom1: *const PxGeometry, pose1: *const PxTransform, sweepHit: *mut PxGeomSweepHit, hitFlags: PxHitFlags, inflation: f32, threadContext: *mut PxQueryThreadContext) -> bool;

    /// Compute custom geometry mass properties. For geometries usable with dynamic rigidbodies.
    pub fn PxCustomGeometryCallbacks_computeMassProperties(self_: *const PxCustomGeometryCallbacks, geometry: *const PxGeometry, massProperties: *mut PxMassProperties);

    /// Compatible with PhysX's PCM feature. Allows to optimize contact generation.
    pub fn PxCustomGeometryCallbacks_usePersistentContactManifold(self_: *const PxCustomGeometryCallbacks, geometry: *const PxGeometry, breakingThreshold: *mut f32) -> bool;

    pub fn PxCustomGeometryCallbacks_delete(self_: *mut PxCustomGeometryCallbacks);

    /// Default constructor.
    ///
    /// Creates an empty object with a NULL callbacks pointer.
    pub fn PxCustomGeometry_new() -> PxCustomGeometry;

    /// Constructor.
    pub fn PxCustomGeometry_new_1(_callbacks: *mut PxCustomGeometryCallbacks) -> PxCustomGeometry;

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    pub fn PxCustomGeometry_isValid(self_: *const PxCustomGeometry) -> bool;

    /// Returns the custom type of the custom geometry.
    pub fn PxCustomGeometry_getCustomType(self_: *const PxCustomGeometry) -> PxCustomGeometryType;

    pub fn PxGeometryHolder_getType(self_: *const PxGeometryHolder) -> PxGeometryType;

    pub fn PxGeometryHolder_any_mut(self_: *mut PxGeometryHolder) -> *mut PxGeometry;

    pub fn PxGeometryHolder_any(self_: *const PxGeometryHolder) -> *const PxGeometry;

    pub fn PxGeometryHolder_sphere_mut(self_: *mut PxGeometryHolder) -> *mut PxSphereGeometry;

    pub fn PxGeometryHolder_sphere(self_: *const PxGeometryHolder) -> *const PxSphereGeometry;

    pub fn PxGeometryHolder_plane_mut(self_: *mut PxGeometryHolder) -> *mut PxPlaneGeometry;

    pub fn PxGeometryHolder_plane(self_: *const PxGeometryHolder) -> *const PxPlaneGeometry;

    pub fn PxGeometryHolder_capsule_mut(self_: *mut PxGeometryHolder) -> *mut PxCapsuleGeometry;

    pub fn PxGeometryHolder_capsule(self_: *const PxGeometryHolder) -> *const PxCapsuleGeometry;

    pub fn PxGeometryHolder_box_mut(self_: *mut PxGeometryHolder) -> *mut PxBoxGeometry;

    pub fn PxGeometryHolder_box(self_: *const PxGeometryHolder) -> *const PxBoxGeometry;

    pub fn PxGeometryHolder_convexMesh_mut(self_: *mut PxGeometryHolder) -> *mut PxConvexMeshGeometry;

    pub fn PxGeometryHolder_convexMesh(self_: *const PxGeometryHolder) -> *const PxConvexMeshGeometry;

    pub fn PxGeometryHolder_tetMesh_mut(self_: *mut PxGeometryHolder) -> *mut PxTetrahedronMeshGeometry;

    pub fn PxGeometryHolder_tetMesh(self_: *const PxGeometryHolder) -> *const PxTetrahedronMeshGeometry;

    pub fn PxGeometryHolder_triangleMesh_mut(self_: *mut PxGeometryHolder) -> *mut PxTriangleMeshGeometry;

    pub fn PxGeometryHolder_triangleMesh(self_: *const PxGeometryHolder) -> *const PxTriangleMeshGeometry;

    pub fn PxGeometryHolder_heightField_mut(self_: *mut PxGeometryHolder) -> *mut PxHeightFieldGeometry;

    pub fn PxGeometryHolder_heightField(self_: *const PxGeometryHolder) -> *const PxHeightFieldGeometry;

    pub fn PxGeometryHolder_particleSystem_mut(self_: *mut PxGeometryHolder) -> *mut PxParticleSystemGeometry;

    pub fn PxGeometryHolder_particleSystem(self_: *const PxGeometryHolder) -> *const PxParticleSystemGeometry;

    pub fn PxGeometryHolder_hairSystem_mut(self_: *mut PxGeometryHolder) -> *mut PxHairSystemGeometry;

    pub fn PxGeometryHolder_hairSystem(self_: *const PxGeometryHolder) -> *const PxHairSystemGeometry;

    pub fn PxGeometryHolder_custom_mut(self_: *mut PxGeometryHolder) -> *mut PxCustomGeometry;

    pub fn PxGeometryHolder_custom(self_: *const PxGeometryHolder) -> *const PxCustomGeometry;

    pub fn PxGeometryHolder_storeAny_mut(self_: *mut PxGeometryHolder, geometry: *const PxGeometry);

    pub fn PxGeometryHolder_new() -> PxGeometryHolder;

    pub fn PxGeometryHolder_new_1(geometry: *const PxGeometry) -> PxGeometryHolder;

    /// Raycast test against a geometry object.
    ///
    /// All geometry types are supported except PxParticleSystemGeometry, PxTetrahedronMeshGeometry and PxHairSystemGeometry.
    ///
    /// Number of hits between the ray and the geometry object
    pub fn PxGeometryQuery_raycast(origin: *const PxVec3, unitDir: *const PxVec3, geom: *const PxGeometry, pose: *const PxTransform, maxDist: f32, hitFlags: PxHitFlags, maxHits: u32, rayHits: *mut PxGeomRaycastHit, stride: u32, queryFlags: PxGeometryQueryFlags, threadContext: *mut PxQueryThreadContext) -> u32;

    /// Overlap test for two geometry objects.
    ///
    /// All combinations are supported except:
    ///
    /// PxPlaneGeometry vs. {PxPlaneGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxTriangleMeshGeometry vs. PxHeightFieldGeometry
    ///
    /// PxHeightFieldGeometry vs. PxHeightFieldGeometry
    ///
    /// Anything involving PxParticleSystemGeometry, PxTetrahedronMeshGeometry or PxHairSystemGeometry.
    ///
    /// True if the two geometry objects overlap
    pub fn PxGeometryQuery_overlap(geom0: *const PxGeometry, pose0: *const PxTransform, geom1: *const PxGeometry, pose1: *const PxTransform, queryFlags: PxGeometryQueryFlags, threadContext: *mut PxQueryThreadContext) -> bool;

    /// Sweep a specified geometry object in space and test for collision with a given object.
    ///
    /// The following combinations are supported.
    ///
    /// PxSphereGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxCapsuleGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxBoxGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxConvexMeshGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// True if the swept geometry object geom0 hits the object geom1
    pub fn PxGeometryQuery_sweep(unitDir: *const PxVec3, maxDist: f32, geom0: *const PxGeometry, pose0: *const PxTransform, geom1: *const PxGeometry, pose1: *const PxTransform, sweepHit: *mut PxGeomSweepHit, hitFlags: PxHitFlags, inflation: f32, queryFlags: PxGeometryQueryFlags, threadContext: *mut PxQueryThreadContext) -> bool;

    /// Compute minimum translational distance (MTD) between two geometry objects.
    ///
    /// All combinations of geom objects are supported except:
    /// - plane/plane
    /// - plane/mesh
    /// - plane/heightfield
    /// - mesh/mesh
    /// - mesh/heightfield
    /// - heightfield/heightfield
    /// - anything involving PxParticleSystemGeometry, PxTetrahedronMeshGeometry or PxHairSystemGeometry
    ///
    /// The function returns a unit vector ('direction') and a penetration depth ('depth').
    ///
    /// The depenetration vector D = direction * depth should be applied to the first object, to
    /// get out of the second object.
    ///
    /// Returned depth should always be positive or null.
    ///
    /// If objects do not overlap, the function can not compute the MTD and returns false.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
    pub fn PxGeometryQuery_computePenetration(direction: *mut PxVec3, depth: *mut f32, geom0: *const PxGeometry, pose0: *const PxTransform, geom1: *const PxGeometry, pose1: *const PxTransform, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Computes distance between a point and a geometry object.
    ///
    /// Currently supported geometry objects: box, sphere, capsule, convex, mesh.
    ///
    /// For meshes, only the BVH34 midphase data-structure is supported.
    ///
    /// Square distance between the point and the geom object, or 0.0 if the point is inside the object, or -1.0 if an error occured (geometry type is not supported, or invalid pose)
    pub fn PxGeometryQuery_pointDistance(point: *const PxVec3, geom: *const PxGeometry, pose: *const PxTransform, closestPoint: *mut PxVec3, closestIndex: *mut u32, queryFlags: PxGeometryQueryFlags) -> f32;

    /// computes the bounds for a geometry object
    pub fn PxGeometryQuery_computeGeomBounds(bounds: *mut PxBounds3, geom: *const PxGeometry, pose: *const PxTransform, offset: f32, inflation: f32, queryFlags: PxGeometryQueryFlags);

    /// Checks if provided geometry is valid.
    ///
    /// True if geometry is valid.
    pub fn PxGeometryQuery_isValid(geom: *const PxGeometry) -> bool;

    pub fn PxHeightFieldSample_tessFlag(self_: *const PxHeightFieldSample) -> u8;

    pub fn PxHeightFieldSample_setTessFlag_mut(self_: *mut PxHeightFieldSample);

    pub fn PxHeightFieldSample_clearTessFlag_mut(self_: *mut PxHeightFieldSample);

    /// Decrements the reference count of a height field and releases it if the new reference count is zero.
    pub fn PxHeightField_release_mut(self_: *mut PxHeightField);

    /// Writes out the sample data array.
    ///
    /// The user provides destBufferSize bytes storage at destBuffer.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// The number of bytes written.
    pub fn PxHeightField_saveCells(self_: *const PxHeightField, destBuffer: *mut std::ffi::c_void, destBufferSize: u32) -> u32;

    /// Replaces a rectangular subfield in the sample data array.
    ///
    /// The user provides the description of a rectangular subfield in subfieldDesc.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// True on success, false on failure. Failure can occur due to format mismatch.
    ///
    /// Modified samples are constrained to the same height quantization range as the original heightfield.
    /// Source samples that are out of range of target heightfield will be clipped with no error.
    /// PhysX does not keep a mapping from the heightfield to heightfield shapes that reference it.
    /// Call PxShape::setGeometry on each shape which references the height field, to ensure that internal data structures are updated to reflect the new geometry.
    /// Please note that PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    pub fn PxHeightField_modifySamples_mut(self_: *mut PxHeightField, startCol: i32, startRow: i32, subfieldDesc: *const PxHeightFieldDesc, shrinkBounds: bool) -> bool;

    /// Retrieves the number of sample rows in the samples array.
    ///
    /// The number of sample rows in the samples array.
    pub fn PxHeightField_getNbRows(self_: *const PxHeightField) -> u32;

    /// Retrieves the number of sample columns in the samples array.
    ///
    /// The number of sample columns in the samples array.
    pub fn PxHeightField_getNbColumns(self_: *const PxHeightField) -> u32;

    /// Retrieves the format of the sample data.
    ///
    /// The format of the sample data.
    pub fn PxHeightField_getFormat(self_: *const PxHeightField) -> PxHeightFieldFormat;

    /// Retrieves the offset in bytes between consecutive samples in the array.
    ///
    /// The offset in bytes between consecutive samples in the array.
    pub fn PxHeightField_getSampleStride(self_: *const PxHeightField) -> u32;

    /// Retrieves the convex edge threshold.
    ///
    /// The convex edge threshold.
    pub fn PxHeightField_getConvexEdgeThreshold(self_: *const PxHeightField) -> f32;

    /// Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.
    ///
    /// The flags bits, combined from values of the enum ::PxHeightFieldFlag.
    pub fn PxHeightField_getFlags(self_: *const PxHeightField) -> PxHeightFieldFlags;

    /// Retrieves the height at the given coordinates in grid space.
    ///
    /// The height at the given coordinates or 0 if the coordinates are out of range.
    pub fn PxHeightField_getHeight(self_: *const PxHeightField, x: f32, z: f32) -> f32;

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    pub fn PxHeightField_getTriangleMaterialIndex(self_: *const PxHeightField, triangleIndex: u32) -> u16;

    /// Returns a triangle face normal for a given triangle index
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Triangle normal for a given triangle index
    pub fn PxHeightField_getTriangleNormal(self_: *const PxHeightField, triangleIndex: u32) -> PxVec3;

    /// Returns heightfield sample of given row and column
    ///
    /// Heightfield sample
    pub fn PxHeightField_getSample(self_: *const PxHeightField, row: u32, column: u32) -> *const PxHeightFieldSample;

    /// Returns the number of times the heightfield data has been modified
    ///
    /// This method returns the number of times modifySamples has been called on this heightfield, so that code that has
    /// retained state that depends on the heightfield can efficiently determine whether it has been modified.
    ///
    /// the number of times the heightfield sample data has been modified.
    pub fn PxHeightField_getTimestamp(self_: *const PxHeightField) -> u32;

    pub fn PxHeightField_getConcreteTypeName(self_: *const PxHeightField) -> *const std::ffi::c_char;

    /// Constructor sets to default.
    pub fn PxHeightFieldDesc_new() -> PxHeightFieldDesc;

    /// (re)sets the structure to the default.
    pub fn PxHeightFieldDesc_setToDefault_mut(self_: *mut PxHeightFieldDesc);

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid.
    pub fn PxHeightFieldDesc_isValid(self_: *const PxHeightFieldDesc) -> bool;

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapTriangleMesh`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    pub fn PxMeshQuery_getTriangle(triGeom: *const PxTriangleMeshGeometry, transform: *const PxTransform, triangleIndex: u32, triangle: *mut PxTriangle, vertexIndices: *mut u32, adjacencyIndices: *mut u32);

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapHeightField`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    ///
    /// TriangleIndex is an index used in internal format, which does have an index out of the bounds in last row.
    /// To traverse all tri indices in the HF, the following code can be applied:
    /// for (PxU32 row = 0; row
    /// <
    /// (nbRows - 1); row++)
    /// {
    /// for (PxU32 col = 0; col
    /// <
    /// (nbCols - 1); col++)
    /// {
    /// for (PxU32 k = 0; k
    /// <
    /// 2; k++)
    /// {
    /// const PxU32 triIndex = 2 * (row*nbCols + col) + k;
    /// ....
    /// }
    /// }
    /// }
    pub fn PxMeshQuery_getTriangle_1(hfGeom: *const PxHeightFieldGeometry, transform: *const PxTransform, triangleIndex: u32, triangle: *mut PxTriangle, vertexIndices: *mut u32, adjacencyIndices: *mut u32);

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// For mesh-vs-mesh overlap tests, please use the specialized function below.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    pub fn PxMeshQuery_findOverlapTriangleMesh(geom: *const PxGeometry, geomPose: *const PxTransform, meshGeom: *const PxTriangleMeshGeometry, meshPose: *const PxTransform, results: *mut u32, maxResults: u32, startIndex: u32, overflow: *mut bool, queryFlags: PxGeometryQueryFlags) -> u32;

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
    pub fn PxMeshQuery_findOverlapHeightField(geom: *const PxGeometry, geomPose: *const PxTransform, hfGeom: *const PxHeightFieldGeometry, hfPose: *const PxTransform, results: *mut u32, maxResults: u32, startIndex: u32, overflow: *mut bool, queryFlags: PxGeometryQueryFlags) -> u32;

    /// Sweep a specified geometry object in space and test for collision with a set of given triangles.
    ///
    /// This function simply sweeps input geometry against each input triangle, in the order they are given.
    /// This is an O(N) operation with N = number of input triangles. It does not use any particular acceleration structure.
    ///
    /// True if the swept geometry object hits the specified triangles
    ///
    /// Only the following geometry types are currently supported: PxSphereGeometry, PxCapsuleGeometry, PxBoxGeometry
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// This function returns a single closest hit across all the input triangles. Multiple hits are not supported.
    ///
    /// Supported hitFlags are PxHitFlag::eDEFAULT, PxHitFlag::eASSUME_NO_INITIAL_OVERLAP, PxHitFlag::ePRECISE_SWEEP, PxHitFlag::eMESH_BOTH_SIDES, PxHitFlag::eMESH_ANY.
    ///
    /// ePOSITION is only defined when there is no initial overlap (sweepHit.hadInitialOverlap() == false)
    ///
    /// The returned normal for initially overlapping sweeps is set to -unitDir.
    ///
    /// Otherwise the returned normal is the front normal of the triangle even if PxHitFlag::eMESH_BOTH_SIDES is set.
    ///
    /// The returned PxGeomSweepHit::faceIndex parameter will hold the index of the hit triangle in input array, i.e. the range is [0; triangleCount). For initially overlapping sweeps, this is the index of overlapping triangle.
    ///
    /// The inflation parameter is not compatible with PxHitFlag::ePRECISE_SWEEP.
    pub fn PxMeshQuery_sweep(unitDir: *const PxVec3, distance: f32, geom: *const PxGeometry, pose: *const PxTransform, triangleCount: u32, triangles: *const PxTriangle, sweepHit: *mut PxGeomSweepHit, hitFlags: PxHitFlags, cachedIndex: *const u32, inflation: f32, doubleSided: bool, queryFlags: PxGeometryQueryFlags) -> bool;

    /// constructor sets to default.
    pub fn PxSimpleTriangleMesh_new() -> PxSimpleTriangleMesh;

    /// (re)sets the structure to the default.
    pub fn PxSimpleTriangleMesh_setToDefault_mut(self_: *mut PxSimpleTriangleMesh);

    /// returns true if the current settings are valid
    pub fn PxSimpleTriangleMesh_isValid(self_: *const PxSimpleTriangleMesh) -> bool;

    /// Constructor
    pub fn PxTriangle_new_alloc() -> *mut PxTriangle;

    /// Constructor
    pub fn PxTriangle_new_alloc_1(p0: *const PxVec3, p1: *const PxVec3, p2: *const PxVec3) -> *mut PxTriangle;

    /// Destructor
    pub fn PxTriangle_delete(self_: *mut PxTriangle);

    /// Compute the normal of the Triangle.
    pub fn PxTriangle_normal(self_: *const PxTriangle, _normal: *mut PxVec3);

    /// Compute the unnormalized normal of the triangle.
    pub fn PxTriangle_denormalizedNormal(self_: *const PxTriangle, _normal: *mut PxVec3);

    /// Compute the area of the triangle.
    ///
    /// Area of the triangle.
    pub fn PxTriangle_area(self_: *const PxTriangle) -> f32;

    /// Computes a point on the triangle from u and v barycentric coordinates.
    pub fn PxTriangle_pointFromUV(self_: *const PxTriangle, u: f32, v: f32) -> PxVec3;

    pub fn PxTrianglePadded_new_alloc() -> *mut PxTrianglePadded;

    pub fn PxTrianglePadded_delete(self_: *mut PxTrianglePadded);

    /// Returns the number of vertices.
    ///
    /// number of vertices
    pub fn PxTriangleMesh_getNbVertices(self_: *const PxTriangleMesh) -> u32;

    /// Returns the vertices.
    ///
    /// array of vertices
    pub fn PxTriangleMesh_getVertices(self_: *const PxTriangleMesh) -> *const PxVec3;

    /// Returns all mesh vertices for modification.
    ///
    /// This function will return the vertices of the mesh so that their positions can be changed in place.
    /// After modifying the vertices you must call refitBVH for the refitting to actually take place.
    /// This function maintains the old mesh topology (triangle indices).
    ///
    /// inplace vertex coordinates for each existing mesh vertex.
    ///
    /// It is recommended to use this feature for scene queries only.
    ///
    /// Size of array returned is equal to the number returned by getNbVertices().
    ///
    /// This function operates on cooked vertex indices.
    ///
    /// This means the index mapping and vertex count can be different from what was provided as an input to the cooking routine.
    ///
    /// To achieve unchanged 1-to-1 index mapping with orignal mesh data (before cooking) please use the following cooking flags:
    ///
    /// eWELD_VERTICES = 0, eDISABLE_CLEAN_MESH = 1.
    ///
    /// It is also recommended to make sure that a call to validateTriangleMesh returns true if mesh cleaning is disabled.
    pub fn PxTriangleMesh_getVerticesForModification_mut(self_: *mut PxTriangleMesh) -> *mut PxVec3;

    /// Refits BVH for mesh vertices.
    ///
    /// This function will refit the mesh BVH to correctly enclose the new positions updated by getVerticesForModification.
    /// Mesh BVH will not be reoptimized by this function so significantly different new positions will cause significantly reduced performance.
    ///
    /// New bounds for the entire mesh.
    ///
    /// For PxMeshMidPhase::eBVH34 trees the refit operation is only available on non-quantized trees (see PxBVH34MidphaseDesc::quantized)
    ///
    /// PhysX does not keep a mapping from the mesh to mesh shapes that reference it.
    ///
    /// Call PxShape::setGeometry on each shape which references the mesh, to ensure that internal data structures are updated to reflect the new geometry.
    ///
    /// PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    ///
    /// It is also recommended to make sure that a call to validateTriangleMesh returns true if mesh cleaning is disabled.
    ///
    /// Active edges information will be lost during refit, the rigid body mesh contact generation might not perform as expected.
    pub fn PxTriangleMesh_refitBVH_mut(self_: *mut PxTriangleMesh) -> PxBounds3;

    /// Returns the number of triangles.
    ///
    /// number of triangles
    pub fn PxTriangleMesh_getNbTriangles(self_: *const PxTriangleMesh) -> u32;

    /// Returns the triangle indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of triangles in the mesh.
    /// Call getTriangleMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of triangles * 3.
    ///
    /// array of triangles
    pub fn PxTriangleMesh_getTriangles(self_: *const PxTriangleMesh) -> *const std::ffi::c_void;

    /// Reads the PxTriangleMesh flags.
    ///
    /// See the list of flags [`PxTriangleMeshFlag`]
    ///
    /// The values of the PxTriangleMesh flags.
    pub fn PxTriangleMesh_getTriangleMeshFlags(self_: *const PxTriangleMesh) -> PxTriangleMeshFlags;

    /// Returns the triangle remapping table.
    ///
    /// The triangles are internally sorted according to various criteria. Hence the internal triangle order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTriangleIndex ] = originalTriangleIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
    pub fn PxTriangleMesh_getTrianglesRemap(self_: *const PxTriangleMesh) -> *const u32;

    /// Decrements the reference count of a triangle mesh and releases it if the new reference count is zero.
    pub fn PxTriangleMesh_release_mut(self_: *mut PxTriangleMesh);

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
    pub fn PxTriangleMesh_getTriangleMaterialIndex(self_: *const PxTriangleMesh, triangleIndex: u32) -> u16;

    /// Returns the local-space (vertex space) AABB from the triangle mesh.
    ///
    /// local-space bounds
    pub fn PxTriangleMesh_getLocalBounds(self_: *const PxTriangleMesh) -> PxBounds3;

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
    pub fn PxTriangleMesh_getSDF(self_: *const PxTriangleMesh) -> *const f32;

    /// Returns the resolution of the local-space dense SDF.
    pub fn PxTriangleMesh_getSDFDimensions(self_: *const PxTriangleMesh, numX: *mut u32, numY: *mut u32, numZ: *mut u32);

    /// Sets whether this mesh should be preferred for SDF projection.
    ///
    /// By default, meshes are flagged as preferring projection and the decisions on which mesh to project is based on the triangle and vertex
    /// count. The model with the fewer triangles is projected onto the SDF of the more detailed mesh.
    /// If one of the meshes is set to prefer SDF projection (default) and the other is set to not prefer SDF projection, model flagged as
    /// preferring SDF projection will be projected onto the model flagged as not preferring, regardless of the detail of the respective meshes.
    /// Where both models are flagged as preferring no projection, the less detailed model will be projected as before.
    pub fn PxTriangleMesh_setPreferSDFProjection_mut(self_: *mut PxTriangleMesh, preferProjection: bool);

    /// Returns whether this mesh prefers SDF projection.
    ///
    /// whether this mesh prefers SDF projection.
    pub fn PxTriangleMesh_getPreferSDFProjection(self_: *const PxTriangleMesh) -> bool;

    /// Returns the mass properties of the mesh assuming unit density.
    ///
    /// The following relationship holds between mass and volume:
    ///
    /// mass = volume * density
    ///
    /// The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.
    ///
    /// Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
    /// localInertia of the unit density mesh by d.
    pub fn PxTriangleMesh_getMassInformation(self_: *const PxTriangleMesh, mass: *mut f32, localInertia: *mut PxMat33, localCenterOfMass: *mut PxVec3);

    /// Constructor
    pub fn PxTetrahedron_new_alloc() -> *mut PxTetrahedron;

    /// Constructor
    pub fn PxTetrahedron_new_alloc_1(p0: *const PxVec3, p1: *const PxVec3, p2: *const PxVec3, p3: *const PxVec3) -> *mut PxTetrahedron;

    /// Destructor
    pub fn PxTetrahedron_delete(self_: *mut PxTetrahedron);

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    pub fn PxSoftBodyAuxData_release_mut(self_: *mut PxSoftBodyAuxData);

    /// Returns the number of vertices.
    ///
    /// number of vertices
    pub fn PxTetrahedronMesh_getNbVertices(self_: *const PxTetrahedronMesh) -> u32;

    /// Returns the vertices
    ///
    /// array of vertices
    pub fn PxTetrahedronMesh_getVertices(self_: *const PxTetrahedronMesh) -> *const PxVec3;

    /// Returns the number of tetrahedrons.
    ///
    /// number of tetrahedrons
    pub fn PxTetrahedronMesh_getNbTetrahedrons(self_: *const PxTetrahedronMesh) -> u32;

    /// Returns the tetrahedron indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of tetrahedrons in the mesh.
    /// Call getTetrahedronMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of tetrahedrons * 4.
    ///
    /// array of tetrahedrons
    pub fn PxTetrahedronMesh_getTetrahedrons(self_: *const PxTetrahedronMesh) -> *const std::ffi::c_void;

    /// Reads the PxTetrahedronMesh flags.
    ///
    /// See the list of flags [`PxTetrahedronMeshFlags`]
    ///
    /// The values of the PxTetrahedronMesh flags.
    pub fn PxTetrahedronMesh_getTetrahedronMeshFlags(self_: *const PxTetrahedronMesh) -> PxTetrahedronMeshFlags;

    /// Returns the tetrahedra remapping table.
    ///
    /// The tetrahedra are internally sorted according to various criteria. Hence the internal tetrahedron order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTetrahedronIndex ] = originalTetrahedronIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
    pub fn PxTetrahedronMesh_getTetrahedraRemap(self_: *const PxTetrahedronMesh) -> *const u32;

    /// Returns the local-space (vertex space) AABB from the tetrahedron mesh.
    ///
    /// local-space bounds
    pub fn PxTetrahedronMesh_getLocalBounds(self_: *const PxTetrahedronMesh) -> PxBounds3;

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    pub fn PxTetrahedronMesh_release_mut(self_: *mut PxTetrahedronMesh);

    /// Const accecssor to the softbody's collision mesh.
    pub fn PxSoftBodyMesh_getCollisionMesh(self_: *const PxSoftBodyMesh) -> *const PxTetrahedronMesh;

    /// Accecssor to the softbody's collision mesh.
    pub fn PxSoftBodyMesh_getCollisionMesh_mut(self_: *mut PxSoftBodyMesh) -> *mut PxTetrahedronMesh;

    /// Const accessor to the softbody's simulation mesh.
    pub fn PxSoftBodyMesh_getSimulationMesh(self_: *const PxSoftBodyMesh) -> *const PxTetrahedronMesh;

    /// Accecssor to the softbody's simulation mesh.
    pub fn PxSoftBodyMesh_getSimulationMesh_mut(self_: *mut PxSoftBodyMesh) -> *mut PxTetrahedronMesh;

    /// Const accessor to the softbodies simulation state.
    pub fn PxSoftBodyMesh_getSoftBodyAuxData(self_: *const PxSoftBodyMesh) -> *const PxSoftBodyAuxData;

    /// Accessor to the softbody's auxilary data like mass and rest pose information
    pub fn PxSoftBodyMesh_getSoftBodyAuxData_mut(self_: *mut PxSoftBodyMesh) -> *mut PxSoftBodyAuxData;

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
    pub fn PxSoftBodyMesh_release_mut(self_: *mut PxSoftBodyMesh);

    pub fn PxCollisionMeshMappingData_release_mut(self_: *mut PxCollisionMeshMappingData);

    pub fn PxCollisionTetrahedronMeshData_getMesh(self_: *const PxCollisionTetrahedronMeshData) -> *const PxTetrahedronMeshData;

    pub fn PxCollisionTetrahedronMeshData_getMesh_mut(self_: *mut PxCollisionTetrahedronMeshData) -> *mut PxTetrahedronMeshData;

    pub fn PxCollisionTetrahedronMeshData_getData(self_: *const PxCollisionTetrahedronMeshData) -> *const PxSoftBodyCollisionData;

    pub fn PxCollisionTetrahedronMeshData_getData_mut(self_: *mut PxCollisionTetrahedronMeshData) -> *mut PxSoftBodyCollisionData;

    pub fn PxCollisionTetrahedronMeshData_release_mut(self_: *mut PxCollisionTetrahedronMeshData);

    pub fn PxSimulationTetrahedronMeshData_getMesh_mut(self_: *mut PxSimulationTetrahedronMeshData) -> *mut PxTetrahedronMeshData;

    pub fn PxSimulationTetrahedronMeshData_getData_mut(self_: *mut PxSimulationTetrahedronMeshData) -> *mut PxSoftBodySimulationData;

    pub fn PxSimulationTetrahedronMeshData_release_mut(self_: *mut PxSimulationTetrahedronMeshData);

    /// Deletes the actor.
    ///
    /// Do not keep a reference to the deleted instance.
    ///
    /// If the actor belongs to a [`PxAggregate`] object, it is automatically removed from the aggregate.
    pub fn PxActor_release_mut(self_: *mut PxActor);

    /// Retrieves the type of actor.
    ///
    /// The actor type of the actor.
    pub fn PxActor_getType(self_: *const PxActor) -> PxActorType;

    /// Retrieves the scene which this actor belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    pub fn PxActor_getScene(self_: *const PxActor) -> *mut PxScene;

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    ///
    /// Default:
    /// NULL
    pub fn PxActor_setName_mut(self_: *mut PxActor, name: *const std::ffi::c_char);

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    pub fn PxActor_getName(self_: *const PxActor) -> *const std::ffi::c_char;

    /// Retrieves the axis aligned bounding box enclosing the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The actor's bounding box.
    pub fn PxActor_getWorldBounds(self_: *const PxActor, inflation: f32) -> PxBounds3;

    /// Raises or clears a particular actor flag.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxActor_setActorFlag_mut(self_: *mut PxActor, flag: PxActorFlag, value: bool);

    /// Sets the actor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    pub fn PxActor_setActorFlags_mut(self_: *mut PxActor, inFlags: PxActorFlags);

    /// Reads the PxActor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// The values of the PxActor flags.
    pub fn PxActor_getActorFlags(self_: *const PxActor) -> PxActorFlags;

    /// Assigns dynamic actors a dominance group identifier.
    ///
    /// PxDominanceGroup is a 5 bit group identifier (legal range from 0 to 31).
    ///
    /// The PxScene::setDominanceGroupPair() lets you set certain behaviors for pairs of dominance groups.
    /// By default every dynamic actor is created in group 0.
    ///
    /// Default:
    /// 0
    ///
    /// Sleeping:
    /// Changing the dominance group does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxActor_setDominanceGroup_mut(self_: *mut PxActor, dominanceGroup: u8);

    /// Retrieves the value set with setDominanceGroup().
    ///
    /// The dominance group of this actor.
    pub fn PxActor_getDominanceGroup(self_: *const PxActor) -> u8;

    /// Sets the owner client of an actor.
    ///
    /// This cannot be done once the actor has been placed into a scene.
    ///
    /// Default:
    /// PX_DEFAULT_CLIENT
    pub fn PxActor_setOwnerClient_mut(self_: *mut PxActor, inClient: u8);

    /// Returns the owner client that was specified at creation time.
    ///
    /// This value cannot be changed once the object is placed into the scene.
    pub fn PxActor_getOwnerClient(self_: *const PxActor) -> u8;

    /// Retrieves the aggregate the actor might be a part of.
    ///
    /// The aggregate the actor is a part of, or NULL if the actor does not belong to an aggregate.
    pub fn PxActor_getAggregate(self_: *const PxActor) -> *mut PxAggregate;

    pub fn phys_PxGetAggregateFilterHint(type_: PxAggregateType, enableSelfCollision: bool) -> u32;

    pub fn phys_PxGetAggregateSelfCollisionBit(hint: u32) -> u32;

    pub fn phys_PxGetAggregateType(hint: u32) -> PxAggregateType;

    /// Deletes the aggregate object.
    ///
    /// Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
    /// belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
    /// to delete both the PxAggregate and its actors, it is best to release the actors first, then release
    /// the PxAggregate when it is empty.
    pub fn PxAggregate_release_mut(self_: *mut PxAggregate);

    /// Adds an actor to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached, or if the incoming actor already belongs
    /// to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an actor to the aggregate also adds the actor to that scene.
    ///
    /// If the actor already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the actor from the scene first, before adding it to the aggregate.
    ///
    /// When a BVH is provided the actor shapes are grouped together.
    /// The scene query pruning structure inside PhysX SDK will store/update one
    /// bound per actor. The scene queries against such an actor will query actor
    /// bounds and then make a local space query against the provided BVH, which is in actor's local space.
    pub fn PxAggregate_addActor_mut(self_: *mut PxAggregate, actor: *mut PxActor, bvh: *const PxBVH) -> bool;

    /// Removes an actor from the aggregate object.
    ///
    /// A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
    /// removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
    /// scene. If you intend to delete the actor, it is best to call [`PxActor::release`]() directly. That way
    /// the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    pub fn PxAggregate_removeActor_mut(self_: *mut PxAggregate, actor: *mut PxActor) -> bool;

    /// Adds an articulation to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached (every articulation link counts as an actor),
    /// or if the incoming articulation already belongs to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an articulation to the aggregate also adds the articulation to that scene.
    ///
    /// If the articulation already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the articulation from the scene first, before adding it to the aggregate.
    pub fn PxAggregate_addArticulation_mut(self_: *mut PxAggregate, articulation: *mut PxArticulationReducedCoordinate) -> bool;

    /// Removes an articulation from the aggregate object.
    ///
    /// A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
    /// removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
    /// scene. If you intend to delete the articulation, it is best to call [`PxArticulationReducedCoordinate::release`]() directly. That way
    /// the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.
    pub fn PxAggregate_removeArticulation_mut(self_: *mut PxAggregate, articulation: *mut PxArticulationReducedCoordinate) -> bool;

    /// Returns the number of actors contained in the aggregate.
    ///
    /// You can use [`getActors`]() to retrieve the actor pointers.
    ///
    /// Number of actors contained in the aggregate.
    pub fn PxAggregate_getNbActors(self_: *const PxAggregate) -> u32;

    /// Retrieves max amount of shapes that can be contained in the aggregate.
    ///
    /// Max shape size.
    pub fn PxAggregate_getMaxNbShapes(self_: *const PxAggregate) -> u32;

    /// Retrieve all actors contained in the aggregate.
    ///
    /// You can retrieve the number of actor pointers by calling [`getNbActors`]()
    ///
    /// Number of actor pointers written to the buffer.
    pub fn PxAggregate_getActors(self_: *const PxAggregate, userBuffer: *mut *mut PxActor, bufferSize: u32, startIndex: u32) -> u32;

    /// Retrieves the scene which this aggregate belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    pub fn PxAggregate_getScene_mut(self_: *mut PxAggregate) -> *mut PxScene;

    /// Retrieves aggregate's self-collision flag.
    ///
    /// self-collision flag
    pub fn PxAggregate_getSelfCollision(self_: *const PxAggregate) -> bool;

    pub fn PxAggregate_getConcreteTypeName(self_: *const PxAggregate) -> *const std::ffi::c_char;

    pub fn PxConstraintInvMassScale_new() -> PxConstraintInvMassScale;

    pub fn PxConstraintInvMassScale_new_1(lin0: f32, ang0: f32, lin1: f32, ang1: f32) -> PxConstraintInvMassScale;

    /// Visualize joint frames
    pub fn PxConstraintVisualizer_visualizeJointFrames_mut(self_: *mut PxConstraintVisualizer, parent: *const PxTransform, child: *const PxTransform);

    /// Visualize joint linear limit
    pub fn PxConstraintVisualizer_visualizeLinearLimit_mut(self_: *mut PxConstraintVisualizer, t0: *const PxTransform, t1: *const PxTransform, value: f32, active: bool);

    /// Visualize joint angular limit
    pub fn PxConstraintVisualizer_visualizeAngularLimit_mut(self_: *mut PxConstraintVisualizer, t0: *const PxTransform, lower: f32, upper: f32, active: bool);

    /// Visualize limit cone
    pub fn PxConstraintVisualizer_visualizeLimitCone_mut(self_: *mut PxConstraintVisualizer, t: *const PxTransform, tanQSwingY: f32, tanQSwingZ: f32, active: bool);

    /// Visualize joint double cone
    pub fn PxConstraintVisualizer_visualizeDoubleCone_mut(self_: *mut PxConstraintVisualizer, t: *const PxTransform, angle: f32, active: bool);

    /// Visualize line
    pub fn PxConstraintVisualizer_visualizeLine_mut(self_: *mut PxConstraintVisualizer, p0: *const PxVec3, p1: *const PxVec3, color: u32);

    /// Pre-simulation data preparation
    /// when the constraint is marked dirty, this function is called at the start of the simulation
    /// step for the SDK to copy the constraint data block.
    pub fn PxConstraintConnector_prepareData_mut(self_: *mut PxConstraintConnector) -> *mut std::ffi::c_void;

    /// Constraint release callback
    ///
    /// When the SDK deletes a PxConstraint object this function is called by the SDK. In general
    /// custom constraints should not be deleted directly by applications: rather, the constraint
    /// should respond to a release() request by calling PxConstraint::release(), then wait for
    /// this call to release its own resources.
    ///
    /// This function is also called when a PxConstraint object is deleted on cleanup due to
    /// destruction of the PxPhysics object.
    pub fn PxConstraintConnector_onConstraintRelease_mut(self_: *mut PxConstraintConnector);

    /// Center-of-mass shift callback
    ///
    /// This function is called by the SDK when the CoM of one of the actors is moved. Since the
    /// API specifies constraint positions relative to actors, and the constraint shader functions
    /// are supplied with coordinates relative to bodies, some synchronization is usually required
    /// when the application moves an object's center of mass.
    pub fn PxConstraintConnector_onComShift_mut(self_: *mut PxConstraintConnector, actor: u32);

    /// Origin shift callback
    ///
    /// This function is called by the SDK when the scene origin gets shifted and allows to adjust
    /// custom data which contains world space transforms.
    ///
    /// If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
    /// to make sure that the data gets synced at the beginning of the next simulation step.
    pub fn PxConstraintConnector_onOriginShift_mut(self_: *mut PxConstraintConnector, shift: *const PxVec3);

    /// Obtain a reference to a PxBase interface if the constraint has one.
    ///
    /// If the constraint does not implement the PxBase interface, it should return NULL.
    pub fn PxConstraintConnector_getSerializable_mut(self_: *mut PxConstraintConnector) -> *mut PxBase;

    /// Obtain the pointer to the constraint's constant data
    pub fn PxConstraintConnector_getConstantBlock(self_: *const PxConstraintConnector) -> *const std::ffi::c_void;

    /// Let the connector know it has been connected to a constraint.
    pub fn PxConstraintConnector_connectToConstraint_mut(self_: *mut PxConstraintConnector, anon_param0: *mut PxConstraint);

    /// virtual destructor
    pub fn PxConstraintConnector_delete(self_: *mut PxConstraintConnector);

    pub fn PxSolverBody_new() -> PxSolverBody;

    pub fn PxSolverBodyData_projectVelocity(self_: *const PxSolverBodyData, lin: *const PxVec3, ang: *const PxVec3) -> f32;

    pub fn PxSolverConstraintPrepDesc_delete(self_: *mut PxSolverConstraintPrepDesc);

    /// Allocates constraint data. It is the application's responsibility to release this memory after PxSolveConstraints has completed.
    ///
    /// The allocated memory. This address must be 16-byte aligned.
    pub fn PxConstraintAllocator_reserveConstraintData_mut(self_: *mut PxConstraintAllocator, byteSize: u32) -> *mut u8;

    /// Allocates friction data. Friction data can be retained by the application for a given pair and provided as an input to PxSolverContactDesc to improve simulation stability.
    /// It is the application's responsibility to release this memory. If this memory is released, the application should ensure it does not pass pointers to this memory to PxSolverContactDesc.
    ///
    /// The allocated memory. This address must be 4-byte aligned.
    pub fn PxConstraintAllocator_reserveFrictionData_mut(self_: *mut PxConstraintAllocator, byteSize: u32) -> *mut u8;

    pub fn PxConstraintAllocator_delete(self_: *mut PxConstraintAllocator);

    pub fn PxArticulationLimit_new() -> PxArticulationLimit;

    pub fn PxArticulationLimit_new_1(low_: f32, high_: f32) -> PxArticulationLimit;

    pub fn PxArticulationDrive_new() -> PxArticulationDrive;

    pub fn PxArticulationDrive_new_1(stiffness_: f32, damping_: f32, maxForce_: f32, driveType_: PxArticulationDriveType) -> PxArticulationDrive;

    pub fn PxTGSSolverBodyVel_projectVelocity(self_: *const PxTGSSolverBodyVel, lin: *const PxVec3, ang: *const PxVec3) -> f32;

    pub fn PxTGSSolverBodyData_projectVelocity(self_: *const PxTGSSolverBodyData, linear: *const PxVec3, angular: *const PxVec3) -> f32;

    pub fn PxTGSSolverConstraintPrepDesc_delete(self_: *mut PxTGSSolverConstraintPrepDesc);

    /// Sets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    pub fn PxArticulationAttachment_setRestLength_mut(self_: *mut PxArticulationAttachment, restLength: f32);

    /// Gets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// The rest length.
    pub fn PxArticulationAttachment_getRestLength(self_: *const PxArticulationAttachment) -> f32;

    /// Sets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
    pub fn PxArticulationAttachment_setLimitParameters_mut(self_: *mut PxArticulationAttachment, parameters: *const PxArticulationTendonLimit);

    /// Gets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Struct with the low and high limit.
    pub fn PxArticulationAttachment_getLimitParameters(self_: *const PxArticulationAttachment) -> PxArticulationTendonLimit;

    /// Sets the attachment's relative offset in the link actor frame.
    pub fn PxArticulationAttachment_setRelativeOffset_mut(self_: *mut PxArticulationAttachment, offset: *const PxVec3);

    /// Gets the attachment's relative offset in the link actor frame.
    ///
    /// The relative offset in the link actor frame.
    pub fn PxArticulationAttachment_getRelativeOffset(self_: *const PxArticulationAttachment) -> PxVec3;

    /// Sets the attachment coefficient.
    pub fn PxArticulationAttachment_setCoefficient_mut(self_: *mut PxArticulationAttachment, coefficient: f32);

    /// Gets the attachment coefficient.
    ///
    /// The scale that the distance between this attachment and its parent is multiplied by when summing up the spatial tendon's length.
    pub fn PxArticulationAttachment_getCoefficient(self_: *const PxArticulationAttachment) -> f32;

    /// Gets the articulation link.
    ///
    /// The articulation link that this attachment is attached to.
    pub fn PxArticulationAttachment_getLink(self_: *const PxArticulationAttachment) -> *mut PxArticulationLink;

    /// Gets the parent attachment.
    ///
    /// The parent attachment.
    pub fn PxArticulationAttachment_getParent(self_: *const PxArticulationAttachment) -> *mut PxArticulationAttachment;

    /// Indicates that this attachment is a leaf, and thus defines a sub-tendon from the root to this attachment.
    ///
    /// True: This attachment is a leaf and has zero children; False: Not a leaf.
    pub fn PxArticulationAttachment_isLeaf(self_: *const PxArticulationAttachment) -> bool;

    /// Gets the spatial tendon that the attachment is a part of.
    ///
    /// The tendon.
    pub fn PxArticulationAttachment_getTendon(self_: *const PxArticulationAttachment) -> *mut PxArticulationSpatialTendon;

    /// Releases the attachment.
    ///
    /// Releasing the attachment is not allowed while the articulation is in a scene. In order to
    /// release the attachment, remove and then re-add the articulation to the scene.
    pub fn PxArticulationAttachment_release_mut(self_: *mut PxArticulationAttachment);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationAttachment_getConcreteTypeName(self_: *const PxArticulationAttachment) -> *const std::ffi::c_char;

    /// Sets the tendon joint coefficient.
    ///
    /// RecipCoefficient is commonly expected to be 1/coefficient, but it can be set to different values to tune behavior; for example, zero can be used to
    /// have a joint axis only participate in the length computation of the tendon, but not have any tendon force applied to it.
    pub fn PxArticulationTendonJoint_setCoefficient_mut(self_: *mut PxArticulationTendonJoint, axis: PxArticulationAxis, coefficient: f32, recipCoefficient: f32);

    /// Gets the tendon joint coefficient.
    pub fn PxArticulationTendonJoint_getCoefficient(self_: *const PxArticulationTendonJoint, axis: *mut PxArticulationAxis, coefficient: *mut f32, recipCoefficient: *mut f32);

    /// Gets the articulation link.
    ///
    /// The articulation link (and its incoming joint in particular) that this tendon joint is associated with.
    pub fn PxArticulationTendonJoint_getLink(self_: *const PxArticulationTendonJoint) -> *mut PxArticulationLink;

    /// Gets the parent tendon joint.
    ///
    /// The parent tendon joint.
    pub fn PxArticulationTendonJoint_getParent(self_: *const PxArticulationTendonJoint) -> *mut PxArticulationTendonJoint;

    /// Gets the tendon that the joint is a part of.
    ///
    /// The tendon.
    pub fn PxArticulationTendonJoint_getTendon(self_: *const PxArticulationTendonJoint) -> *mut PxArticulationFixedTendon;

    /// Releases a tendon joint.
    ///
    /// Releasing a tendon joint is not allowed while the articulation is in a scene. In order to
    /// release the joint, remove and then re-add the articulation to the scene.
    pub fn PxArticulationTendonJoint_release_mut(self_: *mut PxArticulationTendonJoint);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationTendonJoint_getConcreteTypeName(self_: *const PxArticulationTendonJoint) -> *const std::ffi::c_char;

    /// Sets the spring stiffness term acting on the tendon length.
    pub fn PxArticulationTendon_setStiffness_mut(self_: *mut PxArticulationTendon, stiffness: f32);

    /// Gets the spring stiffness of the tendon.
    ///
    /// The spring stiffness.
    pub fn PxArticulationTendon_getStiffness(self_: *const PxArticulationTendon) -> f32;

    /// Sets the damping term acting both on the tendon length and tendon-length limits.
    pub fn PxArticulationTendon_setDamping_mut(self_: *mut PxArticulationTendon, damping: f32);

    /// Gets the damping term acting both on the tendon length and tendon-length limits.
    ///
    /// The damping term.
    pub fn PxArticulationTendon_getDamping(self_: *const PxArticulationTendon) -> f32;

    /// Sets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    pub fn PxArticulationTendon_setLimitStiffness_mut(self_: *mut PxArticulationTendon, stiffness: f32);

    /// Gets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    ///
    /// The limit stiffness term.
    pub fn PxArticulationTendon_getLimitStiffness(self_: *const PxArticulationTendon) -> f32;

    /// Sets the length offset term for the tendon.
    ///
    /// An offset defines an amount to be added to the accumulated length computed for the tendon. It allows the
    /// application to actuate the tendon by shortening or lengthening it.
    pub fn PxArticulationTendon_setOffset_mut(self_: *mut PxArticulationTendon, offset: f32, autowake: bool);

    /// Gets the length offset term for the tendon.
    ///
    /// The offset term.
    pub fn PxArticulationTendon_getOffset(self_: *const PxArticulationTendon) -> f32;

    /// Gets the articulation that the tendon is a part of.
    ///
    /// The articulation.
    pub fn PxArticulationTendon_getArticulation(self_: *const PxArticulationTendon) -> *mut PxArticulationReducedCoordinate;

    /// Releases a tendon to remove it from the articulation and free its associated memory.
    ///
    /// When an articulation is released, its attached tendons are automatically released.
    ///
    /// Releasing a tendon is not allowed while the articulation is in a scene. In order to
    /// release the tendon, remove and then re-add the articulation to the scene.
    pub fn PxArticulationTendon_release_mut(self_: *mut PxArticulationTendon);

    /// Creates an articulation attachment and adds it to the list of children in the parent attachment.
    ///
    /// Creating an attachment is not allowed while the articulation is in a scene. In order to
    /// add the attachment, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created attachment if creation was successful, otherwise a null pointer.
    pub fn PxArticulationSpatialTendon_createAttachment_mut(self_: *mut PxArticulationSpatialTendon, parent: *mut PxArticulationAttachment, coefficient: f32, relativeOffset: PxVec3, link: *mut PxArticulationLink) -> *mut PxArticulationAttachment;

    /// Fills a user-provided buffer of attachment pointers with the set of attachments.
    ///
    /// The number of attachments that were filled into the user buffer.
    pub fn PxArticulationSpatialTendon_getAttachments(self_: *const PxArticulationSpatialTendon, userBuffer: *mut *mut PxArticulationAttachment, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of attachments in the tendon.
    ///
    /// The number of attachments.
    pub fn PxArticulationSpatialTendon_getNbAttachments(self_: *const PxArticulationSpatialTendon) -> u32;

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationSpatialTendon_getConcreteTypeName(self_: *const PxArticulationSpatialTendon) -> *const std::ffi::c_char;

    /// Creates an articulation tendon joint and adds it to the list of children in the parent tendon joint.
    ///
    /// Creating a tendon joint is not allowed while the articulation is in a scene. In order to
    /// add the joint, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created tendon joint if creation was successful, otherwise a null pointer.
    ///
    /// - The axis motion must not be configured as PxArticulationMotion::eLOCKED.
    /// - The axis cannot be part of a fixed joint, i.e. joint configured as PxArticulationJointType::eFIX.
    pub fn PxArticulationFixedTendon_createTendonJoint_mut(self_: *mut PxArticulationFixedTendon, parent: *mut PxArticulationTendonJoint, axis: PxArticulationAxis, coefficient: f32, recipCoefficient: f32, link: *mut PxArticulationLink) -> *mut PxArticulationTendonJoint;

    /// Fills a user-provided buffer of tendon-joint pointers with the set of tendon joints.
    ///
    /// The number of tendon joints filled into the user buffer.
    pub fn PxArticulationFixedTendon_getTendonJoints(self_: *const PxArticulationFixedTendon, userBuffer: *mut *mut PxArticulationTendonJoint, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of tendon joints in the tendon.
    ///
    /// The number of tendon joints.
    pub fn PxArticulationFixedTendon_getNbTendonJoints(self_: *const PxArticulationFixedTendon) -> u32;

    /// Sets the spring rest length of the tendon.
    ///
    /// The accumulated "length" of a fixed tendon is a linear combination of the joint axis positions that the tendon is
    /// associated with, scaled by the respective tendon joints' coefficients. As such, when the joint positions of all
    /// joints are zero, the accumulated length of a fixed tendon is zero.
    ///
    /// The spring of the tendon is not exerting any force on the articulation when the rest length is equal to the
    /// tendon's accumulated length plus the tendon offset.
    pub fn PxArticulationFixedTendon_setRestLength_mut(self_: *mut PxArticulationFixedTendon, restLength: f32);

    /// Gets the spring rest length of the tendon.
    ///
    /// The spring rest length of the tendon.
    pub fn PxArticulationFixedTendon_getRestLength(self_: *const PxArticulationFixedTendon) -> f32;

    /// Sets the low and high limit on the length of the tendon.
    ///
    /// The limits, together with the damping and limit stiffness parameters, act on the accumulated length of the tendon.
    pub fn PxArticulationFixedTendon_setLimitParameters_mut(self_: *mut PxArticulationFixedTendon, parameter: *const PxArticulationTendonLimit);

    /// Gets the low and high limit on the length of the tendon.
    ///
    /// Struct with the low and high limit.
    pub fn PxArticulationFixedTendon_getLimitParameters(self_: *const PxArticulationFixedTendon) -> PxArticulationTendonLimit;

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationFixedTendon_getConcreteTypeName(self_: *const PxArticulationFixedTendon) -> *const std::ffi::c_char;

    pub fn PxArticulationCache_new() -> PxArticulationCache;

    /// Releases an articulation cache.
    pub fn PxArticulationCache_release_mut(self_: *mut PxArticulationCache);

    /// Releases the sensor.
    ///
    /// Releasing a sensor is not allowed while the articulation is in a scene. In order to
    /// release a sensor, remove and then re-add the articulation to the scene.
    pub fn PxArticulationSensor_release_mut(self_: *mut PxArticulationSensor);

    /// Returns the spatial force in the local frame of the sensor.
    ///
    /// The spatial force.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationSensor_getForces(self_: *const PxArticulationSensor) -> PxSpatialForce;

    /// Returns the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// The transform link body frame -> sensor frame.
    pub fn PxArticulationSensor_getRelativePose(self_: *const PxArticulationSensor) -> PxTransform;

    /// Sets the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// Setting the sensor relative pose is not allowed while the articulation is in a scene. In order to
    /// set the pose, remove and then re-add the articulation to the scene.
    pub fn PxArticulationSensor_setRelativePose_mut(self_: *mut PxArticulationSensor, pose: *const PxTransform);

    /// Returns the link that this sensor is attached to.
    ///
    /// A pointer to the link.
    pub fn PxArticulationSensor_getLink(self_: *const PxArticulationSensor) -> *mut PxArticulationLink;

    /// Returns the index of this sensor inside the articulation.
    ///
    /// The return value is only valid for sensors attached to articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    pub fn PxArticulationSensor_getIndex(self_: *const PxArticulationSensor) -> u32;

    /// Returns the articulation that this sensor is part of.
    ///
    /// A pointer to the articulation.
    pub fn PxArticulationSensor_getArticulation(self_: *const PxArticulationSensor) -> *mut PxArticulationReducedCoordinate;

    /// Returns the sensor's flags.
    ///
    /// The current set of flags of the sensor.
    pub fn PxArticulationSensor_getFlags(self_: *const PxArticulationSensor) -> PxArticulationSensorFlags;

    /// Sets a flag of the sensor.
    ///
    /// Setting the sensor flags is not allowed while the articulation is in a scene. In order to
    /// set the flags, remove and then re-add the articulation to the scene.
    pub fn PxArticulationSensor_setFlag_mut(self_: *mut PxArticulationSensor, flag: PxArticulationSensorFlag, enabled: bool);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationSensor_getConcreteTypeName(self_: *const PxArticulationSensor) -> *const std::ffi::c_char;

    /// Returns the scene which this articulation belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    pub fn PxArticulationReducedCoordinate_getScene(self_: *const PxArticulationReducedCoordinate) -> *mut PxScene;

    /// Sets the solver iteration counts for the articulation.
    ///
    /// The solver iteration count determines how accurately contacts, drives, and limits are resolved.
    /// Setting a higher position iteration count may therefore help in scenarios where the articulation
    /// is subject to many constraints; for example, a manipulator articulation with drives and joint limits
    /// that is grasping objects, or several such articulations interacting through contacts. Other situations
    /// where higher position iterations may improve simulation fidelity are: large mass ratios within the
    /// articulation or between the articulation and an object in contact with it; or strong drives in the
    /// articulation being used to manipulate a light object.
    ///
    /// If intersecting bodies are being depenetrated too violently, increase the number of velocity
    /// iterations. More velocity iterations will drive the relative exit velocity of the intersecting
    /// objects closer to the correct value given the restitution.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setSolverIterationCounts_mut(self_: *mut PxArticulationReducedCoordinate, minPositionIters: u32, minVelocityIters: u32);

    /// Returns the solver iteration counts.
    pub fn PxArticulationReducedCoordinate_getSolverIterationCounts(self_: *const PxArticulationReducedCoordinate, minPositionIters: *mut u32, minVelocityIters: *mut u32);

    /// Returns true if this articulation is sleeping.
    ///
    /// When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
    /// is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
    /// or a sleep-affecting property is changed by the user, the entire sleep mechanism should be transparent to the user.
    ///
    /// An articulation can only go to sleep if all links are ready for sleeping. An articulation is guaranteed to be awake
    /// if at least one of the following holds:
    ///
    /// The wake counter is positive (see [`setWakeCounter`]()).
    ///
    /// The linear or angular velocity of any link is non-zero.
    ///
    /// A non-zero force or torque has been applied to the articulation or any of its links.
    ///
    /// If an articulation is sleeping, the following state is guaranteed:
    ///
    /// The wake counter is zero.
    ///
    /// The linear and angular velocity of all links is zero.
    ///
    /// There is no force update pending.
    ///
    /// When an articulation gets inserted into a scene, it will be considered asleep if all the points above hold, else it will
    /// be treated as awake.
    ///
    /// If an articulation is asleep after the call to [`PxScene::fetchResults`]() returns, it is guaranteed that the poses of the
    /// links were not changed. You can use this information to avoid updating the transforms of associated objects.
    ///
    /// True if the articulation is sleeping.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    pub fn PxArticulationReducedCoordinate_isSleeping(self_: *const PxArticulationReducedCoordinate) -> bool;

    /// Sets the mass-normalized energy threshold below which the articulation may go to sleep.
    ///
    /// The articulation will sleep if the energy of each link is below this threshold.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setSleepThreshold_mut(self_: *mut PxArticulationReducedCoordinate, threshold: f32);

    /// Returns the mass-normalized energy below which the articulation may go to sleep.
    ///
    /// The energy threshold for sleeping.
    pub fn PxArticulationReducedCoordinate_getSleepThreshold(self_: *const PxArticulationReducedCoordinate) -> f32;

    /// Sets the mass-normalized kinetic energy threshold below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 0.01 * PxTolerancesScale::speed * PxTolerancesScale::speed
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setStabilizationThreshold_mut(self_: *mut PxArticulationReducedCoordinate, threshold: f32);

    /// Returns the mass-normalized kinetic energy below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    pub fn PxArticulationReducedCoordinate_getStabilizationThreshold(self_: *const PxArticulationReducedCoordinate) -> f32;

    /// Sets the wake counter for the articulation in seconds.
    ///
    /// - The wake counter value determines the minimum amount of time until the articulation can be put to sleep.
    /// - An articulation will not be put to sleep if the energy is above the specified threshold (see [`setSleepThreshold`]())
    /// or if other awake objects are touching it.
    /// - Passing in a positive value will wake up the articulation automatically.
    ///
    /// Default:
    /// 0.4s (which corresponds to 20 frames for a time step of 0.02s)
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    pub fn PxArticulationReducedCoordinate_setWakeCounter_mut(self_: *mut PxArticulationReducedCoordinate, wakeCounterValue: f32);

    /// Returns the wake counter of the articulation in seconds.
    ///
    /// The wake counter of the articulation in seconds.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    pub fn PxArticulationReducedCoordinate_getWakeCounter(self_: *const PxArticulationReducedCoordinate) -> f32;

    /// Wakes up the articulation if it is sleeping.
    ///
    /// - The articulation will get woken up and might cause other touching objects to wake up as well during the next simulation step.
    /// - This will set the wake counter of the articulation to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    pub fn PxArticulationReducedCoordinate_wakeUp_mut(self_: *mut PxArticulationReducedCoordinate);

    /// Forces the articulation to sleep.
    ///
    /// - The articulation will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    /// - This will set any applied force, the velocity, and the wake counter of all bodies in the articulation to zero.
    ///
    /// This call may not be made during simulation, and may only be made on articulations that are in a scene.
    pub fn PxArticulationReducedCoordinate_putToSleep_mut(self_: *mut PxArticulationReducedCoordinate);

    /// Sets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// - The limit acts on the linear velocity of the entire articulation. The velocity is calculated from the total momentum
    /// and the spatial inertia of the articulation.
    /// - The limit only applies to floating-base articulations.
    /// - A benefit of the COM velocity limit is that it is evenly applied to the whole articulation, which results in fewer visual
    /// artifacts compared to link rigid-body damping or joint-velocity limits. However, these per-link or per-degree-of-freedom
    /// limits may still help avoid numerical issues.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut(self_: *mut PxArticulationReducedCoordinate, maxLinearVelocity: f32);

    /// Gets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// The maximal linear velocity magnitude.
    pub fn PxArticulationReducedCoordinate_getMaxCOMLinearVelocity(self_: *const PxArticulationReducedCoordinate) -> f32;

    /// Sets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// - The limit acts on the angular velocity of the entire articulation. The velocity is calculated from the total momentum
    /// and the spatial inertia of the articulation.
    /// - The limit only applies to floating-base articulations.
    /// - A benefit of the COM velocity limit is that it is evenly applied to the whole articulation, which results in fewer visual
    /// artifacts compared to link rigid-body damping or joint-velocity limits. However, these per-link or per-degree-of-freedom
    /// limits may still help avoid numerical issues.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut(self_: *mut PxArticulationReducedCoordinate, maxAngularVelocity: f32);

    /// Gets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// The maximal angular velocity magnitude.
    pub fn PxArticulationReducedCoordinate_getMaxCOMAngularVelocity(self_: *const PxArticulationReducedCoordinate) -> f32;

    /// Adds a link to the articulation with default attribute values.
    ///
    /// The new link, or NULL if the link cannot be created.
    ///
    /// Creating a link is not allowed while the articulation is in a scene. In order to add a link,
    /// remove and then re-add the articulation to the scene.
    pub fn PxArticulationReducedCoordinate_createLink_mut(self_: *mut PxArticulationReducedCoordinate, parent: *mut PxArticulationLink, pose: *const PxTransform) -> *mut PxArticulationLink;

    /// Releases the articulation, and all its links and corresponding joints.
    ///
    /// Attached sensors and tendons are released automatically when the articulation is released.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_release_mut(self_: *mut PxArticulationReducedCoordinate);

    /// Returns the number of links in the articulation.
    ///
    /// The number of links.
    pub fn PxArticulationReducedCoordinate_getNbLinks(self_: *const PxArticulationReducedCoordinate) -> u32;

    /// Returns the set of links in the articulation in the order that they were added to the articulation using createLink.
    ///
    /// The number of links written into the buffer.
    pub fn PxArticulationReducedCoordinate_getLinks(self_: *const PxArticulationReducedCoordinate, userBuffer: *mut *mut PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of shapes in the articulation.
    ///
    /// The number of shapes.
    pub fn PxArticulationReducedCoordinate_getNbShapes(self_: *const PxArticulationReducedCoordinate) -> u32;

    /// Sets a name string for the articulation that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    pub fn PxArticulationReducedCoordinate_setName_mut(self_: *mut PxArticulationReducedCoordinate, name: *const std::ffi::c_char);

    /// Returns the name string set with setName().
    ///
    /// Name string associated with the articulation.
    pub fn PxArticulationReducedCoordinate_getName(self_: *const PxArticulationReducedCoordinate) -> *const std::ffi::c_char;

    /// Returns the axis-aligned bounding box enclosing the articulation.
    ///
    /// The articulation's bounding box.
    ///
    /// It is not allowed to use this method while the simulation is running, except in a split simulation
    /// during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationReducedCoordinate_getWorldBounds(self_: *const PxArticulationReducedCoordinate, inflation: f32) -> PxBounds3;

    /// Returns the aggregate the articulation might be a part of.
    ///
    /// The aggregate the articulation is a part of, or NULL if the articulation does not belong to an aggregate.
    pub fn PxArticulationReducedCoordinate_getAggregate(self_: *const PxArticulationReducedCoordinate) -> *mut PxAggregate;

    /// Sets flags on the articulation.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setArticulationFlags_mut(self_: *mut PxArticulationReducedCoordinate, flags: PxArticulationFlags);

    /// Raises or clears a flag on the articulation.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setArticulationFlag_mut(self_: *mut PxArticulationReducedCoordinate, flag: PxArticulationFlag, value: bool);

    /// Returns the articulation's flags.
    ///
    /// The flags.
    pub fn PxArticulationReducedCoordinate_getArticulationFlags(self_: *const PxArticulationReducedCoordinate) -> PxArticulationFlags;

    /// Returns the total number of joint degrees-of-freedom (DOFs) of the articulation.
    ///
    /// - The six DOFs of the base of a floating-base articulation are not included in this count.
    /// - Example: Both a fixed-base and a floating-base double-pendulum with two revolute joints will have getDofs() == 2.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of joint DOFs, or 0xFFFFFFFF if the articulation is not in a scene.
    pub fn PxArticulationReducedCoordinate_getDofs(self_: *const PxArticulationReducedCoordinate) -> u32;

    /// Creates an articulation cache that can be used to read and write internal articulation data.
    ///
    /// - When the structure of the articulation changes (e.g. adding a link or sensor) after the cache was created,
    /// the cache needs to be released and recreated.
    /// - Free the memory allocated for the cache by calling the release() method on the cache.
    /// - Caches can only be created by articulations that are in a scene.
    ///
    /// The cache, or NULL if the articulation is not in a scene.
    pub fn PxArticulationReducedCoordinate_createCache(self_: *const PxArticulationReducedCoordinate) -> *mut PxArticulationCache;

    /// Returns the size of the articulation cache in bytes.
    ///
    /// - The size does not include: the user-allocated memory for the coefficient matrix or lambda values;
    /// the scratch-related memory/members; and the cache version. See comment in [`PxArticulationCache`].
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The byte size of the cache, or 0xFFFFFFFF if the articulation is not in a scene.
    pub fn PxArticulationReducedCoordinate_getCacheDataSize(self_: *const PxArticulationReducedCoordinate) -> u32;

    /// Zeroes all data in the articulation cache, except user-provided and scratch memory, and cache version.
    ///
    /// This call may only be made on articulations that are in a scene.
    pub fn PxArticulationReducedCoordinate_zeroCache(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Applies the data in the cache to the articulation.
    ///
    /// This call wakes the articulation if it is sleeping, and the autowake parameter is true (default) or:
    /// - a nonzero joint velocity is applied or
    /// - a nonzero joint force is applied or
    /// - a nonzero root velocity is applied
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_applyCache_mut(self_: *mut PxArticulationReducedCoordinate, cache: *mut PxArticulationCache, flags: PxArticulationCacheFlags, autowake: bool);

    /// Copies internal data of the articulation to the cache.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_copyInternalStateToCache(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache, flags: PxArticulationCacheFlags);

    /// Converts maximal-coordinate joint DOF data to reduced coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    pub fn PxArticulationReducedCoordinate_packJointData(self_: *const PxArticulationReducedCoordinate, maximum: *const f32, reduced: *mut f32);

    /// Converts reduced-coordinate joint DOF data to maximal coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
    pub fn PxArticulationReducedCoordinate_unpackJointData(self_: *const PxArticulationReducedCoordinate, reduced: *const f32, maximum: *mut f32);

    /// Prepares common articulation data based on articulation pose for inverse dynamics calculations.
    ///
    /// Usage:
    /// 1. Set articulation pose (joint positions and base transform) via articulation cache and applyCache().
    /// 1. Call commonInit.
    /// 1. Call inverse dynamics computation method.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_commonInit(self_: *const PxArticulationReducedCoordinate);

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// - Inputs - Articulation pose (joint positions + base transform).
    /// - Outputs - Joint forces to counteract gravity (in cache).
    ///
    /// - The joint forces returned are determined purely by gravity for the articulation in the current joint and base pose, and joints at rest;
    /// i.e. external forces, joint velocities, and joint accelerations are set to zero. Joint drives are also not considered in the computation.
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeGeneralizedGravityForce(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Computes the joint DOF forces required to counteract Coriolis and centrifugal forces for the given articulation state.
    ///
    /// - Inputs - Articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint forces to counteract Coriolis and centrifugal forces (in cache).
    ///
    /// - The joint forces returned are determined purely by the articulation's state; i.e. external forces, gravity, and joint accelerations are set to zero.
    /// Joint drives and potential damping terms, such as link angular or linear damping, or joint friction, are also not considered in the computation.
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Computes the joint DOF forces required to counteract external spatial forces applied to articulation links.
    ///
    /// - Inputs - External forces on links (in cache), articulation pose (joint positions + base transform).
    /// - Outputs - Joint forces to counteract the external forces (in cache).
    ///
    /// - Only the external spatial forces provided in the cache and the articulation pose are considered in the computation.
    /// - The external spatial forces are with respect to the links' centers of mass, and not the actor's origin.
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeGeneralizedExternalForce(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Computes the joint accelerations for the given articulation state and joint forces.
    ///
    /// - Inputs - Joint forces (in cache) and articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint accelerations (in cache).
    ///
    /// - The computation includes Coriolis terms and gravity. However, joint drives and potential damping terms are not considered in the computation
    /// (for example, linear link damping or joint friction).
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeJointAcceleration(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Computes the joint forces for the given articulation state and joint accelerations, not considering gravity.
    ///
    /// - Inputs - Joint accelerations (in cache) and articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint forces (in cache).
    ///
    /// - The computation includes Coriolis terms. However, joint drives and potential damping terms are not considered in the computation
    /// (for example, linear link damping or joint friction).
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeJointForce(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Compute the dense Jacobian for the articulation in world space, including the DOFs of a potentially floating base.
    ///
    /// This computes the dense representation of an inherently sparse matrix. Multiplication with this matrix maps
    /// joint space velocities to world-space linear and angular (i.e. spatial) velocities of the centers of mass of the links.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeDenseJacobian(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache, nRows: *mut u32, nCols: *mut u32);

    /// Computes the coefficient matrix for contact forces.
    ///
    /// - The matrix dimension is getCoefficientMatrixSize() = getDofs() * getNbLoopJoints(), and the DOF (column) indexing follows the internal DOF order, see PxArticulationCache::jointVelocity.
    /// - Each column in the matrix is the joint forces effected by a contact based on impulse strength 1.
    /// - The user must allocate memory for PxArticulationCache::coefficientMatrix where the required size of the PxReal array is equal to getCoefficientMatrixSize().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeCoefficientMatrix(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Computes the lambda values when the test impulse is 1.
    ///
    /// - The user must allocate memory for PxArticulationCache::lambda where the required size of the PxReal array is equal to getNbLoopJoints().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// True if convergence was achieved within maxIter; False if convergence was not achieved or the operation failed otherwise.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeLambda(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache, initialState: *mut PxArticulationCache, jointTorque: *const f32, maxIter: u32) -> bool;

    /// Compute the joint-space inertia matrix that maps joint accelerations to joint forces: forces = M * accelerations.
    ///
    /// - Inputs - Articulation pose (joint positions and base transform).
    /// - Outputs - Mass matrix (in cache).
    ///
    /// commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_computeGeneralizedMassMatrix(self_: *const PxArticulationReducedCoordinate, cache: *mut PxArticulationCache);

    /// Adds a loop joint to the articulation system for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_addLoopJoint_mut(self_: *mut PxArticulationReducedCoordinate, joint: *mut PxConstraint);

    /// Removes a loop joint from the articulation for inverse dynamics.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_removeLoopJoint_mut(self_: *mut PxArticulationReducedCoordinate, joint: *mut PxConstraint);

    /// Returns the number of loop joints in the articulation for inverse dynamics.
    ///
    /// The number of loop joints.
    pub fn PxArticulationReducedCoordinate_getNbLoopJoints(self_: *const PxArticulationReducedCoordinate) -> u32;

    /// Returns the set of loop constraints (i.e. joints) in the articulation.
    ///
    /// The number of constraints written into the buffer.
    pub fn PxArticulationReducedCoordinate_getLoopJoints(self_: *const PxArticulationReducedCoordinate, userBuffer: *mut *mut PxConstraint, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the required size of the coefficient matrix in the articulation.
    ///
    /// Size of the coefficient matrix (equal to getDofs() * getNbLoopJoints()).
    ///
    /// This call may only be made on articulations that are in a scene.
    pub fn PxArticulationReducedCoordinate_getCoefficientMatrixSize(self_: *const PxArticulationReducedCoordinate) -> u32;

    /// Sets the root link transform (world to actor frame).
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link transform in a batch articulation state update.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_setRootGlobalPose_mut(self_: *mut PxArticulationReducedCoordinate, pose: *const PxTransform, autowake: bool);

    /// Returns the root link transform (world to actor frame).
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link transform in a batch query.
    ///
    /// The root link transform.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationReducedCoordinate_getRootGlobalPose(self_: *const PxArticulationReducedCoordinate) -> PxTransform;

    /// Sets the root link linear center-of-mass velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    pub fn PxArticulationReducedCoordinate_setRootLinearVelocity_mut(self_: *mut PxArticulationReducedCoordinate, linearVelocity: *const PxVec3, autowake: bool);

    /// Gets the root link center-of-mass linear velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link center-of-mass linear velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationReducedCoordinate_getRootLinearVelocity(self_: *const PxArticulationReducedCoordinate) -> PxVec3;

    /// Sets the root link angular velocity.
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
    pub fn PxArticulationReducedCoordinate_setRootAngularVelocity_mut(self_: *mut PxArticulationReducedCoordinate, angularVelocity: *const PxVec3, autowake: bool);

    /// Gets the root link angular velocity.
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link angular velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationReducedCoordinate_getRootAngularVelocity(self_: *const PxArticulationReducedCoordinate) -> PxVec3;

    /// Returns the (classical) link acceleration in world space for the given low-level link index.
    ///
    /// - The returned acceleration is not a spatial, but a classical, i.e. body-fixed acceleration (https://en.wikipedia.org/wiki/Spatial_acceleration).
    /// - The (linear) acceleration is with respect to the link's center of mass and not the actor frame origin.
    ///
    /// The link's center-of-mass classical acceleration, or 0 if the call is made before the articulation participated in a first simulation step.
    ///
    /// This call may only be made on articulations that are in a scene, and it is not allowed to use this method while the simulation
    /// is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationReducedCoordinate_getLinkAcceleration_mut(self_: *mut PxArticulationReducedCoordinate, linkId: u32) -> PxSpatialVelocity;

    /// Returns the GPU articulation index.
    ///
    /// The GPU index, or 0xFFFFFFFF if the articulation is not in a scene or PxSceneFlag::eSUPPRESS_READBACK is not set.
    pub fn PxArticulationReducedCoordinate_getGpuArticulationIndex_mut(self_: *mut PxArticulationReducedCoordinate) -> u32;

    /// Creates a spatial tendon to attach to the articulation with default attribute values.
    ///
    /// The new spatial tendon.
    ///
    /// Creating a spatial tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    pub fn PxArticulationReducedCoordinate_createSpatialTendon_mut(self_: *mut PxArticulationReducedCoordinate) -> *mut PxArticulationSpatialTendon;

    /// Creates a fixed tendon to attach to the articulation with default attribute values.
    ///
    /// The new fixed tendon.
    ///
    /// Creating a fixed tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
    pub fn PxArticulationReducedCoordinate_createFixedTendon_mut(self_: *mut PxArticulationReducedCoordinate) -> *mut PxArticulationFixedTendon;

    /// Creates a force sensor attached to a link of the articulation.
    ///
    /// The new sensor.
    ///
    /// Creating a sensor is not allowed while the articulation is in a scene. In order to
    /// add the sensor, remove and then re-add the articulation to the scene.
    pub fn PxArticulationReducedCoordinate_createSensor_mut(self_: *mut PxArticulationReducedCoordinate, link: *mut PxArticulationLink, relativePose: *const PxTransform) -> *mut PxArticulationSensor;

    /// Returns the spatial tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    pub fn PxArticulationReducedCoordinate_getSpatialTendons(self_: *const PxArticulationReducedCoordinate, userBuffer: *mut *mut PxArticulationSpatialTendon, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of spatial tendons in the articulation.
    ///
    /// The number of tendons.
    pub fn PxArticulationReducedCoordinate_getNbSpatialTendons_mut(self_: *mut PxArticulationReducedCoordinate) -> u32;

    /// Returns the fixed tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
    pub fn PxArticulationReducedCoordinate_getFixedTendons(self_: *const PxArticulationReducedCoordinate, userBuffer: *mut *mut PxArticulationFixedTendon, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of fixed tendons in the articulation.
    ///
    /// The number of tendons.
    pub fn PxArticulationReducedCoordinate_getNbFixedTendons_mut(self_: *mut PxArticulationReducedCoordinate) -> u32;

    /// Returns the sensors attached to the articulation.
    ///
    /// The order of the sensors in the buffer is not necessarily identical to the order in which the sensors were added to the articulation.
    ///
    /// The number of sensors written into the buffer.
    pub fn PxArticulationReducedCoordinate_getSensors(self_: *const PxArticulationReducedCoordinate, userBuffer: *mut *mut PxArticulationSensor, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of sensors in the articulation.
    ///
    /// The number of sensors.
    pub fn PxArticulationReducedCoordinate_getNbSensors_mut(self_: *mut PxArticulationReducedCoordinate) -> u32;

    /// Update link velocities and/or positions in the articulation.
    ///
    /// For performance, prefer the PxArticulationCache API that performs batch articulation state updates.
    ///
    /// If the application updates the root state (position and velocity) or joint state via any combination of
    /// the non-cache API calls
    ///
    /// - setRootGlobalPose(), setRootLinearVelocity(), setRootAngularVelocity()
    /// - PxArticulationJointReducedCoordinate::setJointPosition(), PxArticulationJointReducedCoordinate::setJointVelocity()
    ///
    /// the application needs to call this method after the state setting in order to update the link states for
    /// the next simulation frame or querying.
    ///
    /// Use
    /// - PxArticulationKinematicFlag::ePOSITION after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    /// - PxArticulationKinematicFlag::eVELOCITY after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities only.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
    pub fn PxArticulationReducedCoordinate_updateKinematic_mut(self_: *mut PxArticulationReducedCoordinate, flags: PxArticulationKinematicFlags);

    /// Gets the parent articulation link of this joint.
    ///
    /// The parent link.
    pub fn PxArticulationJointReducedCoordinate_getParentArticulationLink(self_: *const PxArticulationJointReducedCoordinate) -> *mut PxArticulationLink;

    /// Sets the joint pose in the parent link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setParentPose_mut(self_: *mut PxArticulationJointReducedCoordinate, pose: *const PxTransform);

    /// Gets the joint pose in the parent link actor frame.
    ///
    /// The joint pose.
    pub fn PxArticulationJointReducedCoordinate_getParentPose(self_: *const PxArticulationJointReducedCoordinate) -> PxTransform;

    /// Gets the child articulation link of this joint.
    ///
    /// The child link.
    pub fn PxArticulationJointReducedCoordinate_getChildArticulationLink(self_: *const PxArticulationJointReducedCoordinate) -> *mut PxArticulationLink;

    /// Sets the joint pose in the child link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setChildPose_mut(self_: *mut PxArticulationJointReducedCoordinate, pose: *const PxTransform);

    /// Gets the joint pose in the child link actor frame.
    ///
    /// The joint pose.
    pub fn PxArticulationJointReducedCoordinate_getChildPose(self_: *const PxArticulationJointReducedCoordinate) -> PxTransform;

    /// Sets the joint type (e.g. revolute).
    ///
    /// Setting the joint type is not allowed while the articulation is in a scene.
    /// In order to set the joint type, remove and then re-add the articulation to the scene.
    pub fn PxArticulationJointReducedCoordinate_setJointType_mut(self_: *mut PxArticulationJointReducedCoordinate, jointType: PxArticulationJointType);

    /// Gets the joint type.
    ///
    /// The joint type.
    pub fn PxArticulationJointReducedCoordinate_getJointType(self_: *const PxArticulationJointReducedCoordinate) -> PxArticulationJointType;

    /// Sets the joint motion for a given axis.
    ///
    /// Setting the motion of joint axes is not allowed while the articulation is in a scene.
    /// In order to set the motion, remove and then re-add the articulation to the scene.
    pub fn PxArticulationJointReducedCoordinate_setMotion_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, motion: PxArticulationMotion);

    /// Returns the joint motion for the given axis.
    ///
    /// The joint motion of the given axis.
    pub fn PxArticulationJointReducedCoordinate_getMotion(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> PxArticulationMotion;

    /// Sets the joint limits for a given axis.
    ///
    /// - The motion of the corresponding axis should be set to PxArticulationMotion::eLIMITED in order for the limits to be enforced.
    /// - The lower limit should be strictly smaller than the higher limit. If the limits should be equal, use PxArticulationMotion::eLOCKED
    /// and an appropriate offset in the parent/child joint frames.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, limit.min and limit.max must both be in range [-Pi, Pi].
    pub fn PxArticulationJointReducedCoordinate_setLimitParams_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, limit: *const PxArticulationLimit);

    /// Returns the joint limits for a given axis.
    ///
    /// The joint limits.
    pub fn PxArticulationJointReducedCoordinate_getLimitParams(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> PxArticulationLimit;

    /// Configures a joint drive for the given axis.
    ///
    /// See PxArticulationDrive for parameter details; and the manual for further information, and the drives' implicit spring-damper (i.e. PD control) implementation in particular.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setDriveParams_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, drive: *const PxArticulationDrive);

    /// Gets the joint drive configuration for the given axis.
    ///
    /// The drive parameters.
    pub fn PxArticulationJointReducedCoordinate_getDriveParams(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> PxArticulationDrive;

    /// Sets the joint drive position target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) for a translational axis, or rad for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, target must be in range [-Pi, Pi].
    ///
    /// The target is specified in the parent frame of the joint. If Gp, Gc are the parent and child actor poses in the world frame and Lp, Lc are the parent and child joint frames expressed in the parent and child actor frames then the joint will drive the parent and child links to poses that obey Gp * Lp * J = Gc * Lc. For joints restricted to angular motion, J has the form PxTranfsorm(PxVec3(PxZero), PxExp(PxVec3(twistTarget, swing1Target, swing2Target))).  For joints restricted to linear motion, J has the form PxTransform(PxVec3(XTarget, YTarget, ZTarget), PxQuat(PxIdentity)).
    ///
    /// For spherical joints with more than 1 degree of freedom, the joint target angles taken together can collectively represent a rotation of greater than Pi around a vector. When this happens the rotation that matches the joint drive target is not the shortest path rotation.  The joint pose J that is the outcome after driving to the target pose will always be the equivalent of the shortest path rotation.
    pub fn PxArticulationJointReducedCoordinate_setDriveTarget_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, target: f32, autowake: bool);

    /// Returns the joint drive position target for the given axis.
    ///
    /// The target position.
    pub fn PxArticulationJointReducedCoordinate_getDriveTarget(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> f32;

    /// Sets the joint drive velocity target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setDriveVelocity_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, targetVel: f32, autowake: bool);

    /// Returns the joint drive velocity target for the given axis.
    ///
    /// The target velocity.
    pub fn PxArticulationJointReducedCoordinate_getDriveVelocity(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> f32;

    /// Sets the joint armature for the given axis.
    ///
    /// - The armature is directly added to the joint-space spatial inertia of the corresponding axis.
    /// - The armature is in mass units for a prismatic (i.e. linear) joint, and in mass units * (scene linear units)^2 for a rotational joint.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setArmature_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, armature: f32);

    /// Gets the joint armature for the given axis.
    ///
    /// The armature set on the given axis.
    pub fn PxArticulationJointReducedCoordinate_getArmature(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> f32;

    /// Sets the joint friction coefficient, which applies to all joint axes.
    ///
    /// - The joint friction is unitless and relates the magnitude of the spatial force [F_trans, T_trans] transmitted from parent to child link to
    /// the maximal friction force F_resist that may be applied by the solver to resist joint motion, per axis; i.e. |F_resist|
    /// <
    /// = coefficient * (|F_trans| + |T_trans|),
    /// where F_resist may refer to a linear force or torque depending on the joint axis.
    /// - The simulated friction effect is therefore similar to static and Coulomb friction. In order to simulate dynamic joint friction, use a joint drive with
    /// zero stiffness and zero velocity target, and an appropriately dimensioned damping parameter.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut(self_: *mut PxArticulationJointReducedCoordinate, coefficient: f32);

    /// Gets the joint friction coefficient.
    ///
    /// The joint friction coefficient.
    pub fn PxArticulationJointReducedCoordinate_getFrictionCoefficient(self_: *const PxArticulationJointReducedCoordinate) -> f32;

    /// Sets the maximal joint velocity enforced for all axes.
    ///
    /// - The solver will apply appropriate joint-space impulses in order to enforce the per-axis joint-velocity limit.
    /// - The velocity units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut(self_: *mut PxArticulationJointReducedCoordinate, maxJointV: f32);

    /// Gets the maximal joint velocity enforced for all axes.
    ///
    /// The maximal per-axis joint velocity.
    pub fn PxArticulationJointReducedCoordinate_getMaxJointVelocity(self_: *const PxArticulationJointReducedCoordinate) -> f32;

    /// Sets the joint position for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointPosition to set joint positions in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, jointPos must be in range [-Pi, Pi].
    ///
    /// Joint position is specified in the parent frame of the joint. If Gp, Gc are the parent and child actor poses in the world frame and Lp, Lc are the parent and child joint frames expressed in the parent and child actor frames then the parent and child links will be given poses that obey Gp * Lp * J = Gc * Lc with J denoting the joint pose. For joints restricted to angular motion, J has the form PxTranfsorm(PxVec3(PxZero), PxExp(PxVec3(twistPos, swing1Pos, swing2Pos))).  For joints restricted to linear motion, J has the form PxTransform(PxVec3(xPos, yPos, zPos), PxQuat(PxIdentity)).
    ///
    /// For spherical joints with more than 1 degree of freedom, the input joint positions taken together can collectively represent a rotation of greater than Pi around a vector. When this happens the rotation that matches the joint positions is not the shortest path rotation.  The joint pose J that is the outcome of setting and applying the joint positions will always be the equivalent of the shortest path rotation.
    pub fn PxArticulationJointReducedCoordinate_setJointPosition_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, jointPos: f32);

    /// Gets the joint position for the given axis, i.e. joint degree of freedom (DOF).
    ///
    /// For performance, prefer PxArticulationCache::jointPosition to get joint positions in a batch query.
    ///
    /// The joint position in linear units (equivalent to scene units) for a translational axis, or radians for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationJointReducedCoordinate_getJointPosition(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> f32;

    /// Sets the joint velocity for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointVelocity to set joint velocities in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationJointReducedCoordinate_setJointVelocity_mut(self_: *mut PxArticulationJointReducedCoordinate, axis: PxArticulationAxis, jointVel: f32);

    /// Gets the joint velocity for the given axis.
    ///
    /// For performance, prefer PxArticulationCache::jointVelocity to get joint velocities in a batch query.
    ///
    /// The joint velocity in linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationJointReducedCoordinate_getJointVelocity(self_: *const PxArticulationJointReducedCoordinate, axis: PxArticulationAxis) -> f32;

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationJointReducedCoordinate_getConcreteTypeName(self_: *const PxArticulationJointReducedCoordinate) -> *const std::ffi::c_char;

    /// Decrements the reference count of a shape and releases it if the new reference count is zero.
    ///
    /// Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape
    /// created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
    /// a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
    /// and so when the shape is detached it will also be destroyed.
    pub fn PxShape_release_mut(self_: *mut PxShape);

    /// Adjust the geometry of the shape.
    ///
    /// The type of the passed in geometry must match the geometry type of the shape.
    ///
    /// It is not allowed to change the geometry type of a shape.
    ///
    /// This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    pub fn PxShape_setGeometry_mut(self_: *mut PxShape, geometry: *const PxGeometry);

    /// Retrieve a reference to the shape's geometry.
    ///
    /// The returned reference has the same lifetime as the PxShape it comes from.
    ///
    /// Reference to internal PxGeometry object.
    pub fn PxShape_getGeometry(self_: *const PxShape) -> *const PxGeometry;

    /// Retrieves the actor which this shape is associated with.
    ///
    /// The actor this shape is associated with, if it is an exclusive shape, else NULL
    pub fn PxShape_getActor(self_: *const PxShape) -> *mut PxRigidActor;

    /// Sets the pose of the shape in actor space, i.e. relative to the actors to which they are attached.
    ///
    /// This transformation is identity by default.
    ///
    /// The local pose is an attribute of the shape, and so will apply to all actors to which the shape is attached.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Note:
    /// Does not automatically update the inertia properties of the owning actor (if applicable); use the
    /// PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]() to do this.
    ///
    /// Default:
    /// the identity transform
    pub fn PxShape_setLocalPose_mut(self_: *mut PxShape, pose: *const PxTransform);

    /// Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.
    ///
    /// This transformation is identity by default.
    ///
    /// Pose of shape relative to the actor's frame.
    pub fn PxShape_getLocalPose(self_: *const PxShape) -> PxTransform;

    /// Sets the user definable collision filter data.
    ///
    /// Sleeping:
    /// Does wake up the actor if the filter data change causes a formerly suppressed
    /// collision pair to be enabled.
    ///
    /// Default:
    /// (0,0,0,0)
    pub fn PxShape_setSimulationFilterData_mut(self_: *mut PxShape, data: *const PxFilterData);

    /// Retrieves the shape's collision filter data.
    pub fn PxShape_getSimulationFilterData(self_: *const PxShape) -> PxFilterData;

    /// Sets the user definable query filter data.
    ///
    /// Default:
    /// (0,0,0,0)
    pub fn PxShape_setQueryFilterData_mut(self_: *mut PxShape, data: *const PxFilterData);

    /// Retrieves the shape's Query filter data.
    pub fn PxShape_getQueryFilterData(self_: *const PxShape) -> PxFilterData;

    /// Assigns material(s) to the shape. Will remove existing materials from the shape.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    pub fn PxShape_setMaterials_mut(self_: *mut PxShape, materials: *const *mut PxMaterial, materialCount: u16);

    /// Returns the number of materials assigned to the shape.
    ///
    /// You can use [`getMaterials`]() to retrieve the material pointers.
    ///
    /// Number of materials associated with this shape.
    pub fn PxShape_getNbMaterials(self_: *const PxShape) -> u16;

    /// Retrieve all the material pointers associated with the shape.
    ///
    /// You can retrieve the number of material pointers by calling [`getNbMaterials`]()
    ///
    /// Note: The returned data may contain invalid pointers if you release materials using [`PxMaterial::release`]().
    ///
    /// Number of material pointers written to the buffer.
    pub fn PxShape_getMaterials(self_: *const PxShape, userBuffer: *mut *mut PxMaterial, bufferSize: u32, startIndex: u32) -> u32;

    /// Retrieve material from given triangle index.
    ///
    /// The input index is the internal triangle index as used inside the SDK. This is the index
    /// returned to users by various SDK functions such as raycasts.
    ///
    /// This function is only useful for triangle meshes or heightfields, which have per-triangle
    /// materials. For other shapes or SDF triangle meshes, the function returns the single material
    /// associated with the shape, regardless of the index.
    ///
    /// Material from input triangle
    ///
    /// If faceIndex value of 0xFFFFffff is passed as an input for mesh and heightfield shapes, this function will issue a warning and return NULL.
    ///
    /// Scene queries set the value of PxQueryHit::faceIndex to 0xFFFFffff whenever it is undefined or does not apply.
    pub fn PxShape_getMaterialFromInternalFaceIndex(self_: *const PxShape, faceIndex: u32) -> *mut PxBaseMaterial;

    /// Sets the contact offset.
    ///
    /// Shapes whose distance is less than the sum of their contactOffset values will generate contacts. The contact offset must be positive and
    /// greater than the rest offset. Having a contactOffset greater than than the restOffset allows the collision detection system to
    /// predictively enforce the contact constraint even when the objects are slightly separated. This prevents jitter that would occur
    /// if the constraint were enforced only when shapes were within the rest distance.
    ///
    /// Default:
    /// 0.02f * PxTolerancesScale::length
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    pub fn PxShape_setContactOffset_mut(self_: *mut PxShape, contactOffset: f32);

    /// Retrieves the contact offset.
    ///
    /// The contact offset of the shape.
    pub fn PxShape_getContactOffset(self_: *const PxShape) -> f32;

    /// Sets the rest offset.
    ///
    /// Two shapes will come to rest at a distance equal to the sum of their restOffset values. If the restOffset is 0, they should converge to touching
    /// exactly.  Having a restOffset greater than zero is useful to have objects slide smoothly, so that they do not get hung up on irregularities of
    /// each others' surfaces.
    ///
    /// Default:
    /// 0.0f
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    pub fn PxShape_setRestOffset_mut(self_: *mut PxShape, restOffset: f32);

    /// Retrieves the rest offset.
    ///
    /// The rest offset of the shape.
    pub fn PxShape_getRestOffset(self_: *const PxShape) -> f32;

    /// Sets the density used to interact with fluids.
    ///
    /// To be physically accurate, the density of a rigid body should be computed as its mass divided by its volume. To
    /// simplify tuning the interaction of fluid and rigid bodies, the density for fluid can differ from the real density. This
    /// allows to create floating bodies, even if they are supposed to sink with their mass and volume.
    ///
    /// Default:
    /// 800.0f
    pub fn PxShape_setDensityForFluid_mut(self_: *mut PxShape, densityForFluid: f32);

    /// Retrieves the density used to interact with fluids.
    ///
    /// The density of the body when interacting with fluid.
    pub fn PxShape_getDensityForFluid(self_: *const PxShape) -> f32;

    /// Sets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// Default:
    /// 0.0
    pub fn PxShape_setTorsionalPatchRadius_mut(self_: *mut PxShape, radius: f32);

    /// Gets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// The torsional patch radius of the shape.
    pub fn PxShape_getTorsionalPatchRadius(self_: *const PxShape) -> f32;

    /// Sets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// Default:
    /// 0.0
    pub fn PxShape_setMinTorsionalPatchRadius_mut(self_: *mut PxShape, radius: f32);

    /// Gets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// The minimum torsional patch radius of the shape.
    pub fn PxShape_getMinTorsionalPatchRadius(self_: *const PxShape) -> f32;

    /// Sets shape flags
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Default:
    /// PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE
    pub fn PxShape_setFlag_mut(self_: *mut PxShape, flag: PxShapeFlag, value: bool);

    /// Sets shape flags
    pub fn PxShape_setFlags_mut(self_: *mut PxShape, inFlags: PxShapeFlags);

    /// Retrieves shape flags.
    ///
    /// The values of the shape flags.
    pub fn PxShape_getFlags(self_: *const PxShape) -> PxShapeFlags;

    /// Returns true if the shape is exclusive to an actor.
    pub fn PxShape_isExclusive(self_: *const PxShape) -> bool;

    /// Sets a name string for the object that can be retrieved with [`getName`]().
    ///
    /// This is for debugging and is not used by the SDK.
    /// The string is not copied by the SDK, only the pointer is stored.
    ///
    /// Default:
    /// NULL
    pub fn PxShape_setName_mut(self_: *mut PxShape, name: *const std::ffi::c_char);

    /// retrieves the name string set with setName().
    ///
    /// The name associated with the shape.
    pub fn PxShape_getName(self_: *const PxShape) -> *const std::ffi::c_char;

    pub fn PxShape_getConcreteTypeName(self_: *const PxShape) -> *const std::ffi::c_char;

    /// Deletes the rigid actor object.
    ///
    /// Also releases any shapes associated with the actor.
    ///
    /// Releasing an actor will affect any objects that are connected to the actor (constraint shaders like joints etc.).
    /// Such connected objects will be deleted upon scene deletion, or explicitly by the user by calling release()
    /// on these objects. It is recommended to always remove all objects that reference actors before the actors
    /// themselves are removed. It is not possible to retrieve list of dead connected objects.
    ///
    /// Sleeping:
    /// This call will awaken any sleeping actors contacting the deleted actor (directly or indirectly).
    ///
    /// Calls [`PxActor::release`]() so you might want to check the documentation of that method as well.
    pub fn PxRigidActor_release_mut(self_: *mut PxRigidActor);

    /// Returns the internal actor index.
    ///
    /// This is only defined for actors that have been added to a scene.
    ///
    /// The internal actor index, or 0xffffffff if the actor is not part of a scene.
    pub fn PxRigidActor_getInternalActorIndex(self_: *const PxRigidActor) -> u32;

    /// Retrieves the actors world space transform.
    ///
    /// The getGlobalPose() method retrieves the actor's current actor space to world space transformation.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// Global pose of object.
    pub fn PxRigidActor_getGlobalPose(self_: *const PxRigidActor) -> PxTransform;

    /// Method for setting an actor's pose in the world.
    ///
    /// This method instantaneously changes the actor space to world space transformation.
    ///
    /// This method is mainly for dynamic rigid bodies (see [`PxRigidDynamic`]). Calling this method on static actors is
    /// likely to result in a performance penalty, since internal optimization structures for static actors may need to be
    /// recomputed. In addition, moving static actors will not interact correctly with dynamic actors or joints.
    ///
    /// To directly control an actor's position and have it correctly interact with dynamic bodies and joints, create a dynamic
    /// body with the PxRigidBodyFlag::eKINEMATIC flag, then use the setKinematicTarget() commands to define its path.
    ///
    /// Even when moving dynamic actors, exercise restraint in making use of this method. Where possible, avoid:
    ///
    /// moving actors into other actors, thus causing overlap (an invalid physical state)
    ///
    /// moving an actor that is connected by a joint to another away from the other (thus causing joint error)
    ///
    /// Sleeping:
    /// This call wakes dynamic actors if they are sleeping and the autowake parameter is true (default).
    pub fn PxRigidActor_setGlobalPose_mut(self_: *mut PxRigidActor, pose: *const PxTransform, autowake: bool);

    /// Attach a shape to an actor
    ///
    /// This call will increment the reference count of the shape.
    ///
    /// Mass properties of dynamic rigid actors will not automatically be recomputed
    /// to reflect the new mass distribution implied by the shape. Follow this call with a call to
    /// the PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]() to do that.
    ///
    /// Attaching a triangle mesh, heightfield or plane geometry shape configured as eSIMULATION_SHAPE is not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// True if success.
    pub fn PxRigidActor_attachShape_mut(self_: *mut PxRigidActor, shape: *mut PxShape) -> bool;

    /// Detach a shape from an actor.
    ///
    /// This will also decrement the reference count of the PxShape, and if the reference count is zero, will cause it to be deleted.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxRigidActor_detachShape_mut(self_: *mut PxRigidActor, shape: *mut PxShape, wakeOnLostTouch: bool);

    /// Returns the number of shapes assigned to the actor.
    ///
    /// You can use [`getShapes`]() to retrieve the shape pointers.
    ///
    /// Number of shapes associated with this actor.
    pub fn PxRigidActor_getNbShapes(self_: *const PxRigidActor) -> u32;

    /// Retrieve all the shape pointers belonging to the actor.
    ///
    /// These are the shapes used by the actor for collision detection.
    ///
    /// You can retrieve the number of shape pointers by calling [`getNbShapes`]()
    ///
    /// Note: Removing shapes with [`PxShape::release`]() will invalidate the pointer of the released shape.
    ///
    /// Number of shape pointers written to the buffer.
    pub fn PxRigidActor_getShapes(self_: *const PxRigidActor, userBuffer: *mut *mut PxShape, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of constraint shaders attached to the actor.
    ///
    /// You can use [`getConstraints`]() to retrieve the constraint shader pointers.
    ///
    /// Number of constraint shaders attached to this actor.
    pub fn PxRigidActor_getNbConstraints(self_: *const PxRigidActor) -> u32;

    /// Retrieve all the constraint shader pointers belonging to the actor.
    ///
    /// You can retrieve the number of constraint shader pointers by calling [`getNbConstraints`]()
    ///
    /// Note: Removing constraint shaders with [`PxConstraint::release`]() will invalidate the pointer of the released constraint.
    ///
    /// Number of constraint shader pointers written to the buffer.
    pub fn PxRigidActor_getConstraints(self_: *const PxRigidActor, userBuffer: *mut *mut PxConstraint, bufferSize: u32, startIndex: u32) -> u32;

    pub fn PxNodeIndex_new(id: u32, articLinkId: u32) -> PxNodeIndex;

    pub fn PxNodeIndex_new_1(id: u32) -> PxNodeIndex;

    pub fn PxNodeIndex_index(self_: *const PxNodeIndex) -> u32;

    pub fn PxNodeIndex_articulationLinkId(self_: *const PxNodeIndex) -> u32;

    pub fn PxNodeIndex_isArticulation(self_: *const PxNodeIndex) -> u32;

    pub fn PxNodeIndex_isStaticBody(self_: *const PxNodeIndex) -> bool;

    pub fn PxNodeIndex_isValid(self_: *const PxNodeIndex) -> bool;

    pub fn PxNodeIndex_setIndices_mut(self_: *mut PxNodeIndex, index: u32, articLinkId: u32);

    pub fn PxNodeIndex_setIndices_mut_1(self_: *mut PxNodeIndex, index: u32);

    pub fn PxNodeIndex_getInd(self_: *const PxNodeIndex) -> u64;

    /// Sets the pose of the center of mass relative to the actor.
    ///
    /// Changing this transform will not move the actor in the world!
    ///
    /// Setting an unrealistic center of mass which is a long way from the body can make it difficult for
    /// the SDK to solve constraints. Perhaps leading to instability and jittering bodies.
    ///
    /// Default:
    /// the identity transform
    pub fn PxRigidBody_setCMassLocalPose_mut(self_: *mut PxRigidBody, pose: *const PxTransform);

    /// Retrieves the center of mass pose relative to the actor frame.
    ///
    /// The center of mass pose relative to the actor frame.
    pub fn PxRigidBody_getCMassLocalPose(self_: *const PxRigidBody) -> PxTransform;

    /// Sets the mass of a dynamic actor.
    ///
    /// The mass must be non-negative.
    ///
    /// setMass() does not update the inertial properties of the body, to change the inertia tensor
    /// use setMassSpaceInertiaTensor() or the PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]().
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic.
    ///
    /// Default:
    /// 1.0
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxRigidBody_setMass_mut(self_: *mut PxRigidBody, mass: f32);

    /// Retrieves the mass of the actor.
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// The mass of this actor.
    pub fn PxRigidBody_getMass(self_: *const PxRigidBody) -> f32;

    /// Retrieves the inverse mass of the actor.
    ///
    /// The inverse mass of this actor.
    pub fn PxRigidBody_getInvMass(self_: *const PxRigidBody) -> f32;

    /// Sets the inertia tensor, using a parameter specified in mass space coordinates.
    ///
    /// Note that such matrices are diagonal -- the passed vector is the diagonal.
    ///
    /// If you have a non diagonal world/actor space inertia tensor(3x3 matrix). Then you need to
    /// diagonalize it and set an appropriate mass space transform. See [`setCMassLocalPose`]().
    ///
    /// The inertia tensor elements must be non-negative.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic.
    ///
    /// Default:
    /// (1.0, 1.0, 1.0)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxRigidBody_setMassSpaceInertiaTensor_mut(self_: *mut PxRigidBody, m: *const PxVec3);

    /// Retrieves the diagonal inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inertia vector.
    ///
    /// The mass space inertia tensor of this actor.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    pub fn PxRigidBody_getMassSpaceInertiaTensor(self_: *const PxRigidBody) -> PxVec3;

    /// Retrieves the diagonal inverse inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inverse inertia vector.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// The mass space inverse inertia tensor of this actor.
    pub fn PxRigidBody_getMassSpaceInvInertiaTensor(self_: *const PxRigidBody) -> PxVec3;

    /// Sets the linear damping coefficient.
    ///
    /// Zero represents no damping. The damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.0
    pub fn PxRigidBody_setLinearDamping_mut(self_: *mut PxRigidBody, linDamp: f32);

    /// Retrieves the linear damping coefficient.
    ///
    /// The linear damping coefficient associated with this actor.
    pub fn PxRigidBody_getLinearDamping(self_: *const PxRigidBody) -> f32;

    /// Sets the angular damping coefficient.
    ///
    /// Zero represents no damping.
    ///
    /// The angular damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.05
    pub fn PxRigidBody_setAngularDamping_mut(self_: *mut PxRigidBody, angDamp: f32);

    /// Retrieves the angular damping coefficient.
    ///
    /// The angular damping coefficient associated with this actor.
    pub fn PxRigidBody_getAngularDamping(self_: *const PxRigidBody) -> f32;

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    pub fn PxRigidBody_getLinearVelocity(self_: *const PxRigidBody) -> PxVec3;

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    pub fn PxRigidBody_getAngularVelocity(self_: *const PxRigidBody) -> PxVec3;

    /// Lets you set the maximum linear velocity permitted for this actor.
    ///
    /// With this function, you can set the  maximum linear velocity permitted for this rigid body.
    /// Higher angular velocities are clamped to this value.
    ///
    /// Note: The angular velocity is clamped to the set value
    /// before
    /// the solver, which means that
    /// the limit may still be momentarily exceeded.
    ///
    /// Default:
    /// PX_MAX_F32
    pub fn PxRigidBody_setMaxLinearVelocity_mut(self_: *mut PxRigidBody, maxLinVel: f32);

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    pub fn PxRigidBody_getMaxLinearVelocity(self_: *const PxRigidBody) -> f32;

    /// Lets you set the maximum angular velocity permitted for this actor.
    ///
    /// For various internal computations, very quickly rotating actors introduce error
    /// into the simulation, which leads to undesired results.
    ///
    /// With this function, you can set the  maximum angular velocity permitted for this rigid body.
    /// Higher angular velocities are clamped to this value.
    ///
    /// Note: The angular velocity is clamped to the set value
    /// before
    /// the solver, which means that
    /// the limit may still be momentarily exceeded.
    ///
    /// Default:
    /// 100.0
    pub fn PxRigidBody_setMaxAngularVelocity_mut(self_: *mut PxRigidBody, maxAngVel: f32);

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
    pub fn PxRigidBody_getMaxAngularVelocity(self_: *const PxRigidBody) -> f32;

    /// Applies a force (or impulse) defined in the global coordinate frame to the actor at its center of mass.
    ///
    /// This will not induce a torque
    /// .
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive.
    ///
    /// Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators and are just short hand for multiplying the vector parameter by inverse mass and then using PxForceMode::eACCELERATION and
    /// PxForceMode::eVELOCITY_CHANGE respectively.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// if this is called on an articulation link, only the link is updated, not the entire articulation.
    ///
    /// see [`PxRigidBodyExt::computeVelocityDeltaFromImpulse`] for details of how to compute the change in linear velocity that
    /// will arise from the application of an impulsive force, where an impulsive force is applied force multiplied by a timestep.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the force is non-zero.
    pub fn PxRigidBody_addForce_mut(self_: *mut PxRigidBody, force: *const PxVec3, mode: PxForceMode, autowake: bool);

    /// Applies an impulsive torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the torque is to be conventional or impulsive.
    ///
    /// Each actor has an angular acceleration and an angular velocity change accumulator which are directly modified using the modes
    /// PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE
    /// also modify these same accumulators and are just short hand for multiplying the vector parameter by inverse inertia and then
    /// using PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// if this called on an articulation link, only the link is updated, not the entire articulation.
    ///
    /// see [`PxRigidBodyExt::computeVelocityDeltaFromImpulse`] for details of how to compute the change in angular velocity that
    /// will arise from the application of an impulsive torque, where an impulsive torque is an applied torque multiplied by a timestep.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the torque is non-zero.
    pub fn PxRigidBody_addTorque_mut(self_: *mut PxRigidBody, torque: *const PxVec3, mode: PxForceMode, autowake: bool);

    /// Clears the accumulated forces (sets the accumulated force back to zero).
    ///
    /// Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators (see PxRigidBody::addForce() for details); therefore the effect of calling clearForce(PxForceMode::eFORCE) is equivalent to calling
    /// clearForce(PxForceMode::eACCELERATION), and the effect of calling clearForce(PxForceMode::eIMPULSE) is equivalent to calling
    /// clearForce(PxForceMode::eVELOCITY_CHANGE).
    ///
    /// ::PxForceMode determines if the cleared force is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    pub fn PxRigidBody_clearForce_mut(self_: *mut PxRigidBody, mode: PxForceMode);

    /// Clears the impulsive torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// Each actor has an angular acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators (see PxRigidBody::addTorque() for details); therefore the effect of calling clearTorque(PxForceMode::eFORCE) is equivalent to calling
    /// clearTorque(PxForceMode::eACCELERATION), and the effect of calling clearTorque(PxForceMode::eIMPULSE) is equivalent to calling
    /// clearTorque(PxForceMode::eVELOCITY_CHANGE).
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    pub fn PxRigidBody_clearTorque_mut(self_: *mut PxRigidBody, mode: PxForceMode);

    /// Sets the impulsive force and torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    pub fn PxRigidBody_setForceAndTorque_mut(self_: *mut PxRigidBody, force: *const PxVec3, torque: *const PxVec3, mode: PxForceMode);

    /// Raises or clears a particular rigid body flag.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// Default:
    /// no flags are set
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxRigidBody_setRigidBodyFlag_mut(self_: *mut PxRigidBody, flag: PxRigidBodyFlag, value: bool);

    pub fn PxRigidBody_setRigidBodyFlags_mut(self_: *mut PxRigidBody, inFlags: PxRigidBodyFlags);

    /// Reads the PxRigidBody flags.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// The values of the PxRigidBody flags.
    pub fn PxRigidBody_getRigidBodyFlags(self_: *const PxRigidBody) -> PxRigidBodyFlags;

    /// Sets the CCD minimum advance coefficient.
    ///
    /// The CCD minimum advance coefficient is a value in the range [0, 1] that is used to control the minimum amount of time a body is integrated when
    /// it has a CCD contact. The actual minimum amount of time that is integrated depends on various properties, including the relative speed and collision shapes
    /// of the bodies involved in the contact. From these properties, a numeric value is calculated that determines the maximum distance (and therefore maximum time)
    /// which these bodies could be integrated forwards that would ensure that these bodies did not pass through each-other. This value is then scaled by CCD minimum advance
    /// coefficient to determine the amount of time that will be consumed in the CCD pass.
    ///
    /// Things to consider:
    /// A large value (approaching 1) ensures that the objects will always advance some time. However, larger values increase the chances of objects gently drifting through each-other in
    /// scenes which the constraint solver can't converge, e.g. scenes where an object is being dragged through a wall with a constraint.
    /// A value of 0 ensures that the pair of objects stop at the exact time-of-impact and will not gently drift through each-other. However, with very small/thin objects initially in
    /// contact, this can lead to a large amount of time being dropped and increases the chances of jamming. Jamming occurs when the an object is persistently in contact with an object
    /// such that the time-of-impact is 0, which results in no time being advanced for those objects in that CCD pass.
    ///
    /// The chances of jamming can be reduced by increasing the number of CCD mass
    pub fn PxRigidBody_setMinCCDAdvanceCoefficient_mut(self_: *mut PxRigidBody, advanceCoefficient: f32);

    /// Gets the CCD minimum advance coefficient.
    ///
    /// The value of the CCD min advance coefficient.
    pub fn PxRigidBody_getMinCCDAdvanceCoefficient(self_: *const PxRigidBody) -> f32;

    /// Sets the maximum depenetration velocity permitted to be introduced by the solver.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    pub fn PxRigidBody_setMaxDepenetrationVelocity_mut(self_: *mut PxRigidBody, biasClamp: f32);

    /// Returns the maximum depenetration velocity the solver is permitted to introduced.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    ///
    /// The maximum penetration bias applied by the solver.
    pub fn PxRigidBody_getMaxDepenetrationVelocity(self_: *const PxRigidBody) -> f32;

    /// Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
    /// bodies will be the minimum of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
    /// by the value for the dynamic body.
    pub fn PxRigidBody_setMaxContactImpulse_mut(self_: *mut PxRigidBody, maxImpulse: f32);

    /// Returns the maximum impulse that may be applied at a contact.
    ///
    /// The maximum impulse that may be applied at a contact
    pub fn PxRigidBody_getMaxContactImpulse(self_: *const PxRigidBody) -> f32;

    /// Sets a distance scale whereby the angular influence of a contact on the normal constraint in a contact is
    /// zeroed if normal.cross(offset) falls below this tolerance. Rather than acting as an absolute value, this tolerance
    /// is scaled by the ratio rXn.dot(angVel)/normal.dot(linVel) such that contacts that have relatively larger angular velocity
    /// than linear normal velocity (e.g. rolling wheels) achieve larger slop values as the angular velocity increases.
    pub fn PxRigidBody_setContactSlopCoefficient_mut(self_: *mut PxRigidBody, slopCoefficient: f32);

    /// Returns the contact slop coefficient.
    ///
    /// The contact slop coefficient.
    pub fn PxRigidBody_getContactSlopCoefficient(self_: *const PxRigidBody) -> f32;

    /// Returns the island node index
    ///
    /// The island node index.
    pub fn PxRigidBody_getInternalIslandNodeIndex(self_: *const PxRigidBody) -> PxNodeIndex;

    /// Releases the link from the articulation.
    ///
    /// Only a leaf articulation link can be released.
    ///
    /// Releasing a link is not allowed while the articulation link is in a scene. In order to release a link,
    /// remove and then re-add the corresponding articulation to the scene.
    pub fn PxArticulationLink_release_mut(self_: *mut PxArticulationLink);

    /// Gets the articulation that the link is a part of.
    ///
    /// The articulation.
    pub fn PxArticulationLink_getArticulation(self_: *const PxArticulationLink) -> *mut PxArticulationReducedCoordinate;

    /// Gets the joint which connects this link to its parent.
    ///
    /// The joint connecting the link to the parent. NULL for the root link.
    pub fn PxArticulationLink_getInboundJoint(self_: *const PxArticulationLink) -> *mut PxArticulationJointReducedCoordinate;

    /// Gets the number of degrees of freedom of the joint which connects this link to its parent.
    ///
    /// - The root link DOF-count is defined to be 0 regardless of PxArticulationFlag::eFIX_BASE.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of degrees of freedom, or 0xFFFFFFFF if the articulation is not in a scene.
    pub fn PxArticulationLink_getInboundJointDof(self_: *const PxArticulationLink) -> u32;

    /// Gets the number of child links.
    ///
    /// The number of child links.
    pub fn PxArticulationLink_getNbChildren(self_: *const PxArticulationLink) -> u32;

    /// Gets the low-level link index that may be used to index into members of PxArticulationCache.
    ///
    /// The return value is only valid for articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
    pub fn PxArticulationLink_getLinkIndex(self_: *const PxArticulationLink) -> u32;

    /// Retrieves the child links.
    ///
    /// The number of articulation links written to the buffer.
    pub fn PxArticulationLink_getChildren(self_: *const PxArticulationLink, userBuffer: *mut *mut PxArticulationLink, bufferSize: u32, startIndex: u32) -> u32;

    /// Set the constraint-force-mixing scale term.
    ///
    /// The cfm scale term is a stabilization term that helps avoid instabilities with over-constrained
    /// configurations. It should be a small value that is multiplied by 1/mass internally to produce
    /// an additional bias added to the unit response term in the solver.
    ///
    /// Default:
    /// 0.025
    /// Range:
    /// [0, 1]
    ///
    /// This call is not allowed while the simulation is running.
    pub fn PxArticulationLink_setCfmScale_mut(self_: *mut PxArticulationLink, cfm: f32);

    /// Get the constraint-force-mixing scale term.
    ///
    /// The constraint-force-mixing scale term.
    pub fn PxArticulationLink_getCfmScale(self_: *const PxArticulationLink) -> f32;

    /// Get the linear velocity of the link.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The linear velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationLink_getLinearVelocity(self_: *const PxArticulationLink) -> PxVec3;

    /// Get the angular velocity of the link.
    ///
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The angular velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
    pub fn PxArticulationLink_getAngularVelocity(self_: *const PxArticulationLink) -> PxVec3;

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
    pub fn PxArticulationLink_getConcreteTypeName(self_: *const PxArticulationLink) -> *const std::ffi::c_char;

    pub fn PxConeLimitedConstraint_new() -> PxConeLimitedConstraint;

    /// Releases a PxConstraint instance.
    ///
    /// This call does not wake up the connected rigid bodies.
    pub fn PxConstraint_release_mut(self_: *mut PxConstraint);

    /// Retrieves the scene which this constraint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    pub fn PxConstraint_getScene(self_: *const PxConstraint) -> *mut PxScene;

    /// Retrieves the actors for this constraint.
    pub fn PxConstraint_getActors(self_: *const PxConstraint, actor0: *mut *mut PxRigidActor, actor1: *mut *mut PxRigidActor);

    /// Sets the actors for this constraint.
    pub fn PxConstraint_setActors_mut(self_: *mut PxConstraint, actor0: *mut PxRigidActor, actor1: *mut PxRigidActor);

    /// Notify the scene that the constraint shader data has been updated by the application
    pub fn PxConstraint_markDirty_mut(self_: *mut PxConstraint);

    /// Retrieve the flags for this constraint
    ///
    /// the constraint flags
    pub fn PxConstraint_getFlags(self_: *const PxConstraint) -> PxConstraintFlags;

    /// Set the flags for this constraint
    ///
    /// default: PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    pub fn PxConstraint_setFlags_mut(self_: *mut PxConstraint, flags: PxConstraintFlags);

    /// Set a flag for this constraint
    pub fn PxConstraint_setFlag_mut(self_: *mut PxConstraint, flag: PxConstraintFlag, value: bool);

    /// Retrieve the constraint force most recently applied to maintain this constraint.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    pub fn PxConstraint_getForce(self_: *const PxConstraint, linear: *mut PxVec3, angular: *mut PxVec3);

    /// whether the constraint is valid.
    ///
    /// A constraint is valid if it has at least one dynamic rigid body or articulation link. A constraint that
    /// is not valid may not be inserted into a scene, and therefore a static actor to which an invalid constraint
    /// is attached may not be inserted into a scene.
    ///
    /// Invalid constraints arise only when an actor to which the constraint is attached has been deleted.
    pub fn PxConstraint_isValid(self_: *const PxConstraint) -> bool;

    /// Set the break force and torque thresholds for this constraint.
    ///
    /// If either the force or torque measured at the constraint exceed these thresholds the constraint will break.
    pub fn PxConstraint_setBreakForce_mut(self_: *mut PxConstraint, linear: f32, angular: f32);

    /// Retrieve the constraint break force and torque thresholds
    pub fn PxConstraint_getBreakForce(self_: *const PxConstraint, linear: *mut f32, angular: *mut f32);

    /// Set the minimum response threshold for a constraint row
    ///
    /// When using mass modification for a joint or infinite inertia for a jointed body, very stiff solver constraints can be generated which
    /// can destabilize simulation. Setting this value to a small positive value (e.g. 1e-8) will cause constraint rows to be ignored if very
    /// large changes in impulses will generate only small changes in velocity. When setting this value, also set
    /// PxConstraintFlag::eDISABLE_PREPROCESSING. The solver accuracy for this joint may be reduced.
    pub fn PxConstraint_setMinResponseThreshold_mut(self_: *mut PxConstraint, threshold: f32);

    /// Retrieve the constraint break force and torque thresholds
    ///
    /// the minimum response threshold for a constraint row
    pub fn PxConstraint_getMinResponseThreshold(self_: *const PxConstraint) -> f32;

    /// Fetch external owner of the constraint.
    ///
    /// Provides a reference to the external owner of a constraint and a unique owner type ID.
    ///
    /// Reference to the external object which owns the constraint.
    pub fn PxConstraint_getExternalReference_mut(self_: *mut PxConstraint, typeID: *mut u32) -> *mut std::ffi::c_void;

    /// Set the constraint functions for this constraint
    pub fn PxConstraint_setConstraintFunctions_mut(self_: *mut PxConstraint, connector: *mut PxConstraintConnector, shaders: *const PxConstraintShaderTable);

    pub fn PxConstraint_getConcreteTypeName(self_: *const PxConstraint) -> *const std::ffi::c_char;

    /// Constructor
    pub fn PxContactStreamIterator_new(contactPatches: *const u8, contactPoints: *const u8, contactFaceIndices: *const u32, nbPatches: u32, nbContacts: u32) -> PxContactStreamIterator;

    /// Returns whether there are more patches in this stream.
    ///
    /// Whether there are more patches in this stream.
    pub fn PxContactStreamIterator_hasNextPatch(self_: *const PxContactStreamIterator) -> bool;

    /// Returns the total contact count.
    ///
    /// Total contact count.
    pub fn PxContactStreamIterator_getTotalContactCount(self_: *const PxContactStreamIterator) -> u32;

    /// Returns the total patch count.
    ///
    /// Total patch count.
    pub fn PxContactStreamIterator_getTotalPatchCount(self_: *const PxContactStreamIterator) -> u32;

    /// Advances iterator to next contact patch.
    pub fn PxContactStreamIterator_nextPatch_mut(self_: *mut PxContactStreamIterator);

    /// Returns if the current patch has more contacts.
    ///
    /// If there are more contacts in the current patch.
    pub fn PxContactStreamIterator_hasNextContact(self_: *const PxContactStreamIterator) -> bool;

    /// Advances to the next contact in the patch.
    pub fn PxContactStreamIterator_nextContact_mut(self_: *mut PxContactStreamIterator);

    /// Gets the current contact's normal
    ///
    /// The current contact's normal.
    pub fn PxContactStreamIterator_getContactNormal(self_: *const PxContactStreamIterator) -> *const PxVec3;

    /// Gets the inverse mass scale for body 0.
    ///
    /// The inverse mass scale for body 0.
    pub fn PxContactStreamIterator_getInvMassScale0(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the inverse mass scale for body 1.
    ///
    /// The inverse mass scale for body 1.
    pub fn PxContactStreamIterator_getInvMassScale1(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the inverse inertia scale for body 0.
    ///
    /// The inverse inertia scale for body 0.
    pub fn PxContactStreamIterator_getInvInertiaScale0(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the inverse inertia scale for body 1.
    ///
    /// The inverse inertia scale for body 1.
    pub fn PxContactStreamIterator_getInvInertiaScale1(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's max impulse.
    ///
    /// The contact's max impulse.
    pub fn PxContactStreamIterator_getMaxImpulse(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's target velocity.
    ///
    /// The contact's target velocity.
    pub fn PxContactStreamIterator_getTargetVel(self_: *const PxContactStreamIterator) -> *const PxVec3;

    /// Gets the contact's contact point.
    ///
    /// The contact's contact point.
    pub fn PxContactStreamIterator_getContactPoint(self_: *const PxContactStreamIterator) -> *const PxVec3;

    /// Gets the contact's separation.
    ///
    /// The contact's separation.
    pub fn PxContactStreamIterator_getSeparation(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's face index for shape 0.
    ///
    /// The contact's face index for shape 0.
    pub fn PxContactStreamIterator_getFaceIndex0(self_: *const PxContactStreamIterator) -> u32;

    /// Gets the contact's face index for shape 1.
    ///
    /// The contact's face index for shape 1.
    pub fn PxContactStreamIterator_getFaceIndex1(self_: *const PxContactStreamIterator) -> u32;

    /// Gets the contact's static friction coefficient.
    ///
    /// The contact's static friction coefficient.
    pub fn PxContactStreamIterator_getStaticFriction(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's dynamic friction coefficient.
    ///
    /// The contact's dynamic friction coefficient.
    pub fn PxContactStreamIterator_getDynamicFriction(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's restitution coefficient.
    ///
    /// The contact's restitution coefficient.
    pub fn PxContactStreamIterator_getRestitution(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's damping value.
    ///
    /// The contact's damping value.
    pub fn PxContactStreamIterator_getDamping(self_: *const PxContactStreamIterator) -> f32;

    /// Gets the contact's material flags.
    ///
    /// The contact's material flags.
    pub fn PxContactStreamIterator_getMaterialFlags(self_: *const PxContactStreamIterator) -> u32;

    /// Gets the contact's material index for shape 0.
    ///
    /// The contact's material index for shape 0.
    pub fn PxContactStreamIterator_getMaterialIndex0(self_: *const PxContactStreamIterator) -> u16;

    /// Gets the contact's material index for shape 1.
    ///
    /// The contact's material index for shape 1.
    pub fn PxContactStreamIterator_getMaterialIndex1(self_: *const PxContactStreamIterator) -> u16;

    /// Advances the contact stream iterator to a specific contact index.
    ///
    /// True if advancing was possible
    pub fn PxContactStreamIterator_advanceToIndex_mut(self_: *mut PxContactStreamIterator, initialIndex: u32) -> bool;

    /// Get the position of a specific contact point in the set.
    ///
    /// Position to the requested point in world space
    pub fn PxContactSet_getPoint(self_: *const PxContactSet, i: u32) -> *const PxVec3;

    /// Alter the position of a specific contact point in the set.
    pub fn PxContactSet_setPoint_mut(self_: *mut PxContactSet, i: u32, p: *const PxVec3);

    /// Get the contact normal of a specific contact point in the set.
    ///
    /// The requested normal in world space
    pub fn PxContactSet_getNormal(self_: *const PxContactSet, i: u32) -> *const PxVec3;

    /// Alter the contact normal of a specific contact point in the set.
    ///
    /// Changing the normal can cause contact points to be ignored.
    pub fn PxContactSet_setNormal_mut(self_: *mut PxContactSet, i: u32, n: *const PxVec3);

    /// Get the separation distance of a specific contact point in the set.
    ///
    /// The separation. Negative implies penetration.
    pub fn PxContactSet_getSeparation(self_: *const PxContactSet, i: u32) -> f32;

    /// Alter the separation of a specific contact point in the set.
    pub fn PxContactSet_setSeparation_mut(self_: *mut PxContactSet, i: u32, s: f32);

    /// Get the target velocity of a specific contact point in the set.
    ///
    /// The target velocity in world frame
    pub fn PxContactSet_getTargetVelocity(self_: *const PxContactSet, i: u32) -> *const PxVec3;

    /// Alter the target velocity of a specific contact point in the set.
    pub fn PxContactSet_setTargetVelocity_mut(self_: *mut PxContactSet, i: u32, v: *const PxVec3);

    /// Get the face index with respect to the first shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the first shape
    ///
    /// At the moment, the first shape is never a tri-mesh, therefore this function always returns PXC_CONTACT_NO_FACE_INDEX
    pub fn PxContactSet_getInternalFaceIndex0(self_: *const PxContactSet, i: u32) -> u32;

    /// Get the face index with respect to the second shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the second shape
    pub fn PxContactSet_getInternalFaceIndex1(self_: *const PxContactSet, i: u32) -> u32;

    /// Get the maximum impulse for a specific contact point in the set.
    ///
    /// The maximum impulse
    pub fn PxContactSet_getMaxImpulse(self_: *const PxContactSet, i: u32) -> f32;

    /// Alter the maximum impulse for a specific contact point in the set.
    ///
    /// Must be nonnegative. If set to zero, the contact point will be ignored
    pub fn PxContactSet_setMaxImpulse_mut(self_: *mut PxContactSet, i: u32, s: f32);

    /// Get the restitution coefficient for a specific contact point in the set.
    ///
    /// The restitution coefficient
    pub fn PxContactSet_getRestitution(self_: *const PxContactSet, i: u32) -> f32;

    /// Alter the restitution coefficient for a specific contact point in the set.
    ///
    /// Valid ranges [0,1]
    pub fn PxContactSet_setRestitution_mut(self_: *mut PxContactSet, i: u32, r: f32);

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient (dimensionless)
    pub fn PxContactSet_getStaticFriction(self_: *const PxContactSet, i: u32) -> f32;

    /// Alter the static friction coefficient for a specific contact point in the set.
    pub fn PxContactSet_setStaticFriction_mut(self_: *mut PxContactSet, i: u32, f: f32);

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient
    pub fn PxContactSet_getDynamicFriction(self_: *const PxContactSet, i: u32) -> f32;

    /// Alter the static dynamic coefficient for a specific contact point in the set.
    pub fn PxContactSet_setDynamicFriction_mut(self_: *mut PxContactSet, i: u32, f: f32);

    /// Ignore the contact point.
    ///
    /// If a contact point is ignored then no force will get applied at this point. This can be used to disable collision in certain areas of a shape, for example.
    pub fn PxContactSet_ignore_mut(self_: *mut PxContactSet, i: u32);

    /// The number of contact points in the set.
    pub fn PxContactSet_size(self_: *const PxContactSet) -> u32;

    /// Returns the invMassScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    pub fn PxContactSet_getInvMassScale0(self_: *const PxContactSet) -> f32;

    /// Returns the invMassScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    pub fn PxContactSet_getInvMassScale1(self_: *const PxContactSet) -> f32;

    /// Returns the invInertiaScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    pub fn PxContactSet_getInvInertiaScale0(self_: *const PxContactSet) -> f32;

    /// Returns the invInertiaScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    pub fn PxContactSet_getInvInertiaScale1(self_: *const PxContactSet) -> f32;

    /// Sets the invMassScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    pub fn PxContactSet_setInvMassScale0_mut(self_: *mut PxContactSet, scale: f32);

    /// Sets the invMassScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
    pub fn PxContactSet_setInvMassScale1_mut(self_: *mut PxContactSet, scale: f32);

    /// Sets the invInertiaScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    pub fn PxContactSet_setInvInertiaScale0_mut(self_: *mut PxContactSet, scale: f32);

    /// Sets the invInertiaScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
    pub fn PxContactSet_setInvInertiaScale1_mut(self_: *mut PxContactSet, scale: f32);

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    pub fn PxContactModifyCallback_onContactModify_mut(self_: *mut PxContactModifyCallback, pairs: *mut PxContactModifyPair, count: u32);

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
    pub fn PxCCDContactModifyCallback_onCCDContactModify_mut(self_: *mut PxCCDContactModifyCallback, pairs: *mut PxContactModifyPair, count: u32);

    /// Notification if an object or its memory gets released
    ///
    /// If release() gets called on a PxBase object, an eUSER_RELEASE event will get fired immediately. The object state can be queried in the callback but
    /// it is not allowed to change the state. Furthermore, when reading from the object it is the user's responsibility to make sure that no other thread
    /// is writing at the same time to the object (this includes the simulation itself, i.e., [`PxScene::fetchResults`]() must not get called at the same time).
    ///
    /// Calling release() on a PxBase object does not necessarily trigger its destructor immediately. For example, the object can be shared and might still
    /// be referenced by other objects or the simulation might still be running and accessing the object state. In such cases the destructor will be called
    /// as soon as it is safe to do so. After the destruction of the object and its memory, an eMEMORY_RELEASE event will get fired. In this case it is not
    /// allowed to dereference the object pointer in the callback.
    pub fn PxDeletionListener_onRelease_mut(self_: *mut PxDeletionListener, observed: *const PxBase, userData: *mut std::ffi::c_void, deletionEvent: PxDeletionEventFlag);

    pub fn PxBaseMaterial_isKindOf(self_: *const PxBaseMaterial, name: *const std::ffi::c_char) -> bool;

    /// Sets young's modulus which defines the body's stiffness
    pub fn PxFEMMaterial_setYoungsModulus_mut(self_: *mut PxFEMMaterial, young: f32);

    /// Retrieves the young's modulus value.
    ///
    /// The young's modulus value.
    pub fn PxFEMMaterial_getYoungsModulus(self_: *const PxFEMMaterial) -> f32;

    /// Sets the Poisson's ratio which defines the body's volume preservation. Completely incompressible materials have a poisson ratio of 0.5. Its value should not be set to exactly 0.5 because this leads to numerical problems.
    pub fn PxFEMMaterial_setPoissons_mut(self_: *mut PxFEMMaterial, poisson: f32);

    /// Retrieves the Poisson's ratio.
    ///
    /// The Poisson's ratio.
    pub fn PxFEMMaterial_getPoissons(self_: *const PxFEMMaterial) -> f32;

    /// Sets the dynamic friction value which defines the strength of resistance when two objects slide relative to each other while in contact.
    pub fn PxFEMMaterial_setDynamicFriction_mut(self_: *mut PxFEMMaterial, dynamicFriction: f32);

    /// Retrieves the dynamic friction value
    ///
    /// The dynamic friction value
    pub fn PxFEMMaterial_getDynamicFriction(self_: *const PxFEMMaterial) -> f32;

    pub fn PxFilterData_new(anon_param0: PxEMPTY) -> PxFilterData;

    /// Default constructor.
    pub fn PxFilterData_new_1() -> PxFilterData;

    /// Constructor to set filter data initially.
    pub fn PxFilterData_new_2(w0: u32, w1: u32, w2: u32, w3: u32) -> PxFilterData;

    /// (re)sets the structure to the default.
    pub fn PxFilterData_setToDefault_mut(self_: *mut PxFilterData);

    /// Extract filter object type from the filter attributes of a collision pair object
    ///
    /// The type of the collision pair object.
    pub fn phys_PxGetFilterObjectType(attr: u32) -> PxFilterObjectType;

    /// Specifies whether the collision object belongs to a kinematic rigid body
    ///
    /// True if the object belongs to a kinematic rigid body, else false
    pub fn phys_PxFilterObjectIsKinematic(attr: u32) -> bool;

    /// Specifies whether the collision object is a trigger shape
    ///
    /// True if the object is a trigger shape, else false
    pub fn phys_PxFilterObjectIsTrigger(attr: u32) -> bool;

    /// Filter method to specify how a pair of potentially colliding objects should be processed.
    ///
    /// This method gets called when the filter flags returned by the filter shader (see [`PxSimulationFilterShader`])
    /// indicate that the filter callback should be invoked ([`PxFilterFlag::eCALLBACK`] or #PxFilterFlag::eNOTIFY set).
    /// Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given
    /// collision pair.
    ///
    /// Filter flags defining whether the pair should be discarded, temporarily ignored or processed and whether the pair
    /// should be tracked and send a report on pair deletion through the filter callback
    pub fn PxSimulationFilterCallback_pairFound_mut(self_: *mut PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: PxFilterData, a0: *const PxActor, s0: *const PxShape, attributes1: u32, filterData1: PxFilterData, a1: *const PxActor, s1: *const PxShape, pairFlags: *mut PxPairFlags) -> PxFilterFlags;

    /// Callback to inform that a tracked collision pair is gone.
    ///
    /// This method gets called when a collision pair disappears or gets re-filtered. Only applies to
    /// collision pairs which have been marked as filter callback pairs ([`PxFilterFlag::eNOTIFY`] set in #pairFound()).
    pub fn PxSimulationFilterCallback_pairLost_mut(self_: *mut PxSimulationFilterCallback, pairID: u32, attributes0: u32, filterData0: PxFilterData, attributes1: u32, filterData1: PxFilterData, objectRemoved: bool);

    /// Callback to give the opportunity to change the filter state of a tracked collision pair.
    ///
    /// This method gets called once per simulation step to let the application change the filter and pair
    /// flags of a collision pair that has been reported in [`pairFound`]() and requested callbacks by
    /// setting [`PxFilterFlag::eNOTIFY`]. To request a change of filter status, the target pair has to be
    /// specified by its ID, the new filter and pair flags have to be provided and the method should return true.
    ///
    /// If this method changes the filter status of a collision pair and the pair should keep being tracked
    /// by the filter callbacks then [`PxFilterFlag::eNOTIFY`] has to be set.
    ///
    /// The application is responsible to ensure that this method does not get called for pairs that have been
    /// reported as lost, see [`pairLost`]().
    ///
    /// True if the changes should be applied. In this case the method will get called again. False if
    /// no more status changes should be done in the current simulation step. In that case the provided flags will be discarded.
    pub fn PxSimulationFilterCallback_statusChange_mut(self_: *mut PxSimulationFilterCallback, pairID: *mut u32, pairFlags: *mut PxPairFlags, filterFlags: *mut PxFilterFlags) -> bool;

    /// Any combination of PxDataAccessFlag::eREADABLE and PxDataAccessFlag::eWRITABLE
    pub fn PxLockedData_getDataAccessFlags_mut(self_: *mut PxLockedData) -> PxDataAccessFlags;

    /// Unlocks the bulk data.
    pub fn PxLockedData_unlock_mut(self_: *mut PxLockedData);

    /// virtual destructor
    pub fn PxLockedData_delete(self_: *mut PxLockedData);

    /// Sets the coefficient of dynamic friction.
    ///
    /// The coefficient of dynamic friction should be in [0, PX_MAX_F32). If set to greater than staticFriction, the effective value of staticFriction will be increased to match.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setDynamicFriction_mut(self_: *mut PxMaterial, coef: f32);

    /// Retrieves the DynamicFriction value.
    ///
    /// The coefficient of dynamic friction.
    pub fn PxMaterial_getDynamicFriction(self_: *const PxMaterial) -> f32;

    /// Sets the coefficient of static friction
    ///
    /// The coefficient of static friction should be in the range [0, PX_MAX_F32)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setStaticFriction_mut(self_: *mut PxMaterial, coef: f32);

    /// Retrieves the coefficient of static friction.
    ///
    /// The coefficient of static friction.
    pub fn PxMaterial_getStaticFriction(self_: *const PxMaterial) -> f32;

    /// Sets the coefficient of restitution
    ///
    /// A coefficient of 0 makes the object bounce as little as possible, higher values up to 1.0 result in more bounce.
    ///
    /// This property is overloaded when PxMaterialFlag::eCOMPLIANT_CONTACT flag is enabled. This permits negative values for restitution to be provided.
    /// The negative values are converted into spring stiffness terms for an implicit spring simulated at the contact site, with the spring positional error defined by
    /// the contact separation value. Higher stiffness terms produce stiffer springs that behave more like a rigid contact.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setRestitution_mut(self_: *mut PxMaterial, rest: f32);

    /// Retrieves the coefficient of restitution.
    ///
    /// See [`setRestitution`].
    ///
    /// The coefficient of restitution.
    pub fn PxMaterial_getRestitution(self_: *const PxMaterial) -> f32;

    /// Sets the coefficient of damping
    ///
    /// This property only affects the simulation if PxMaterialFlag::eCOMPLIANT_CONTACT is raised.
    /// Damping works together with spring stiffness (set through a negative restitution value). Spring stiffness corrects positional error while
    /// damping resists relative velocity. Setting a high damping coefficient can produce spongy contacts.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setDamping_mut(self_: *mut PxMaterial, damping: f32);

    /// Retrieves the coefficient of damping.
    ///
    /// See [`setDamping`].
    ///
    /// The coefficient of damping.
    pub fn PxMaterial_getDamping(self_: *const PxMaterial) -> f32;

    /// Raises or clears a particular material flag.
    ///
    /// See the list of flags [`PxMaterialFlag`]
    ///
    /// Default:
    /// eIMPROVED_PATCH_FRICTION
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setFlag_mut(self_: *mut PxMaterial, flag: PxMaterialFlag, b: bool);

    /// sets all the material flags.
    ///
    /// See the list of flags [`PxMaterialFlag`]
    ///
    /// Default:
    /// eIMPROVED_PATCH_FRICTION
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setFlags_mut(self_: *mut PxMaterial, flags: PxMaterialFlags);

    /// Retrieves the flags. See [`PxMaterialFlag`].
    ///
    /// The material flags.
    pub fn PxMaterial_getFlags(self_: *const PxMaterial) -> PxMaterialFlags;

    /// Sets the friction combine mode.
    ///
    /// See the enum ::PxCombineMode .
    ///
    /// Default:
    /// PxCombineMode::eAVERAGE
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setFrictionCombineMode_mut(self_: *mut PxMaterial, combMode: PxCombineMode);

    /// Retrieves the friction combine mode.
    ///
    /// See [`setFrictionCombineMode`].
    ///
    /// The friction combine mode for this material.
    pub fn PxMaterial_getFrictionCombineMode(self_: *const PxMaterial) -> PxCombineMode;

    /// Sets the restitution combine mode.
    ///
    /// See the enum ::PxCombineMode .
    ///
    /// Default:
    /// PxCombineMode::eAVERAGE
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
    pub fn PxMaterial_setRestitutionCombineMode_mut(self_: *mut PxMaterial, combMode: PxCombineMode);

    /// Retrieves the restitution combine mode.
    ///
    /// See [`setRestitutionCombineMode`].
    ///
    /// The coefficient of restitution combine mode for this material.
    pub fn PxMaterial_getRestitutionCombineMode(self_: *const PxMaterial) -> PxCombineMode;

    pub fn PxMaterial_getConcreteTypeName(self_: *const PxMaterial) -> *const std::ffi::c_char;

    /// Construct parameters with default values.
    pub fn PxDiffuseParticleParams_new() -> PxDiffuseParticleParams;

    /// (re)sets the structure to the default.
    pub fn PxDiffuseParticleParams_setToDefault_mut(self_: *mut PxDiffuseParticleParams);

    /// Sets friction
    pub fn PxParticleMaterial_setFriction_mut(self_: *mut PxParticleMaterial, friction: f32);

    /// Retrieves the friction value.
    ///
    /// The friction value.
    pub fn PxParticleMaterial_getFriction(self_: *const PxParticleMaterial) -> f32;

    /// Sets velocity damping term
    pub fn PxParticleMaterial_setDamping_mut(self_: *mut PxParticleMaterial, damping: f32);

    /// Retrieves the velocity damping term
    ///
    /// The velocity damping term.
    pub fn PxParticleMaterial_getDamping(self_: *const PxParticleMaterial) -> f32;

    /// Sets adhesion term
    pub fn PxParticleMaterial_setAdhesion_mut(self_: *mut PxParticleMaterial, adhesion: f32);

    /// Retrieves the adhesion term
    ///
    /// The adhesion term.
    pub fn PxParticleMaterial_getAdhesion(self_: *const PxParticleMaterial) -> f32;

    /// Sets gravity scale term
    pub fn PxParticleMaterial_setGravityScale_mut(self_: *mut PxParticleMaterial, scale: f32);

    /// Retrieves the gravity scale term
    ///
    /// The gravity scale term.
    pub fn PxParticleMaterial_getGravityScale(self_: *const PxParticleMaterial) -> f32;

    /// Sets material adhesion radius scale. This is multiplied by the particle rest offset to compute the fall-off distance
    /// at which point adhesion ceases to operate.
    pub fn PxParticleMaterial_setAdhesionRadiusScale_mut(self_: *mut PxParticleMaterial, scale: f32);

    /// Retrieves the adhesion radius scale.
    ///
    /// The adhesion radius scale.
    pub fn PxParticleMaterial_getAdhesionRadiusScale(self_: *const PxParticleMaterial) -> f32;

    /// Destroys the instance it is called on.
    ///
    /// Use this release method to destroy an instance of this class. Be sure
    /// to not keep a reference to this object after calling release.
    /// Avoid release calls while a scene is simulating (in between simulate() and fetchResults() calls).
    ///
    /// Note that this must be called once for each prior call to PxCreatePhysics, as
    /// there is a reference counter. Also note that you mustn't destroy the PxFoundation instance (holding the allocator, error callback etc.)
    /// until after the reference count reaches 0 and the SDK is actually removed.
    ///
    /// Releasing an SDK will also release any objects created through it (scenes, triangle meshes, convex meshes, heightfields, shapes etc.),
    /// provided the user hasn't already done so.
    ///
    /// Releasing the PxPhysics instance is a prerequisite to releasing the PxFoundation instance.
    pub fn PxPhysics_release_mut(self_: *mut PxPhysics);

    /// Retrieves the Foundation instance.
    ///
    /// A reference to the Foundation object.
    pub fn PxPhysics_getFoundation_mut(self_: *mut PxPhysics) -> *mut PxFoundation;

    /// Creates an aggregate with the specified maximum size and filtering hint.
    ///
    /// The previous API used "bool enableSelfCollision" which should now silently evaluates
    /// to a PxAggregateType::eGENERIC aggregate with its self-collision bit.
    ///
    /// Use PxAggregateType::eSTATIC or PxAggregateType::eKINEMATIC for aggregates that will
    /// only contain static or kinematic actors. This provides faster filtering when used in
    /// combination with PxPairFilteringMode.
    ///
    /// The new aggregate.
    pub fn PxPhysics_createAggregate_mut(self_: *mut PxPhysics, maxActor: u32, maxShape: u32, filterHint: u32) -> *mut PxAggregate;

    /// Returns the simulation tolerance parameters.
    ///
    /// The current simulation tolerance parameters.
    pub fn PxPhysics_getTolerancesScale(self_: *const PxPhysics) -> *const PxTolerancesScale;

    /// Creates a triangle mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new triangle mesh.
    pub fn PxPhysics_createTriangleMesh_mut(self_: *mut PxPhysics, stream: *mut PxInputStream) -> *mut PxTriangleMesh;

    /// Return the number of triangle meshes that currently exist.
    ///
    /// Number of triangle meshes.
    pub fn PxPhysics_getNbTriangleMeshes(self_: *const PxPhysics) -> u32;

    /// Writes the array of triangle mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the triangle meshes in the array is not specified.
    ///
    /// The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getTriangleMeshes(self_: *const PxPhysics, userBuffer: *mut *mut PxTriangleMesh, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a tetrahedron mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new tetrahedron mesh.
    pub fn PxPhysics_createTetrahedronMesh_mut(self_: *mut PxPhysics, stream: *mut PxInputStream) -> *mut PxTetrahedronMesh;

    /// Creates a softbody mesh object.
    ///
    /// The new softbody mesh.
    pub fn PxPhysics_createSoftBodyMesh_mut(self_: *mut PxPhysics, stream: *mut PxInputStream) -> *mut PxSoftBodyMesh;

    /// Return the number of tetrahedron meshes that currently exist.
    ///
    /// Number of tetrahedron meshes.
    pub fn PxPhysics_getNbTetrahedronMeshes(self_: *const PxPhysics) -> u32;

    /// Writes the array of tetrahedron mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the tetrahedron meshes in the array is not specified.
    ///
    /// The number of tetrahedron mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getTetrahedronMeshes(self_: *const PxPhysics, userBuffer: *mut *mut PxTetrahedronMesh, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a heightfield object from previously cooked stream.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new heightfield.
    pub fn PxPhysics_createHeightField_mut(self_: *mut PxPhysics, stream: *mut PxInputStream) -> *mut PxHeightField;

    /// Return the number of heightfields that currently exist.
    ///
    /// Number of heightfields.
    pub fn PxPhysics_getNbHeightFields(self_: *const PxPhysics) -> u32;

    /// Writes the array of heightfield pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the heightfields in the array is not specified.
    ///
    /// The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getHeightFields(self_: *const PxPhysics, userBuffer: *mut *mut PxHeightField, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a convex mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new convex mesh.
    pub fn PxPhysics_createConvexMesh_mut(self_: *mut PxPhysics, stream: *mut PxInputStream) -> *mut PxConvexMesh;

    /// Return the number of convex meshes that currently exist.
    ///
    /// Number of convex meshes.
    pub fn PxPhysics_getNbConvexMeshes(self_: *const PxPhysics) -> u32;

    /// Writes the array of convex mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the convex meshes in the array is not specified.
    ///
    /// The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getConvexMeshes(self_: *const PxPhysics, userBuffer: *mut *mut PxConvexMesh, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a bounding volume hierarchy.
    ///
    /// The new BVH.
    pub fn PxPhysics_createBVH_mut(self_: *mut PxPhysics, stream: *mut PxInputStream) -> *mut PxBVH;

    /// Return the number of bounding volume hierarchies that currently exist.
    ///
    /// Number of bounding volume hierarchies.
    pub fn PxPhysics_getNbBVHs(self_: *const PxPhysics) -> u32;

    /// Writes the array of bounding volume hierarchy pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the BVHs in the array is not specified.
    ///
    /// The number of BVH pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getBVHs(self_: *const PxPhysics, userBuffer: *mut *mut PxBVH, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a scene.
    ///
    /// Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
    /// number of scenes that can be created.
    ///
    /// The new scene object.
    pub fn PxPhysics_createScene_mut(self_: *mut PxPhysics, sceneDesc: *const PxSceneDesc) -> *mut PxScene;

    /// Gets number of created scenes.
    ///
    /// The number of scenes created.
    pub fn PxPhysics_getNbScenes(self_: *const PxPhysics) -> u32;

    /// Writes the array of scene pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the scene pointers in the array is not specified.
    ///
    /// The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getScenes(self_: *const PxPhysics, userBuffer: *mut *mut PxScene, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a static rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    pub fn PxPhysics_createRigidStatic_mut(self_: *mut PxPhysics, pose: *const PxTransform) -> *mut PxRigidStatic;

    /// Creates a dynamic rigid actor with the specified pose and all other fields initialized
    /// to their default values.
    pub fn PxPhysics_createRigidDynamic_mut(self_: *mut PxPhysics, pose: *const PxTransform) -> *mut PxRigidDynamic;

    /// Creates a pruning structure from actors.
    ///
    /// Every provided actor needs at least one shape with the eSCENE_QUERY_SHAPE flag set.
    ///
    /// Both static and dynamic actors can be provided.
    ///
    /// It is not allowed to pass in actors which are already part of a scene.
    ///
    /// Articulation links cannot be provided.
    ///
    /// Pruning structure created from given actors, or NULL if any of the actors did not comply with the above requirements.
    pub fn PxPhysics_createPruningStructure_mut(self_: *mut PxPhysics, actors: *const *mut PxRigidActor, nbActors: u32) -> *mut PxPruningStructure;

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    pub fn PxPhysics_createShape_mut(self_: *mut PxPhysics, geometry: *const PxGeometry, material: *const PxMaterial, isExclusive: bool, shapeFlags: PxShapeFlags) -> *mut PxShape;

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    ///
    /// Shapes created from *SDF* triangle-mesh geometries do not support more than one material.
    pub fn PxPhysics_createShape_mut_1(self_: *mut PxPhysics, geometry: *const PxGeometry, materials: *const *mut PxMaterial, materialCount: u16, isExclusive: bool, shapeFlags: PxShapeFlags) -> *mut PxShape;

    /// Return the number of shapes that currently exist.
    ///
    /// Number of shapes.
    pub fn PxPhysics_getNbShapes(self_: *const PxPhysics) -> u32;

    /// Writes the array of shape pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the shapes in the array is not specified.
    ///
    /// The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getShapes(self_: *const PxPhysics, userBuffer: *mut *mut PxShape, bufferSize: u32, startIndex: u32) -> u32;

    /// Creates a constraint shader.
    ///
    /// A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
    /// be NULL to denote attachment to the world.
    ///
    /// The new constraint shader.
    pub fn PxPhysics_createConstraint_mut(self_: *mut PxPhysics, actor0: *mut PxRigidActor, actor1: *mut PxRigidActor, connector: *mut PxConstraintConnector, shaders: *const PxConstraintShaderTable, dataSize: u32) -> *mut PxConstraint;

    /// Creates a reduced-coordinate articulation with all fields initialized to their default values.
    ///
    /// the new articulation
    pub fn PxPhysics_createArticulationReducedCoordinate_mut(self_: *mut PxPhysics) -> *mut PxArticulationReducedCoordinate;

    /// Creates a new rigid body material with certain default properties.
    ///
    /// The new rigid body material.
    pub fn PxPhysics_createMaterial_mut(self_: *mut PxPhysics, staticFriction: f32, dynamicFriction: f32, restitution: f32) -> *mut PxMaterial;

    /// Return the number of rigid body materials that currently exist.
    ///
    /// Number of rigid body materials.
    pub fn PxPhysics_getNbMaterials(self_: *const PxPhysics) -> u32;

    /// Writes the array of rigid body material pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the materials in the array is not specified.
    ///
    /// The number of material pointers written to userBuffer, this should be less or equal to bufferSize.
    pub fn PxPhysics_getMaterials(self_: *const PxPhysics, userBuffer: *mut *mut PxMaterial, bufferSize: u32, startIndex: u32) -> u32;

    /// Register a deletion listener. Listeners will be called whenever an object is deleted.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    ///
    /// By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use [`registerDeletionListenerObjects`] to restrict the received events to specific objects.
    ///
    /// The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.
    pub fn PxPhysics_registerDeletionListener_mut(self_: *mut PxPhysics, observer: *mut PxDeletionListener, deletionEvents: *const PxDeletionEventFlags, restrictedObjectSet: bool);

    /// Unregister a deletion listener.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    pub fn PxPhysics_unregisterDeletionListener_mut(self_: *mut PxPhysics, observer: *mut PxDeletionListener);

    /// Register specific objects for deletion events.
    ///
    /// This method allows for a deletion listener to limit deletion events to specific objects only.
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    pub fn PxPhysics_registerDeletionListenerObjects_mut(self_: *mut PxPhysics, observer: *mut PxDeletionListener, observables: *const *const PxBase, observableCount: u32);

    /// Unregister specific objects for deletion events.
    ///
    /// This method allows to clear previously registered objects for a deletion listener (see [`registerDeletionListenerObjects`]()).
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
    pub fn PxPhysics_unregisterDeletionListenerObjects_mut(self_: *mut PxPhysics, observer: *mut PxDeletionListener, observables: *const *const PxBase, observableCount: u32);

    /// Gets PxPhysics object insertion interface.
    ///
    /// The insertion interface is needed for PxCreateTriangleMesh, PxCooking::createTriangleMesh etc., this allows runtime mesh creation.
    pub fn PxPhysics_getPhysicsInsertionCallback_mut(self_: *mut PxPhysics) -> *mut PxInsertionCallback;

    /// Creates an instance of the physics SDK.
    ///
    /// Creates an instance of this class. May not be a class member to avoid name mangling.
    /// Pass the constant [`PX_PHYSICS_VERSION`] as the argument.
    /// There may be only one instance of this class per process. Calling this method after an instance
    /// has been created already will result in an error message and NULL will be returned.
    ///
    /// Calling this will register all optional code modules (Articulations and HeightFields), preparing them for use.
    /// If you do not need some of these modules, consider calling PxCreateBasePhysics() instead and registering needed
    /// modules manually.
    ///
    /// PxPhysics instance on success, NULL if operation failed
    pub fn phys_PxCreatePhysics(version: u32, foundation: *mut PxFoundation, scale: *const PxTolerancesScale, trackOutstandingAllocations: bool, pvd: *mut PxPvd, omniPvd: *mut PxOmniPvd) -> *mut PxPhysics;

    pub fn phys_PxGetPhysics() -> *mut PxPhysics;

    pub fn PxActorShape_new() -> PxActorShape;

    pub fn PxActorShape_new_1(a: *mut PxRigidActor, s: *mut PxShape) -> PxActorShape;

    /// constructor sets to default
    pub fn PxQueryCache_new() -> PxQueryCache;

    /// constructor to set properties
    pub fn PxQueryCache_new_1(s: *mut PxShape, findex: u32) -> PxQueryCache;

    /// default constructor
    pub fn PxQueryFilterData_new() -> PxQueryFilterData;

    /// constructor to set both filter data and filter flags
    pub fn PxQueryFilterData_new_1(fd: *const PxFilterData, f: PxQueryFlags) -> PxQueryFilterData;

    /// constructor to set filter flags only
    pub fn PxQueryFilterData_new_2(f: PxQueryFlags) -> PxQueryFilterData;

    /// This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.
    ///
    /// the updated type for this hit  (see [`PxQueryHitType`])
    pub fn PxQueryFilterCallback_preFilter_mut(self_: *mut PxQueryFilterCallback, filterData: *const PxFilterData, shape: *const PxShape, actor: *const PxRigidActor, queryFlags: *mut PxHitFlags) -> PxQueryHitType;

    /// This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.
    ///
    /// the updated hit type for this hit  (see [`PxQueryHitType`])
    pub fn PxQueryFilterCallback_postFilter_mut(self_: *mut PxQueryFilterCallback, filterData: *const PxFilterData, hit: *const PxQueryHit, shape: *const PxShape, actor: *const PxRigidActor) -> PxQueryHitType;

    /// virtual destructor
    pub fn PxQueryFilterCallback_delete(self_: *mut PxQueryFilterCallback);

    /// Moves kinematically controlled dynamic actors through the game world.
    ///
    /// You set a dynamic actor to be kinematic using the PxRigidBodyFlag::eKINEMATIC flag
    /// with setRigidBodyFlag().
    ///
    /// The move command will result in a velocity that will move the body into
    /// the desired pose. After the move is carried out during a single time step,
    /// the velocity is returned to zero. Thus, you must continuously call
    /// this in every time step for kinematic actors so that they move along a path.
    ///
    /// This function simply stores the move destination until the next simulation
    /// step is processed, so consecutive calls will simply overwrite the stored target variable.
    ///
    /// The motion is always fully carried out.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and will set the wake counter to [`PxSceneDesc::wakeCounterResetValue`].
    pub fn PxRigidDynamic_setKinematicTarget_mut(self_: *mut PxRigidDynamic, destination: *const PxTransform);

    /// Get target pose of a kinematically controlled dynamic actor.
    ///
    /// True if the actor is a kinematically controlled dynamic and the target has been set, else False.
    pub fn PxRigidDynamic_getKinematicTarget(self_: *const PxRigidDynamic, target: *mut PxTransform) -> bool;

    /// Returns true if this body is sleeping.
    ///
    /// When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
    /// is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
    /// or one of its properties is changed by the user, the entire sleep mechanism should be transparent to the user.
    ///
    /// In general, a dynamic rigid actor is guaranteed to be awake if at least one of the following holds:
    ///
    /// The wake counter is positive (see [`setWakeCounter`]()).
    ///
    /// The linear or angular velocity is non-zero.
    ///
    /// A non-zero force or torque has been applied.
    ///
    /// If a dynamic rigid actor is sleeping, the following state is guaranteed:
    ///
    /// The wake counter is zero.
    ///
    /// The linear and angular velocity is zero.
    ///
    /// There is no force update pending.
    ///
    /// When an actor gets inserted into a scene, it will be considered asleep if all the points above hold, else it will be treated as awake.
    ///
    /// If an actor is asleep after the call to PxScene::fetchResults() returns, it is guaranteed that the pose of the actor
    /// was not changed. You can use this information to avoid updating the transforms of associated objects.
    ///
    /// A kinematic actor is asleep unless a target pose has been set (in which case it will stay awake until two consecutive
    /// simulation steps without a target pose being set have passed). The wake counter will get set to zero or to the reset value
    /// [`PxSceneDesc::wakeCounterResetValue`] in the case where a target pose has been set to be consistent with the definitions above.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// True if the actor is sleeping.
    pub fn PxRigidDynamic_isSleeping(self_: *const PxRigidDynamic) -> bool;

    /// Sets the mass-normalized kinetic energy threshold below which an actor may go to sleep.
    ///
    /// Actors whose kinetic energy divided by their mass is below this threshold will be candidates for sleeping.
    ///
    /// Default:
    /// 5e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    pub fn PxRigidDynamic_setSleepThreshold_mut(self_: *mut PxRigidDynamic, threshold: f32);

    /// Returns the mass-normalized kinetic energy below which an actor may go to sleep.
    ///
    /// The energy threshold for sleeping.
    pub fn PxRigidDynamic_getSleepThreshold(self_: *const PxRigidDynamic) -> f32;

    /// Sets the mass-normalized kinetic energy threshold below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 1e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
    pub fn PxRigidDynamic_setStabilizationThreshold_mut(self_: *mut PxRigidDynamic, threshold: f32);

    /// Returns the mass-normalized kinetic energy below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
    pub fn PxRigidDynamic_getStabilizationThreshold(self_: *const PxRigidDynamic) -> f32;

    /// Reads the PxRigidDynamic lock flags.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// The values of the PxRigidDynamicLock flags.
    pub fn PxRigidDynamic_getRigidDynamicLockFlags(self_: *const PxRigidDynamic) -> PxRigidDynamicLockFlags;

    /// Raises or clears a particular rigid dynamic lock flag.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// Default:
    /// no flags are set
    pub fn PxRigidDynamic_setRigidDynamicLockFlag_mut(self_: *mut PxRigidDynamic, flag: PxRigidDynamicLockFlag, value: bool);

    pub fn PxRigidDynamic_setRigidDynamicLockFlags_mut(self_: *mut PxRigidDynamic, flags: PxRigidDynamicLockFlags);

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
    pub fn PxRigidDynamic_getLinearVelocity(self_: *const PxRigidDynamic) -> PxVec3;

    /// Sets the linear velocity of the actor.
    ///
    /// Note that if you continuously set the velocity of an actor yourself,
    /// forces such as gravity or friction will not be able to manifest themselves, because forces directly
    /// influence only the velocity/momentum of an actor.
    ///
    /// Default:
    /// (0.0, 0.0, 0.0)
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the
    /// new velocity is non-zero.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    pub fn PxRigidDynamic_setLinearVelocity_mut(self_: *mut PxRigidDynamic, linVel: *const PxVec3, autowake: bool);

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
    pub fn PxRigidDynamic_getAngularVelocity(self_: *const PxRigidDynamic) -> PxVec3;

    /// Sets the angular velocity of the actor.
    ///
    /// Note that if you continuously set the angular velocity of an actor yourself,
    /// forces such as friction will not be able to rotate the actor, because forces directly influence only the velocity/momentum.
    ///
    /// Default:
    /// (0.0, 0.0, 0.0)
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the
    /// new velocity is non-zero.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    pub fn PxRigidDynamic_setAngularVelocity_mut(self_: *mut PxRigidDynamic, angVel: *const PxVec3, autowake: bool);

    /// Sets the wake counter for the actor.
    ///
    /// The wake counter value determines the minimum amount of time until the body can be put to sleep. Please note
    /// that a body will not be put to sleep if the energy is above the specified threshold (see [`setSleepThreshold`]())
    /// or if other awake bodies are touching it.
    ///
    /// Passing in a positive value will wake the actor up automatically.
    ///
    /// It is invalid to use this method for kinematic actors since the wake counter for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Default:
    /// 0.4 (which corresponds to 20 frames for a time step of 0.02)
    pub fn PxRigidDynamic_setWakeCounter_mut(self_: *mut PxRigidDynamic, wakeCounterValue: f32);

    /// Returns the wake counter of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// The wake counter of the actor.
    pub fn PxRigidDynamic_getWakeCounter(self_: *const PxRigidDynamic) -> f32;

    /// Wakes up the actor if it is sleeping.
    ///
    /// The actor will get woken up and might cause other touching actors to wake up as well during the next simulation step.
    ///
    /// This will set the wake counter of the actor to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    pub fn PxRigidDynamic_wakeUp_mut(self_: *mut PxRigidDynamic);

    /// Forces the actor to sleep.
    ///
    /// The actor will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    ///
    /// Any applied force will be cleared and the velocity and the wake counter of the actor will be set to 0.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    pub fn PxRigidDynamic_putToSleep_mut(self_: *mut PxRigidDynamic);

    /// Sets the solver iteration counts for the body.
    ///
    /// The solver iteration count determines how accurately joints and contacts are resolved.
    /// If you are having trouble with jointed bodies oscillating and behaving erratically, then
    /// setting a higher position iteration count may improve their stability.
    ///
    /// If intersecting bodies are being depenetrated too violently, increase the number of velocity
    /// iterations. More velocity iterations will drive the relative exit velocity of the intersecting
    /// objects closer to the correct value given the restitution.
    ///
    /// Default:
    /// 4 position iterations, 1 velocity iteration
    pub fn PxRigidDynamic_setSolverIterationCounts_mut(self_: *mut PxRigidDynamic, minPositionIters: u32, minVelocityIters: u32);

    /// Retrieves the solver iteration counts.
    pub fn PxRigidDynamic_getSolverIterationCounts(self_: *const PxRigidDynamic, minPositionIters: *mut u32, minVelocityIters: *mut u32);

    /// Retrieves the force threshold for contact reports.
    ///
    /// The contact report threshold is a force threshold. If the force between
    /// two actors exceeds this threshold for either of the two actors, a contact report
    /// will be generated according to the contact report threshold flags provided by
    /// the filter shader/callback.
    /// See [`PxPairFlag`].
    ///
    /// The threshold used for a collision between a dynamic actor and the static environment is
    /// the threshold of the dynamic actor, and all contacts with static actors are summed to find
    /// the total normal force.
    ///
    /// Default:
    /// PX_MAX_F32
    ///
    /// Force threshold for contact reports.
    pub fn PxRigidDynamic_getContactReportThreshold(self_: *const PxRigidDynamic) -> f32;

    /// Sets the force threshold for contact reports.
    ///
    /// See [`getContactReportThreshold`]().
    pub fn PxRigidDynamic_setContactReportThreshold_mut(self_: *mut PxRigidDynamic, threshold: f32);

    pub fn PxRigidDynamic_getConcreteTypeName(self_: *const PxRigidDynamic) -> *const std::ffi::c_char;

    pub fn PxRigidStatic_getConcreteTypeName(self_: *const PxRigidStatic) -> *const std::ffi::c_char;

    /// constructor sets to default.
    pub fn PxSceneQueryDesc_new() -> PxSceneQueryDesc;

    /// (re)sets the structure to the default.
    pub fn PxSceneQueryDesc_setToDefault_mut(self_: *mut PxSceneQueryDesc);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    pub fn PxSceneQueryDesc_isValid(self_: *const PxSceneQueryDesc) -> bool;

    /// Sets the rebuild rate of the dynamic tree pruning structures.
    pub fn PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut(self_: *mut PxSceneQuerySystemBase, dynamicTreeRebuildRateHint: u32);

    /// Retrieves the rebuild rate of the dynamic tree pruning structures.
    ///
    /// The rebuild rate of the dynamic tree pruning structures.
    pub fn PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint(self_: *const PxSceneQuerySystemBase) -> u32;

    /// Forces dynamic trees to be immediately rebuilt.
    ///
    /// PxScene will call this function with the PX_SCENE_PRUNER_STATIC or PX_SCENE_PRUNER_DYNAMIC value.
    pub fn PxSceneQuerySystemBase_forceRebuildDynamicTree_mut(self_: *mut PxSceneQuerySystemBase, prunerIndex: u32);

    /// Sets scene query update mode
    pub fn PxSceneQuerySystemBase_setUpdateMode_mut(self_: *mut PxSceneQuerySystemBase, updateMode: PxSceneQueryUpdateMode);

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    pub fn PxSceneQuerySystemBase_getUpdateMode(self_: *const PxSceneQuerySystemBase) -> PxSceneQueryUpdateMode;

    /// Retrieves the system's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    pub fn PxSceneQuerySystemBase_getStaticTimestamp(self_: *const PxSceneQuerySystemBase) -> u32;

    /// Flushes any changes to the scene query representation.
    ///
    /// This method updates the state of the scene query representation to match changes in the scene state.
    ///
    /// By default, these changes are buffered until the next query is submitted. Calling this function will not change
    /// the results from scene queries, but can be used to ensure that a query will not perform update work in the course of
    /// its execution.
    ///
    /// A thread performing updates will hold a write lock on the query structure, and thus stall other querying threads. In multithread
    /// scenarios it can be useful to explicitly schedule the period where this lock may be held for a significant period, so that
    /// subsequent queries issued from multiple threads will not block.
    pub fn PxSceneQuerySystemBase_flushUpdates_mut(self_: *mut PxSceneQuerySystemBase);

    /// Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
    /// or via a custom user callback implementation inheriting from PxRaycastCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    pub fn PxSceneQuerySystemBase_raycast(self_: *const PxSceneQuerySystemBase, origin: *const PxVec3, unitDir: *const PxVec3, distance: f32, hitCall: *mut PxRaycastCallback, hitFlags: PxHitFlags, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Performs a sweep test against objects in the scene, returns results in a PxSweepBuffer object
    /// or via a custom user callback implementation inheriting from PxSweepCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    pub fn PxSceneQuerySystemBase_sweep(self_: *const PxSceneQuerySystemBase, geometry: *const PxGeometry, pose: *const PxTransform, unitDir: *const PxVec3, distance: f32, hitCall: *mut PxSweepCallback, hitFlags: PxHitFlags, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, inflation: f32, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Performs an overlap test of a given geometry against objects in the scene, returns results in a PxOverlapBuffer object
    /// or via a custom user callback implementation inheriting from PxOverlapCallback.
    ///
    /// Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see [`PxQueryHitType`]).
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    ///
    /// eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
    ///
    /// If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
    pub fn PxSceneQuerySystemBase_overlap(self_: *const PxSceneQuerySystemBase, geometry: *const PxGeometry, pose: *const PxTransform, hitCall: *mut PxOverlapCallback, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, queryFlags: PxGeometryQueryFlags) -> bool;

    /// Sets scene query update mode
    pub fn PxSceneSQSystem_setSceneQueryUpdateMode_mut(self_: *mut PxSceneSQSystem, updateMode: PxSceneQueryUpdateMode);

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
    pub fn PxSceneSQSystem_getSceneQueryUpdateMode(self_: *const PxSceneSQSystem) -> PxSceneQueryUpdateMode;

    /// Retrieves the scene's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
    pub fn PxSceneSQSystem_getSceneQueryStaticTimestamp(self_: *const PxSceneSQSystem) -> u32;

    /// Flushes any changes to the scene query representation.
    pub fn PxSceneSQSystem_flushQueryUpdates_mut(self_: *mut PxSceneSQSystem);

    /// Forces dynamic trees to be immediately rebuilt.
    pub fn PxSceneSQSystem_forceDynamicTreeRebuild_mut(self_: *mut PxSceneSQSystem, rebuildStaticStructure: bool, rebuildDynamicStructure: bool);

    /// Return the value of PxSceneQueryDesc::staticStructure that was set when creating the scene with PxPhysics::createScene
    pub fn PxSceneSQSystem_getStaticStructure(self_: *const PxSceneSQSystem) -> PxPruningStructureType;

    /// Return the value of PxSceneQueryDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene
    pub fn PxSceneSQSystem_getDynamicStructure(self_: *const PxSceneSQSystem) -> PxPruningStructureType;

    /// Executes scene queries update tasks.
    ///
    /// This function will refit dirty shapes within the pruner and will execute a task to build a new AABB tree, which is
    /// build on a different thread. The new AABB tree is built based on the dynamic tree rebuild hint rate. Once
    /// the new tree is ready it will be commited in next fetchQueries call, which must be called after.
    ///
    /// This function is equivalent to the following PxSceneQuerySystem calls:
    /// Synchronous calls:
    /// - PxSceneQuerySystemBase::flushUpdates()
    /// - handle0 = PxSceneQuerySystem::prepareSceneQueryBuildStep(PX_SCENE_PRUNER_STATIC)
    /// - handle1 = PxSceneQuerySystem::prepareSceneQueryBuildStep(PX_SCENE_PRUNER_DYNAMIC)
    /// Asynchronous calls:
    /// - PxSceneQuerySystem::sceneQueryBuildStep(handle0);
    /// - PxSceneQuerySystem::sceneQueryBuildStep(handle1);
    ///
    /// This function is part of the PxSceneSQSystem interface because it uses the PxScene task system under the hood. But
    /// it calls PxSceneQuerySystem functions, which are independent from this system and could be called in a similar
    /// fashion by a separate, possibly user-defined task manager.
    ///
    /// If PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED is used, it is required to update the scene queries
    /// using this function.
    pub fn PxSceneSQSystem_sceneQueriesUpdate_mut(self_: *mut PxSceneSQSystem, completionTask: *mut PxBaseTask, controlSimulation: bool);

    /// This checks to see if the scene queries update has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    pub fn PxSceneSQSystem_checkQueries_mut(self_: *mut PxSceneSQSystem, block: bool) -> bool;

    /// This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call,
    /// the SDK will issue an error message.
    ///
    /// If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree.
    pub fn PxSceneSQSystem_fetchQueries_mut(self_: *mut PxSceneSQSystem, block: bool) -> bool;

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
    pub fn PxSceneQuerySystem_release_mut(self_: *mut PxSceneQuerySystem);

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
    pub fn PxSceneQuerySystem_acquireReference_mut(self_: *mut PxSceneQuerySystem);

    /// Preallocates internal arrays to minimize the amount of reallocations.
    ///
    /// The system does not prevent more allocations than given numbers. It is legal to not call this function at all,
    /// or to add more shapes to the system than the preallocated amounts.
    pub fn PxSceneQuerySystem_preallocate_mut(self_: *mut PxSceneQuerySystem, prunerIndex: u32, nbShapes: u32);

    /// Frees internal memory that may not be in-use anymore.
    ///
    /// This is an entry point for reclaiming transient memory allocated at some point by the SQ system,
    /// but which wasn't been immediately freed for performance reason. Calling this function might free
    /// some memory, but it might also produce a new set of allocations in the next frame.
    pub fn PxSceneQuerySystem_flushMemory_mut(self_: *mut PxSceneQuerySystem);

    /// Adds a shape to the SQ system.
    ///
    /// The same function is used to add either a regular shape, or a SQ compound shape.
    pub fn PxSceneQuerySystem_addSQShape_mut(self_: *mut PxSceneQuerySystem, actor: *const PxRigidActor, shape: *const PxShape, bounds: *const PxBounds3, transform: *const PxTransform, compoundHandle: *const u32, hasPruningStructure: bool);

    /// Removes a shape from the SQ system.
    ///
    /// The same function is used to remove either a regular shape, or a SQ compound shape.
    pub fn PxSceneQuerySystem_removeSQShape_mut(self_: *mut PxSceneQuerySystem, actor: *const PxRigidActor, shape: *const PxShape);

    /// Updates a shape in the SQ system.
    ///
    /// The same function is used to update either a regular shape, or a SQ compound shape.
    ///
    /// The transforms are eager-evaluated, but the bounds are lazy-evaluated. This means that
    /// the updated transform has to be passed to the update function, while the bounds are automatically
    /// recomputed by the system whenever needed.
    pub fn PxSceneQuerySystem_updateSQShape_mut(self_: *mut PxSceneQuerySystem, actor: *const PxRigidActor, shape: *const PxShape, transform: *const PxTransform);

    /// Adds a compound to the SQ system.
    ///
    /// SQ compound handle
    pub fn PxSceneQuerySystem_addSQCompound_mut(self_: *mut PxSceneQuerySystem, actor: *const PxRigidActor, shapes: *mut *const PxShape, bvh: *const PxBVH, transforms: *const PxTransform) -> u32;

    /// Removes a compound from the SQ system.
    pub fn PxSceneQuerySystem_removeSQCompound_mut(self_: *mut PxSceneQuerySystem, compoundHandle: u32);

    /// Updates a compound in the SQ system.
    ///
    /// The compound structures are immediately updated when the call occurs.
    pub fn PxSceneQuerySystem_updateSQCompound_mut(self_: *mut PxSceneQuerySystem, compoundHandle: u32, compoundTransform: *const PxTransform);

    /// Shift the data structures' origin by the specified vector.
    ///
    /// Please refer to the notes of the similar function in PxScene.
    pub fn PxSceneQuerySystem_shiftOrigin_mut(self_: *mut PxSceneQuerySystem, shift: *const PxVec3);

    /// Merges a pruning structure with the SQ system's internal pruners.
    pub fn PxSceneQuerySystem_merge_mut(self_: *mut PxSceneQuerySystem, pruningStructure: *const PxPruningStructure);

    /// Shape to SQ-pruner-handle mapping function.
    ///
    /// This function finds and returns the SQ pruner handle associated with a given (actor/shape) couple
    /// that was previously added to the system. This is needed for the sync function.
    ///
    /// Associated SQ pruner handle.
    pub fn PxSceneQuerySystem_getHandle(self_: *const PxSceneQuerySystem, actor: *const PxRigidActor, shape: *const PxShape, prunerIndex: *mut u32) -> u32;

    /// Synchronizes the scene-query system with another system that references the same objects.
    ///
    /// This function is used when the scene-query objects also exist in another system that can also update them. For example the scene-query objects
    /// (used for raycast, overlap or sweep queries) might be driven by equivalent objects in an external rigid-body simulation engine. In this case
    /// the rigid-body simulation engine computes the new poses and transforms, and passes them to the scene-query system using this function. It is
    /// more efficient than calling updateSQShape on each object individually, since updateSQShape would end up recomputing the bounds already available
    /// in the rigid-body engine.
    pub fn PxSceneQuerySystem_sync_mut(self_: *mut PxSceneQuerySystem, prunerIndex: u32, handles: *const u32, indices: *const u32, bounds: *const PxBounds3, transforms: *const PxTransformPadded, count: u32, ignoredIndices: *const PxBitMap);

    /// Finalizes updates made to the SQ system.
    ///
    /// This function should be called after updates have been made to the SQ system, to fully reflect the changes
    /// inside the internal pruners. In particular it should be called:
    /// - after calls to updateSQShape
    /// - after calls to sync
    ///
    /// This function:
    /// - recomputes bounds of manually updated shapes (i.e. either regular or SQ compound shapes modified by updateSQShape)
    /// - updates dynamic pruners (refit operations)
    /// - incrementally rebuilds AABB-trees
    ///
    /// The amount of work performed in this function depends on PxSceneQueryUpdateMode.
    pub fn PxSceneQuerySystem_finalizeUpdates_mut(self_: *mut PxSceneQuerySystem);

    /// Prepares asynchronous build step.
    ///
    /// This is directly called (synchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function is called to let the system execute any necessary synchronous operation before the
    /// asynchronous sceneQueryBuildStep() function is called.
    ///
    /// If there is any work to do for the specific pruner, the function returns a pruner-specific handle that
    /// will be passed to the corresponding, asynchronous sceneQueryBuildStep function.
    ///
    /// A pruner-specific handle that will be sent to sceneQueryBuildStep if there is any work to do, i.e. to execute the corresponding sceneQueryBuildStep() call.
    ///
    /// Null if there is no work to do, otherwise a pruner-specific handle.
    pub fn PxSceneQuerySystem_prepareSceneQueryBuildStep_mut(self_: *mut PxSceneQuerySystem, prunerIndex: u32) -> *mut std::ffi::c_void;

    /// Executes asynchronous build step.
    ///
    /// This is directly called (asynchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function incrementally builds the internal trees/pruners. It is called asynchronously, i.e. this can be
    /// called from different threads for building multiple trees at the same time.
    pub fn PxSceneQuerySystem_sceneQueryBuildStep_mut(self_: *mut PxSceneQuerySystem, handle: *mut std::ffi::c_void);

    pub fn PxBroadPhaseDesc_new(type_: PxBroadPhaseType) -> PxBroadPhaseDesc;

    pub fn PxBroadPhaseDesc_isValid(self_: *const PxBroadPhaseDesc) -> bool;

    /// Retrieves the filter group for static objects.
    ///
    /// Mark static objects with this group when adding them to the broadphase.
    /// Overlaps between static objects will not be detected. All static objects
    /// should have the same group.
    ///
    /// Filter group for static objects.
    pub fn phys_PxGetBroadPhaseStaticFilterGroup() -> u32;

    /// Retrieves a filter group for dynamic objects.
    ///
    /// Mark dynamic objects with this group when adding them to the broadphase.
    /// Each dynamic object must have an ID, and overlaps between dynamic objects that have
    /// the same ID will not be detected. This is useful to dismiss overlaps between shapes
    /// of the same (compound) actor directly within the broadphase.
    ///
    /// Filter group for the object.
    pub fn phys_PxGetBroadPhaseDynamicFilterGroup(id: u32) -> u32;

    /// Retrieves a filter group for kinematic objects.
    ///
    /// Mark kinematic objects with this group when adding them to the broadphase.
    /// Each kinematic object must have an ID, and overlaps between kinematic objects that have
    /// the same ID will not be detected.
    ///
    /// Filter group for the object.
    pub fn phys_PxGetBroadPhaseKinematicFilterGroup(id: u32) -> u32;

    pub fn PxBroadPhaseUpdateData_new(created: *const u32, nbCreated: u32, updated: *const u32, nbUpdated: u32, removed: *const u32, nbRemoved: u32, bounds: *const PxBounds3, groups: *const u32, distances: *const f32, capacity: u32) -> PxBroadPhaseUpdateData;

    pub fn PxBroadPhaseResults_new() -> PxBroadPhaseResults;

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    pub fn PxBroadPhaseRegions_getNbRegions(self_: *const PxBroadPhaseRegions) -> u32;

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions.
    pub fn PxBroadPhaseRegions_getRegions(self_: *const PxBroadPhaseRegions, userBuffer: *mut PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32;

    /// Adds a new broad-phase region.
    ///
    /// The total number of regions is limited to PxBroadPhaseCaps::mMaxNbRegions. If that number is exceeded, the call is ignored.
    ///
    /// The newly added region will be automatically populated with already existing objects that touch it, if the
    /// 'populateRegion' parameter is set to true. Otherwise the newly added region will be empty, and it will only be
    /// populated with objects when those objects are added to the simulation, or updated if they already exist.
    ///
    /// Using 'populateRegion=true' has a cost, so it is best to avoid it if possible. In particular it is more efficient
    /// to create the empty regions first (with populateRegion=false) and then add the objects afterwards (rather than
    /// the opposite).
    ///
    /// Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
    /// regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
    /// object leaves all regions, or is created outside of all regions, several things happen:
    /// - collisions get disabled for this object
    /// - the object appears in the getOutOfBoundsObjects() array
    ///
    /// If an out-of-bounds object, whose collisions are disabled, re-enters a valid broadphase region, then collisions
    /// are re-enabled for that object.
    ///
    /// Handle for newly created region, or 0xffffffff in case of failure.
    pub fn PxBroadPhaseRegions_addRegion_mut(self_: *mut PxBroadPhaseRegions, region: *const PxBroadPhaseRegion, populateRegion: bool, bounds: *const PxBounds3, distances: *const f32) -> u32;

    /// Removes a broad-phase region.
    ///
    /// If the region still contains objects, and if those objects do not overlap any region any more, they are not
    /// automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
    /// is used for each object. Users are responsible for removing the objects from the simulation if this is the
    /// desired behavior.
    ///
    /// If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    ///
    /// True if success
    pub fn PxBroadPhaseRegions_removeRegion_mut(self_: *mut PxBroadPhaseRegions, handle: u32) -> bool;

    pub fn PxBroadPhaseRegions_getNbOutOfBoundsObjects(self_: *const PxBroadPhaseRegions) -> u32;

    pub fn PxBroadPhaseRegions_getOutOfBoundsObjects(self_: *const PxBroadPhaseRegions) -> *const u32;

    pub fn PxBroadPhase_release_mut(self_: *mut PxBroadPhase);

    /// Gets the broadphase type.
    ///
    /// Broadphase type.
    pub fn PxBroadPhase_getType(self_: *const PxBroadPhase) -> PxBroadPhaseType;

    /// Gets broad-phase caps.
    pub fn PxBroadPhase_getCaps(self_: *const PxBroadPhase, caps: *mut PxBroadPhaseCaps);

    /// Retrieves the regions API if applicable.
    ///
    /// For broadphases that do not use explicit user-defined regions, this call returns NULL.
    ///
    /// Region API, or NULL.
    pub fn PxBroadPhase_getRegions_mut(self_: *mut PxBroadPhase) -> *mut PxBroadPhaseRegions;

    /// Retrieves the broadphase allocator.
    ///
    /// User-provided buffers should ideally be allocated with this allocator, for best performance.
    /// This is especially true for the GPU broadphases, whose buffers need to be allocated in CUDA
    /// host memory.
    ///
    /// The broadphase allocator.
    pub fn PxBroadPhase_getAllocator_mut(self_: *mut PxBroadPhase) -> *mut PxAllocatorCallback;

    /// Retrieves the profiler's context ID.
    ///
    /// The context ID.
    pub fn PxBroadPhase_getContextID(self_: *const PxBroadPhase) -> u64;

    /// Sets a scratch buffer
    ///
    /// Some broadphases might take advantage of a scratch buffer to limit runtime allocations.
    ///
    /// All broadphases still work without providing a scratch buffer, this is an optional function
    /// that can potentially reduce runtime allocations.
    pub fn PxBroadPhase_setScratchBlock_mut(self_: *mut PxBroadPhase, scratchBlock: *mut std::ffi::c_void, size: u32);

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The provided update data describes changes to objects since the last broadphase update.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    pub fn PxBroadPhase_update_mut(self_: *mut PxBroadPhase, updateData: *const PxBroadPhaseUpdateData, continuation: *mut PxBaseTask);

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    pub fn PxBroadPhase_fetchResults_mut(self_: *mut PxBroadPhase, results: *mut PxBroadPhaseResults);

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    pub fn PxBroadPhase_update_mut_1(self_: *mut PxBroadPhase, results: *mut PxBroadPhaseResults, updateData: *const PxBroadPhaseUpdateData);

    /// Broadphase factory function.
    ///
    /// Use this function to create a new standalone broadphase.
    ///
    /// Newly created broadphase, or NULL
    pub fn phys_PxCreateBroadPhase(desc: *const PxBroadPhaseDesc) -> *mut PxBroadPhase;

    pub fn PxAABBManager_release_mut(self_: *mut PxAABBManager);

    /// Retrieves the underlying broadphase.
    ///
    /// The managed broadphase.
    pub fn PxAABBManager_getBroadPhase_mut(self_: *mut PxAABBManager) -> *mut PxBroadPhase;

    /// Retrieves the managed bounds.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object bounds.
    pub fn PxAABBManager_getBounds(self_: *const PxAABBManager) -> *const PxBounds3;

    /// Retrieves the managed distances.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object distances.
    pub fn PxAABBManager_getDistances(self_: *const PxAABBManager) -> *const f32;

    /// Retrieves the managed filter groups.
    ///
    /// The managed object groups.
    pub fn PxAABBManager_getGroups(self_: *const PxAABBManager) -> *const u32;

    /// Retrieves the managed buffers' capacity.
    ///
    /// Bounds, distances and groups buffers have the same capacity.
    ///
    /// The managed buffers' capacity.
    pub fn PxAABBManager_getCapacity(self_: *const PxAABBManager) -> u32;

    /// Adds an object to the manager.
    ///
    /// Objects' indices are externally managed, i.e. they must be provided by users (as opposed to handles
    /// that could be returned by this manager). The design allows users to identify an object by a single ID,
    /// and use the same ID in multiple sub-systems.
    pub fn PxAABBManager_addObject_mut(self_: *mut PxAABBManager, index: u32, bounds: *const PxBounds3, group: u32, distance: f32);

    /// Removes an object from the manager.
    pub fn PxAABBManager_removeObject_mut(self_: *mut PxAABBManager, index: u32);

    /// Updates an object in the manager.
    ///
    /// This call can update an object's bounds, distance, or both.
    /// It is not possible to update an object's filter group.
    pub fn PxAABBManager_updateObject_mut(self_: *mut PxAABBManager, index: u32, bounds: *const PxBounds3, distance: *const f32);

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The data necessary for updating the broadphase is internally computed by the AABB manager.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
    pub fn PxAABBManager_update_mut(self_: *mut PxAABBManager, continuation: *mut PxBaseTask);

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
    pub fn PxAABBManager_fetchResults_mut(self_: *mut PxAABBManager, results: *mut PxBroadPhaseResults);

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
    pub fn PxAABBManager_update_mut_1(self_: *mut PxAABBManager, results: *mut PxBroadPhaseResults);

    /// AABB manager factory function.
    ///
    /// Use this function to create a new standalone high-level broadphase.
    ///
    /// Newly created AABB manager, or NULL
    pub fn phys_PxCreateAABBManager(broadphase: *mut PxBroadPhase) -> *mut PxAABBManager;

    /// constructor sets to default
    pub fn PxSceneLimits_new() -> PxSceneLimits;

    /// (re)sets the structure to the default
    pub fn PxSceneLimits_setToDefault_mut(self_: *mut PxSceneLimits);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    pub fn PxSceneLimits_isValid(self_: *const PxSceneLimits) -> bool;

    pub fn PxgDynamicsMemoryConfig_new() -> PxgDynamicsMemoryConfig;

    pub fn PxgDynamicsMemoryConfig_isValid(self_: *const PxgDynamicsMemoryConfig) -> bool;

    /// constructor sets to default.
    pub fn PxSceneDesc_new(scale: *const PxTolerancesScale) -> PxSceneDesc;

    /// (re)sets the structure to the default.
    pub fn PxSceneDesc_setToDefault_mut(self_: *mut PxSceneDesc, scale: *const PxTolerancesScale);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    pub fn PxSceneDesc_isValid(self_: *const PxSceneDesc) -> bool;

    pub fn PxSceneDesc_getTolerancesScale(self_: *const PxSceneDesc) -> *const PxTolerancesScale;

    /// Get number of broadphase volumes added for the current simulation step.
    ///
    /// Number of broadphase volumes added.
    pub fn PxSimulationStatistics_getNbBroadPhaseAdds(self_: *const PxSimulationStatistics) -> u32;

    /// Get number of broadphase volumes removed for the current simulation step.
    ///
    /// Number of broadphase volumes removed.
    pub fn PxSimulationStatistics_getNbBroadPhaseRemoves(self_: *const PxSimulationStatistics) -> u32;

    /// Get number of shape collision pairs of a certain type processed for the current simulation step.
    ///
    /// There is an entry for each geometry pair type.
    ///
    /// entry[i][j] = entry[j][i], hence, if you want the sum of all pair
    /// types, you need to discard the symmetric entries
    ///
    /// Number of processed pairs of the specified geometry types.
    pub fn PxSimulationStatistics_getRbPairStats(self_: *const PxSimulationStatistics, pairType: RbPairStatsType, g0: PxGeometryType, g1: PxGeometryType) -> u32;

    pub fn PxSimulationStatistics_new() -> PxSimulationStatistics;

    /// Sets the PVD flag. See PxPvdSceneFlag.
    pub fn PxPvdSceneClient_setScenePvdFlag_mut(self_: *mut PxPvdSceneClient, flag: PxPvdSceneFlag, value: bool);

    /// Sets the PVD flags. See PxPvdSceneFlags.
    pub fn PxPvdSceneClient_setScenePvdFlags_mut(self_: *mut PxPvdSceneClient, flags: PxPvdSceneFlags);

    /// Retrieves the PVD flags. See PxPvdSceneFlags.
    pub fn PxPvdSceneClient_getScenePvdFlags(self_: *const PxPvdSceneClient) -> PxPvdSceneFlags;

    /// update camera on PVD application's render window
    pub fn PxPvdSceneClient_updateCamera_mut(self_: *mut PxPvdSceneClient, name: *const std::ffi::c_char, origin: *const PxVec3, up: *const PxVec3, target: *const PxVec3);

    /// draw points on PVD application's render window
    pub fn PxPvdSceneClient_drawPoints_mut(self_: *mut PxPvdSceneClient, points: *const PxDebugPoint, count: u32);

    /// draw lines on PVD application's render window
    pub fn PxPvdSceneClient_drawLines_mut(self_: *mut PxPvdSceneClient, lines: *const PxDebugLine, count: u32);

    /// draw triangles on PVD application's render window
    pub fn PxPvdSceneClient_drawTriangles_mut(self_: *mut PxPvdSceneClient, triangles: *const PxDebugTriangle, count: u32);

    /// draw text on PVD application's render window
    pub fn PxPvdSceneClient_drawText_mut(self_: *mut PxPvdSceneClient, text: *const PxDebugText);

    pub fn PxDominanceGroupPair_new(a: u8, b: u8) -> PxDominanceGroupPair;

    pub fn PxBroadPhaseCallback_delete(self_: *mut PxBroadPhaseCallback);

    /// Out-of-bounds notification.
    ///
    /// This function is called when an object leaves the broad-phase.
    pub fn PxBroadPhaseCallback_onObjectOutOfBounds_mut(self_: *mut PxBroadPhaseCallback, shape: *mut PxShape, actor: *mut PxActor);

    /// Out-of-bounds notification.
    ///
    /// This function is called when an aggregate leaves the broad-phase.
    pub fn PxBroadPhaseCallback_onObjectOutOfBounds_mut_1(self_: *mut PxBroadPhaseCallback, aggregate: *mut PxAggregate);

    /// Deletes the scene.
    ///
    /// Removes any actors and constraint shaders from this scene
    /// (if the user hasn't already done so).
    ///
    /// Be sure to not keep a reference to this object after calling release.
    /// Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
    pub fn PxScene_release_mut(self_: *mut PxScene);

    /// Sets a scene flag. You can only set one flag at a time.
    ///
    /// Not all flags are mutable and changing some will result in an error. Please check [`PxSceneFlag`] to see which flags can be changed.
    pub fn PxScene_setFlag_mut(self_: *mut PxScene, flag: PxSceneFlag, value: bool);

    /// Get the scene flags.
    ///
    /// The scene flags. See [`PxSceneFlag`]
    pub fn PxScene_getFlags(self_: *const PxScene) -> PxSceneFlags;

    /// Set new scene limits.
    ///
    /// Increase the maximum capacity of various data structures in the scene. The new capacities will be
    /// at least as large as required to deal with the objects currently in the scene. Further, these values
    /// are for preallocation and do not represent hard limits.
    pub fn PxScene_setLimits_mut(self_: *mut PxScene, limits: *const PxSceneLimits);

    /// Get current scene limits.
    ///
    /// Current scene limits.
    pub fn PxScene_getLimits(self_: *const PxScene) -> PxSceneLimits;

    /// Call this method to retrieve the Physics SDK.
    ///
    /// The physics SDK this scene is associated with.
    pub fn PxScene_getPhysics_mut(self_: *mut PxScene) -> *mut PxPhysics;

    /// Retrieves the scene's internal timestamp, increased each time a simulation step is completed.
    ///
    /// scene timestamp
    pub fn PxScene_getTimestamp(self_: *const PxScene) -> u32;

    /// Adds an articulation to this scene.
    ///
    /// If the articulation is already assigned to a scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// True if success
    pub fn PxScene_addArticulation_mut(self_: *mut PxScene, articulation: *mut PxArticulationReducedCoordinate) -> bool;

    /// Removes an articulation from this scene.
    ///
    /// If the articulation is not part of this scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the articulation is in an aggregate it will be removed from the aggregate.
    pub fn PxScene_removeArticulation_mut(self_: *mut PxScene, articulation: *mut PxArticulationReducedCoordinate, wakeOnLostTouch: bool);

    /// Adds an actor to this scene.
    ///
    /// If the actor is already assigned to a scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the actor has an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// You can not add individual articulation links (see [`PxArticulationLink`]) to the scene. Use #addArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// When a BVH is provided the actor shapes are grouped together.
    /// The scene query pruning structure inside PhysX SDK will store/update one
    /// bound per actor. The scene queries against such an actor will query actor
    /// bounds and then make a local space query against the provided BVH, which is in actor's local space.
    ///
    /// True if success
    pub fn PxScene_addActor_mut(self_: *mut PxScene, actor: *mut PxActor, bvh: *const PxBVH) -> bool;

    /// Adds actors to this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use addActor() instead.
    /// For articulation links, use addArticulation().
    ///
    /// If one of the actors is already assigned to a scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// If an actor in the array contains an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If an actor in the array is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// this method is optimized for high performance.
    ///
    /// True if success
    pub fn PxScene_addActors_mut(self_: *mut PxScene, actors: *const *mut PxActor, nbActors: u32) -> bool;

    /// Adds a pruning structure together with its actors to this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use addActor() instead.
    /// For articulation links, use addArticulation().
    ///
    /// If an actor in the pruning structure contains an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// For all actors in the pruning structure each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// This method is optimized for high performance.
    ///
    /// Merging a PxPruningStructure into an active scene query optimization AABB tree might unbalance the tree. A typical use case for
    /// PxPruningStructure is a large world scenario where blocks of closely positioned actors get streamed in. The merge process finds the
    /// best node in the active scene query optimization AABB tree and inserts the PxPruningStructure. Therefore using PxPruningStructure
    /// for actors scattered throughout the world will result in an unbalanced tree.
    ///
    /// True if success
    pub fn PxScene_addActors_mut_1(self_: *mut PxScene, pruningStructure: *const PxPruningStructure) -> bool;

    /// Removes an actor from this scene.
    ///
    /// If the actor is not part of this scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
    ///
    /// If the actor is in an aggregate it will be removed from the aggregate.
    pub fn PxScene_removeActor_mut(self_: *mut PxScene, actor: *mut PxActor, wakeOnLostTouch: bool);

    /// Removes actors from this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use removeActor() instead.
    /// For articulation links, use removeArticulation().
    ///
    /// If some actor is not part of this scene (see [`PxActor::getScene`]), the actor remove is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
    pub fn PxScene_removeActors_mut(self_: *mut PxScene, actors: *const *mut PxActor, nbActors: u32, wakeOnLostTouch: bool);

    /// Adds an aggregate to this scene.
    ///
    /// If the aggregate is already assigned to a scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If the aggregate already contains actors, those actors are added to the scene as well.
    ///
    /// True if success
    pub fn PxScene_addAggregate_mut(self_: *mut PxScene, aggregate: *mut PxAggregate) -> bool;

    /// Removes an aggregate from this scene.
    ///
    /// If the aggregate is not part of this scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains actors, those actors are removed from the scene as well.
    pub fn PxScene_removeAggregate_mut(self_: *mut PxScene, aggregate: *mut PxAggregate, wakeOnLostTouch: bool);

    /// Adds objects in the collection to this scene.
    ///
    /// This function adds the following types of objects to this scene: PxRigidActor (except PxArticulationLink), PxAggregate, PxArticulationReducedCoordinate.
    /// This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.
    ///
    /// If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// True if success
    pub fn PxScene_addCollection_mut(self_: *mut PxScene, collection: *const PxCollection) -> bool;

    /// Retrieve the number of actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// the number of actors.
    pub fn PxScene_getNbActors(self_: *const PxScene, types: PxActorTypeFlags) -> u32;

    /// Retrieve an array of all the actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// Number of actors written to the buffer.
    pub fn PxScene_getActors(self_: *const PxScene, types: PxActorTypeFlags, userBuffer: *mut *mut PxActor, bufferSize: u32, startIndex: u32) -> u32;

    /// Queries the PxScene for a list of the PxActors whose transforms have been
    /// updated during the previous simulation step. Only includes actors of type PxRigidDynamic and PxArticulationLink.
    ///
    /// PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.
    ///
    /// A pointer to the list of active PxActors generated during the last call to fetchResults().
    pub fn PxScene_getActiveActors_mut(self_: *mut PxScene, nbActorsOut: *mut u32) -> *mut *mut PxActor;

    /// Returns the number of articulations in the scene.
    ///
    /// the number of articulations in this scene.
    pub fn PxScene_getNbArticulations(self_: *const PxScene) -> u32;

    /// Retrieve all the articulations in the scene.
    ///
    /// Number of articulations written to the buffer.
    pub fn PxScene_getArticulations(self_: *const PxScene, userBuffer: *mut *mut PxArticulationReducedCoordinate, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of constraint shaders in the scene.
    ///
    /// the number of constraint shaders in this scene.
    pub fn PxScene_getNbConstraints(self_: *const PxScene) -> u32;

    /// Retrieve all the constraint shaders in the scene.
    ///
    /// Number of constraint shaders written to the buffer.
    pub fn PxScene_getConstraints(self_: *const PxScene, userBuffer: *mut *mut PxConstraint, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of aggregates in the scene.
    ///
    /// the number of aggregates in this scene.
    pub fn PxScene_getNbAggregates(self_: *const PxScene) -> u32;

    /// Retrieve all the aggregates in the scene.
    ///
    /// Number of aggregates written to the buffer.
    pub fn PxScene_getAggregates(self_: *const PxScene, userBuffer: *mut *mut PxAggregate, bufferSize: u32, startIndex: u32) -> u32;

    /// Specifies the dominance behavior of contacts between two actors with two certain dominance groups.
    ///
    /// It is possible to assign each actor to a dominance groups using [`PxActor::setDominanceGroup`]().
    ///
    /// With dominance groups one can have all contacts created between actors act in one direction only. This is useful, for example, if you
    /// want an object to push debris out of its way and be unaffected,while still responding physically to forces and collisions
    /// with non-debris objects.
    ///
    /// Whenever a contact between two actors (a0, a1) needs to be solved, the groups (g0, g1) of both
    /// actors are retrieved. Then the PxDominanceGroupPair setting for this group pair is retrieved with getDominanceGroupPair(g0, g1).
    ///
    /// In the contact, PxDominanceGroupPair::dominance0 becomes the dominance setting for a0, and
    /// PxDominanceGroupPair::dominance1 becomes the dominance setting for a1. A dominanceN setting of 1.0f, the default,
    /// will permit aN to be pushed or pulled by a(1-N) through the contact. A dominanceN setting of 0.0f, will however
    /// prevent aN to be pushed by a(1-N) via the contact. Thus, a PxDominanceGroupPair of (1.0f, 0.0f) makes
    /// the interaction one-way.
    ///
    /// The matrix sampled by getDominanceGroupPair(g1, g2) is initialised by default such that:
    ///
    /// if g1 == g2, then (1.0f, 1.0f) is returned
    /// if g1
    /// <
    /// g2, then (0.0f, 1.0f) is returned
    /// if g1 >  g2, then (1.0f, 0.0f) is returned
    ///
    /// In other words, we permit actors in higher groups to be pushed around by actors in lower groups by default.
    ///
    /// These settings should cover most applications, and in fact not overriding these settings may likely result in higher performance.
    ///
    /// It is not possible to make the matrix asymetric, or to change the diagonal. In other words:
    ///
    /// it is not possible to change (g1, g2) if (g1==g2)
    /// if you set
    ///
    /// (g1, g2) to X, then (g2, g1) will implicitly and automatically be set to ~X, where:
    ///
    /// ~(1.0f, 1.0f) is (1.0f, 1.0f)
    /// ~(0.0f, 1.0f) is (1.0f, 0.0f)
    /// ~(1.0f, 0.0f) is (0.0f, 1.0f)
    ///
    /// These two restrictions are to make sure that contacts between two actors will always evaluate to the same dominance
    /// setting, regardless of the order of the actors.
    ///
    /// Dominance settings are currently specified as floats 0.0f or 1.0f because in the future we may permit arbitrary
    /// fractional settings to express 'partly-one-way' interactions.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake actors up automatically.
    pub fn PxScene_setDominanceGroupPair_mut(self_: *mut PxScene, group1: u8, group2: u8, dominance: *const PxDominanceGroupPair);

    /// Samples the dominance matrix.
    pub fn PxScene_getDominanceGroupPair(self_: *const PxScene, group1: u8, group2: u8) -> PxDominanceGroupPair;

    /// Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene
    pub fn PxScene_getCpuDispatcher(self_: *const PxScene) -> *mut PxCpuDispatcher;

    /// Reserves a new client ID.
    ///
    /// PX_DEFAULT_CLIENT is always available as the default clientID.
    /// Additional clients are returned by this function. Clients cannot be released once created.
    /// An error is reported when more than a supported number of clients (currently 128) are created.
    pub fn PxScene_createClient_mut(self_: *mut PxScene) -> u8;

    /// Sets a user notify object which receives special simulation events when they occur.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    pub fn PxScene_setSimulationEventCallback_mut(self_: *mut PxScene, callback: *mut PxSimulationEventCallback);

    /// Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().
    ///
    /// The current user notify pointer. See [`PxSimulationEventCallback`].
    pub fn PxScene_getSimulationEventCallback(self_: *const PxScene) -> *mut PxSimulationEventCallback;

    /// Sets a user callback object, which receives callbacks on all contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    pub fn PxScene_setContactModifyCallback_mut(self_: *mut PxScene, callback: *mut PxContactModifyCallback);

    /// Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    pub fn PxScene_setCCDContactModifyCallback_mut(self_: *mut PxScene, callback: *mut PxCCDContactModifyCallback);

    /// Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    pub fn PxScene_getContactModifyCallback(self_: *const PxScene) -> *mut PxContactModifyCallback;

    /// Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
    pub fn PxScene_getCCDContactModifyCallback(self_: *const PxScene) -> *mut PxCCDContactModifyCallback;

    /// Sets a broad-phase user callback object.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
    pub fn PxScene_setBroadPhaseCallback_mut(self_: *mut PxScene, callback: *mut PxBroadPhaseCallback);

    /// Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().
    ///
    /// The current broad-phase callback pointer. See [`PxBroadPhaseCallback`].
    pub fn PxScene_getBroadPhaseCallback(self_: *const PxScene) -> *mut PxBroadPhaseCallback;

    /// Sets the shared global filter data which will get passed into the filter shader.
    ///
    /// It is the user's responsibility to ensure that changing the shared global filter data does not change the filter output value for existing pairs.
    /// If the filter output for existing pairs does change nonetheless then such a change will not take effect until the pair gets refiltered.
    /// resetFiltering() can be used to explicitly refilter the pairs of specific objects.
    ///
    /// The provided data will get copied to internal buffers and this copy will be used for filtering calls.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    pub fn PxScene_setFilterShaderData_mut(self_: *mut PxScene, data: *const std::ffi::c_void, dataSize: u32);

    /// Gets the shared global filter data in use for this scene.
    ///
    /// The reference points to a copy of the original filter data specified in [`PxSceneDesc`].filterShaderData or provided by #setFilterShaderData().
    ///
    /// Shared filter data for filter shader.
    pub fn PxScene_getFilterShaderData(self_: *const PxScene) -> *const std::ffi::c_void;

    /// Gets the size of the shared global filter data ([`PxSceneDesc`].filterShaderData)
    ///
    /// Size of shared filter data [bytes].
    pub fn PxScene_getFilterShaderDataSize(self_: *const PxScene) -> u32;

    /// Marks the object to reset interactions and re-run collision filters in the next simulation step.
    ///
    /// This call forces the object to remove all existing collision interactions, to search anew for existing contact
    /// pairs and to run the collision filters again for found collision pairs.
    ///
    /// The operation is supported for PxRigidActor objects only.
    ///
    /// All persistent state of existing interactions will be lost and can not be retrieved even if the same collison pair
    /// is found again in the next step. This will mean, for example, that you will not get notified about persistent contact
    /// for such an interaction (see [`PxPairFlag::eNOTIFY_TOUCH_PERSISTS`]), the contact pair will be interpreted as newly found instead.
    ///
    /// Lost touch contact reports will be sent for every collision pair which includes this shape, if they have
    /// been requested through [`PxPairFlag::eNOTIFY_TOUCH_LOST`] or #PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST.
    ///
    /// This is an expensive operation, don't use it if you don't have to.
    ///
    /// Can be used to retrieve collision pairs that were killed by the collision filters (see [`PxFilterFlag::eKILL`])
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does wake up the actor.
    ///
    /// True if success
    pub fn PxScene_resetFiltering_mut(self_: *mut PxScene, actor: *mut PxActor) -> bool;

    /// Marks the object to reset interactions and re-run collision filters for specified shapes in the next simulation step.
    ///
    /// This is a specialization of the resetFiltering(PxActor
    /// &
    /// actor) method and allows to reset interactions for specific shapes of
    /// a PxRigidActor.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does wake up the actor.
    pub fn PxScene_resetFiltering_mut_1(self_: *mut PxScene, actor: *mut PxRigidActor, shapes: *const *mut PxShape, shapeCount: u32) -> bool;

    /// Gets the pair filtering mode for kinematic-kinematic pairs.
    ///
    /// Filtering mode for kinematic-kinematic pairs.
    pub fn PxScene_getKinematicKinematicFilteringMode(self_: *const PxScene) -> PxPairFilteringMode;

    /// Gets the pair filtering mode for static-kinematic pairs.
    ///
    /// Filtering mode for static-kinematic pairs.
    pub fn PxScene_getStaticKinematicFilteringMode(self_: *const PxScene) -> PxPairFilteringMode;

    /// Advances the simulation by an elapsedTime time.
    ///
    /// Large elapsedTime values can lead to instabilities. In such cases elapsedTime
    /// should be subdivided into smaller time intervals and simulate() should be called
    /// multiple times for each interval.
    ///
    /// Calls to simulate() should pair with calls to fetchResults():
    /// Each fetchResults() invocation corresponds to exactly one simulate()
    /// invocation; calling simulate() twice without an intervening fetchResults()
    /// or fetchResults() twice without an intervening simulate() causes an error
    /// condition.
    ///
    /// scene->simulate();
    /// ...do some processing until physics is computed...
    /// scene->fetchResults();
    /// ...now results of run may be retrieved.
    ///
    /// True if success
    pub fn PxScene_simulate_mut(self_: *mut PxScene, elapsedTime: f32, completionTask: *mut PxBaseTask, scratchMemBlock: *mut std::ffi::c_void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool;

    /// Performs dynamics phase of the simulation pipeline.
    ///
    /// Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.
    ///
    /// True if success
    pub fn PxScene_advance_mut(self_: *mut PxScene, completionTask: *mut PxBaseTask) -> bool;

    /// Performs collision detection for the scene over elapsedTime
    ///
    /// Calls to collide() should be the first method called to simulate a frame.
    ///
    /// True if success
    pub fn PxScene_collide_mut(self_: *mut PxScene, elapsedTime: f32, completionTask: *mut PxBaseTask, scratchMemBlock: *mut std::ffi::c_void, scratchMemBlockSize: u32, controlSimulation: bool) -> bool;

    /// This checks to see if the simulation run has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
    pub fn PxScene_checkResults_mut(self_: *mut PxScene, block: bool) -> bool;

    /// This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error
    /// message.
    pub fn PxScene_fetchCollision_mut(self_: *mut PxScene, block: bool) -> bool;

    /// This is the big brother to checkResults() it basically does the following:
    ///
    /// True if the results have been fetched.
    pub fn PxScene_fetchResults_mut(self_: *mut PxScene, block: bool, errorState: *mut u32) -> bool;

    /// This call performs the first section of fetchResults, and returns a pointer to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
    /// for fetchResults() performance.
    ///
    /// After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
    /// permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().
    ///
    /// True if the results have been fetched.
    pub fn PxScene_fetchResultsStart_mut(self_: *mut PxScene, contactPairs: *mut *const PxContactPairHeader, nbContactPairs: *mut u32, block: bool) -> bool;

    /// This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.
    ///
    /// This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with
    /// fetchResultsStart(...) and fetchResultsFinish(...)
    pub fn PxScene_processCallbacks_mut(self_: *mut PxScene, continuation: *mut PxBaseTask);

    /// This call performs the second section of fetchResults.
    ///
    /// It must be called after fetchResultsStart() returns and contact reports have been processed.
    ///
    /// Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.
    pub fn PxScene_fetchResultsFinish_mut(self_: *mut PxScene, errorState: *mut u32);

    /// This call performs the synchronization of particle system data copies.
    pub fn PxScene_fetchResultsParticleSystem_mut(self_: *mut PxScene);

    /// Clear internal buffers and free memory.
    ///
    /// This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
    /// the physics data gets streamed in and a checkpoint with a clean state should be created.
    ///
    /// It is not allowed to call this method while the simulation is running. The call will fail.
    pub fn PxScene_flushSimulation_mut(self_: *mut PxScene, sendPendingReports: bool);

    /// Sets a constant gravity for the entire scene.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    pub fn PxScene_setGravity_mut(self_: *mut PxScene, vec: *const PxVec3);

    /// Retrieves the current gravity setting.
    ///
    /// The current gravity for the scene.
    pub fn PxScene_getGravity(self_: *const PxScene) -> PxVec3;

    /// Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setBounceThresholdVelocity_mut(self_: *mut PxScene, t: f32);

    /// Return the bounce threshold velocity.
    pub fn PxScene_getBounceThresholdVelocity(self_: *const PxScene) -> f32;

    /// Sets the maximum number of CCD passes
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setCCDMaxPasses_mut(self_: *mut PxScene, ccdMaxPasses: u32);

    /// Gets the maximum number of CCD passes.
    ///
    /// The maximum number of CCD passes.
    pub fn PxScene_getCCDMaxPasses(self_: *const PxScene) -> u32;

    /// Set the maximum CCD separation.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setCCDMaxSeparation_mut(self_: *mut PxScene, t: f32);

    /// Gets the maximum CCD separation.
    ///
    /// The maximum CCD separation.
    pub fn PxScene_getCCDMaxSeparation(self_: *const PxScene) -> f32;

    /// Set the CCD threshold.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setCCDThreshold_mut(self_: *mut PxScene, t: f32);

    /// Gets the CCD threshold.
    ///
    /// The CCD threshold.
    pub fn PxScene_getCCDThreshold(self_: *const PxScene) -> f32;

    /// Set the max bias coefficient.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setMaxBiasCoefficient_mut(self_: *mut PxScene, t: f32);

    /// Gets the max bias coefficient.
    ///
    /// The max bias coefficient.
    pub fn PxScene_getMaxBiasCoefficient(self_: *const PxScene) -> f32;

    /// Set the friction offset threshold.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setFrictionOffsetThreshold_mut(self_: *mut PxScene, t: f32);

    /// Gets the friction offset threshold.
    pub fn PxScene_getFrictionOffsetThreshold(self_: *const PxScene) -> f32;

    /// Set the friction correlation distance.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setFrictionCorrelationDistance_mut(self_: *mut PxScene, t: f32);

    /// Gets the friction correlation distance.
    pub fn PxScene_getFrictionCorrelationDistance(self_: *const PxScene) -> f32;

    /// Return the friction model.
    pub fn PxScene_getFrictionType(self_: *const PxScene) -> PxFrictionType;

    /// Return the solver model.
    pub fn PxScene_getSolverType(self_: *const PxScene) -> PxSolverType;

    /// Function that lets you set debug visualization parameters.
    ///
    /// Returns false if the value passed is out of range for usage specified by the enum.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// False if the parameter is out of range.
    pub fn PxScene_setVisualizationParameter_mut(self_: *mut PxScene, param: PxVisualizationParameter, value: f32) -> bool;

    /// Function that lets you query debug visualization parameters.
    ///
    /// The value of the parameter.
    pub fn PxScene_getVisualizationParameter(self_: *const PxScene, paramEnum: PxVisualizationParameter) -> f32;

    /// Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setVisualizationCullingBox_mut(self_: *mut PxScene, box_: *const PxBounds3);

    /// Retrieves the visualization culling box.
    ///
    /// the box to which the geometry will be culled.
    pub fn PxScene_getVisualizationCullingBox(self_: *const PxScene) -> PxBounds3;

    /// Retrieves the render buffer.
    ///
    /// This will contain the results of any active visualization for this scene.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will result in undefined behaviour.
    ///
    /// The render buffer.
    pub fn PxScene_getRenderBuffer_mut(self_: *mut PxScene) -> *const PxRenderBuffer;

    /// Call this method to retrieve statistics for the current simulation step.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    pub fn PxScene_getSimulationStatistics(self_: *const PxScene, stats: *mut PxSimulationStatistics);

    /// Returns broad-phase type.
    ///
    /// Broad-phase type
    pub fn PxScene_getBroadPhaseType(self_: *const PxScene) -> PxBroadPhaseType;

    /// Gets broad-phase caps.
    ///
    /// True if success
    pub fn PxScene_getBroadPhaseCaps(self_: *const PxScene, caps: *mut PxBroadPhaseCaps) -> bool;

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
    pub fn PxScene_getNbBroadPhaseRegions(self_: *const PxScene) -> u32;

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions
    pub fn PxScene_getBroadPhaseRegions(self_: *const PxScene, userBuffer: *mut PxBroadPhaseRegionInfo, bufferSize: u32, startIndex: u32) -> u32;

    /// Adds a new broad-phase region.
    ///
    /// The bounds for the new region must be non-empty, otherwise an error occurs and the call is ignored.
    ///
    /// Note that by default, objects already existing in the SDK that might touch this region will not be automatically
    /// added to the region. In other words the newly created region will be empty, and will only be populated with new
    /// objects when they are added to the simulation, or with already existing objects when they are updated.
    ///
    /// It is nonetheless possible to override this default behavior and let the SDK populate the new region automatically
    /// with already existing objects overlapping the incoming region. This has a cost though, and it should only be used
    /// when the game can not guarantee that all objects within the new region will be added to the simulation after the
    /// region itself.
    ///
    /// Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
    /// regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
    /// object leaves all regions, or is created outside of all regions, several things happen:
    /// - collisions get disabled for this object
    /// - if a PxBroadPhaseCallback object is provided, an "out-of-bounds" event is generated via that callback
    /// - if a PxBroadPhaseCallback object is not provided, a warning/error message is sent to the error stream
    ///
    /// If an object goes out-of-bounds and user deletes it during the same frame, neither the out-of-bounds event nor the
    /// error message is generated.
    ///
    /// Handle for newly created region, or 0xffffffff in case of failure.
    pub fn PxScene_addBroadPhaseRegion_mut(self_: *mut PxScene, region: *const PxBroadPhaseRegion, populateRegion: bool) -> u32;

    /// Removes a new broad-phase region.
    ///
    /// If the region still contains objects, and if those objects do not overlap any region any more, they are not
    /// automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
    /// is used for each object. Users are responsible for removing the objects from the simulation if this is the
    /// desired behavior.
    ///
    /// If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    ///
    /// True if success
    pub fn PxScene_removeBroadPhaseRegion_mut(self_: *mut PxScene, handle: u32) -> bool;

    /// Get the task manager associated with this scene
    ///
    /// the task manager associated with the scene
    pub fn PxScene_getTaskManager(self_: *const PxScene) -> *mut PxTaskManager;

    /// Lock the scene for reading from the calling thread.
    ///
    /// When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockRead() must be
    /// called before any read calls are made on the scene.
    ///
    /// Multiple threads may read at the same time, no threads may read while a thread is writing.
    /// If a call to lockRead() is made while another thread is holding a write lock
    /// then the calling thread will be blocked until the writing thread calls unlockWrite().
    ///
    /// Lock upgrading is *not* supported, that means it is an error to
    /// call lockRead() followed by lockWrite().
    ///
    /// Recursive locking is supported but each lockRead() call must be paired with an unlockRead().
    pub fn PxScene_lockRead_mut(self_: *mut PxScene, file: *const std::ffi::c_char, line: u32);

    /// Unlock the scene from reading.
    ///
    /// Each unlockRead() must be paired with a lockRead() from the same thread.
    pub fn PxScene_unlockRead_mut(self_: *mut PxScene);

    /// Lock the scene for writing from this thread.
    ///
    /// When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockWrite() must be
    /// called before any write calls are made on the scene.
    ///
    /// Only one thread may write at a time and no threads may read while a thread is writing.
    /// If a call to lockWrite() is made and there are other threads reading then the
    /// calling thread will be blocked until the readers complete.
    ///
    /// Writers have priority. If a thread is blocked waiting to write then subsequent calls to
    /// lockRead() from other threads will be blocked until the writer completes.
    ///
    /// If multiple threads are waiting to write then the thread that is first
    /// granted access depends on OS scheduling.
    ///
    /// Recursive locking is supported but each lockWrite() call must be paired
    /// with an unlockWrite().
    ///
    /// If a thread has already locked the scene for writing then it may call
    /// lockRead().
    pub fn PxScene_lockWrite_mut(self_: *mut PxScene, file: *const std::ffi::c_char, line: u32);

    /// Unlock the scene from writing.
    ///
    /// Each unlockWrite() must be paired with a lockWrite() from the same thread.
    pub fn PxScene_unlockWrite_mut(self_: *mut PxScene);

    /// set the cache blocks that can be used during simulate().
    ///
    /// Each frame the simulation requires memory to store contact, friction, and contact cache data. This memory is used in blocks of 16K.
    /// Each frame the blocks used by the previous frame are freed, and may be retrieved by the application using PxScene::flushSimulation()
    ///
    /// This call will force allocation of cache blocks if the numBlocks parameter is greater than the currently allocated number
    /// of blocks, and less than the max16KContactDataBlocks parameter specified at scene creation time.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setNbContactDataBlocks_mut(self_: *mut PxScene, numBlocks: u32);

    /// get the number of cache blocks currently used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the number of cache blocks currently used by the scene
    pub fn PxScene_getNbContactDataBlocksUsed(self_: *const PxScene) -> u32;

    /// get the maximum number of cache blocks used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the maximum number of cache blocks everused by the scene
    pub fn PxScene_getMaxNbContactDataBlocksUsed(self_: *const PxScene) -> u32;

    /// Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene
    pub fn PxScene_getContactReportStreamBufferSize(self_: *const PxScene) -> u32;

    /// Sets the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setSolverBatchSize_mut(self_: *mut PxScene, solverBatchSize: u32);

    /// Retrieves the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Current number of actors required to spawn a separate rigid body solver thread.
    pub fn PxScene_getSolverBatchSize(self_: *const PxScene) -> u32;

    /// Sets the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
    pub fn PxScene_setSolverArticulationBatchSize_mut(self_: *mut PxScene, solverBatchSize: u32);

    /// Retrieves the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Current number of articulations required to spawn a separate rigid body solver thread.
    pub fn PxScene_getSolverArticulationBatchSize(self_: *const PxScene) -> u32;

    /// Returns the wake counter reset value.
    ///
    /// Wake counter reset value
    pub fn PxScene_getWakeCounterResetValue(self_: *const PxScene) -> f32;

    /// Shift the scene origin by the specified vector.
    ///
    /// The poses of all objects in the scene and the corresponding data structures will get adjusted to reflect the new origin location
    /// (the shift vector will get subtracted from all object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysX accordingly.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    ///
    /// Make sure to propagate the origin shift to other dependent modules (for example, the character controller module etc.).
    ///
    /// This is an expensive operation and we recommend to use it only in the case where distance related precision issues may arise in areas far from the origin.
    pub fn PxScene_shiftOrigin_mut(self_: *mut PxScene, shift: *const PxVec3);

    /// Returns the Pvd client associated with the scene.
    ///
    /// the client, NULL if no PVD supported.
    pub fn PxScene_getScenePvdClient_mut(self_: *mut PxScene) -> *mut PxPvdSceneClient;

    /// Copy GPU articulation data from the internal GPU buffer to a user-provided device buffer.
    pub fn PxScene_copyArticulationData_mut(self_: *mut PxScene, data: *mut std::ffi::c_void, index: *mut std::ffi::c_void, dataType: PxArticulationGpuDataType, nbCopyArticulations: u32, copyEvent: *mut std::ffi::c_void);

    /// Apply GPU articulation data from a user-provided device buffer to the internal GPU buffer.
    pub fn PxScene_applyArticulationData_mut(self_: *mut PxScene, data: *mut std::ffi::c_void, index: *mut std::ffi::c_void, dataType: PxArticulationGpuDataType, nbUpdatedArticulations: u32, waitEvent: *mut std::ffi::c_void, signalEvent: *mut std::ffi::c_void);

    /// Copy GPU softbody data from the internal GPU buffer to a user-provided device buffer.
    pub fn PxScene_copySoftBodyData_mut(self_: *mut PxScene, data: *mut *mut std::ffi::c_void, dataSizes: *mut std::ffi::c_void, softBodyIndices: *mut std::ffi::c_void, flag: PxSoftBodyDataFlag, nbCopySoftBodies: u32, maxSize: u32, copyEvent: *mut std::ffi::c_void);

    /// Apply user-provided data to the internal softbody system.
    pub fn PxScene_applySoftBodyData_mut(self_: *mut PxScene, data: *mut *mut std::ffi::c_void, dataSizes: *mut std::ffi::c_void, softBodyIndices: *mut std::ffi::c_void, flag: PxSoftBodyDataFlag, nbUpdatedSoftBodies: u32, maxSize: u32, applyEvent: *mut std::ffi::c_void);

    /// Copy contact data from the internal GPU buffer to a user-provided device buffer.
    ///
    /// The contact data contains pointers to internal state and is only valid until the next call to simulate().
    pub fn PxScene_copyContactData_mut(self_: *mut PxScene, data: *mut std::ffi::c_void, maxContactPairs: u32, numContactPairs: *mut std::ffi::c_void, copyEvent: *mut std::ffi::c_void);

    /// Copy GPU rigid body data from the internal GPU buffer to a user-provided device buffer.
    pub fn PxScene_copyBodyData_mut(self_: *mut PxScene, data: *mut PxGpuBodyData, index: *mut PxGpuActorPair, nbCopyActors: u32, copyEvent: *mut std::ffi::c_void);

    /// Apply user-provided data to rigid body.
    pub fn PxScene_applyActorData_mut(self_: *mut PxScene, data: *mut std::ffi::c_void, index: *mut PxGpuActorPair, flag: PxActorCacheFlag, nbUpdatedActors: u32, waitEvent: *mut std::ffi::c_void, signalEvent: *mut std::ffi::c_void);

    /// Compute dense Jacobian matrices for specified articulations on the GPU.
    ///
    /// The size of Jacobians can vary by articulation, since it depends on the number of links, degrees-of-freedom, and whether the base is fixed.
    ///
    /// The size is determined using these formulas:
    /// nCols = (fixedBase ? 0 : 6) + dofCount
    /// nRows = (fixedBase ? 0 : 6) + (linkCount - 1) * 6;
    ///
    /// The user must ensure that adequate space is provided for each Jacobian matrix.
    pub fn PxScene_computeDenseJacobians_mut(self_: *mut PxScene, indices: *const PxIndexDataPair, nbIndices: u32, computeEvent: *mut std::ffi::c_void);

    /// Compute the joint-space inertia matrices that maps joint accelerations to joint forces: forces = M * accelerations on the GPU.
    ///
    /// The size of matrices can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount * dofCount
    ///
    /// The user must ensure that adequate space is provided for each mass matrix.
    pub fn PxScene_computeGeneralizedMassMatrices_mut(self_: *mut PxScene, indices: *const PxIndexDataPair, nbIndices: u32, computeEvent: *mut std::ffi::c_void);

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    pub fn PxScene_computeGeneralizedGravityForces_mut(self_: *mut PxScene, indices: *const PxIndexDataPair, nbIndices: u32, computeEvent: *mut std::ffi::c_void);

    /// Computes the joint DOF forces required to counteract coriolis and centrifugal forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
    pub fn PxScene_computeCoriolisAndCentrifugalForces_mut(self_: *mut PxScene, indices: *const PxIndexDataPair, nbIndices: u32, computeEvent: *mut std::ffi::c_void);

    pub fn PxScene_getGpuDynamicsConfig(self_: *const PxScene) -> PxgDynamicsMemoryConfig;

    /// Apply user-provided data to particle buffers.
    ///
    /// This function should be used if the particle buffer flags are already on the device. Otherwise, use PxParticleBuffer::raiseFlags()
    /// from the CPU.
    ///
    /// This assumes the data has been changed directly in the PxParticleBuffer.
    pub fn PxScene_applyParticleBufferData_mut(self_: *mut PxScene, indices: *const u32, bufferIndexPair: *const PxGpuParticleBufferIndexPair, flags: *const PxParticleBufferFlags, nbUpdatedBuffers: u32, waitEvent: *mut std::ffi::c_void, signalEvent: *mut std::ffi::c_void);

    /// Constructor
    pub fn PxSceneReadLock_new_alloc(scene: *mut PxScene, file: *const std::ffi::c_char, line: u32) -> *mut PxSceneReadLock;

    pub fn PxSceneReadLock_delete(self_: *mut PxSceneReadLock);

    /// Constructor
    pub fn PxSceneWriteLock_new_alloc(scene: *mut PxScene, file: *const std::ffi::c_char, line: u32) -> *mut PxSceneWriteLock;

    pub fn PxSceneWriteLock_delete(self_: *mut PxSceneWriteLock);

    pub fn PxContactPairExtraDataItem_new() -> PxContactPairExtraDataItem;

    pub fn PxContactPairVelocity_new() -> PxContactPairVelocity;

    pub fn PxContactPairPose_new() -> PxContactPairPose;

    pub fn PxContactPairIndex_new() -> PxContactPairIndex;

    /// Constructor
    pub fn PxContactPairExtraDataIterator_new(stream: *const u8, size: u32) -> PxContactPairExtraDataIterator;

    /// Advances the iterator to next set of extra data items.
    ///
    /// The contact pair extra data stream contains sets of items as requested by the corresponding [`PxPairFlag`] flags
    /// [`PxPairFlag::ePRE_SOLVER_VELOCITY`], #PxPairFlag::ePOST_SOLVER_VELOCITY, #PxPairFlag::eCONTACT_EVENT_POSE. A set can contain one
    /// item of each plus the PxContactPairIndex item. This method parses the stream and points the iterator
    /// member variables to the corresponding items of the current set, if they are available. If CCD is not enabled,
    /// you should only get one set of items. If CCD with multiple passes is enabled, you might get more than one item
    /// set.
    ///
    /// Even though contact pair extra data is requested per shape pair, you will not get an item set per shape pair
    /// but one per actor pair. If, for example, an actor has two shapes and both collide with another actor, then
    /// there will only be one item set (since it applies to both shape pairs).
    ///
    /// True if there was another set of extra data items in the stream, else false.
    pub fn PxContactPairExtraDataIterator_nextItemSet_mut(self_: *mut PxContactPairExtraDataIterator) -> bool;

    pub fn PxContactPairHeader_new() -> PxContactPairHeader;

    pub fn PxContactPair_new() -> PxContactPair;

    /// Extracts the contact points from the stream and stores them in a convenient format.
    ///
    /// Number of contact points written to the buffer.
    pub fn PxContactPair_extractContacts(self_: *const PxContactPair, userBuffer: *mut PxContactPairPoint, bufferSize: u32) -> u32;

    /// Helper method to clone the contact pair and copy the contact data stream into a user buffer.
    ///
    /// The contact data stream is only accessible during the contact report callback. This helper function provides copy functionality
    /// to buffer the contact stream information such that it can get accessed at a later stage.
    pub fn PxContactPair_bufferContacts(self_: *const PxContactPair, newPair: *mut PxContactPair, bufferMemory: *mut u8);

    pub fn PxContactPair_getInternalFaceIndices(self_: *const PxContactPair) -> *const u32;

    pub fn PxTriggerPair_new() -> PxTriggerPair;

    pub fn PxConstraintInfo_new() -> PxConstraintInfo;

    pub fn PxConstraintInfo_new_1(c: *mut PxConstraint, extRef: *mut std::ffi::c_void, t: u32) -> PxConstraintInfo;

    /// This is called when a breakable constraint breaks.
    ///
    /// The user should not release the constraint shader inside this call!
    ///
    /// No event will get reported if the constraint breaks but gets deleted while the time step is still being simulated.
    pub fn PxSimulationEventCallback_onConstraintBreak_mut(self_: *mut PxSimulationEventCallback, constraints: *mut PxConstraintInfo, count: u32);

    /// This is called with the actors which have just been woken up.
    ///
    /// Only supported by rigid bodies yet.
    ///
    /// Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
    ///
    /// Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
    /// will get reported. For example, let us assume actor A is awake, then A->putToSleep() gets called, then later A->wakeUp() gets called.
    /// At the next simulate/fetchResults() step only an onWake() event will get triggered because that was the last transition.
    ///
    /// If an actor gets newly added to a scene with properties such that it is awake and the sleep state does not get changed by
    /// the user or simulation, then an onWake() event will get sent at the next simulate/fetchResults() step.
    pub fn PxSimulationEventCallback_onWake_mut(self_: *mut PxSimulationEventCallback, actors: *mut *mut PxActor, count: u32);

    /// This is called with the actors which have just been put to sleep.
    ///
    /// Only supported by rigid bodies yet.
    ///
    /// Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
    ///
    /// Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
    /// will get reported. For example, let us assume actor A is asleep, then A->wakeUp() gets called, then later A->putToSleep() gets called.
    /// At the next simulate/fetchResults() step only an onSleep() event will get triggered because that was the last transition (assuming the simulation
    /// does not wake the actor up).
    ///
    /// If an actor gets newly added to a scene with properties such that it is asleep and the sleep state does not get changed by
    /// the user or simulation, then an onSleep() event will get sent at the next simulate/fetchResults() step.
    pub fn PxSimulationEventCallback_onSleep_mut(self_: *mut PxSimulationEventCallback, actors: *mut *mut PxActor, count: u32);

    /// This is called when certain contact events occur.
    ///
    /// The method will be called for a pair of actors if one of the colliding shape pairs requested contact notification.
    /// You request which events are reported using the filter shader/callback mechanism (see [`PxSimulationFilterShader`],
    /// [`PxSimulationFilterCallback`], #PxPairFlag).
    ///
    /// Do not keep references to the passed objects, as they will be
    /// invalid after this function returns.
    pub fn PxSimulationEventCallback_onContact_mut(self_: *mut PxSimulationEventCallback, pairHeader: *const PxContactPairHeader, pairs: *const PxContactPair, nbPairs: u32);

    /// This is called with the current trigger pair events.
    ///
    /// Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
    /// according to the pair flag specification in the filter shader (see [`PxPairFlag`], #PxSimulationFilterShader).
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
    pub fn PxSimulationEventCallback_onTrigger_mut(self_: *mut PxSimulationEventCallback, pairs: *mut PxTriggerPair, count: u32);

    /// Provides early access to the new pose of moving rigid bodies.
    ///
    /// When this call occurs, rigid bodies having the [`PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW`]
    /// flag set, were moved by the simulation and their new poses can be accessed through the provided buffers.
    ///
    /// The provided buffers are valid and can be read until the next call to [`PxScene::simulate`]() or #PxScene::collide().
    ///
    /// This callback gets triggered while the simulation is running. If the provided rigid body references are used to
    /// read properties of the object, then the callback has to guarantee no other thread is writing to the same body at the same
    /// time.
    ///
    /// The code in this callback should be lightweight as it can block the simulation, that is, the
    /// [`PxScene::fetchResults`]() call.
    pub fn PxSimulationEventCallback_onAdvance_mut(self_: *mut PxSimulationEventCallback, bodyBuffer: *const *const PxRigidBody, poseBuffer: *const PxTransform, count: u32);

    pub fn PxSimulationEventCallback_delete(self_: *mut PxSimulationEventCallback);

    pub fn PxFEMParameters_new() -> PxFEMParameters;

    /// Release this object.
    pub fn PxPruningStructure_release_mut(self_: *mut PxPruningStructure);

    /// Retrieve rigid actors in the pruning structure.
    ///
    /// You can retrieve the number of rigid actor pointers by calling [`getNbRigidActors`]()
    ///
    /// Number of rigid actor pointers written to the buffer.
    pub fn PxPruningStructure_getRigidActors(self_: *const PxPruningStructure, userBuffer: *mut *mut PxRigidActor, bufferSize: u32, startIndex: u32) -> u32;

    /// Returns the number of rigid actors in the pruning structure.
    ///
    /// You can use [`getRigidActors`]() to retrieve the rigid actor pointers.
    ///
    /// Number of rigid actors in the pruning structure.
    pub fn PxPruningStructure_getNbRigidActors(self_: *const PxPruningStructure) -> u32;

    /// Gets the merge data for static actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for static actors.
    pub fn PxPruningStructure_getStaticMergeData(self_: *const PxPruningStructure) -> *const std::ffi::c_void;

    /// Gets the merge data for dynamic actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for dynamic actors.
    pub fn PxPruningStructure_getDynamicMergeData(self_: *const PxPruningStructure) -> *const std::ffi::c_void;

    pub fn PxPruningStructure_getConcreteTypeName(self_: *const PxPruningStructure) -> *const std::ffi::c_char;

    pub fn PxExtendedVec3_new() -> PxExtendedVec3;

    pub fn PxExtendedVec3_new_1(_x: f64, _y: f64, _z: f64) -> PxExtendedVec3;

    pub fn PxExtendedVec3_isZero(self_: *const PxExtendedVec3) -> bool;

    pub fn PxExtendedVec3_dot(self_: *const PxExtendedVec3, v: *const PxVec3) -> f64;

    pub fn PxExtendedVec3_distanceSquared(self_: *const PxExtendedVec3, v: *const PxExtendedVec3) -> f64;

    pub fn PxExtendedVec3_magnitudeSquared(self_: *const PxExtendedVec3) -> f64;

    pub fn PxExtendedVec3_magnitude(self_: *const PxExtendedVec3) -> f64;

    pub fn PxExtendedVec3_normalize_mut(self_: *mut PxExtendedVec3) -> f64;

    pub fn PxExtendedVec3_isFinite(self_: *const PxExtendedVec3) -> bool;

    pub fn PxExtendedVec3_maximum_mut(self_: *mut PxExtendedVec3, v: *const PxExtendedVec3);

    pub fn PxExtendedVec3_minimum_mut(self_: *mut PxExtendedVec3, v: *const PxExtendedVec3);

    pub fn PxExtendedVec3_set_mut(self_: *mut PxExtendedVec3, x_: f64, y_: f64, z_: f64);

    pub fn PxExtendedVec3_setPlusInfinity_mut(self_: *mut PxExtendedVec3);

    pub fn PxExtendedVec3_setMinusInfinity_mut(self_: *mut PxExtendedVec3);

    pub fn PxExtendedVec3_cross_mut(self_: *mut PxExtendedVec3, left: *const PxExtendedVec3, right: *const PxVec3);

    pub fn PxExtendedVec3_cross_mut_1(self_: *mut PxExtendedVec3, left: *const PxExtendedVec3, right: *const PxExtendedVec3);

    pub fn PxExtendedVec3_cross(self_: *const PxExtendedVec3, v: *const PxExtendedVec3) -> PxExtendedVec3;

    pub fn PxExtendedVec3_cross_mut_2(self_: *mut PxExtendedVec3, left: *const PxVec3, right: *const PxExtendedVec3);

    pub fn phys_toVec3(v: *const PxExtendedVec3) -> PxVec3;

    pub fn PxObstacle_getType(self_: *const PxObstacle) -> PxGeometryType;

    pub fn PxBoxObstacle_new() -> PxBoxObstacle;

    pub fn PxCapsuleObstacle_new() -> PxCapsuleObstacle;

    /// Releases the context.
    pub fn PxObstacleContext_release_mut(self_: *mut PxObstacleContext);

    /// Retrieves the controller manager associated with this context.
    ///
    /// The associated controller manager
    pub fn PxObstacleContext_getControllerManager(self_: *const PxObstacleContext) -> *mut PxControllerManager;

    /// Adds an obstacle to the context.
    ///
    /// Handle for newly-added obstacle
    pub fn PxObstacleContext_addObstacle_mut(self_: *mut PxObstacleContext, obstacle: *const PxObstacle) -> u32;

    /// Removes an obstacle from the context.
    ///
    /// True if success
    pub fn PxObstacleContext_removeObstacle_mut(self_: *mut PxObstacleContext, handle: u32) -> bool;

    /// Updates data for an existing obstacle.
    ///
    /// True if success
    pub fn PxObstacleContext_updateObstacle_mut(self_: *mut PxObstacleContext, handle: u32, obstacle: *const PxObstacle) -> bool;

    /// Retrieves number of obstacles in the context.
    ///
    /// Number of obstacles in the context
    pub fn PxObstacleContext_getNbObstacles(self_: *const PxObstacleContext) -> u32;

    /// Retrieves desired obstacle.
    ///
    /// Desired obstacle
    pub fn PxObstacleContext_getObstacle(self_: *const PxObstacleContext, i: u32) -> *const PxObstacle;

    /// Retrieves desired obstacle by given handle.
    ///
    /// Desired obstacle
    pub fn PxObstacleContext_getObstacleByHandle(self_: *const PxObstacleContext, handle: u32) -> *const PxObstacle;

    /// Called when current controller hits a shape.
    ///
    /// This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.
    pub fn PxUserControllerHitReport_onShapeHit_mut(self_: *mut PxUserControllerHitReport, hit: *const PxControllerShapeHit);

    /// Called when current controller hits another controller.
    pub fn PxUserControllerHitReport_onControllerHit_mut(self_: *mut PxUserControllerHitReport, hit: *const PxControllersHit);

    /// Called when current controller hits a user-defined obstacle.
    pub fn PxUserControllerHitReport_onObstacleHit_mut(self_: *mut PxUserControllerHitReport, hit: *const PxControllerObstacleHit);

    pub fn PxControllerFilterCallback_delete(self_: *mut PxControllerFilterCallback);

    /// Filtering method for CCT-vs-CCT.
    ///
    /// true to keep the pair, false to filter it out
    pub fn PxControllerFilterCallback_filter_mut(self_: *mut PxControllerFilterCallback, a: *const PxController, b: *const PxController) -> bool;

    pub fn PxControllerFilters_new(filterData: *const PxFilterData, cb: *mut PxQueryFilterCallback, cctFilterCb: *mut PxControllerFilterCallback) -> PxControllerFilters;

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    pub fn PxControllerDesc_isValid(self_: *const PxControllerDesc) -> bool;

    /// Returns the character controller type
    ///
    /// The controllers type.
    pub fn PxControllerDesc_getType(self_: *const PxControllerDesc) -> PxControllerShapeType;

    /// Return the type of controller
    pub fn PxController_getType(self_: *const PxController) -> PxControllerShapeType;

    /// Releases the controller.
    pub fn PxController_release_mut(self_: *mut PxController);

    /// Moves the character using a "collide-and-slide" algorithm.
    ///
    /// Collision flags, collection of ::PxControllerCollisionFlags
    pub fn PxController_move_mut(self_: *mut PxController, disp: *const PxVec3, minDist: f32, elapsedTime: f32, filters: *const PxControllerFilters, obstacles: *const PxObstacleContext) -> PxControllerCollisionFlags;

    /// Sets controller's position.
    ///
    /// The position controlled by this function is the center of the collision shape.
    ///
    /// This is a 'teleport' function, it doesn't check for collisions.
    ///
    /// The character's position must be such that it does not overlap the static geometry.
    ///
    /// To move the character under normal conditions use the [`move`]() function.
    ///
    /// Currently always returns true.
    pub fn PxController_setPosition_mut(self_: *mut PxController, position: *const PxExtendedVec3) -> bool;

    /// Retrieve the raw position of the controller.
    ///
    /// The position retrieved by this function is the center of the collision shape. To retrieve the bottom position of the shape,
    /// a.k.a. the foot position, use the getFootPosition() function.
    ///
    /// The position is updated by calls to move(). Calling this method without calling
    /// move() will return the last position or the initial position of the controller.
    ///
    /// The controller's center position
    pub fn PxController_getPosition(self_: *const PxController) -> *const PxExtendedVec3;

    /// Set controller's foot position.
    ///
    /// The position controlled by this function is the bottom of the collision shape, a.k.a. the foot position.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// This is a 'teleport' function, it doesn't check for collisions.
    ///
    /// To move the character under normal conditions use the [`move`]() function.
    ///
    /// Currently always returns true.
    pub fn PxController_setFootPosition_mut(self_: *mut PxController, position: *const PxExtendedVec3) -> bool;

    /// Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// The controller's foot position
    pub fn PxController_getFootPosition(self_: *const PxController) -> PxExtendedVec3;

    /// Get the rigid body actor associated with this controller (see PhysX documentation).
    /// The behavior upon manually altering this actor is undefined, you should primarily
    /// use it for reading const properties.
    ///
    /// the actor associated with the controller.
    pub fn PxController_getActor(self_: *const PxController) -> *mut PxRigidDynamic;

    /// The step height.
    pub fn PxController_setStepOffset_mut(self_: *mut PxController, offset: f32);

    /// Retrieve the step height.
    ///
    /// The step offset for the controller.
    pub fn PxController_getStepOffset(self_: *const PxController) -> f32;

    /// Sets the non-walkable mode for the CCT.
    pub fn PxController_setNonWalkableMode_mut(self_: *mut PxController, flag: PxControllerNonWalkableMode);

    /// Retrieves the non-walkable mode for the CCT.
    ///
    /// The current non-walkable mode.
    pub fn PxController_getNonWalkableMode(self_: *const PxController) -> PxControllerNonWalkableMode;

    /// Retrieve the contact offset.
    ///
    /// The contact offset for the controller.
    pub fn PxController_getContactOffset(self_: *const PxController) -> f32;

    /// Sets the contact offset.
    pub fn PxController_setContactOffset_mut(self_: *mut PxController, offset: f32);

    /// Retrieve the 'up' direction.
    ///
    /// The up direction for the controller.
    pub fn PxController_getUpDirection(self_: *const PxController) -> PxVec3;

    /// Sets the 'up' direction.
    pub fn PxController_setUpDirection_mut(self_: *mut PxController, up: *const PxVec3);

    /// Retrieve the slope limit.
    ///
    /// The slope limit for the controller.
    pub fn PxController_getSlopeLimit(self_: *const PxController) -> f32;

    /// Sets the slope limit.
    ///
    /// This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
    /// (which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
    /// will be ignored.
    pub fn PxController_setSlopeLimit_mut(self_: *mut PxController, slopeLimit: f32);

    /// Flushes internal geometry cache.
    ///
    /// The character controller uses caching in order to speed up collision testing. The cache is
    /// automatically flushed when a change to static objects is detected in the scene. For example when a
    /// static shape is added, updated, or removed from the scene, the cache is automatically invalidated.
    ///
    /// However there may be situations that cannot be automatically detected, and those require manual
    /// invalidation of the cache. Currently the user must call this when the filtering behavior changes (the
    /// PxControllerFilters parameter of the PxController::move call).  While the controller in principle
    /// could detect a change in these parameters, it cannot detect a change in the behavior of the filtering
    /// function.
    pub fn PxController_invalidateCache_mut(self_: *mut PxController);

    /// Retrieve the scene associated with the controller.
    ///
    /// The physics scene
    pub fn PxController_getScene_mut(self_: *mut PxController) -> *mut PxScene;

    /// Returns the user data associated with this controller.
    ///
    /// The user pointer associated with the controller.
    pub fn PxController_getUserData(self_: *const PxController) -> *mut std::ffi::c_void;

    /// Sets the user data associated with this controller.
    pub fn PxController_setUserData_mut(self_: *mut PxController, userData: *mut std::ffi::c_void);

    /// Returns information about the controller's internal state.
    pub fn PxController_getState(self_: *const PxController, state: *mut PxControllerState);

    /// Returns the controller's internal statistics.
    pub fn PxController_getStats(self_: *const PxController, stats: *mut PxControllerStats);

    /// Resizes the controller.
    ///
    /// This function attempts to resize the controller to a given size, while making sure the bottom
    /// position of the controller remains constant. In other words the function modifies both the
    /// height and the (center) position of the controller. This is a helper function that can be used
    /// to implement a 'crouch' functionality for example.
    pub fn PxController_resize_mut(self_: *mut PxController, height: f32);

    /// constructor sets to default.
    pub fn PxBoxControllerDesc_new_alloc() -> *mut PxBoxControllerDesc;

    pub fn PxBoxControllerDesc_delete(self_: *mut PxBoxControllerDesc);

    /// (re)sets the structure to the default.
    pub fn PxBoxControllerDesc_setToDefault_mut(self_: *mut PxBoxControllerDesc);

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    pub fn PxBoxControllerDesc_isValid(self_: *const PxBoxControllerDesc) -> bool;

    /// Gets controller's half height.
    ///
    /// The half height of the controller.
    pub fn PxBoxController_getHalfHeight(self_: *const PxBoxController) -> f32;

    /// Gets controller's half side extent.
    ///
    /// The half side extent of the controller.
    pub fn PxBoxController_getHalfSideExtent(self_: *const PxBoxController) -> f32;

    /// Gets controller's half forward extent.
    ///
    /// The half forward extent of the controller.
    pub fn PxBoxController_getHalfForwardExtent(self_: *const PxBoxController) -> f32;

    /// Sets controller's half height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    pub fn PxBoxController_setHalfHeight_mut(self_: *mut PxBoxController, halfHeight: f32) -> bool;

    /// Sets controller's half side extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    pub fn PxBoxController_setHalfSideExtent_mut(self_: *mut PxBoxController, halfSideExtent: f32) -> bool;

    /// Sets controller's half forward extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    pub fn PxBoxController_setHalfForwardExtent_mut(self_: *mut PxBoxController, halfForwardExtent: f32) -> bool;

    /// constructor sets to default.
    pub fn PxCapsuleControllerDesc_new_alloc() -> *mut PxCapsuleControllerDesc;

    pub fn PxCapsuleControllerDesc_delete(self_: *mut PxCapsuleControllerDesc);

    /// (re)sets the structure to the default.
    pub fn PxCapsuleControllerDesc_setToDefault_mut(self_: *mut PxCapsuleControllerDesc);

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
    pub fn PxCapsuleControllerDesc_isValid(self_: *const PxCapsuleControllerDesc) -> bool;

    /// Gets controller's radius.
    ///
    /// The radius of the controller.
    pub fn PxCapsuleController_getRadius(self_: *const PxCapsuleController) -> f32;

    /// Sets controller's radius.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    pub fn PxCapsuleController_setRadius_mut(self_: *mut PxCapsuleController, radius: f32) -> bool;

    /// Gets controller's height.
    ///
    /// The height of the capsule controller.
    pub fn PxCapsuleController_getHeight(self_: *const PxCapsuleController) -> f32;

    /// Resets controller's height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
    pub fn PxCapsuleController_setHeight_mut(self_: *mut PxCapsuleController, height: f32) -> bool;

    /// Gets controller's climbing mode.
    ///
    /// The capsule controller's climbing mode.
    pub fn PxCapsuleController_getClimbingMode(self_: *const PxCapsuleController) -> PxCapsuleClimbingMode;

    /// Sets controller's climbing mode.
    pub fn PxCapsuleController_setClimbingMode_mut(self_: *mut PxCapsuleController, mode: PxCapsuleClimbingMode) -> bool;

    /// Retrieve behavior flags for a shape.
    ///
    /// When the CCT touches a shape, the CCT's behavior w.r.t. this shape can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given shape
    pub fn PxControllerBehaviorCallback_getBehaviorFlags_mut(self_: *mut PxControllerBehaviorCallback, shape: *const PxShape, actor: *const PxActor) -> PxControllerBehaviorFlags;

    /// Retrieve behavior flags for a controller.
    ///
    /// When the CCT touches a controller, the CCT's behavior w.r.t. this controller can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// The flag PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT is not supported.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given controller
    pub fn PxControllerBehaviorCallback_getBehaviorFlags_mut_1(self_: *mut PxControllerBehaviorCallback, controller: *const PxController) -> PxControllerBehaviorFlags;

    /// Retrieve behavior flags for an obstacle.
    ///
    /// When the CCT touches an obstacle, the CCT's behavior w.r.t. this obstacle can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given obstacle
    pub fn PxControllerBehaviorCallback_getBehaviorFlags_mut_2(self_: *mut PxControllerBehaviorCallback, obstacle: *const PxObstacle) -> PxControllerBehaviorFlags;

    /// Releases the controller manager.
    ///
    /// This will release all associated controllers and obstacle contexts.
    ///
    /// This function is required to be called to release foundation usage.
    pub fn PxControllerManager_release_mut(self_: *mut PxControllerManager);

    /// Returns the scene the manager is adding the controllers to.
    ///
    /// The associated physics scene.
    pub fn PxControllerManager_getScene(self_: *const PxControllerManager) -> *mut PxScene;

    /// Returns the number of controllers that are being managed.
    ///
    /// The number of controllers.
    pub fn PxControllerManager_getNbControllers(self_: *const PxControllerManager) -> u32;

    /// Retrieve one of the controllers in the manager.
    ///
    /// The controller with the specified index.
    pub fn PxControllerManager_getController_mut(self_: *mut PxControllerManager, index: u32) -> *mut PxController;

    /// Creates a new character controller.
    ///
    /// The new controller
    pub fn PxControllerManager_createController_mut(self_: *mut PxControllerManager, desc: *const PxControllerDesc) -> *mut PxController;

    /// Releases all the controllers that are being managed.
    pub fn PxControllerManager_purgeControllers_mut(self_: *mut PxControllerManager);

    /// Retrieves debug data.
    ///
    /// The render buffer filled with debug-render data
    pub fn PxControllerManager_getRenderBuffer_mut(self_: *mut PxControllerManager) -> *mut PxRenderBuffer;

    /// Sets debug rendering flags
    pub fn PxControllerManager_setDebugRenderingFlags_mut(self_: *mut PxControllerManager, flags: PxControllerDebugRenderFlags);

    /// Returns the number of obstacle contexts that are being managed.
    ///
    /// The number of obstacle contexts.
    pub fn PxControllerManager_getNbObstacleContexts(self_: *const PxControllerManager) -> u32;

    /// Retrieve one of the obstacle contexts in the manager.
    ///
    /// The obstacle context with the specified index.
    pub fn PxControllerManager_getObstacleContext_mut(self_: *mut PxControllerManager, index: u32) -> *mut PxObstacleContext;

    /// Creates an obstacle context.
    ///
    /// New obstacle context
    pub fn PxControllerManager_createObstacleContext_mut(self_: *mut PxControllerManager) -> *mut PxObstacleContext;

    /// Computes character-character interactions.
    ///
    /// This function is an optional helper to properly resolve interactions between characters, in case they overlap (which can happen for gameplay reasons, etc).
    ///
    /// You should call this once per frame, before your PxController::move() calls. The function will not move the characters directly, but it will
    /// compute overlap information for each character that will be used in the next move() call.
    ///
    /// You need to provide a proper time value here so that interactions are resolved in a way that do not depend on the framerate.
    ///
    /// If you only have one character in the scene, or if you can guarantee your characters will never overlap, then you do not need to call this function.
    ///
    /// Releasing the manager will automatically release all the associated obstacle contexts.
    pub fn PxControllerManager_computeInteractions_mut(self_: *mut PxControllerManager, elapsedTime: f32, cctFilterCb: *mut PxControllerFilterCallback);

    /// Enables or disables runtime tessellation.
    ///
    /// Large triangles can create accuracy issues in the sweep code, which in turn can lead to characters not sliding smoothly
    /// against geometries, or even penetrating them. This feature allows one to reduce those issues by tessellating large
    /// triangles at runtime, before performing sweeps against them. The amount of tessellation is controlled by the 'maxEdgeLength' parameter.
    /// Any triangle with at least one edge length greater than the maxEdgeLength will get recursively tessellated, until resulting triangles are small enough.
    ///
    /// This features only applies to triangle meshes, convex meshes, heightfields and boxes.
    pub fn PxControllerManager_setTessellation_mut(self_: *mut PxControllerManager, flag: bool, maxEdgeLength: f32);

    /// Enables or disables the overlap recovery module.
    ///
    /// The overlap recovery module can be used to depenetrate CCTs from static objects when an overlap is detected. This can happen
    /// in three main cases:
    /// - when the CCT is directly spawned or teleported in another object
    /// - when the CCT algorithm fails due to limited FPU accuracy
    /// - when the "up vector" is modified, making the rotated CCT shape overlap surrounding objects
    ///
    /// When activated, the CCT module will automatically try to resolve the penetration, and move the CCT to a safe place where it does
    /// not overlap other objects anymore. This only concerns static objects, dynamic objects are ignored by the recovery module.
    ///
    /// When the recovery module is not activated, it is possible for the CCTs to go through static objects. By default, the recovery
    /// module is enabled.
    ///
    /// The recovery module currently works with all geometries except heightfields.
    pub fn PxControllerManager_setOverlapRecoveryModule_mut(self_: *mut PxControllerManager, flag: bool);

    /// Enables or disables the precise sweeps.
    ///
    /// Precise sweeps are more accurate, but also potentially slower than regular sweeps.
    ///
    /// By default, precise sweeps are enabled.
    pub fn PxControllerManager_setPreciseSweeps_mut(self_: *mut PxControllerManager, flag: bool);

    /// Enables or disables vertical sliding against ceilings.
    ///
    /// Geometry is seen as "ceilings" when the following condition is met:
    ///
    /// dot product(contact normal, up direction)
    /// <
    /// 0.0f
    ///
    /// This flag controls whether characters should slide vertically along the geometry in that case.
    ///
    /// By default, sliding is allowed.
    pub fn PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut(self_: *mut PxControllerManager, flag: bool);

    /// Shift the origin of the character controllers and obstacle objects by the specified vector.
    ///
    /// The positions of all character controllers, obstacle objects and the corresponding data structures will get adjusted to reflect the shifted origin location
    /// (the shift vector will get subtracted from all character controller and obstacle object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysXCharacterKinematic accordingly.
    ///
    /// This call will not automatically shift the PhysX scene and its objects. You need to call PxScene::shiftOrigin() seperately to keep the systems in sync.
    pub fn PxControllerManager_shiftOrigin_mut(self_: *mut PxControllerManager, shift: *const PxVec3);

    /// Creates the controller manager.
    ///
    /// The character controller is informed by [`PxDeletionListener::onRelease`]() when actors or shapes are released, and updates its internal
    /// caches accordingly. If character controller movement or a call to [`PxControllerManager::shiftOrigin`]() may overlap with actor/shape releases,
    /// internal data structures must be guarded against concurrent access.
    ///
    /// Locking guarantees thread safety in such scenarios.
    ///
    /// locking may result in significant slowdown for release of actors or shapes.
    ///
    /// By default, locking is disabled.
    pub fn phys_PxCreateControllerManager(scene: *mut PxScene, lockingEnabled: bool) -> *mut PxControllerManager;

    pub fn PxDim3_new() -> PxDim3;

    /// Constructor
    pub fn PxSDFDesc_new() -> PxSDFDesc;

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    pub fn PxSDFDesc_isValid(self_: *const PxSDFDesc) -> bool;

    /// constructor sets to default.
    pub fn PxConvexMeshDesc_new() -> PxConvexMeshDesc;

    /// (re)sets the structure to the default.
    pub fn PxConvexMeshDesc_setToDefault_mut(self_: *mut PxConvexMeshDesc);

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid
    pub fn PxConvexMeshDesc_isValid(self_: *const PxConvexMeshDesc) -> bool;

    /// Constructor sets to default.
    pub fn PxTriangleMeshDesc_new() -> PxTriangleMeshDesc;

    /// (re)sets the structure to the default.
    pub fn PxTriangleMeshDesc_setToDefault_mut(self_: *mut PxTriangleMeshDesc);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
    pub fn PxTriangleMeshDesc_isValid(self_: *const PxTriangleMeshDesc) -> bool;

    /// Constructor to build an empty tetmesh description
    pub fn PxTetrahedronMeshDesc_new() -> PxTetrahedronMeshDesc;

    pub fn PxTetrahedronMeshDesc_isValid(self_: *const PxTetrahedronMeshDesc) -> bool;

    /// Constructor to build an empty simulation description
    pub fn PxSoftBodySimulationDataDesc_new() -> PxSoftBodySimulationDataDesc;

    pub fn PxSoftBodySimulationDataDesc_isValid(self_: *const PxSoftBodySimulationDataDesc) -> bool;

    /// Desc initialization to default value.
    pub fn PxBVH34MidphaseDesc_setToDefault_mut(self_: *mut PxBVH34MidphaseDesc);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    pub fn PxBVH34MidphaseDesc_isValid(self_: *const PxBVH34MidphaseDesc) -> bool;

    pub fn PxMidphaseDesc_new() -> PxMidphaseDesc;

    /// Returns type of midphase mesh structure.
    ///
    /// PxMeshMidPhase::Enum
    pub fn PxMidphaseDesc_getType(self_: *const PxMidphaseDesc) -> PxMeshMidPhase;

    /// Initialize the midphase mesh structure descriptor
    pub fn PxMidphaseDesc_setToDefault_mut(self_: *mut PxMidphaseDesc, type_: PxMeshMidPhase);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    pub fn PxMidphaseDesc_isValid(self_: *const PxMidphaseDesc) -> bool;

    pub fn PxBVHDesc_new() -> PxBVHDesc;

    /// Initialize the BVH descriptor
    pub fn PxBVHDesc_setToDefault_mut(self_: *mut PxBVHDesc);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
    pub fn PxBVHDesc_isValid(self_: *const PxBVHDesc) -> bool;

    pub fn PxCookingParams_new(sc: *const PxTolerancesScale) -> PxCookingParams;

    pub fn phys_PxGetStandaloneInsertionCallback() -> *mut PxInsertionCallback;

    /// Cooks a bounding volume hierarchy. The results are written to the stream.
    ///
    /// PxCookBVH() allows a BVH description to be cooked into a binary stream
    /// suitable for loading and performing BVH detection at runtime.
    ///
    /// true on success.
    pub fn phys_PxCookBVH(desc: *const PxBVHDesc, stream: *mut PxOutputStream) -> bool;

    /// Cooks and creates a bounding volume hierarchy without going through a stream.
    ///
    /// This method does the same as cookBVH, but the produced BVH is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxBVH pointer on success
    pub fn phys_PxCreateBVH(desc: *const PxBVHDesc, insertionCallback: *mut PxInsertionCallback) -> *mut PxBVH;

    /// Cooks a heightfield. The results are written to the stream.
    ///
    /// To create a heightfield object there is an option to precompute some of calculations done while loading the heightfield data.
    ///
    /// cookHeightField() allows a heightfield description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    pub fn phys_PxCookHeightField(desc: *const PxHeightFieldDesc, stream: *mut PxOutputStream) -> bool;

    /// Cooks and creates a heightfield mesh and inserts it into PxPhysics.
    ///
    /// PxHeightField pointer on success
    pub fn phys_PxCreateHeightField(desc: *const PxHeightFieldDesc, insertionCallback: *mut PxInsertionCallback) -> *mut PxHeightField;

    /// Cooks a convex mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// cookConvexMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// The number of vertices and the number of convex polygons in a cooked convex mesh is limited to 255.
    ///
    /// If those limits are exceeded in either the user-provided data or the final cooked mesh, an error is reported.
    ///
    /// true on success.
    pub fn phys_PxCookConvexMesh(params: *const PxCookingParams, desc: *const PxConvexMeshDesc, stream: *mut PxOutputStream, condition: *mut PxConvexMeshCookingResult) -> bool;

    /// Cooks and creates a convex mesh without going through a stream.
    ///
    /// This method does the same as cookConvexMesh, but the produced mesh is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxConvexMesh pointer on success
    pub fn phys_PxCreateConvexMesh(params: *const PxCookingParams, desc: *const PxConvexMeshDesc, insertionCallback: *mut PxInsertionCallback, condition: *mut PxConvexMeshCookingResult) -> *mut PxConvexMesh;

    /// Verifies if the convex mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The convex mesh descriptor must contain an already created convex mesh - the vertices, indices and polygons must be provided.
    ///
    /// This function should be used if PxConvexFlag::eDISABLE_MESH_VALIDATION is planned to be used in release builds.
    ///
    /// true if all the validity conditions hold, false otherwise.
    pub fn phys_PxValidateConvexMesh(params: *const PxCookingParams, desc: *const PxConvexMeshDesc) -> bool;

    /// Computed hull polygons from given vertices and triangles. Polygons are needed for PxConvexMeshDesc rather than triangles.
    ///
    /// Please note that the resulting polygons may have different number of vertices. Some vertices may be removed.
    /// The output vertices, indices and polygons must be used to construct a hull.
    ///
    /// The provided PxAllocatorCallback does allocate the out array's. It is the user responsibility to deallocated those
    /// array's.
    ///
    /// true on success
    pub fn phys_PxComputeHullPolygons(params: *const PxCookingParams, mesh: *const PxSimpleTriangleMesh, inCallback: *mut PxAllocatorCallback, nbVerts: *mut u32, vertices: *mut *mut PxVec3, nbIndices: *mut u32, indices: *mut *mut u32, nbPolygons: *mut u32, hullPolygons: *mut *mut PxHullPolygon) -> bool;

    /// Verifies if the triangle mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The following conditions are true for a valid triangle mesh:
    /// 1. There are no duplicate vertices (within specified vertexWeldTolerance. See PxCookingParams::meshWeldTolerance)
    /// 2. There are no large triangles (within specified PxTolerancesScale.)
    ///
    /// true if all the validity conditions hold, false otherwise.
    pub fn phys_PxValidateTriangleMesh(params: *const PxCookingParams, desc: *const PxTriangleMeshDesc) -> bool;

    /// Cooks and creates a triangle mesh without going through a stream.
    ///
    /// This method does the same as cookTriangleMesh, but the produced mesh is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxTriangleMesh pointer on success.
    pub fn phys_PxCreateTriangleMesh(params: *const PxCookingParams, desc: *const PxTriangleMeshDesc, insertionCallback: *mut PxInsertionCallback, condition: *mut PxTriangleMeshCookingResult) -> *mut PxTriangleMesh;

    /// Cooks a triangle mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// PxCookTriangleMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
    pub fn phys_PxCookTriangleMesh(params: *const PxCookingParams, desc: *const PxTriangleMeshDesc, stream: *mut PxOutputStream, condition: *mut PxTriangleMeshCookingResult) -> bool;

    pub fn PxDefaultMemoryOutputStream_new_alloc(allocator: *mut PxAllocatorCallback) -> *mut PxDefaultMemoryOutputStream;

    pub fn PxDefaultMemoryOutputStream_delete(self_: *mut PxDefaultMemoryOutputStream);

    pub fn PxDefaultMemoryOutputStream_write_mut(self_: *mut PxDefaultMemoryOutputStream, src: *const std::ffi::c_void, count: u32) -> u32;

    pub fn PxDefaultMemoryOutputStream_getSize(self_: *const PxDefaultMemoryOutputStream) -> u32;

    pub fn PxDefaultMemoryOutputStream_getData(self_: *const PxDefaultMemoryOutputStream) -> *mut u8;

    pub fn PxDefaultMemoryInputData_new_alloc(data: *mut u8, length: u32) -> *mut PxDefaultMemoryInputData;

    pub fn PxDefaultMemoryInputData_read_mut(self_: *mut PxDefaultMemoryInputData, dest: *mut std::ffi::c_void, count: u32) -> u32;

    pub fn PxDefaultMemoryInputData_getLength(self_: *const PxDefaultMemoryInputData) -> u32;

    pub fn PxDefaultMemoryInputData_seek_mut(self_: *mut PxDefaultMemoryInputData, pos: u32);

    pub fn PxDefaultMemoryInputData_tell(self_: *const PxDefaultMemoryInputData) -> u32;

    pub fn PxDefaultFileOutputStream_new_alloc(name: *const std::ffi::c_char) -> *mut PxDefaultFileOutputStream;

    pub fn PxDefaultFileOutputStream_delete(self_: *mut PxDefaultFileOutputStream);

    pub fn PxDefaultFileOutputStream_write_mut(self_: *mut PxDefaultFileOutputStream, src: *const std::ffi::c_void, count: u32) -> u32;

    pub fn PxDefaultFileOutputStream_isValid_mut(self_: *mut PxDefaultFileOutputStream) -> bool;

    pub fn PxDefaultFileInputData_new_alloc(name: *const std::ffi::c_char) -> *mut PxDefaultFileInputData;

    pub fn PxDefaultFileInputData_delete(self_: *mut PxDefaultFileInputData);

    pub fn PxDefaultFileInputData_read_mut(self_: *mut PxDefaultFileInputData, dest: *mut std::ffi::c_void, count: u32) -> u32;

    pub fn PxDefaultFileInputData_seek_mut(self_: *mut PxDefaultFileInputData, pos: u32);

    pub fn PxDefaultFileInputData_tell(self_: *const PxDefaultFileInputData) -> u32;

    pub fn PxDefaultFileInputData_getLength(self_: *const PxDefaultFileInputData) -> u32;

    pub fn PxDefaultFileInputData_isValid(self_: *const PxDefaultFileInputData) -> bool;

    pub fn phys_platformAlignedAlloc(size: usize) -> *mut std::ffi::c_void;

    pub fn phys_platformAlignedFree(ptr: *mut std::ffi::c_void);

    pub fn PxDefaultAllocator_allocate_mut(self_: *mut PxDefaultAllocator, size: usize, anon_param1: *const std::ffi::c_char, anon_param2: *const std::ffi::c_char, anon_param3: i32) -> *mut std::ffi::c_void;

    pub fn PxDefaultAllocator_deallocate_mut(self_: *mut PxDefaultAllocator, ptr: *mut std::ffi::c_void);

    pub fn PxDefaultAllocator_delete(self_: *mut PxDefaultAllocator);

    /// Set the actors for this joint.
    ///
    /// An actor may be NULL to indicate the world frame. At most one of the actors may be NULL.
    pub fn PxJoint_setActors_mut(self_: *mut PxJoint, actor0: *mut PxRigidActor, actor1: *mut PxRigidActor);

    /// Get the actors for this joint.
    pub fn PxJoint_getActors(self_: *const PxJoint, actor0: *mut *mut PxRigidActor, actor1: *mut *mut PxRigidActor);

    /// Set the joint local pose for an actor.
    ///
    /// This is the relative pose which locates the joint frame relative to the actor.
    pub fn PxJoint_setLocalPose_mut(self_: *mut PxJoint, actor: PxJointActorIndex, localPose: *const PxTransform);

    /// get the joint local pose for an actor.
    ///
    /// return the local pose for this joint
    pub fn PxJoint_getLocalPose(self_: *const PxJoint, actor: PxJointActorIndex) -> PxTransform;

    /// get the relative pose for this joint
    ///
    /// This function returns the pose of the joint frame of actor1 relative to actor0
    pub fn PxJoint_getRelativeTransform(self_: *const PxJoint) -> PxTransform;

    /// get the relative linear velocity of the joint
    ///
    /// This function returns the linear velocity of the origin of the constraint frame of actor1, relative to the origin of the constraint
    /// frame of actor0. The value is returned in the constraint frame of actor0
    pub fn PxJoint_getRelativeLinearVelocity(self_: *const PxJoint) -> PxVec3;

    /// get the relative angular velocity of the joint
    ///
    /// This function returns the angular velocity of  actor1 relative to actor0. The value is returned in the constraint frame of actor0
    pub fn PxJoint_getRelativeAngularVelocity(self_: *const PxJoint) -> PxVec3;

    /// set the break force for this joint.
    ///
    /// if the constraint force or torque on the joint exceeds the specified values, the joint will break,
    /// at which point it will not constrain the two actors and the flag PxConstraintFlag::eBROKEN will be set. The
    /// force and torque are measured in the joint frame of the first actor
    pub fn PxJoint_setBreakForce_mut(self_: *mut PxJoint, force: f32, torque: f32);

    /// get the break force for this joint.
    pub fn PxJoint_getBreakForce(self_: *const PxJoint, force: *mut f32, torque: *mut f32);

    /// set the constraint flags for this joint.
    pub fn PxJoint_setConstraintFlags_mut(self_: *mut PxJoint, flags: PxConstraintFlags);

    /// set a constraint flags for this joint to a specified value.
    pub fn PxJoint_setConstraintFlag_mut(self_: *mut PxJoint, flag: PxConstraintFlag, value: bool);

    /// get the constraint flags for this joint.
    ///
    /// the constraint flags
    pub fn PxJoint_getConstraintFlags(self_: *const PxJoint) -> PxConstraintFlags;

    /// set the inverse mass scale for actor0.
    pub fn PxJoint_setInvMassScale0_mut(self_: *mut PxJoint, invMassScale: f32);

    /// get the inverse mass scale for actor0.
    ///
    /// inverse mass scale for actor0
    pub fn PxJoint_getInvMassScale0(self_: *const PxJoint) -> f32;

    /// set the inverse inertia scale for actor0.
    pub fn PxJoint_setInvInertiaScale0_mut(self_: *mut PxJoint, invInertiaScale: f32);

    /// get the inverse inertia scale for actor0.
    ///
    /// inverse inertia scale for actor0
    pub fn PxJoint_getInvInertiaScale0(self_: *const PxJoint) -> f32;

    /// set the inverse mass scale for actor1.
    pub fn PxJoint_setInvMassScale1_mut(self_: *mut PxJoint, invMassScale: f32);

    /// get the inverse mass scale for actor1.
    ///
    /// inverse mass scale for actor1
    pub fn PxJoint_getInvMassScale1(self_: *const PxJoint) -> f32;

    /// set the inverse inertia scale for actor1.
    pub fn PxJoint_setInvInertiaScale1_mut(self_: *mut PxJoint, invInertiaScale: f32);

    /// get the inverse inertia scale for actor1.
    ///
    /// inverse inertia scale for actor1
    pub fn PxJoint_getInvInertiaScale1(self_: *const PxJoint) -> f32;

    /// Retrieves the PxConstraint corresponding to this joint.
    ///
    /// This can be used to determine, among other things, the force applied at the joint.
    ///
    /// the constraint
    pub fn PxJoint_getConstraint(self_: *const PxJoint) -> *mut PxConstraint;

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    pub fn PxJoint_setName_mut(self_: *mut PxJoint, name: *const std::ffi::c_char);

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
    pub fn PxJoint_getName(self_: *const PxJoint) -> *const std::ffi::c_char;

    /// Deletes the joint.
    ///
    /// This call does not wake up the connected rigid bodies.
    pub fn PxJoint_release_mut(self_: *mut PxJoint);

    /// Retrieves the scene which this joint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
    pub fn PxJoint_getScene(self_: *const PxJoint) -> *mut PxScene;

    /// Put class meta data in stream, used for serialization
    pub fn PxJoint_getBinaryMetaData(stream: *mut PxOutputStream);

    pub fn PxSpring_new(stiffness_: f32, damping_: f32) -> PxSpring;

    /// Helper function to setup a joint's global frame
    ///
    /// This replaces the following functions from previous SDK versions:
    ///
    /// void NxJointDesc::setGlobalAnchor(const NxVec3
    /// &
    /// wsAnchor);
    /// void NxJointDesc::setGlobalAxis(const NxVec3
    /// &
    /// wsAxis);
    ///
    /// The function sets the joint's localPose using world-space input parameters.
    pub fn phys_PxSetJointGlobalFrame(joint: *mut PxJoint, wsAnchor: *const PxVec3, wsAxis: *const PxVec3);

    /// Create a distance Joint.
    pub fn phys_PxDistanceJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxDistanceJoint;

    /// Return the current distance of the joint
    pub fn PxDistanceJoint_getDistance(self_: *const PxDistanceJoint) -> f32;

    /// Set the allowed minimum distance for the joint.
    ///
    /// The minimum distance must be no more than the maximum distance
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    pub fn PxDistanceJoint_setMinDistance_mut(self_: *mut PxDistanceJoint, distance: f32);

    /// Get the allowed minimum distance for the joint.
    ///
    /// the allowed minimum distance
    pub fn PxDistanceJoint_getMinDistance(self_: *const PxDistanceJoint) -> f32;

    /// Set the allowed maximum distance for the joint.
    ///
    /// The maximum distance must be no less than the minimum distance.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    pub fn PxDistanceJoint_setMaxDistance_mut(self_: *mut PxDistanceJoint, distance: f32);

    /// Get the allowed maximum distance for the joint.
    ///
    /// the allowed maximum distance
    pub fn PxDistanceJoint_getMaxDistance(self_: *const PxDistanceJoint) -> f32;

    /// Set the error tolerance of the joint.
    pub fn PxDistanceJoint_setTolerance_mut(self_: *mut PxDistanceJoint, tolerance: f32);

    /// Get the error tolerance of the joint.
    ///
    /// the distance beyond the joint's [min, max] range before the joint becomes active.
    ///
    /// Default
    /// 0.25f * PxTolerancesScale::length
    /// Range
    /// (0, PX_MAX_F32)
    ///
    /// This value should be used to ensure that if the minimum distance is zero and the
    /// spring function is in use, the rest length of the spring is non-zero.
    pub fn PxDistanceJoint_getTolerance(self_: *const PxDistanceJoint) -> f32;

    /// Set the strength of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    pub fn PxDistanceJoint_setStiffness_mut(self_: *mut PxDistanceJoint, stiffness: f32);

    /// Get the strength of the joint spring.
    ///
    /// stiffness the spring strength of the joint
    pub fn PxDistanceJoint_getStiffness(self_: *const PxDistanceJoint) -> f32;

    /// Set the damping of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    pub fn PxDistanceJoint_setDamping_mut(self_: *mut PxDistanceJoint, damping: f32);

    /// Get the damping of the joint spring.
    ///
    /// the degree of damping of the joint spring of the joint
    pub fn PxDistanceJoint_getDamping(self_: *const PxDistanceJoint) -> f32;

    /// Set the contact distance for the min
    /// &
    /// max distance limits.
    ///
    /// This is similar to the PxJointLimitParameters::contactDistance parameter for regular limits.
    ///
    /// The two most common values are 0 and infinite. Infinite means the internal constraints are
    /// always created, resulting in the best simulation quality but slower performance. Zero means
    /// the internal constraints are only created when the limits are violated, resulting in best
    /// performance but worse simulation quality.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
    pub fn PxDistanceJoint_setContactDistance_mut(self_: *mut PxDistanceJoint, contactDistance: f32);

    /// Get the contact distance.
    ///
    /// the contact distance
    pub fn PxDistanceJoint_getContactDistance(self_: *const PxDistanceJoint) -> f32;

    /// Set the flags specific to the Distance Joint.
    ///
    /// Default
    /// PxDistanceJointFlag::eMAX_DISTANCE_ENABLED
    pub fn PxDistanceJoint_setDistanceJointFlags_mut(self_: *mut PxDistanceJoint, flags: PxDistanceJointFlags);

    /// Set a single flag specific to a Distance Joint to true or false.
    pub fn PxDistanceJoint_setDistanceJointFlag_mut(self_: *mut PxDistanceJoint, flag: PxDistanceJointFlag, value: bool);

    /// Get the flags specific to the Distance Joint.
    ///
    /// the joint flags
    pub fn PxDistanceJoint_getDistanceJointFlags(self_: *const PxDistanceJoint) -> PxDistanceJointFlags;

    /// Returns string name of PxDistanceJoint, used for serialization
    pub fn PxDistanceJoint_getConcreteTypeName(self_: *const PxDistanceJoint) -> *const std::ffi::c_char;

    /// Create a distance Joint.
    pub fn phys_PxContactJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxContactJoint;

    pub fn PxJacobianRow_new() -> PxJacobianRow;

    pub fn PxJacobianRow_new_1(lin0: *const PxVec3, lin1: *const PxVec3, ang0: *const PxVec3, ang1: *const PxVec3) -> PxJacobianRow;

    /// Set the current contact of the joint
    pub fn PxContactJoint_setContact_mut(self_: *mut PxContactJoint, contact: *const PxVec3);

    /// Set the current contact normal of the joint
    pub fn PxContactJoint_setContactNormal_mut(self_: *mut PxContactJoint, contactNormal: *const PxVec3);

    /// Set the current penetration of the joint
    pub fn PxContactJoint_setPenetration_mut(self_: *mut PxContactJoint, penetration: f32);

    /// Return the current contact of the joint
    pub fn PxContactJoint_getContact(self_: *const PxContactJoint) -> PxVec3;

    /// Return the current contact normal of the joint
    pub fn PxContactJoint_getContactNormal(self_: *const PxContactJoint) -> PxVec3;

    /// Return the current penetration value of the joint
    pub fn PxContactJoint_getPenetration(self_: *const PxContactJoint) -> f32;

    pub fn PxContactJoint_getRestitution(self_: *const PxContactJoint) -> f32;

    pub fn PxContactJoint_setRestitution_mut(self_: *mut PxContactJoint, restitution: f32);

    pub fn PxContactJoint_getBounceThreshold(self_: *const PxContactJoint) -> f32;

    pub fn PxContactJoint_setBounceThreshold_mut(self_: *mut PxContactJoint, bounceThreshold: f32);

    /// Returns string name of PxContactJoint, used for serialization
    pub fn PxContactJoint_getConcreteTypeName(self_: *const PxContactJoint) -> *const std::ffi::c_char;

    pub fn PxContactJoint_computeJacobians(self_: *const PxContactJoint, jacobian: *mut PxJacobianRow);

    pub fn PxContactJoint_getNbJacobianRows(self_: *const PxContactJoint) -> u32;

    /// Create a fixed joint.
    pub fn phys_PxFixedJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxFixedJoint;

    /// Returns string name of PxFixedJoint, used for serialization
    pub fn PxFixedJoint_getConcreteTypeName(self_: *const PxFixedJoint) -> *const std::ffi::c_char;

    pub fn PxJointLimitParameters_new_alloc() -> *mut PxJointLimitParameters;

    /// Returns true if the current settings are valid.
    ///
    /// true if the current settings are valid
    pub fn PxJointLimitParameters_isValid(self_: *const PxJointLimitParameters) -> bool;

    pub fn PxJointLimitParameters_isSoft(self_: *const PxJointLimitParameters) -> bool;

    /// construct a linear hard limit
    pub fn PxJointLinearLimit_new(scale: *const PxTolerancesScale, extent: f32, contactDist_deprecated: f32) -> PxJointLinearLimit;

    /// construct a linear soft limit
    pub fn PxJointLinearLimit_new_1(extent: f32, spring: *const PxSpring) -> PxJointLinearLimit;

    /// Returns true if the limit is valid
    ///
    /// true if the current settings are valid
    pub fn PxJointLinearLimit_isValid(self_: *const PxJointLinearLimit) -> bool;

    pub fn PxJointLinearLimit_delete(self_: *mut PxJointLinearLimit);

    /// Construct a linear hard limit pair. The lower distance value must be less than the upper distance value.
    pub fn PxJointLinearLimitPair_new(scale: *const PxTolerancesScale, lowerLimit: f32, upperLimit: f32, contactDist_deprecated: f32) -> PxJointLinearLimitPair;

    /// construct a linear soft limit pair
    pub fn PxJointLinearLimitPair_new_1(lowerLimit: f32, upperLimit: f32, spring: *const PxSpring) -> PxJointLinearLimitPair;

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    pub fn PxJointLinearLimitPair_isValid(self_: *const PxJointLinearLimitPair) -> bool;

    pub fn PxJointLinearLimitPair_delete(self_: *mut PxJointLinearLimitPair);

    /// construct an angular hard limit pair.
    ///
    /// The lower value must be less than the upper value.
    pub fn PxJointAngularLimitPair_new(lowerLimit: f32, upperLimit: f32, contactDist_deprecated: f32) -> PxJointAngularLimitPair;

    /// construct an angular soft limit pair.
    ///
    /// The lower value must be less than the upper value.
    pub fn PxJointAngularLimitPair_new_1(lowerLimit: f32, upperLimit: f32, spring: *const PxSpring) -> PxJointAngularLimitPair;

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    pub fn PxJointAngularLimitPair_isValid(self_: *const PxJointAngularLimitPair) -> bool;

    pub fn PxJointAngularLimitPair_delete(self_: *mut PxJointAngularLimitPair);

    /// Construct a cone hard limit.
    pub fn PxJointLimitCone_new(yLimitAngle: f32, zLimitAngle: f32, contactDist_deprecated: f32) -> PxJointLimitCone;

    /// Construct a cone soft limit.
    pub fn PxJointLimitCone_new_1(yLimitAngle: f32, zLimitAngle: f32, spring: *const PxSpring) -> PxJointLimitCone;

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    pub fn PxJointLimitCone_isValid(self_: *const PxJointLimitCone) -> bool;

    pub fn PxJointLimitCone_delete(self_: *mut PxJointLimitCone);

    /// Construct a pyramid hard limit.
    pub fn PxJointLimitPyramid_new(yLimitAngleMin: f32, yLimitAngleMax: f32, zLimitAngleMin: f32, zLimitAngleMax: f32, contactDist_deprecated: f32) -> PxJointLimitPyramid;

    /// Construct a pyramid soft limit.
    pub fn PxJointLimitPyramid_new_1(yLimitAngleMin: f32, yLimitAngleMax: f32, zLimitAngleMin: f32, zLimitAngleMax: f32, spring: *const PxSpring) -> PxJointLimitPyramid;

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
    pub fn PxJointLimitPyramid_isValid(self_: *const PxJointLimitPyramid) -> bool;

    pub fn PxJointLimitPyramid_delete(self_: *mut PxJointLimitPyramid);

    /// Create a prismatic joint.
    pub fn phys_PxPrismaticJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxPrismaticJoint;

    /// returns the displacement of the joint along its axis.
    pub fn PxPrismaticJoint_getPosition(self_: *const PxPrismaticJoint) -> f32;

    /// returns the velocity of the joint along its axis
    pub fn PxPrismaticJoint_getVelocity(self_: *const PxPrismaticJoint) -> f32;

    /// sets the joint limit  parameters.
    ///
    /// The limit range is [-PX_MAX_F32, PX_MAX_F32], but note that the width of the limit (upper-lower) must also be
    /// a valid float.
    pub fn PxPrismaticJoint_setLimit_mut(self_: *mut PxPrismaticJoint, anon_param0: *const PxJointLinearLimitPair);

    /// gets the joint limit  parameters.
    pub fn PxPrismaticJoint_getLimit(self_: *const PxPrismaticJoint) -> PxJointLinearLimitPair;

    /// Set the flags specific to the Prismatic Joint.
    ///
    /// Default
    /// PxPrismaticJointFlags(0)
    pub fn PxPrismaticJoint_setPrismaticJointFlags_mut(self_: *mut PxPrismaticJoint, flags: PxPrismaticJointFlags);

    /// Set a single flag specific to a Prismatic Joint to true or false.
    pub fn PxPrismaticJoint_setPrismaticJointFlag_mut(self_: *mut PxPrismaticJoint, flag: PxPrismaticJointFlag, value: bool);

    /// Get the flags specific to the Prismatic Joint.
    ///
    /// the joint flags
    pub fn PxPrismaticJoint_getPrismaticJointFlags(self_: *const PxPrismaticJoint) -> PxPrismaticJointFlags;

    /// Returns string name of PxPrismaticJoint, used for serialization
    pub fn PxPrismaticJoint_getConcreteTypeName(self_: *const PxPrismaticJoint) -> *const std::ffi::c_char;

    /// Create a revolute joint.
    pub fn phys_PxRevoluteJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxRevoluteJoint;

    /// return the angle of the joint, in the range (-2*Pi, 2*Pi]
    pub fn PxRevoluteJoint_getAngle(self_: *const PxRevoluteJoint) -> f32;

    /// return the velocity of the joint
    pub fn PxRevoluteJoint_getVelocity(self_: *const PxRevoluteJoint) -> f32;

    /// set the joint limit parameters.
    ///
    /// The limit is activated using the flag PxRevoluteJointFlag::eLIMIT_ENABLED
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    pub fn PxRevoluteJoint_setLimit_mut(self_: *mut PxRevoluteJoint, limits: *const PxJointAngularLimitPair);

    /// get the joint limit parameters.
    ///
    /// the joint limit parameters
    pub fn PxRevoluteJoint_getLimit(self_: *const PxRevoluteJoint) -> PxJointAngularLimitPair;

    /// set the target velocity for the drive model.
    ///
    /// The motor will only be able to reach this velocity if the maxForce is sufficiently large.
    /// If the joint is spinning faster than this velocity, the motor will actually try to brake
    /// (see PxRevoluteJointFlag::eDRIVE_FREESPIN.)
    ///
    /// The sign of this variable determines the rotation direction, with positive values going
    /// the same way as positive joint angles. Setting a very large target velocity may cause
    /// undesirable results.
    ///
    /// Range:
    /// (-PX_MAX_F32, PX_MAX_F32)
    /// Default:
    /// 0.0
    pub fn PxRevoluteJoint_setDriveVelocity_mut(self_: *mut PxRevoluteJoint, velocity: f32, autowake: bool);

    /// gets the target velocity for the drive model.
    ///
    /// the drive target velocity
    pub fn PxRevoluteJoint_getDriveVelocity(self_: *const PxRevoluteJoint) -> f32;

    /// sets the maximum torque the drive can exert.
    ///
    /// The value set here may be used either as an impulse limit or a force limit, depending on the flag PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// PX_MAX_F32
    pub fn PxRevoluteJoint_setDriveForceLimit_mut(self_: *mut PxRevoluteJoint, limit: f32);

    /// gets the maximum torque the drive can exert.
    ///
    /// the torque limit
    pub fn PxRevoluteJoint_getDriveForceLimit(self_: *const PxRevoluteJoint) -> f32;

    /// sets the gear ratio for the drive.
    ///
    /// When setting up the drive constraint, the velocity of the first actor is scaled by this value, and its response to drive torque is scaled down.
    /// So if the drive target velocity is zero, the second actor will be driven to the velocity of the first scaled by the gear ratio
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1.0
    pub fn PxRevoluteJoint_setDriveGearRatio_mut(self_: *mut PxRevoluteJoint, ratio: f32);

    /// gets the gear ratio.
    ///
    /// the drive gear ratio
    pub fn PxRevoluteJoint_getDriveGearRatio(self_: *const PxRevoluteJoint) -> f32;

    /// sets the flags specific to the Revolute Joint.
    ///
    /// Default
    /// PxRevoluteJointFlags(0)
    pub fn PxRevoluteJoint_setRevoluteJointFlags_mut(self_: *mut PxRevoluteJoint, flags: PxRevoluteJointFlags);

    /// sets a single flag specific to a Revolute Joint.
    pub fn PxRevoluteJoint_setRevoluteJointFlag_mut(self_: *mut PxRevoluteJoint, flag: PxRevoluteJointFlag, value: bool);

    /// gets the flags specific to the Revolute Joint.
    ///
    /// the joint flags
    pub fn PxRevoluteJoint_getRevoluteJointFlags(self_: *const PxRevoluteJoint) -> PxRevoluteJointFlags;

    /// Returns string name of PxRevoluteJoint, used for serialization
    pub fn PxRevoluteJoint_getConcreteTypeName(self_: *const PxRevoluteJoint) -> *const std::ffi::c_char;

    /// Create a spherical joint.
    pub fn phys_PxSphericalJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxSphericalJoint;

    /// Set the limit cone.
    ///
    /// If enabled, the limit cone will constrain the angular movement of the joint to lie
    /// within an elliptical cone.
    ///
    /// the limit cone
    pub fn PxSphericalJoint_getLimitCone(self_: *const PxSphericalJoint) -> PxJointLimitCone;

    /// Get the limit cone.
    pub fn PxSphericalJoint_setLimitCone_mut(self_: *mut PxSphericalJoint, limit: *const PxJointLimitCone);

    /// get the swing angle of the joint from the Y axis
    pub fn PxSphericalJoint_getSwingYAngle(self_: *const PxSphericalJoint) -> f32;

    /// get the swing angle of the joint from the Z axis
    pub fn PxSphericalJoint_getSwingZAngle(self_: *const PxSphericalJoint) -> f32;

    /// Set the flags specific to the Spherical Joint.
    ///
    /// Default
    /// PxSphericalJointFlags(0)
    pub fn PxSphericalJoint_setSphericalJointFlags_mut(self_: *mut PxSphericalJoint, flags: PxSphericalJointFlags);

    /// Set a single flag specific to a Spherical Joint to true or false.
    pub fn PxSphericalJoint_setSphericalJointFlag_mut(self_: *mut PxSphericalJoint, flag: PxSphericalJointFlag, value: bool);

    /// Get the flags specific to the Spherical Joint.
    ///
    /// the joint flags
    pub fn PxSphericalJoint_getSphericalJointFlags(self_: *const PxSphericalJoint) -> PxSphericalJointFlags;

    /// Returns string name of PxSphericalJoint, used for serialization
    pub fn PxSphericalJoint_getConcreteTypeName(self_: *const PxSphericalJoint) -> *const std::ffi::c_char;

    /// Create a D6 joint.
    pub fn phys_PxD6JointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxD6Joint;

    /// default constructor for PxD6JointDrive.
    pub fn PxD6JointDrive_new() -> PxD6JointDrive;

    /// constructor a PxD6JointDrive.
    pub fn PxD6JointDrive_new_1(driveStiffness: f32, driveDamping: f32, driveForceLimit: f32, isAcceleration: bool) -> PxD6JointDrive;

    /// returns true if the drive is valid
    pub fn PxD6JointDrive_isValid(self_: *const PxD6JointDrive) -> bool;

    /// Set the motion type around the specified axis.
    ///
    /// Each axis may independently specify that the degree of freedom is locked (blocking relative movement
    /// along or around this axis), limited by the corresponding limit, or free.
    ///
    /// Default:
    /// all degrees of freedom are locked
    pub fn PxD6Joint_setMotion_mut(self_: *mut PxD6Joint, axis: PxD6Axis, type_: PxD6Motion);

    /// Get the motion type around the specified axis.
    ///
    /// the motion type around the specified axis
    pub fn PxD6Joint_getMotion(self_: *const PxD6Joint, axis: PxD6Axis) -> PxD6Motion;

    /// get the twist angle of the joint, in the range (-2*Pi, 2*Pi]
    pub fn PxD6Joint_getTwistAngle(self_: *const PxD6Joint) -> f32;

    /// get the swing angle of the joint from the Y axis
    pub fn PxD6Joint_getSwingYAngle(self_: *const PxD6Joint) -> f32;

    /// get the swing angle of the joint from the Z axis
    pub fn PxD6Joint_getSwingZAngle(self_: *const PxD6Joint) -> f32;

    /// Set the distance limit for the joint.
    ///
    /// A single limit constraints all linear limited degrees of freedom, forming a linear, circular
    /// or spherical constraint on motion depending on the number of limited degrees. This is similar
    /// to a distance limit.
    pub fn PxD6Joint_setDistanceLimit_mut(self_: *mut PxD6Joint, limit: *const PxJointLinearLimit);

    /// Get the distance limit for the joint.
    ///
    /// the distance limit structure
    pub fn PxD6Joint_getDistanceLimit(self_: *const PxD6Joint) -> PxJointLinearLimit;

    /// Set the linear limit for a given linear axis.
    ///
    /// This function extends the previous setDistanceLimit call with the following features:
    /// - there can be a different limit for each linear axis
    /// - each limit is defined by two values, i.e. it can now be asymmetric
    ///
    /// This can be used to create prismatic joints similar to PxPrismaticJoint, or point-in-quad joints,
    /// or point-in-box joints.
    pub fn PxD6Joint_setLinearLimit_mut(self_: *mut PxD6Joint, axis: PxD6Axis, limit: *const PxJointLinearLimitPair);

    /// Get the linear limit for a given linear axis.
    ///
    /// the linear limit pair structure from desired axis
    pub fn PxD6Joint_getLinearLimit(self_: *const PxD6Joint, axis: PxD6Axis) -> PxJointLinearLimitPair;

    /// Set the twist limit for the joint.
    ///
    /// The twist limit controls the range of motion around the twist axis.
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
    pub fn PxD6Joint_setTwistLimit_mut(self_: *mut PxD6Joint, limit: *const PxJointAngularLimitPair);

    /// Get the twist limit for the joint.
    ///
    /// the twist limit structure
    pub fn PxD6Joint_getTwistLimit(self_: *const PxD6Joint) -> PxJointAngularLimitPair;

    /// Set the swing cone limit for the joint.
    ///
    /// The cone limit is used if either or both swing axes are limited. The extents are
    /// symmetrical and measured in the frame of the parent. If only one swing degree of freedom
    /// is limited, the corresponding value from the cone limit defines the limit range.
    pub fn PxD6Joint_setSwingLimit_mut(self_: *mut PxD6Joint, limit: *const PxJointLimitCone);

    /// Get the cone limit for the joint.
    ///
    /// the swing limit structure
    pub fn PxD6Joint_getSwingLimit(self_: *const PxD6Joint) -> PxJointLimitCone;

    /// Set a pyramidal swing limit for the joint.
    ///
    /// The pyramid limits will only be used in the following cases:
    /// - both swing Y and Z are limited. The limit shape is then a pyramid.
    /// - Y is limited and Z is locked, or vice versa. The limit shape is an asymmetric angular section, similar to
    /// what is supported for the twist axis.
    /// The remaining cases (Y limited and Z is free, or vice versa) are not supported.
    pub fn PxD6Joint_setPyramidSwingLimit_mut(self_: *mut PxD6Joint, limit: *const PxJointLimitPyramid);

    /// Get the pyramidal swing limit for the joint.
    ///
    /// the swing limit structure
    pub fn PxD6Joint_getPyramidSwingLimit(self_: *const PxD6Joint) -> PxJointLimitPyramid;

    /// Set the drive parameters for the specified drive type.
    ///
    /// Default
    /// The default drive spring and damping values are zero, the force limit is zero, and no flags are set.
    pub fn PxD6Joint_setDrive_mut(self_: *mut PxD6Joint, index: PxD6Drive, drive: *const PxD6JointDrive);

    /// Get the drive parameters for the specified drive type.
    pub fn PxD6Joint_getDrive(self_: *const PxD6Joint, index: PxD6Drive) -> PxD6JointDrive;

    /// Set the drive goal pose
    ///
    /// The goal is relative to the constraint frame of actor[0]
    ///
    /// Default
    /// the identity transform
    pub fn PxD6Joint_setDrivePosition_mut(self_: *mut PxD6Joint, pose: *const PxTransform, autowake: bool);

    /// Get the drive goal pose.
    pub fn PxD6Joint_getDrivePosition(self_: *const PxD6Joint) -> PxTransform;

    /// Set the target goal velocity for drive.
    ///
    /// The velocity is measured in the constraint frame of actor[0]
    pub fn PxD6Joint_setDriveVelocity_mut(self_: *mut PxD6Joint, linear: *const PxVec3, angular: *const PxVec3, autowake: bool);

    /// Get the target goal velocity for joint drive.
    pub fn PxD6Joint_getDriveVelocity(self_: *const PxD6Joint, linear: *mut PxVec3, angular: *mut PxVec3);

    /// Set the linear tolerance threshold for projection. Projection is enabled if PxConstraintFlag::ePROJECTION
    /// is set for the joint.
    ///
    /// If the joint separates by more than this distance along its locked degrees of freedom, the solver
    /// will move the bodies to close the distance.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    ///
    /// Sometimes it is not possible to project (for example when the joints form a cycle).
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1e10f
    pub fn PxD6Joint_setProjectionLinearTolerance_mut(self_: *mut PxD6Joint, tolerance: f32);

    /// Get the linear tolerance threshold for projection.
    ///
    /// the linear tolerance threshold
    pub fn PxD6Joint_getProjectionLinearTolerance(self_: *const PxD6Joint) -> f32;

    /// Set the angular tolerance threshold for projection. Projection is enabled if
    /// PxConstraintFlag::ePROJECTION is set for the joint.
    ///
    /// If the joint deviates by more than this angle around its locked angular degrees of freedom,
    /// the solver will move the bodies to close the angle.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    ///
    /// Sometimes it is not possible to project (for example when the joints form a cycle).
    ///
    /// Range:
    /// [0,Pi]
    /// Default:
    /// Pi
    ///
    /// Angular projection is implemented only for the case of two or three locked angular degrees of freedom.
    pub fn PxD6Joint_setProjectionAngularTolerance_mut(self_: *mut PxD6Joint, tolerance: f32);

    /// Get the angular tolerance threshold for projection.
    ///
    /// tolerance the angular tolerance threshold in radians
    pub fn PxD6Joint_getProjectionAngularTolerance(self_: *const PxD6Joint) -> f32;

    /// Returns string name of PxD6Joint, used for serialization
    pub fn PxD6Joint_getConcreteTypeName(self_: *const PxD6Joint) -> *const std::ffi::c_char;

    /// Create a gear Joint.
    pub fn phys_PxGearJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxGearJoint;

    /// Set the hinge/revolute joints connected by the gear joint.
    ///
    /// The passed joints can be either PxRevoluteJoint, PxD6Joint or PxArticulationJointReducedCoordinate.
    /// The joints must define degrees of freedom around the twist axis. They cannot be null.
    ///
    /// Note that these joints are only used to compute the positional error correction term,
    /// used to adjust potential drift between jointed actors. The gear joint can run without
    /// calling this function, but in that case some visible overlap may develop over time between
    /// the teeth of the gear meshes.
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    pub fn PxGearJoint_setHinges_mut(self_: *mut PxGearJoint, hinge0: *const PxBase, hinge1: *const PxBase) -> bool;

    /// Set the desired gear ratio.
    ///
    /// For two gears with n0 and n1 teeth respectively, the gear ratio is n0/n1.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    pub fn PxGearJoint_setGearRatio_mut(self_: *mut PxGearJoint, ratio: f32);

    /// Get the gear ratio.
    ///
    /// Current ratio
    pub fn PxGearJoint_getGearRatio(self_: *const PxGearJoint) -> f32;

    pub fn PxGearJoint_getConcreteTypeName(self_: *const PxGearJoint) -> *const std::ffi::c_char;

    /// Create a rack
    /// &
    /// pinion Joint.
    pub fn phys_PxRackAndPinionJointCreate(physics: *mut PxPhysics, actor0: *mut PxRigidActor, localFrame0: *const PxTransform, actor1: *mut PxRigidActor, localFrame1: *const PxTransform) -> *mut PxRackAndPinionJoint;

    /// Set the hinge
    /// &
    /// prismatic joints connected by the rack
    /// &
    /// pinion joint.
    ///
    /// The passed hinge joint can be either PxRevoluteJoint, PxD6Joint or PxArticulationJointReducedCoordinate. It cannot be null.
    /// The passed prismatic joint can be either PxPrismaticJoint or PxD6Joint. It cannot be null.
    ///
    /// Note that these joints are only used to compute the positional error correction term,
    /// used to adjust potential drift between jointed actors. The rack
    /// &
    /// pinion joint can run without
    /// calling this function, but in that case some visible overlap may develop over time between
    /// the teeth of the rack
    /// &
    /// pinion meshes.
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    pub fn PxRackAndPinionJoint_setJoints_mut(self_: *mut PxRackAndPinionJoint, hinge: *const PxBase, prismatic: *const PxBase) -> bool;

    /// Set the desired ratio directly.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
    pub fn PxRackAndPinionJoint_setRatio_mut(self_: *mut PxRackAndPinionJoint, ratio: f32);

    /// Get the ratio.
    ///
    /// Current ratio
    pub fn PxRackAndPinionJoint_getRatio(self_: *const PxRackAndPinionJoint) -> f32;

    /// Set the desired ratio indirectly.
    ///
    /// This is a simple helper function that computes the ratio from passed data:
    ///
    /// ratio = (PI*2*nbRackTeeth)/(rackLength*nbPinionTeeth)
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
    pub fn PxRackAndPinionJoint_setData_mut(self_: *mut PxRackAndPinionJoint, nbRackTeeth: u32, nbPinionTeeth: u32, rackLength: f32) -> bool;

    pub fn PxRackAndPinionJoint_getConcreteTypeName(self_: *const PxRackAndPinionJoint) -> *const std::ffi::c_char;

    pub fn PxGroupsMask_new_alloc() -> *mut PxGroupsMask;

    pub fn PxGroupsMask_delete(self_: *mut PxGroupsMask);

    /// Implementation of a simple filter shader that emulates PhysX 2.8.x filtering
    ///
    /// This shader provides the following logic:
    ///
    /// If one of the two filter objects is a trigger, the pair is acccepted and [`PxPairFlag::eTRIGGER_DEFAULT`] will be used for trigger reports
    ///
    /// Else, if the filter mask logic (see further below) discards the pair it will be suppressed ([`PxFilterFlag::eSUPPRESS`])
    ///
    /// Else, the pair gets accepted and collision response gets enabled ([`PxPairFlag::eCONTACT_DEFAULT`])
    ///
    /// Filter mask logic:
    /// Given the two [`PxFilterData`] structures fd0 and fd1 of two collision objects, the pair passes the filter if the following
    /// conditions are met:
    ///
    /// 1) Collision groups of the pair are enabled
    /// 2) Collision filtering equation is satisfied
    pub fn phys_PxDefaultSimulationFilterShader(attributes0: u32, filterData0: PxFilterData, attributes1: u32, filterData1: PxFilterData, pairFlags: *mut PxPairFlags, constantBlock: *const std::ffi::c_void, constantBlockSize: u32) -> PxFilterFlags;

    /// Determines if collision detection is performed between a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// True if the groups could collide
    pub fn phys_PxGetGroupCollisionFlag(group1: u16, group2: u16) -> bool;

    /// Specifies if collision should be performed by a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    pub fn phys_PxSetGroupCollisionFlag(group1: u16, group2: u16, enable: bool);

    /// Retrieves the value set with PxSetGroup()
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// The collision group this actor belongs to
    pub fn phys_PxGetGroup(actor: *const PxActor) -> u16;

    /// Sets which collision group this actor is part of
    ///
    /// Collision group is an integer between 0 and 31.
    pub fn phys_PxSetGroup(actor: *mut PxActor, collisionGroup: u16);

    /// Retrieves filtering operation. See comments for PxGroupsMask
    pub fn phys_PxGetFilterOps(op0: *mut PxFilterOp, op1: *mut PxFilterOp, op2: *mut PxFilterOp);

    /// Setups filtering operations. See comments for PxGroupsMask
    pub fn phys_PxSetFilterOps(op0: *const PxFilterOp, op1: *const PxFilterOp, op2: *const PxFilterOp);

    /// Retrieves filtering's boolean value. See comments for PxGroupsMask
    ///
    /// flag Boolean value for filter.
    pub fn phys_PxGetFilterBool() -> bool;

    /// Setups filtering's boolean value. See comments for PxGroupsMask
    pub fn phys_PxSetFilterBool(enable: bool);

    /// Gets filtering constant K0 and K1. See comments for PxGroupsMask
    pub fn phys_PxGetFilterConstants(c0: *mut PxGroupsMask, c1: *mut PxGroupsMask);

    /// Setups filtering's K0 and K1 value. See comments for PxGroupsMask
    pub fn phys_PxSetFilterConstants(c0: *const PxGroupsMask, c1: *const PxGroupsMask);

    /// Gets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    ///
    /// The group mask for the actor.
    pub fn phys_PxGetGroupsMask(actor: *const PxActor) -> PxGroupsMask;

    /// Sets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    pub fn phys_PxSetGroupsMask(actor: *mut PxActor, mask: *const PxGroupsMask);

    pub fn PxDefaultErrorCallback_new_alloc() -> *mut PxDefaultErrorCallback;

    pub fn PxDefaultErrorCallback_delete(self_: *mut PxDefaultErrorCallback);

    pub fn PxDefaultErrorCallback_reportError_mut(self_: *mut PxDefaultErrorCallback, code: PxErrorCode, message: *const std::ffi::c_char, file: *const std::ffi::c_char, line: i32);

    /// Creates a new shape with default properties and a list of materials and adds it to the list of shapes of this actor.
    ///
    /// This is equivalent to the following
    ///
    /// ```cpp
    /// // reference count is 1
    /// PxShape* shape(...) = PxGetPhysics().createShape(...);
    /// // increments reference count
    /// actor->attachShape(shape);
    /// // releases user reference, leaving reference count at 1
    /// shape->release();
    /// ```
    ///
    /// As a consequence, detachShape() will result in the release of the last reference, and the shape will be deleted.
    ///
    /// The default shape flags to be set are: eVISUALIZATION, eSIMULATION_SHAPE, eSCENE_QUERY_SHAPE (see [`PxShapeFlag`]).
    /// Triangle mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Creating compounds with a very large number of shapes may adversely affect performance and stability.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// The newly created shape.
    pub fn PxRigidActorExt_createExclusiveShape(actor: *mut PxRigidActor, geometry: *const PxGeometry, materials: *const *mut PxMaterial, materialCount: u16, shapeFlags: PxShapeFlags) -> *mut PxShape;

    /// Creates a new shape with default properties and a single material adds it to the list of shapes of this actor.
    ///
    /// This is equivalent to the following
    ///
    /// ```cpp
    /// // reference count is 1
    /// PxShape* shape(...) = PxGetPhysics().createShape(...);
    /// // increments reference count
    /// actor->attachShape(shape);
    /// // releases user reference, leaving reference count at 1
    /// shape->release();
    /// ```
    ///
    /// As a consequence, detachShape() will result in the release of the last reference, and the shape will be deleted.
    ///
    /// The default shape flags to be set are: eVISUALIZATION, eSIMULATION_SHAPE, eSCENE_QUERY_SHAPE (see [`PxShapeFlag`]).
    /// Triangle mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Creating compounds with a very large number of shapes may adversely affect performance and stability.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// The newly created shape.
    pub fn PxRigidActorExt_createExclusiveShape_1(actor: *mut PxRigidActor, geometry: *const PxGeometry, material: *const PxMaterial, shapeFlags: PxShapeFlags) -> *mut PxShape;

    /// Gets a list of bounds based on shapes in rigid actor. This list can be used to cook/create
    /// bounding volume hierarchy though PxCooking API.
    pub fn PxRigidActorExt_getRigidActorShapeLocalBoundsList(actor: *const PxRigidActor, numBounds: *mut u32) -> *mut PxBounds3;

    /// Convenience function to create a PxBVH object from a PxRigidActor.
    ///
    /// The computed PxBVH can then be used in PxScene::addActor() or PxAggregate::addActor().
    /// After adding the actor
    /// &
    /// BVH to the scene/aggregate, release the PxBVH object by calling PxBVH::release().
    ///
    /// The PxBVH for this actor.
    pub fn PxRigidActorExt_createBVHFromActor(physics: *mut PxPhysics, actor: *const PxRigidActor) -> *mut PxBVH;

    /// Default constructor.
    pub fn PxMassProperties_new() -> PxMassProperties;

    /// Construct from individual elements.
    pub fn PxMassProperties_new_1(m: f32, inertiaT: *const PxMat33, com: *const PxVec3) -> PxMassProperties;

    /// Compute mass properties based on a provided geometry structure.
    ///
    /// This constructor assumes the geometry has a density of 1. Mass and inertia tensor scale linearly with density.
    pub fn PxMassProperties_new_2(geometry: *const PxGeometry) -> PxMassProperties;

    /// Translate the center of mass by a given vector and adjust the inertia tensor accordingly.
    pub fn PxMassProperties_translate_mut(self_: *mut PxMassProperties, t: *const PxVec3);

    /// Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.
    ///
    /// The entries of the diagonalized inertia tensor.
    pub fn PxMassProperties_getMassSpaceInertia(inertia: *const PxMat33, massFrame: *mut PxQuat) -> PxVec3;

    /// Translate an inertia tensor using the parallel axis theorem
    ///
    /// The translated inertia tensor.
    pub fn PxMassProperties_translateInertia(inertia: *const PxMat33, mass: f32, t: *const PxVec3) -> PxMat33;

    /// Rotate an inertia tensor around the center of mass
    ///
    /// The rotated inertia tensor.
    pub fn PxMassProperties_rotateInertia(inertia: *const PxMat33, q: *const PxQuat) -> PxMat33;

    /// Non-uniform scaling of the inertia tensor
    ///
    /// The scaled inertia tensor.
    pub fn PxMassProperties_scaleInertia(inertia: *const PxMat33, scaleRotation: *const PxQuat, scale: *const PxVec3) -> PxMat33;

    /// Sum up individual mass properties.
    ///
    /// The summed up mass properties.
    pub fn PxMassProperties_sum(props: *const PxMassProperties, transforms: *const PxTransform, count: u32) -> PxMassProperties;

    /// Computation of mass properties for a rigid body actor
    ///
    /// To simulate a dynamic rigid actor, the SDK needs a mass and an inertia tensor.
    ///
    /// This method offers functionality to compute the necessary mass and inertia properties based on the shapes declared in
    /// the PxRigidBody descriptor and some additionally specified parameters. For each shape, the shape geometry,
    /// the shape positioning within the actor and the specified shape density are used to compute the body's mass and
    /// inertia properties.
    ///
    /// Shapes without PxShapeFlag::eSIMULATION_SHAPE set are ignored unless includeNonSimShapes is true.
    /// Shapes with plane, triangle mesh or heightfield geometry and PxShapeFlag::eSIMULATION_SHAPE set are not allowed for PxRigidBody collision.
    ///
    /// This method will set the mass, center of mass, and inertia tensor
    ///
    /// if no collision shapes are found, the inertia tensor is set to (1,1,1) and the mass to 1
    ///
    /// if massLocalPose is non-NULL, the rigid body's center of mass parameter  will be set
    /// to the user provided value (massLocalPose) and the inertia tensor will be resolved at that point.
    ///
    /// If all shapes of the actor have the same density then the overloaded method updateMassAndInertia() with a single density parameter can be used instead.
    ///
    /// Boolean. True on success else false.
    pub fn PxRigidBodyExt_updateMassAndInertia(body: *mut PxRigidBody, shapeDensities: *const f32, shapeDensityCount: u32, massLocalPose: *const PxVec3, includeNonSimShapes: bool) -> bool;

    /// Computation of mass properties for a rigid body actor
    ///
    /// See previous method for details.
    ///
    /// Boolean. True on success else false.
    pub fn PxRigidBodyExt_updateMassAndInertia_1(body: *mut PxRigidBody, density: f32, massLocalPose: *const PxVec3, includeNonSimShapes: bool) -> bool;

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the sum of all user-supplied
    /// shape mass values, and the inertia and center of mass are computed according to the rigid body's shapes and the per shape mass input values.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// If a single mass value should be used for the actor as a whole then the overloaded method setMassAndUpdateInertia() with a single mass parameter can be used instead.
    ///
    /// Boolean. True on success else false.
    pub fn PxRigidBodyExt_setMassAndUpdateInertia(body: *mut PxRigidBody, shapeMasses: *const f32, shapeMassCount: u32, massLocalPose: *const PxVec3, includeNonSimShapes: bool) -> bool;

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the user-supplied
    /// value, and the inertia and center of mass are computed according to the rigid body's shapes and the input mass.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// Boolean. True on success else false.
    pub fn PxRigidBodyExt_setMassAndUpdateInertia_1(body: *mut PxRigidBody, mass: f32, massLocalPose: *const PxVec3, includeNonSimShapes: bool) -> bool;

    /// Compute the mass, inertia tensor and center of mass from a list of shapes.
    ///
    /// The mass properties from the combined shapes.
    pub fn PxRigidBodyExt_computeMassPropertiesFromShapes(shapes: *const *const PxShape, shapeCount: u32) -> PxMassProperties;

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in global coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep,
    /// you can maintain a total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    pub fn PxRigidBodyExt_addForceAtPos(body: *mut PxRigidBody, force: *const PxVec3, pos: *const PxVec3, mode: PxForceMode, wakeup: bool);

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in local coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    pub fn PxRigidBodyExt_addForceAtLocalPos(body: *mut PxRigidBody, force: *const PxVec3, pos: *const PxVec3, mode: PxForceMode, wakeup: bool);

    /// Applies a force (or impulse) defined in the actor local coordinate frame, acting at a
    /// particular point in global coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    pub fn PxRigidBodyExt_addLocalForceAtPos(body: *mut PxRigidBody, force: *const PxVec3, pos: *const PxVec3, mode: PxForceMode, wakeup: bool);

    /// Applies a force (or impulse) defined in the actor local coordinate frame, acting at a
    /// particular point in local coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
    pub fn PxRigidBodyExt_addLocalForceAtLocalPos(body: *mut PxRigidBody, force: *const PxVec3, pos: *const PxVec3, mode: PxForceMode, wakeup: bool);

    /// Computes the velocity of a point given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the global frame.
    pub fn PxRigidBodyExt_getVelocityAtPos(body: *const PxRigidBody, pos: *const PxVec3) -> PxVec3;

    /// Computes the velocity of a point given in local coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the local frame.
    pub fn PxRigidBodyExt_getLocalVelocityAtLocalPos(body: *const PxRigidBody, pos: *const PxVec3) -> PxVec3;

    /// Computes the velocity of a point (offset from the origin of the body) given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point (offset from the origin of the body) in the global frame.
    pub fn PxRigidBodyExt_getVelocityAtOffset(body: *const PxRigidBody, pos: *const PxVec3) -> PxVec3;

    /// Compute the change to linear and angular velocity that would occur if an impulsive force and torque were to be applied to a specified rigid body.
    ///
    /// The rigid body is left unaffected unless a subsequent independent call is executed that actually applies the computed changes to velocity and angular velocity.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    pub fn PxRigidBodyExt_computeVelocityDeltaFromImpulse(body: *const PxRigidBody, impulsiveForce: *const PxVec3, impulsiveTorque: *const PxVec3, deltaLinearVelocity: *mut PxVec3, deltaAngularVelocity: *mut PxVec3);

    /// Computes the linear and angular velocity change vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular velocity changes from a contact or joint when the mass/inertia ratios have been adjusted.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
    pub fn PxRigidBodyExt_computeVelocityDeltaFromImpulse_1(body: *const PxRigidBody, globalPose: *const PxTransform, point: *const PxVec3, impulse: *const PxVec3, invMassScale: f32, invInertiaScale: f32, deltaLinearVelocity: *mut PxVec3, deltaAngularVelocity: *mut PxVec3);

    /// Computes the linear and angular impulse vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular impulses from a contact or joint when the mass/inertia ratios have been adjusted.
    pub fn PxRigidBodyExt_computeLinearAngularImpulse(body: *const PxRigidBody, globalPose: *const PxTransform, point: *const PxVec3, impulse: *const PxVec3, invMassScale: f32, invInertiaScale: f32, linearImpulse: *mut PxVec3, angularImpulse: *mut PxVec3);

    /// Performs a linear sweep through space with the body's geometry objects.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// If eTOUCH is returned from the filter callback, it will trigger an error and the hit will be discarded.
    ///
    /// The function sweeps all shapes attached to a given rigid body through space and reports the nearest
    /// object in the scene which intersects any of of the shapes swept paths.
    /// Information about the closest intersection is written to a [`PxSweepHit`] structure.
    ///
    /// True if a blocking hit was found.
    pub fn PxRigidBodyExt_linearSweepSingle(body: *mut PxRigidBody, scene: *mut PxScene, unitDir: *const PxVec3, distance: f32, outputFlags: PxHitFlags, closestHit: *mut PxSweepHit, shapeIndex: *mut u32, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, inflation: f32) -> bool;

    /// Performs a linear sweep through space with the body's geometry objects, returning all overlaps.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// This function sweeps all shapes attached to a given rigid body through space and reports all
    /// objects in the scene that intersect any of the shapes' swept paths until there are no more objects to report
    /// or a blocking hit is encountered.
    ///
    /// the number of touching hits. If overflow is set to true, the results are incomplete. In case of overflow there are also no guarantees that all touching hits returned are closer than the blocking hit.
    pub fn PxRigidBodyExt_linearSweepMultiple(body: *mut PxRigidBody, scene: *mut PxScene, unitDir: *const PxVec3, distance: f32, outputFlags: PxHitFlags, touchHitBuffer: *mut PxSweepHit, touchHitShapeIndices: *mut u32, touchHitBufferSize: u32, block: *mut PxSweepHit, blockingShapeIndex: *mut i32, overflow: *mut bool, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, inflation: f32) -> u32;

    /// Retrieves the world space pose of the shape.
    ///
    /// Global pose of shape.
    pub fn PxShapeExt_getGlobalPose(shape: *const PxShape, actor: *const PxRigidActor) -> PxTransform;

    /// Raycast test against the shape.
    ///
    /// Number of hits between the ray and the shape
    pub fn PxShapeExt_raycast(shape: *const PxShape, actor: *const PxRigidActor, rayOrigin: *const PxVec3, rayDir: *const PxVec3, maxDist: f32, hitFlags: PxHitFlags, maxHits: u32, rayHits: *mut PxRaycastHit) -> u32;

    /// Test overlap between the shape and a geometry object
    ///
    /// True if the shape overlaps the geometry object
    pub fn PxShapeExt_overlap(shape: *const PxShape, actor: *const PxRigidActor, otherGeom: *const PxGeometry, otherGeomPose: *const PxTransform) -> bool;

    /// Sweep a geometry object against the shape.
    ///
    /// Currently only box, sphere, capsule and convex mesh shapes are supported, i.e. the swept geometry object must be one of those types.
    ///
    /// True if the swept geometry object hits the shape
    pub fn PxShapeExt_sweep(shape: *const PxShape, actor: *const PxRigidActor, unitDir: *const PxVec3, distance: f32, otherGeom: *const PxGeometry, otherGeomPose: *const PxTransform, sweepHit: *mut PxSweepHit, hitFlags: PxHitFlags) -> bool;

    /// Retrieves the axis aligned bounding box enclosing the shape.
    ///
    /// The shape's bounding box.
    pub fn PxShapeExt_getWorldBounds(shape: *const PxShape, actor: *const PxRigidActor, inflation: f32) -> PxBounds3;

    pub fn PxMeshOverlapUtil_new_alloc() -> *mut PxMeshOverlapUtil;

    pub fn PxMeshOverlapUtil_delete(self_: *mut PxMeshOverlapUtil);

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    pub fn PxMeshOverlapUtil_findOverlap_mut(self_: *mut PxMeshOverlapUtil, geom: *const PxGeometry, geomPose: *const PxTransform, meshGeom: *const PxTriangleMeshGeometry, meshPose: *const PxTransform) -> u32;

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
    pub fn PxMeshOverlapUtil_findOverlap_mut_1(self_: *mut PxMeshOverlapUtil, geom: *const PxGeometry, geomPose: *const PxTransform, hfGeom: *const PxHeightFieldGeometry, hfPose: *const PxTransform) -> u32;

    /// Retrieves array of triangle indices after a findOverlap call.
    ///
    /// Indices of touched triangles
    pub fn PxMeshOverlapUtil_getResults(self_: *const PxMeshOverlapUtil) -> *const u32;

    /// Retrieves number of triangle indices after a findOverlap call.
    ///
    /// Number of touched triangles
    pub fn PxMeshOverlapUtil_getNbResults(self_: *const PxMeshOverlapUtil) -> u32;

    /// Computes an approximate minimum translational distance (MTD) between a geometry object and a mesh.
    ///
    /// This iterative function computes an approximate vector that can be used to depenetrate a geom object
    /// from a triangle mesh. Returned depenetration vector should be applied to 'geom', to get out of the mesh.
    ///
    /// The function works best when the amount of overlap between the geom object and the mesh is small. If the
    /// geom object's center goes inside the mesh, backface culling usually kicks in, no overlap is detected,
    /// and the function does not compute an MTD vector.
    ///
    /// The function early exits if no overlap is detected after a depenetration attempt. This means that if
    /// maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
    /// been successful. Usually N = 4 gives good results.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
    pub fn phys_PxComputeTriangleMeshPenetration(direction: *mut PxVec3, depth: *mut f32, geom: *const PxGeometry, geomPose: *const PxTransform, meshGeom: *const PxTriangleMeshGeometry, meshPose: *const PxTransform, maxIter: u32, usedIter: *mut u32) -> bool;

    /// Computes an approximate minimum translational distance (MTD) between a geometry object and a heightfield.
    ///
    /// This iterative function computes an approximate vector that can be used to depenetrate a geom object
    /// from a heightfield. Returned depenetration vector should be applied to 'geom', to get out of the heightfield.
    ///
    /// The function works best when the amount of overlap between the geom object and the mesh is small. If the
    /// geom object's center goes inside the heightfield, backface culling usually kicks in, no overlap is detected,
    /// and the function does not compute an MTD vector.
    ///
    /// The function early exits if no overlap is detected after a depenetration attempt. This means that if
    /// maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
    /// been successful. Usually N = 4 gives good results.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
    pub fn phys_PxComputeHeightFieldPenetration(direction: *mut PxVec3, depth: *mut f32, geom: *const PxGeometry, geomPose: *const PxTransform, heightFieldGeom: *const PxHeightFieldGeometry, heightFieldPose: *const PxTransform, maxIter: u32, usedIter: *mut u32) -> bool;

    pub fn PxXmlMiscParameter_new() -> PxXmlMiscParameter;

    pub fn PxXmlMiscParameter_new_1(inUpVector: *mut PxVec3, inScale: PxTolerancesScale) -> PxXmlMiscParameter;

    /// Returns whether the collection is serializable with the externalReferences collection.
    ///
    /// Some definitions to explain whether a collection can be serialized or not:
    ///
    /// For definitions of
    /// requires
    /// and
    /// complete
    /// see [`PxSerialization::complete`]
    ///
    /// A serializable object is
    /// subordinate
    /// if it cannot be serialized on its own
    /// The following objects are subordinate:
    /// - articulation links
    /// - articulation joints
    /// - joints
    ///
    /// A collection C can be serialized with external references collection D iff
    /// - C is complete relative to D (no dangling references)
    /// - Every object in D required by an object in C has a valid ID (no unnamed references)
    /// - Every subordinate object in C is required by another object in C (no orphans)
    ///
    /// Whether the collection is serializable
    pub fn PxSerialization_isSerializable(collection: *mut PxCollection, sr: *mut PxSerializationRegistry, externalReferences: *const PxCollection) -> bool;

    /// Adds to a collection all objects such that it can be successfully serialized.
    ///
    /// A collection C is complete relative to an other collection D if every object required by C is either in C or D.
    /// This function adds objects to a collection, such that it becomes complete with respect to the exceptFor collection.
    /// Completeness is needed for serialization. See [`PxSerialization::serializeCollectionToBinary`],
    /// [`PxSerialization::serializeCollectionToXml`].
    ///
    /// Sdk objects require other sdk object according to the following rules:
    /// - joints require their actors and constraint
    /// - rigid actors require their shapes
    /// - shapes require their material(s) and mesh (triangle mesh, convex mesh or height field), if any
    /// - articulations require their links and joints
    /// - aggregates require their actors
    ///
    /// If followJoints is specified another rule is added:
    /// - actors require their joints
    ///
    /// Specifying followJoints will make whole jointed actor chains being added to the collection. Following chains
    /// is interrupted whenever a object in exceptFor is encountered.
    pub fn PxSerialization_complete(collection: *mut PxCollection, sr: *mut PxSerializationRegistry, exceptFor: *const PxCollection, followJoints: bool);

    /// Creates PxSerialObjectId values for unnamed objects in a collection.
    ///
    /// Creates PxSerialObjectId names for unnamed objects in a collection starting at a base value and incrementing,
    /// skipping values that are already assigned to objects in the collection.
    pub fn PxSerialization_createSerialObjectIds(collection: *mut PxCollection, base: u64);

    /// Creates a PxCollection from XML data.
    ///
    /// a pointer to a PxCollection if successful or NULL if it failed.
    pub fn PxSerialization_createCollectionFromXml(inputData: *mut PxInputData, cooking: *mut PxCooking, sr: *mut PxSerializationRegistry, externalRefs: *const PxCollection, stringTable: *mut PxStringTable, outArgs: *mut PxXmlMiscParameter) -> *mut PxCollection;

    /// Deserializes a PxCollection from memory.
    ///
    /// Creates a collection from memory. If the collection has external dependencies another collection
    /// can be provided to resolve these.
    ///
    /// The memory block provided has to be 128 bytes aligned and contain a contiguous serialized collection as written
    /// by PxSerialization::serializeCollectionToBinary. The contained binary data needs to be compatible with the current binary format version
    /// which is defined by "PX_PHYSICS_VERSION_MAJOR.PX_PHYSICS_VERSION_MINOR.PX_PHYSICS_VERSION_BUGFIX-PX_BINARY_SERIAL_VERSION".
    /// For a list of compatible sdk releases refer to the documentation of PX_BINARY_SERIAL_VERSION.
    pub fn PxSerialization_createCollectionFromBinary(memBlock: *mut std::ffi::c_void, sr: *mut PxSerializationRegistry, externalRefs: *const PxCollection) -> *mut PxCollection;

    /// Serializes a physics collection to an XML output stream.
    ///
    /// The collection to be serialized needs to be complete
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// true if the collection is successfully serialized.
    pub fn PxSerialization_serializeCollectionToXml(outputStream: *mut PxOutputStream, collection: *mut PxCollection, sr: *mut PxSerializationRegistry, cooking: *mut PxCooking, externalRefs: *const PxCollection, inArgs: *mut PxXmlMiscParameter) -> bool;

    /// Serializes a collection to a binary stream.
    ///
    /// Serializes a collection to a stream. In order to resolve external dependencies the externalReferences collection has to be provided.
    /// Optionally names of objects that where set for example with [`PxActor::setName`] are serialized along with the objects.
    ///
    /// The collection can be successfully serialized if isSerializable(collection) returns true. See [`isSerializable`].
    ///
    /// The implementation of the output stream needs to fulfill the requirements on the memory block input taken by
    /// PxSerialization::createCollectionFromBinary.
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// Whether serialization was successful
    pub fn PxSerialization_serializeCollectionToBinary(outputStream: *mut PxOutputStream, collection: *mut PxCollection, sr: *mut PxSerializationRegistry, externalRefs: *const PxCollection, exportNames: bool) -> bool;

    /// Creates an application managed registry for serialization.
    ///
    /// PxSerializationRegistry instance.
    pub fn PxSerialization_createSerializationRegistry(physics: *mut PxPhysics) -> *mut PxSerializationRegistry;

    /// Deletes the dispatcher.
    ///
    /// Do not keep a reference to the deleted instance.
    pub fn PxDefaultCpuDispatcher_release_mut(self_: *mut PxDefaultCpuDispatcher);

    /// Enables profiling at task level.
    ///
    /// By default enabled only in profiling builds.
    pub fn PxDefaultCpuDispatcher_setRunProfiled_mut(self_: *mut PxDefaultCpuDispatcher, runProfiled: bool);

    /// Checks if profiling is enabled at task level.
    ///
    /// True if tasks should be profiled.
    pub fn PxDefaultCpuDispatcher_getRunProfiled(self_: *const PxDefaultCpuDispatcher) -> bool;

    /// Create default dispatcher, extensions SDK needs to be initialized first.
    ///
    /// numThreads may be zero in which case no worker thread are initialized and
    /// simulation tasks will be executed on the thread that calls PxScene::simulate()
    ///
    /// yieldProcessorCount must be greater than zero if eYIELD_PROCESSOR is the chosen mode and equal to zero for all other modes.
    ///
    /// eYIELD_THREAD and eYIELD_PROCESSOR modes will use compute resources even if the simulation is not running.
    /// It is left to users to keep threads inactive, if so desired, when no simulation is running.
    pub fn phys_PxDefaultCpuDispatcherCreate(numThreads: u32, affinityMasks: *mut u32, mode: PxDefaultCpuDispatcherWaitForWorkMode, yieldProcessorCount: u32) -> *mut PxDefaultCpuDispatcher;

    /// Builds smooth vertex normals over a mesh.
    ///
    /// - "smooth" because smoothing groups are not supported here
    /// - takes angles into account for correct cube normals computation
    ///
    /// To use 32bit indices pass a pointer in dFaces and set wFaces to zero. Alternatively pass a pointer to
    /// wFaces and set dFaces to zero.
    ///
    /// True on success.
    pub fn phys_PxBuildSmoothNormals(nbTris: u32, nbVerts: u32, verts: *const PxVec3, dFaces: *const u32, wFaces: *const u16, normals: *mut PxVec3, flip: bool) -> bool;

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    pub fn phys_PxCreateDynamic(sdk: *mut PxPhysics, transform: *const PxTransform, geometry: *const PxGeometry, material: *mut PxMaterial, density: f32, shapeOffset: *const PxTransform) -> *mut PxRigidDynamic;

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
    pub fn phys_PxCreateDynamic_1(sdk: *mut PxPhysics, transform: *const PxTransform, shape: *mut PxShape, density: f32) -> *mut PxRigidDynamic;

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
    pub fn phys_PxCreateKinematic(sdk: *mut PxPhysics, transform: *const PxTransform, geometry: *const PxGeometry, material: *mut PxMaterial, density: f32, shapeOffset: *const PxTransform) -> *mut PxRigidDynamic;

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
    pub fn phys_PxCreateKinematic_1(sdk: *mut PxPhysics, transform: *const PxTransform, shape: *mut PxShape, density: f32) -> *mut PxRigidDynamic;

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    pub fn phys_PxCreateStatic(sdk: *mut PxPhysics, transform: *const PxTransform, geometry: *const PxGeometry, material: *mut PxMaterial, shapeOffset: *const PxTransform) -> *mut PxRigidStatic;

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
    pub fn phys_PxCreateStatic_1(sdk: *mut PxPhysics, transform: *const PxTransform, shape: *mut PxShape) -> *mut PxRigidStatic;

    /// create a shape by copying attributes from another shape
    ///
    /// The function clones a PxShape. The following properties are copied:
    /// - geometry
    /// - flags
    /// - materials
    /// - actor-local pose
    /// - contact offset
    /// - rest offset
    /// - simulation filter data
    /// - query filter data
    /// - torsional patch radius
    /// - minimum torsional patch radius
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - user data
    ///
    /// the newly-created rigid static
    pub fn phys_PxCloneShape(physicsSDK: *mut PxPhysics, shape: *const PxShape, isExclusive: bool) -> *mut PxShape;

    /// create a static body by copying attributes from another rigid actor
    ///
    /// The function clones a PxRigidDynamic or PxRigidStatic as a PxRigidStatic. A uniform scale is applied. The following properties are copied:
    /// - shapes
    /// - actor flags
    /// - owner client and client behavior bits
    /// - dominance group
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - joints or observers
    /// - aggregate or scene membership
    /// - user data
    ///
    /// Transforms are not copied with bit-exact accuracy.
    ///
    /// the newly-created rigid static
    pub fn phys_PxCloneStatic(physicsSDK: *mut PxPhysics, transform: *const PxTransform, actor: *const PxRigidActor) -> *mut PxRigidStatic;

    /// create a dynamic body by copying attributes from an existing body
    ///
    /// The following properties are copied:
    /// - shapes
    /// - actor flags, rigidDynamic flags and rigidDynamic lock flags
    /// - mass, moment of inertia, and center of mass frame
    /// - linear and angular velocity
    /// - linear and angular damping
    /// - maximum linear velocity
    /// - maximum angular velocity
    /// - position and velocity solver iterations
    /// - maximum depenetration velocity
    /// - sleep threshold
    /// - contact report threshold
    /// - dominance group
    /// - owner client and client behavior bits
    /// - name pointer
    /// - kinematic target
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - joints or observers
    /// - aggregate or scene membership
    /// - sleep timer
    /// - user data
    ///
    /// Transforms are not copied with bit-exact accuracy.
    ///
    /// the newly-created rigid static
    pub fn phys_PxCloneDynamic(physicsSDK: *mut PxPhysics, transform: *const PxTransform, body: *const PxRigidDynamic) -> *mut PxRigidDynamic;

    /// create a plane actor. The plane equation is n.x + d = 0
    ///
    /// a new static actor, or NULL if it could not be constructed
    pub fn phys_PxCreatePlane(sdk: *mut PxPhysics, plane: *const PxPlane, material: *mut PxMaterial) -> *mut PxRigidStatic;

    /// scale a rigid actor by a uniform scale
    ///
    /// The geometry and relative positions of the actor are multiplied by the given scale value. If the actor is a rigid body or an
    /// articulation link and the scaleMassProps value is true, the mass properties are scaled assuming the density is constant: the
    /// center of mass is linearly scaled, the mass is multiplied by the cube of the scale, and the inertia tensor by the fifth power of the scale.
    pub fn phys_PxScaleRigidActor(actor: *mut PxRigidActor, scale: f32, scaleMassProps: bool);

    pub fn PxStringTableExt_createStringTable(inAllocator: *mut PxAllocatorCallback) -> *mut PxStringTable;

    /// Creates regions for PxSceneDesc, from a global box.
    ///
    /// This helper simply subdivides the given global box into a 2D grid of smaller boxes. Each one of those smaller boxes
    /// is a region of interest for the broadphase. There are nbSubdiv*nbSubdiv regions in the 2D grid. The function does not
    /// subdivide along the given up axis.
    ///
    /// This is the simplest setup one can use with PxBroadPhaseType::eMBP. A more sophisticated setup would try to cover
    /// the game world with a non-uniform set of regions (i.e. not just a grid).
    ///
    /// number of regions written out to the 'regions' array
    pub fn PxBroadPhaseExt_createRegionsFromWorldBounds(regions: *mut PxBounds3, globalBounds: *const PxBounds3, nbSubdiv: u32, upAxis: u32) -> u32;

    /// Raycast returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the ray.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    pub fn PxSceneQueryExt_raycastAny(scene: *const PxScene, origin: *const PxVec3, unitDir: *const PxVec3, distance: f32, hit: *mut PxQueryHit, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache) -> bool;

    /// Raycast returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
    pub fn PxSceneQueryExt_raycastSingle(scene: *const PxScene, origin: *const PxVec3, unitDir: *const PxVec3, distance: f32, outputFlags: PxHitFlags, hit: *mut PxRaycastHit, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache) -> bool;

    /// Raycast returning multiple results.
    ///
    /// Find all rigid actors that get hit along the ray. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    pub fn PxSceneQueryExt_raycastMultiple(scene: *const PxScene, origin: *const PxVec3, unitDir: *const PxVec3, distance: f32, outputFlags: PxHitFlags, hitBuffer: *mut PxRaycastHit, hitBufferSize: u32, blockingHit: *mut bool, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache) -> i32;

    /// Sweep returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the sweep path.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    pub fn PxSceneQueryExt_sweepAny(scene: *const PxScene, geometry: *const PxGeometry, pose: *const PxTransform, unitDir: *const PxVec3, distance: f32, queryFlags: PxHitFlags, hit: *mut PxQueryHit, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, inflation: f32) -> bool;

    /// Sweep returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
    pub fn PxSceneQueryExt_sweepSingle(scene: *const PxScene, geometry: *const PxGeometry, pose: *const PxTransform, unitDir: *const PxVec3, distance: f32, outputFlags: PxHitFlags, hit: *mut PxSweepHit, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, inflation: f32) -> bool;

    /// Sweep returning multiple results.
    ///
    /// Find all rigid actors that get hit along the sweep. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    pub fn PxSceneQueryExt_sweepMultiple(scene: *const PxScene, geometry: *const PxGeometry, pose: *const PxTransform, unitDir: *const PxVec3, distance: f32, outputFlags: PxHitFlags, hitBuffer: *mut PxSweepHit, hitBufferSize: u32, blockingHit: *mut bool, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback, cache: *const PxQueryCache, inflation: f32) -> i32;

    /// Test overlap between a geometry and objects in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both get written to the hit buffer.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
    pub fn PxSceneQueryExt_overlapMultiple(scene: *const PxScene, geometry: *const PxGeometry, pose: *const PxTransform, hitBuffer: *mut PxOverlapHit, hitBufferSize: u32, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback) -> i32;

    /// Test returning, for a given geometry, any overlapping object in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both trigger a hit.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// True if an overlap was found.
    pub fn PxSceneQueryExt_overlapAny(scene: *const PxScene, geometry: *const PxGeometry, pose: *const PxTransform, hit: *mut PxOverlapHit, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback) -> bool;

    pub fn PxBatchQueryExt_release_mut(self_: *mut PxBatchQueryExt);

    /// Performs a raycast against objects in the scene.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxRaycastBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
    pub fn PxBatchQueryExt_raycast_mut(self_: *mut PxBatchQueryExt, origin: *const PxVec3, unitDir: *const PxVec3, distance: f32, maxNbTouches: u16, hitFlags: PxHitFlags, filterData: *const PxQueryFilterData, cache: *const PxQueryCache) -> *mut PxRaycastBuffer;

    /// Performs a sweep test against objects in the scene.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxSweepBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
    pub fn PxBatchQueryExt_sweep_mut(self_: *mut PxBatchQueryExt, geometry: *const PxGeometry, pose: *const PxTransform, unitDir: *const PxVec3, distance: f32, maxNbTouches: u16, hitFlags: PxHitFlags, filterData: *const PxQueryFilterData, cache: *const PxQueryCache, inflation: f32) -> *mut PxSweepBuffer;

    /// Performs an overlap test of a given geometry against objects in the scene.
    ///
    /// Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see [`PxQueryHitType`]).
    ///
    /// eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
    ///
    /// If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxOverlapBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
    pub fn PxBatchQueryExt_overlap_mut(self_: *mut PxBatchQueryExt, geometry: *const PxGeometry, pose: *const PxTransform, maxNbTouches: u16, filterData: *const PxQueryFilterData, cache: *const PxQueryCache) -> *mut PxOverlapBuffer;

    pub fn PxBatchQueryExt_execute_mut(self_: *mut PxBatchQueryExt);

    /// Create a PxBatchQueryExt without the need for pre-allocated result or touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    pub fn phys_PxCreateBatchQueryExt(scene: *const PxScene, queryFilterCallback: *mut PxQueryFilterCallback, maxNbRaycasts: u32, maxNbRaycastTouches: u32, maxNbSweeps: u32, maxNbSweepTouches: u32, maxNbOverlaps: u32, maxNbOverlapTouches: u32) -> *mut PxBatchQueryExt;

    /// Create a PxBatchQueryExt with user-supplied result and touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
    pub fn phys_PxCreateBatchQueryExt_1(scene: *const PxScene, queryFilterCallback: *mut PxQueryFilterCallback, raycastBuffers: *mut PxRaycastBuffer, maxNbRaycasts: u32, raycastTouches: *mut PxRaycastHit, maxNbRaycastTouches: u32, sweepBuffers: *mut PxSweepBuffer, maxNbSweeps: u32, sweepTouches: *mut PxSweepHit, maxNbSweepTouches: u32, overlapBuffers: *mut PxOverlapBuffer, maxNbOverlaps: u32, overlapTouches: *mut PxOverlapHit, maxNbOverlapTouches: u32) -> *mut PxBatchQueryExt;

    /// Creates an external scene query system.
    ///
    /// An external SQ system is the part of a PxScene that deals with scene queries (SQ). This is usually taken care of
    /// by an internal implementation inside PxScene, but it is also possible to re-route all SQ calls to an external
    /// implementation, potentially opening the door to some customizations in behavior and features for advanced users.
    ///
    /// The following external SQ system is an example of how an implementation would look like. It re-uses much of the
    /// same code as the internal version, but it could be re-implemented in a completely different way to match users'
    /// specific needs.
    ///
    /// An external SQ system instance
    pub fn phys_PxCreateExternalSceneQuerySystem(desc: *const PxSceneQueryDesc, contextID: u64) -> *mut PxSceneQuerySystem;

    /// Adds a pruner to the system.
    ///
    /// The internal PhysX scene-query system uses two regular pruners (one for static shapes, one for dynamic shapes) and an optional
    /// compound pruner. Our custom scene query system supports an arbitrary number of regular pruners.
    ///
    /// This can be useful to reduce the load on each pruner, in particular during updates, when internal trees are rebuilt in the
    /// background. On the other hand this implementation simply iterates over all created pruners to perform queries, so their cost
    /// might increase if a large number of pruners is used.
    ///
    /// In any case this serves as an example of how the PxSceneQuerySystem API can be used to customize scene queries.
    ///
    /// A pruner index
    pub fn PxCustomSceneQuerySystem_addPruner_mut(self_: *mut PxCustomSceneQuerySystem, primaryType: PxPruningStructureType, secondaryType: PxDynamicTreeSecondaryPruner, preallocated: u32) -> u32;

    /// Start custom build-steps for all pruners
    ///
    /// This function is used in combination with customBuildstep() and finishCustomBuildstep() to let users take control
    /// of the pruners' build-step
    /// &
    /// commit calls - basically the pruners' update functions. These functions should be used
    /// with the PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED update mode, otherwise the build-steps will happen
    /// automatically in fetchResults. For N pruners it can be more efficient to use these custom build-step functions to
    /// perform the updates in parallel:
    ///
    /// - call startCustomBuildstep() first (one synchronous call)
    /// - for each pruner, call customBuildstep() (asynchronous calls from multiple threads)
    /// - once it is done, call finishCustomBuildstep() to finish the update (synchronous call)
    ///
    /// The multi-threaded update is more efficient here than what it is in PxScene, because the "flushShapes()" call is
    /// also multi-threaded (while it is not in PxScene).
    ///
    /// Note that users are responsible for locks here, and these calls should not overlap with other SQ calls. In particular
    /// one should not add new objects to the SQ system or perform queries while these calls are happening.
    ///
    /// The number of pruners in the system.
    pub fn PxCustomSceneQuerySystem_startCustomBuildstep_mut(self_: *mut PxCustomSceneQuerySystem) -> u32;

    /// Perform a custom build-step for a given pruner.
    pub fn PxCustomSceneQuerySystem_customBuildstep_mut(self_: *mut PxCustomSceneQuerySystem, index: u32);

    /// Finish custom build-steps
    ///
    /// Call this function once after all the customBuildstep() calls are done.
    pub fn PxCustomSceneQuerySystem_finishCustomBuildstep_mut(self_: *mut PxCustomSceneQuerySystem);

    pub fn PxCustomSceneQuerySystemAdapter_delete(self_: *mut PxCustomSceneQuerySystemAdapter);

    /// Gets a pruner index for an actor/shape.
    ///
    /// This user-defined function tells the system in which pruner a given actor/shape should go.
    ///
    /// The returned index must be valid, i.e. it must have been previously returned to users by PxCustomSceneQuerySystem::addPruner.
    ///
    /// A pruner index for this actor/shape.
    pub fn PxCustomSceneQuerySystemAdapter_getPrunerIndex(self_: *const PxCustomSceneQuerySystemAdapter, actor: *const PxRigidActor, shape: *const PxShape) -> u32;

    /// Pruner filtering callback.
    ///
    /// This will be called for each query to validate whether it should process a given pruner.
    ///
    /// True to process the pruner, false to skip it entirely
    pub fn PxCustomSceneQuerySystemAdapter_processPruner(self_: *const PxCustomSceneQuerySystemAdapter, prunerIndex: u32, context: *const PxQueryThreadContext, filterData: *const PxQueryFilterData, filterCall: *mut PxQueryFilterCallback) -> bool;

    /// Creates a custom scene query system.
    ///
    /// This is similar to PxCreateExternalSceneQuerySystem, except this function creates a PxCustomSceneQuerySystem object.
    /// It can be plugged to PxScene the same way, via PxSceneDesc::sceneQuerySystem.
    ///
    /// A custom SQ system instance
    pub fn phys_PxCreateCustomSceneQuerySystem(sceneQueryUpdateMode: PxSceneQueryUpdateMode, contextID: u64, adapter: *const PxCustomSceneQuerySystemAdapter, usesTreeOfPruners: bool) -> *mut PxCustomSceneQuerySystem;

    /// Computes closest polygon of the convex hull geometry for a given impact point
    /// and impact direction. When doing sweeps against a scene, one might want to delay
    /// the rather expensive computation of the hit face index for convexes until it is clear
    /// the information is really needed and then use this method to get the corresponding
    /// face index.
    ///
    /// Closest face index of the convex geometry.
    pub fn phys_PxFindFaceIndex(convexGeom: *const PxConvexMeshGeometry, geomPose: *const PxTransform, impactPos: *const PxVec3, unitDir: *const PxVec3) -> u32;

    /// Sets the sampling radius
    ///
    /// Returns true if the sampling was successful and false if there was a problem. Usually an internal overflow is the problem for very big meshes or very small sampling radii.
    pub fn PxPoissonSampler_setSamplingRadius_mut(self_: *mut PxPoissonSampler, samplingRadius: f32) -> bool;

    /// Adds new Poisson Samples inside the sphere specified
    pub fn PxPoissonSampler_addSamplesInSphere_mut(self_: *mut PxPoissonSampler, sphereCenter: *const PxVec3, sphereRadius: f32, createVolumeSamples: bool);

    /// Adds new Poisson Samples inside the box specified
    pub fn PxPoissonSampler_addSamplesInBox_mut(self_: *mut PxPoissonSampler, axisAlignedBox: *const PxBounds3, boxOrientation: *const PxQuat, createVolumeSamples: bool);

    pub fn PxPoissonSampler_delete(self_: *mut PxPoissonSampler);

    /// Creates a shape sampler
    ///
    /// Returns the sampler
    pub fn phys_PxCreateShapeSampler(geometry: *const PxGeometry, transform: *const PxTransform, worldBounds: *const PxBounds3, initialSamplingRadius: f32, numSampleAttemptsAroundPoint: i32) -> *mut PxPoissonSampler;

    /// Checks whether a point is inside the triangle mesh
    ///
    /// Returns true if the point is inside the triangle mesh
    pub fn PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut(self_: *mut PxTriangleMeshPoissonSampler, p: *const PxVec3) -> bool;

    pub fn PxTriangleMeshPoissonSampler_delete(self_: *mut PxTriangleMeshPoissonSampler);

    /// Creates a triangle mesh sampler
    ///
    /// Returns the sampler
    pub fn phys_PxCreateTriangleMeshSampler(triangles: *const u32, numTriangles: u32, vertices: *const PxVec3, numVertices: u32, initialSamplingRadius: f32, numSampleAttemptsAroundPoint: i32) -> *mut PxTriangleMeshPoissonSampler;

    /// Returns the index of the tetrahedron that contains a point
    ///
    /// The index of the tetrahedon containing the point, -1 if not tetrahedron contains the opoint
    pub fn PxTetrahedronMeshExt_findTetrahedronContainingPoint(mesh: *const PxTetrahedronMesh, point: *const PxVec3, bary: *mut PxVec4, tolerance: f32) -> i32;

    /// Returns the index of the tetrahedron closest to a point
    ///
    /// The index of the tetrahedon closest to the point
    pub fn PxTetrahedronMeshExt_findTetrahedronClosestToPoint(mesh: *const PxTetrahedronMesh, point: *const PxVec3, bary: *mut PxVec4) -> i32;

    /// Initialize the PhysXExtensions library.
    ///
    /// This should be called before calling any functions or methods in extensions which may require allocation.
    ///
    /// This function does not need to be called before creating a PxDefaultAllocator object.
    pub fn phys_PxInitExtensions(physics: *mut PxPhysics, pvd: *mut PxPvd) -> bool;

    /// Shut down the PhysXExtensions library.
    ///
    /// This function should be called to cleanly shut down the PhysXExtensions library before application exit.
    ///
    /// This function is required to be called to release foundation usage.
    pub fn phys_PxCloseExtensions();

    pub fn PxRepXObject_new(inTypeName: *const std::ffi::c_char, inSerializable: *const std::ffi::c_void, inId: u64) -> PxRepXObject;

    pub fn PxRepXObject_isValid(self_: *const PxRepXObject) -> bool;

    pub fn PxRepXInstantiationArgs_new(inPhysics: *mut PxPhysics, inCooking: *mut PxCooking, inStringTable: *mut PxStringTable) -> PxRepXInstantiationArgs;

    /// The type this Serializer is meant to operate on.
    pub fn PxRepXSerializer_getTypeName_mut(self_: *mut PxRepXSerializer) -> *const std::ffi::c_char;

    /// Convert from a RepX object to a key-value pair hierarchy
    pub fn PxRepXSerializer_objectToFile_mut(self_: *mut PxRepXSerializer, inLiveObject: *const PxRepXObject, inCollection: *mut PxCollection, inWriter: *mut XmlWriter, inTempBuffer: *mut MemoryBuffer, inArgs: *mut PxRepXInstantiationArgs);

    /// Convert from a descriptor to a live object.  Must be an object of this Serializer type.
    ///
    /// The new live object.  It can be an invalid object if the instantiation cannot take place.
    pub fn PxRepXSerializer_fileToObject_mut(self_: *mut PxRepXSerializer, inReader: *mut XmlReader, inAllocator: *mut XmlMemoryAllocator, inArgs: *mut PxRepXInstantiationArgs, inCollection: *mut PxCollection) -> PxRepXObject;

    /// Connects the SDK to the PhysX Visual Debugger application.
    pub fn PxPvd_connect_mut(self_: *mut PxPvd, transport: *mut PxPvdTransport, flags: PxPvdInstrumentationFlags) -> bool;

    /// Disconnects the SDK from the PhysX Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    pub fn PxPvd_disconnect_mut(self_: *mut PxPvd);

    /// Return if connection to PVD is created.
    pub fn PxPvd_isConnected_mut(self_: *mut PxPvd, useCachedStatus: bool) -> bool;

    /// returns the PVD data transport
    /// returns NULL if no transport is present.
    pub fn PxPvd_getTransport_mut(self_: *mut PxPvd) -> *mut PxPvdTransport;

    /// Retrieves the PVD flags. See PxPvdInstrumentationFlags.
    pub fn PxPvd_getInstrumentationFlags_mut(self_: *mut PxPvd) -> PxPvdInstrumentationFlags;

    /// Releases the pvd instance.
    pub fn PxPvd_release_mut(self_: *mut PxPvd);

    /// Create a pvd instance.
    pub fn phys_PxCreatePvd(foundation: *mut PxFoundation) -> *mut PxPvd;

    /// Connects to the Visual Debugger application.
    /// return True if success
    pub fn PxPvdTransport_connect_mut(self_: *mut PxPvdTransport) -> bool;

    /// Disconnects from the Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
    pub fn PxPvdTransport_disconnect_mut(self_: *mut PxPvdTransport);

    /// Return if connection to PVD is created.
    pub fn PxPvdTransport_isConnected_mut(self_: *mut PxPvdTransport) -> bool;

    /// write bytes to the other endpoint of the connection. should lock before witre. If an error occurs
    /// this connection will assume to be dead.
    pub fn PxPvdTransport_write_mut(self_: *mut PxPvdTransport, inBytes: *const u8, inLength: u32) -> bool;

    pub fn PxPvdTransport_lock_mut(self_: *mut PxPvdTransport) -> *mut PxPvdTransport;

    pub fn PxPvdTransport_unlock_mut(self_: *mut PxPvdTransport);

    /// send any data and block until we know it is at least on the wire.
    pub fn PxPvdTransport_flush_mut(self_: *mut PxPvdTransport);

    /// Return size of written data.
    pub fn PxPvdTransport_getWrittenDataSize_mut(self_: *mut PxPvdTransport) -> u64;

    pub fn PxPvdTransport_release_mut(self_: *mut PxPvdTransport);

    /// Create a default socket transport.
    pub fn phys_PxDefaultPvdSocketTransportCreate(host: *const std::ffi::c_char, port: i32, timeoutInMilliseconds: u32) -> *mut PxPvdTransport;

    /// Create a default file transport.
    pub fn phys_PxDefaultPvdFileTransportCreate(name: *const std::ffi::c_char) -> *mut PxPvdTransport;

}
