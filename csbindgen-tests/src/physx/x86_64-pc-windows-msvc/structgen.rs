#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxAllocator {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRawAllocator {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxVirtualAllocator {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxUserAllocated {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTempAllocator {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBitAndByte {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBitMap {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxVec3Padded {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub padding: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxQuat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTransform {
    pub q: PxQuat,
    pub p: PxVec3,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTransformPadded {
    pub transform: PxTransform,
    pub padding: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMat33 {
    pub column0: PxVec3,
    pub column1: PxVec3,
    pub column2: PxVec3,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBounds3 {
    pub minimum: PxVec3,
    pub maximum: PxVec3,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadcastingAllocator {
    pub structgen_pad0: [u8; 176],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadcastingErrorCallback {
    pub structgen_pad0: [u8; 160],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxVec4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMat44 {
    pub column0: PxVec4,
    pub column1: PxVec4,
    pub column2: PxVec4,
    pub column3: PxVec4,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPlane {
    pub n: PxVec3,
    pub d: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct Interpolation {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMutexImpl {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxReadWriteLock {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxProfileScoped {
    pub mCallback: *mut PxProfilerCallback,
    pub mEventName: *const std::ffi::c_char,
    pub mProfilerData: *mut std::ffi::c_void,
    pub mContextId: u64,
    pub mDetached: bool,
    pub structgen_pad0: [u8; 7],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSListEntry {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSListImpl {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSyncImpl {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCounterFrequencyToTensOfNanos {
    pub mNumerator: u64,
    pub mDenominator: u64,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTime {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxVec2 {
    pub x: f32,
    pub y: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxStridedData {
    pub stride: u32,
    pub structgen_pad0: [u8; 4],
    pub data: *const std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBoundedData {
    pub stride: u32,
    pub structgen_pad0: [u8; 4],
    pub data: *const std::ffi::c_void,
    pub count: u32,
    pub structgen_pad1: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDebugPoint {
    pub pos: PxVec3,
    pub color: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDebugLine {
    pub pos0: PxVec3,
    pub color0: u32,
    pub pos1: PxVec3,
    pub color1: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDebugTriangle {
    pub pos0: PxVec3,
    pub color0: u32,
    pub pos1: PxVec3,
    pub color1: u32,
    pub pos2: PxVec3,
    pub color2: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDebugText {
    pub position: PxVec3,
    pub size: f32,
    pub color: u32,
    pub structgen_pad0: [u8; 4],
    pub string: *const std::ffi::c_char,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDeserializationContext {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBase {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRefCounted {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTolerancesScale {
    pub length: f32,
    pub speed: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMetaDataEntry {
    pub type_: *const std::ffi::c_char,
    pub name: *const std::ffi::c_char,
    pub offset: u32,
    pub size: u32,
    pub count: u32,
    pub offsetSize: u32,
    pub flags: u32,
    pub alignment: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBaseTask {
    pub structgen_pad0: [u8; 24],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTask {
    pub structgen_pad0: [u8; 32],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxLightCpuTask {
    pub structgen_pad0: [u8; 40],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBoxGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub halfExtents: PxVec3,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVH {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCapsuleGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub radius: f32,
    pub halfHeight: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxHullPolygon {
    pub mPlane: [f32; 4],
    pub mNbVerts: u16,
    pub mIndexBase: u16,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConvexMesh {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMeshScale {
    pub scale: PxVec3,
    pub rotation: PxQuat,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConvexMeshGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub scale: PxMeshScale,
    pub structgen_pad1: [u8; 4],
    pub convexMesh: *mut PxConvexMesh,
    pub meshFlags: PxConvexMeshGeometryFlags,
    pub structgen_pad2: [u8; 7],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSphereGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub radius: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPlaneGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTriangleMeshGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub scale: PxMeshScale,
    pub meshFlags: PxMeshGeometryFlags,
    pub structgen_pad1: [u8; 3],
    pub triangleMesh: *mut PxTriangleMesh,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxHeightFieldGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub heightField: *mut PxHeightField,
    pub heightScale: f32,
    pub rowScale: f32,
    pub columnScale: f32,
    pub heightFieldFlags: PxMeshGeometryFlags,
    pub structgen_pad1: [u8; 3],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxParticleSystemGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub mSolverType: PxParticleSolverType,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxHairSystemGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTetrahedronMeshGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub tetrahedronMesh: *mut PxTetrahedronMesh,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxQueryHit {
    pub faceIndex: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxLocationHit {
    pub faceIndex: u32,
    pub flags: PxHitFlags,
    pub structgen_pad0: [u8; 2],
    pub position: PxVec3,
    pub normal: PxVec3,
    pub distance: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeomRaycastHit {
    pub faceIndex: u32,
    pub flags: PxHitFlags,
    pub structgen_pad0: [u8; 2],
    pub position: PxVec3,
    pub normal: PxVec3,
    pub distance: f32,
    pub u: f32,
    pub v: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeomOverlapHit {
    pub faceIndex: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeomSweepHit {
    pub faceIndex: u32,
    pub flags: PxHitFlags,
    pub structgen_pad0: [u8; 2],
    pub position: PxVec3,
    pub normal: PxVec3,
    pub distance: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeomIndexPair {
    pub id0: u32,
    pub id1: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxQueryThreadContext {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCustomGeometryType {
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCustomGeometry {
    pub structgen_pad0: [u8; 4],
    pub mTypePadding: f32,
    pub callbacks: *mut PxCustomGeometryCallbacks,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeometryHolder {
    pub structgen_pad0: [u8; 56],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGeometryQuery {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxHeightFieldSample {
    pub height: i16,
    pub materialIndex0: PxBitAndByte,
    pub materialIndex1: PxBitAndByte,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxHeightField {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxHeightFieldDesc {
    pub nbRows: u32,
    pub nbColumns: u32,
    pub format: PxHeightFieldFormat,
    pub structgen_pad0: [u8; 4],
    pub samples: PxStridedData,
    pub convexEdgeThreshold: f32,
    pub flags: PxHeightFieldFlags,
    pub structgen_pad1: [u8; 2],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMeshQuery {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSimpleTriangleMesh {
    pub points: PxBoundedData,
    pub triangles: PxBoundedData,
    pub flags: PxMeshFlags,
    pub structgen_pad0: [u8; 6],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTriangle {
    pub verts: [PxVec3; 3],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTrianglePadded {
    pub verts: [PxVec3; 3],
    pub padding: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTriangleMesh {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVH34TriangleMesh {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTetrahedron {
    pub verts: [PxVec3; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSoftBodyAuxData {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTetrahedronMesh {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSoftBodyMesh {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCollisionMeshMappingData {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSoftBodyCollisionData {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTetrahedronMeshData {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSoftBodySimulationData {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCollisionTetrahedronMeshData {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSimulationTetrahedronMeshData {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxActor {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxAggregate {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSpringModifiers {
    pub stiffness: f32,
    pub damping: f32,
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRestitutionModifiers {
    pub restitution: f32,
    pub velocityThreshold: f32,
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct Px1DConstraint {
    pub linear0: PxVec3,
    pub geometricError: f32,
    pub angular0: PxVec3,
    pub velocityTarget: f32,
    pub linear1: PxVec3,
    pub minImpulse: f32,
    pub angular1: PxVec3,
    pub maxImpulse: f32,
    pub mods: Px1DConstraintMods,
    pub forInternalUse: f32,
    pub flags: u16,
    pub solveHint: u16,
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintInvMassScale {
    pub linear0: f32,
    pub angular0: f32,
    pub linear1: f32,
    pub angular1: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPoint {
    pub normal: PxVec3,
    pub separation: f32,
    pub point: PxVec3,
    pub maxImpulse: f32,
    pub targetVel: PxVec3,
    pub staticFriction: f32,
    pub materialFlags: u8,
    pub structgen_pad0: [u8; 3],
    pub internalFaceIndex1: u32,
    pub dynamicFriction: f32,
    pub restitution: f32,
    pub damping: f32,
    pub structgen_pad1: [u8; 12],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSolverBody {
    pub linearVelocity: PxVec3,
    pub maxSolverNormalProgress: u16,
    pub maxSolverFrictionProgress: u16,
    pub angularState: PxVec3,
    pub solverProgress: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSolverBodyData {
    pub linearVelocity: PxVec3,
    pub invMass: f32,
    pub angularVelocity: PxVec3,
    pub reportThreshold: f32,
    pub sqrtInvInertia: PxMat33,
    pub penBiasClamp: f32,
    pub nodeIndex: u32,
    pub maxContactImpulse: f32,
    pub body2World: PxTransform,
    pub pad: u16,
    pub structgen_pad0: [u8; 2],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintBatchHeader {
    pub startIndex: u32,
    pub stride: u16,
    pub constraintType: u16,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSolverConstraintDesc {
    pub structgen_pad0: [u8; 16],
    pub bodyADataIndex: u32,
    pub bodyBDataIndex: u32,
    pub linkIndexA: u32,
    pub linkIndexB: u32,
    pub constraint: *mut u8,
    pub writeBack: *mut std::ffi::c_void,
    pub progressA: u16,
    pub progressB: u16,
    pub constraintLengthOver16: u16,
    pub padding: [u8; 10],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSolverConstraintPrepDescBase {
    pub invMassScales: PxConstraintInvMassScale,
    pub desc: *mut PxSolverConstraintDesc,
    pub body0: *const PxSolverBody,
    pub body1: *const PxSolverBody,
    pub data0: *const PxSolverBodyData,
    pub data1: *const PxSolverBodyData,
    pub bodyFrame0: PxTransform,
    pub bodyFrame1: PxTransform,
    pub bodyState0: BodyState,
    pub bodyState1: BodyState,
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSolverConstraintPrepDesc {
    pub invMassScales: PxConstraintInvMassScale,
    pub desc: *mut PxSolverConstraintDesc,
    pub body0: *const PxSolverBody,
    pub body1: *const PxSolverBody,
    pub data0: *const PxSolverBodyData,
    pub data1: *const PxSolverBodyData,
    pub bodyFrame0: PxTransform,
    pub bodyFrame1: PxTransform,
    pub bodyState0: BodyState,
    pub bodyState1: BodyState,
    pub structgen_pad0: [u8; 8],
    pub rows: *mut Px1DConstraint,
    pub numRows: u32,
    pub linBreakForce: f32,
    pub angBreakForce: f32,
    pub minResponseThreshold: f32,
    pub writeback: *mut std::ffi::c_void,
    pub disablePreprocessing: bool,
    pub improvedSlerp: bool,
    pub driveLimitsAreForces: bool,
    pub extendedLimits: bool,
    pub disableConstraint: bool,
    pub structgen_pad1: [u8; 3],
    pub body0WorldOffset: PxVec3Padded,
    pub structgen_pad2: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSolverContactDesc {
    pub invMassScales: PxConstraintInvMassScale,
    pub desc: *mut PxSolverConstraintDesc,
    pub body0: *const PxSolverBody,
    pub body1: *const PxSolverBody,
    pub data0: *const PxSolverBodyData,
    pub data1: *const PxSolverBodyData,
    pub bodyFrame0: PxTransform,
    pub bodyFrame1: PxTransform,
    pub bodyState0: BodyState,
    pub bodyState1: BodyState,
    pub structgen_pad0: [u8; 8],
    pub shapeInteraction: *mut std::ffi::c_void,
    pub contacts: *mut PxContactPoint,
    pub numContacts: u32,
    pub hasMaxImpulse: bool,
    pub disableStrongFriction: bool,
    pub hasForceThresholds: bool,
    pub structgen_pad1: [u8; 1],
    pub restDistance: f32,
    pub maxCCDSeparation: f32,
    pub frictionPtr: *mut u8,
    pub frictionCount: u8,
    pub structgen_pad2: [u8; 7],
    pub contactForces: *mut f32,
    pub startFrictionPatchIndex: u32,
    pub numFrictionPatches: u32,
    pub startContactPatchIndex: u32,
    pub numContactPatches: u16,
    pub axisConstraintCount: u16,
    pub offsetSlop: f32,
    pub structgen_pad3: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationLimit {
    pub low: f32,
    pub high: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationDrive {
    pub stiffness: f32,
    pub damping: f32,
    pub maxForce: f32,
    pub driveType: PxArticulationDriveType,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTGSSolverBodyVel {
    pub linearVelocity: PxVec3,
    pub nbStaticInteractions: u16,
    pub maxDynamicPartition: u16,
    pub angularVelocity: PxVec3,
    pub partitionMask: u32,
    pub deltaAngDt: PxVec3,
    pub maxAngVel: f32,
    pub deltaLinDt: PxVec3,
    pub lockFlags: u16,
    pub isKinematic: bool,
    pub pad: u8,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTGSSolverBodyTxInertia {
    pub deltaBody2World: PxTransform,
    pub sqrtInvInertia: PxMat33,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTGSSolverBodyData {
    pub originalLinearVelocity: PxVec3,
    pub maxContactImpulse: f32,
    pub originalAngularVelocity: PxVec3,
    pub penBiasClamp: f32,
    pub invMass: f32,
    pub nodeIndex: u32,
    pub reportThreshold: f32,
    pub pad: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTGSSolverConstraintPrepDescBase {
    pub invMassScales: PxConstraintInvMassScale,
    pub desc: *mut PxSolverConstraintDesc,
    pub body0: *const PxTGSSolverBodyVel,
    pub body1: *const PxTGSSolverBodyVel,
    pub body0TxI: *const PxTGSSolverBodyTxInertia,
    pub body1TxI: *const PxTGSSolverBodyTxInertia,
    pub bodyData0: *const PxTGSSolverBodyData,
    pub bodyData1: *const PxTGSSolverBodyData,
    pub bodyFrame0: PxTransform,
    pub bodyFrame1: PxTransform,
    pub bodyState0: BodyState,
    pub bodyState1: BodyState,
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTGSSolverConstraintPrepDesc {
    pub invMassScales: PxConstraintInvMassScale,
    pub desc: *mut PxSolverConstraintDesc,
    pub body0: *const PxTGSSolverBodyVel,
    pub body1: *const PxTGSSolverBodyVel,
    pub body0TxI: *const PxTGSSolverBodyTxInertia,
    pub body1TxI: *const PxTGSSolverBodyTxInertia,
    pub bodyData0: *const PxTGSSolverBodyData,
    pub bodyData1: *const PxTGSSolverBodyData,
    pub bodyFrame0: PxTransform,
    pub bodyFrame1: PxTransform,
    pub bodyState0: BodyState,
    pub bodyState1: BodyState,
    pub structgen_pad0: [u8; 8],
    pub rows: *mut Px1DConstraint,
    pub numRows: u32,
    pub linBreakForce: f32,
    pub angBreakForce: f32,
    pub minResponseThreshold: f32,
    pub writeback: *mut std::ffi::c_void,
    pub disablePreprocessing: bool,
    pub improvedSlerp: bool,
    pub driveLimitsAreForces: bool,
    pub extendedLimits: bool,
    pub disableConstraint: bool,
    pub structgen_pad1: [u8; 3],
    pub body0WorldOffset: PxVec3Padded,
    pub cA2w: PxVec3Padded,
    pub cB2w: PxVec3Padded,
    pub structgen_pad2: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTGSSolverContactDesc {
    pub invMassScales: PxConstraintInvMassScale,
    pub desc: *mut PxSolverConstraintDesc,
    pub body0: *const PxTGSSolverBodyVel,
    pub body1: *const PxTGSSolverBodyVel,
    pub body0TxI: *const PxTGSSolverBodyTxInertia,
    pub body1TxI: *const PxTGSSolverBodyTxInertia,
    pub bodyData0: *const PxTGSSolverBodyData,
    pub bodyData1: *const PxTGSSolverBodyData,
    pub bodyFrame0: PxTransform,
    pub bodyFrame1: PxTransform,
    pub bodyState0: BodyState,
    pub bodyState1: BodyState,
    pub structgen_pad0: [u8; 8],
    pub shapeInteraction: *mut std::ffi::c_void,
    pub contacts: *mut PxContactPoint,
    pub numContacts: u32,
    pub hasMaxImpulse: bool,
    pub disableStrongFriction: bool,
    pub hasForceThresholds: bool,
    pub structgen_pad1: [u8; 1],
    pub restDistance: f32,
    pub maxCCDSeparation: f32,
    pub frictionPtr: *mut u8,
    pub frictionCount: u8,
    pub structgen_pad2: [u8; 7],
    pub contactForces: *mut f32,
    pub startFrictionPatchIndex: u32,
    pub numFrictionPatches: u32,
    pub startContactPatchIndex: u32,
    pub numContactPatches: u16,
    pub axisConstraintCount: u16,
    pub maxImpulse: f32,
    pub torsionalPatchRadius: f32,
    pub minTorsionalPatchRadius: f32,
    pub offsetSlop: f32,
    pub structgen_pad3: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationTendonLimit {
    pub lowLimit: f32,
    pub highLimit: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationAttachment {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationTendonJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationTendon {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationSpatialTendon {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationFixedTendon {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSpatialForce {
    pub force: PxVec3,
    pub pad0: f32,
    pub torque: PxVec3,
    pub pad1: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSpatialVelocity {
    pub linear: PxVec3,
    pub pad0: f32,
    pub angular: PxVec3,
    pub pad1: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationRootLinkData {
    pub transform: PxTransform,
    pub worldLinVel: PxVec3,
    pub worldAngVel: PxVec3,
    pub worldLinAccel: PxVec3,
    pub worldAngAccel: PxVec3,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationCache {
    pub externalForces: *mut PxSpatialForce,
    pub denseJacobian: *mut f32,
    pub massMatrix: *mut f32,
    pub jointVelocity: *mut f32,
    pub jointAcceleration: *mut f32,
    pub jointPosition: *mut f32,
    pub jointForce: *mut f32,
    pub jointSolverForces: *mut f32,
    pub linkVelocity: *mut PxSpatialVelocity,
    pub linkAcceleration: *mut PxSpatialVelocity,
    pub rootLinkData: *mut PxArticulationRootLinkData,
    pub sensorForces: *mut PxSpatialForce,
    pub coefficientMatrix: *mut f32,
    pub lambda: *mut f32,
    pub scratchMemory: *mut std::ffi::c_void,
    pub scratchAllocator: *mut std::ffi::c_void,
    pub version: u32,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationSensor {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationReducedCoordinate {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationJointReducedCoordinate {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxShape {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRigidActor {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxNodeIndex {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRigidBody {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxArticulationLink {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConeLimitedConstraint {
    pub mAxis: PxVec3,
    pub mAngle: f32,
    pub mLowLimit: f32,
    pub mHighLimit: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConeLimitParams {
    pub lowHighLimits: PxVec4,
    pub axisAngle: PxVec4,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintShaderTable {
    pub solverPrep: *mut std::ffi::c_void,
    pub structgen_pad0: [u8; 8],
    pub visualize: *mut std::ffi::c_void,
    pub flag: PxConstraintFlag,
    pub structgen_pad1: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMassModificationProps {
    pub mInvMassScale0: f32,
    pub mInvInertiaScale0: f32,
    pub mInvMassScale1: f32,
    pub mInvInertiaScale1: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPatch {
    pub mMassModification: PxMassModificationProps,
    pub normal: PxVec3,
    pub restitution: f32,
    pub dynamicFriction: f32,
    pub staticFriction: f32,
    pub damping: f32,
    pub startContactIndex: u16,
    pub nbContacts: u8,
    pub materialFlags: u8,
    pub internalFlags: u16,
    pub materialIndex0: u16,
    pub materialIndex1: u16,
    pub pad: [u16; 5],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContact {
    pub contact: PxVec3,
    pub separation: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxExtendedContact {
    pub contact: PxVec3,
    pub separation: f32,
    pub targetVelocity: PxVec3,
    pub maxImpulse: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxModifiableContact {
    pub contact: PxVec3,
    pub separation: f32,
    pub targetVelocity: PxVec3,
    pub maxImpulse: f32,
    pub normal: PxVec3,
    pub restitution: f32,
    pub materialFlags: u32,
    pub materialIndex0: u16,
    pub materialIndex1: u16,
    pub staticFriction: f32,
    pub dynamicFriction: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactStreamIterator {
    pub zero: PxVec3,
    pub structgen_pad0: [u8; 4],
    pub patch: *const PxContactPatch,
    pub contact: *const PxContact,
    pub faceIndice: *const u32,
    pub totalPatches: u32,
    pub totalContacts: u32,
    pub nextContactIndex: u32,
    pub nextPatchIndex: u32,
    pub contactPatchHeaderSize: u32,
    pub contactPointSize: u32,
    pub mStreamFormat: StreamFormat,
    pub forceNoResponse: u32,
    pub pointStepped: bool,
    pub structgen_pad1: [u8; 3],
    pub hasFaceIndices: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGpuContactPair {
    pub contactPatches: *mut u8,
    pub contactPoints: *mut u8,
    pub contactForces: *mut f32,
    pub transformCacheRef0: u32,
    pub transformCacheRef1: u32,
    pub nodeIndex0: PxNodeIndex,
    pub nodeIndex1: PxNodeIndex,
    pub actor0: *mut PxActor,
    pub actor1: *mut PxActor,
    pub nbContacts: u16,
    pub nbPatches: u16,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactSet {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactModifyPair {
    pub actor: [*const PxRigidActor; 2],
    pub shape: [*const PxShape; 2],
    pub transform: [PxTransform; 2],
    pub contacts: PxContactSet,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBaseMaterial {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxFEMMaterial {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxFilterData {
    pub word0: u32,
    pub word1: u32,
    pub word2: u32,
    pub word3: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxParticleRigidFilterPair {
    pub mID0: u64,
    pub mID1: u64,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMaterial {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGpuParticleBufferIndexPair {
    pub systemIndex: u32,
    pub bufferIndex: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxParticleVolume {
    pub bound: PxBounds3,
    pub particleIndicesOffset: u32,
    pub numParticles: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDiffuseParticleParams {
    pub threshold: f32,
    pub lifetime: f32,
    pub airDrag: f32,
    pub bubbleDrag: f32,
    pub buoyancy: f32,
    pub kineticEnergyWeight: f32,
    pub pressureWeight: f32,
    pub divergenceWeight: f32,
    pub collisionDecay: f32,
    pub useAccurateVelocity: bool,
    pub structgen_pad0: [u8; 3],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxParticleSpring {
    pub ind0: u32,
    pub ind1: u32,
    pub length: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub pad: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxParticleMaterial {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxActorShape {
    pub actor: *mut PxRigidActor,
    pub shape: *mut PxShape,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRaycastHit {
    pub faceIndex: u32,
    pub flags: PxHitFlags,
    pub structgen_pad0: [u8; 2],
    pub position: PxVec3,
    pub normal: PxVec3,
    pub distance: f32,
    pub u: f32,
    pub v: f32,
    pub structgen_pad1: [u8; 4],
    pub actor: *mut PxRigidActor,
    pub shape: *mut PxShape,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxOverlapHit {
    pub faceIndex: u32,
    pub structgen_pad0: [u8; 4],
    pub actor: *mut PxRigidActor,
    pub shape: *mut PxShape,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSweepHit {
    pub faceIndex: u32,
    pub flags: PxHitFlags,
    pub structgen_pad0: [u8; 2],
    pub position: PxVec3,
    pub normal: PxVec3,
    pub distance: f32,
    pub structgen_pad1: [u8; 4],
    pub actor: *mut PxRigidActor,
    pub shape: *mut PxShape,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRaycastCallback {
    pub structgen_pad0: [u8; 8],
    pub block: PxRaycastHit,
    pub hasBlock: bool,
    pub structgen_pad1: [u8; 7],
    pub touches: *mut PxRaycastHit,
    pub maxNbTouches: u32,
    pub nbTouches: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxOverlapCallback {
    pub structgen_pad0: [u8; 8],
    pub block: PxOverlapHit,
    pub hasBlock: bool,
    pub structgen_pad1: [u8; 7],
    pub touches: *mut PxOverlapHit,
    pub maxNbTouches: u32,
    pub nbTouches: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSweepCallback {
    pub structgen_pad0: [u8; 8],
    pub block: PxSweepHit,
    pub hasBlock: bool,
    pub structgen_pad1: [u8; 7],
    pub touches: *mut PxSweepHit,
    pub maxNbTouches: u32,
    pub nbTouches: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRaycastBuffer {
    pub structgen_pad0: [u8; 8],
    pub block: PxRaycastHit,
    pub hasBlock: bool,
    pub structgen_pad1: [u8; 7],
    pub touches: *mut PxRaycastHit,
    pub maxNbTouches: u32,
    pub nbTouches: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxOverlapBuffer {
    pub structgen_pad0: [u8; 8],
    pub block: PxOverlapHit,
    pub hasBlock: bool,
    pub structgen_pad1: [u8; 7],
    pub touches: *mut PxOverlapHit,
    pub maxNbTouches: u32,
    pub nbTouches: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSweepBuffer {
    pub structgen_pad0: [u8; 8],
    pub block: PxSweepHit,
    pub hasBlock: bool,
    pub structgen_pad1: [u8; 7],
    pub touches: *mut PxSweepHit,
    pub maxNbTouches: u32,
    pub nbTouches: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxQueryCache {
    pub shape: *mut PxShape,
    pub actor: *mut PxRigidActor,
    pub faceIndex: u32,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxQueryFilterData {
    pub data: PxFilterData,
    pub flags: PxQueryFlags,
    pub structgen_pad0: [u8; 2],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRigidDynamic {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRigidStatic {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneQueryDesc {
    pub staticStructure: PxPruningStructureType,
    pub dynamicStructure: PxPruningStructureType,
    pub dynamicTreeRebuildRateHint: u32,
    pub dynamicTreeSecondaryPruner: PxDynamicTreeSecondaryPruner,
    pub staticBVHBuildStrategy: PxBVHBuildStrategy,
    pub dynamicBVHBuildStrategy: PxBVHBuildStrategy,
    pub staticNbObjectsPerNode: u32,
    pub dynamicNbObjectsPerNode: u32,
    pub sceneQueryUpdateMode: PxSceneQueryUpdateMode,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseRegion {
    pub mBounds: PxBounds3,
    pub mUserData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseRegionInfo {
    pub mRegion: PxBroadPhaseRegion,
    pub mNbStaticObjects: u32,
    pub mNbDynamicObjects: u32,
    pub mActive: bool,
    pub mOverlap: bool,
    pub structgen_pad0: [u8; 6],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseCaps {
    pub mMaxNbRegions: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseDesc {
    pub mType: PxBroadPhaseType,
    pub structgen_pad0: [u8; 4],
    pub mContextID: u64,
    pub structgen_pad1: [u8; 8],
    pub mFoundLostPairsCapacity: u32,
    pub mDiscardStaticVsKinematic: bool,
    pub mDiscardKinematicVsKinematic: bool,
    pub structgen_pad2: [u8; 2],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseUpdateData {
    pub mCreated: *const u32,
    pub mNbCreated: u32,
    pub structgen_pad0: [u8; 4],
    pub mUpdated: *const u32,
    pub mNbUpdated: u32,
    pub structgen_pad1: [u8; 4],
    pub mRemoved: *const u32,
    pub mNbRemoved: u32,
    pub structgen_pad2: [u8; 4],
    pub mBounds: *const PxBounds3,
    pub mGroups: *const u32,
    pub mDistances: *const f32,
    pub mCapacity: u32,
    pub structgen_pad3: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhasePair {
    pub mID0: u32,
    pub mID1: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseResults {
    pub mNbCreatedPairs: u32,
    pub structgen_pad0: [u8; 4],
    pub mCreatedPairs: *const PxBroadPhasePair,
    pub mNbDeletedPairs: u32,
    pub structgen_pad1: [u8; 4],
    pub mDeletedPairs: *const PxBroadPhasePair,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneLimits {
    pub maxNbActors: u32,
    pub maxNbBodies: u32,
    pub maxNbStaticShapes: u32,
    pub maxNbDynamicShapes: u32,
    pub maxNbAggregates: u32,
    pub maxNbConstraints: u32,
    pub maxNbRegions: u32,
    pub maxNbBroadPhaseOverlaps: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxgDynamicsMemoryConfig {
    pub tempBufferCapacity: u32,
    pub maxRigidContactCount: u32,
    pub maxRigidPatchCount: u32,
    pub heapCapacity: u32,
    pub foundLostPairsCapacity: u32,
    pub foundLostAggregatePairsCapacity: u32,
    pub totalAggregatePairsCapacity: u32,
    pub maxSoftBodyContacts: u32,
    pub maxFemClothContacts: u32,
    pub maxParticleContacts: u32,
    pub collisionStackSize: u32,
    pub maxHairContacts: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneDesc {
    pub staticStructure: PxPruningStructureType,
    pub dynamicStructure: PxPruningStructureType,
    pub dynamicTreeRebuildRateHint: u32,
    pub dynamicTreeSecondaryPruner: PxDynamicTreeSecondaryPruner,
    pub staticBVHBuildStrategy: PxBVHBuildStrategy,
    pub dynamicBVHBuildStrategy: PxBVHBuildStrategy,
    pub staticNbObjectsPerNode: u32,
    pub dynamicNbObjectsPerNode: u32,
    pub sceneQueryUpdateMode: PxSceneQueryUpdateMode,
    pub gravity: PxVec3,
    pub simulationEventCallback: *mut PxSimulationEventCallback,
    pub contactModifyCallback: *mut PxContactModifyCallback,
    pub ccdContactModifyCallback: *mut PxCCDContactModifyCallback,
    pub filterShaderData: *const std::ffi::c_void,
    pub filterShaderDataSize: u32,
    pub structgen_pad0: [u8; 4],
    pub filterShader: *mut std::ffi::c_void,
    pub filterCallback: *mut PxSimulationFilterCallback,
    pub kineKineFilteringMode: PxPairFilteringMode,
    pub staticKineFilteringMode: PxPairFilteringMode,
    pub broadPhaseType: PxBroadPhaseType,
    pub structgen_pad1: [u8; 4],
    pub broadPhaseCallback: *mut PxBroadPhaseCallback,
    pub limits: PxSceneLimits,
    pub frictionType: PxFrictionType,
    pub solverType: PxSolverType,
    pub bounceThresholdVelocity: f32,
    pub frictionOffsetThreshold: f32,
    pub frictionCorrelationDistance: f32,
    pub flags: PxSceneFlags,
    pub cpuDispatcher: *mut PxCpuDispatcher,
    pub structgen_pad2: [u8; 8],
    pub userData: *mut std::ffi::c_void,
    pub solverBatchSize: u32,
    pub solverArticulationBatchSize: u32,
    pub nbContactDataBlocks: u32,
    pub maxNbContactDataBlocks: u32,
    pub maxBiasCoefficient: f32,
    pub contactReportStreamBufferSize: u32,
    pub ccdMaxPasses: u32,
    pub ccdThreshold: f32,
    pub ccdMaxSeparation: f32,
    pub wakeCounterResetValue: f32,
    pub sanityBounds: PxBounds3,
    pub gpuDynamicsConfig: PxgDynamicsMemoryConfig,
    pub gpuMaxNumPartitions: u32,
    pub gpuMaxNumStaticPartitions: u32,
    pub gpuComputeVersion: u32,
    pub contactPairSlabSize: u32,
    pub sceneQuerySystem: *mut PxSceneQuerySystem,
    pub structgen_pad3: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSimulationStatistics {
    pub nbActiveConstraints: u32,
    pub nbActiveDynamicBodies: u32,
    pub nbActiveKinematicBodies: u32,
    pub nbStaticBodies: u32,
    pub nbDynamicBodies: u32,
    pub nbKinematicBodies: u32,
    pub nbShapes: [u32; 11],
    pub nbAggregates: u32,
    pub nbArticulations: u32,
    pub nbAxisSolverConstraints: u32,
    pub compressedContactSize: u32,
    pub requiredContactConstraintMemory: u32,
    pub peakConstraintMemory: u32,
    pub nbDiscreteContactPairsTotal: u32,
    pub nbDiscreteContactPairsWithCacheHits: u32,
    pub nbDiscreteContactPairsWithContacts: u32,
    pub nbNewPairs: u32,
    pub nbLostPairs: u32,
    pub nbNewTouches: u32,
    pub nbLostTouches: u32,
    pub nbPartitions: u32,
    pub structgen_pad0: [u8; 4],
    pub gpuMemParticles: u64,
    pub gpuMemSoftBodies: u64,
    pub gpuMemFEMCloths: u64,
    pub gpuMemHairSystems: u64,
    pub gpuMemHeap: u64,
    pub gpuMemHeapBroadPhase: u64,
    pub gpuMemHeapNarrowPhase: u64,
    pub gpuMemHeapSolver: u64,
    pub gpuMemHeapArticulation: u64,
    pub gpuMemHeapSimulation: u64,
    pub gpuMemHeapSimulationArticulation: u64,
    pub gpuMemHeapSimulationParticles: u64,
    pub gpuMemHeapSimulationSoftBody: u64,
    pub gpuMemHeapSimulationFEMCloth: u64,
    pub gpuMemHeapSimulationHairSystem: u64,
    pub gpuMemHeapParticles: u64,
    pub gpuMemHeapSoftBodies: u64,
    pub gpuMemHeapFEMCloths: u64,
    pub gpuMemHeapHairSystems: u64,
    pub gpuMemHeapOther: u64,
    pub nbBroadPhaseAdds: u32,
    pub nbBroadPhaseRemoves: u32,
    pub nbDiscreteContactPairs: [[u32; 11]; 11],
    pub nbCCDPairs: [[u32; 11]; 11],
    pub nbModifiedContactPairs: [[u32; 11]; 11],
    pub nbTriggerPairs: [[u32; 11]; 11],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGpuBodyData {
    pub quat: PxQuat,
    pub pos: PxVec4,
    pub linVel: PxVec4,
    pub angVel: PxVec4,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGpuActorPair {
    pub srcIndex: u32,
    pub structgen_pad0: [u8; 4],
    pub nodeIndex: PxNodeIndex,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxIndexDataPair {
    pub index: u32,
    pub structgen_pad0: [u8; 4],
    pub data: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDominanceGroupPair {
    pub dominance0: u8,
    pub dominance1: u8,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxScene {
    pub structgen_pad0: [u8; 8],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneReadLock {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneWriteLock {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairExtraDataItem {
    pub type_: u8,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairVelocity {
    pub type_: u8,
    pub structgen_pad0: [u8; 3],
    pub linearVelocity: [PxVec3; 2],
    pub angularVelocity: [PxVec3; 2],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairPose {
    pub type_: u8,
    pub structgen_pad0: [u8; 3],
    pub globalPose: [PxTransform; 2],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairIndex {
    pub type_: u8,
    pub structgen_pad0: [u8; 1],
    pub index: u16,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairExtraDataIterator {
    pub currPtr: *const u8,
    pub endPtr: *const u8,
    pub preSolverVelocity: *const PxContactPairVelocity,
    pub postSolverVelocity: *const PxContactPairVelocity,
    pub eventPose: *const PxContactPairPose,
    pub contactPairIndex: u32,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairHeader {
    pub actors: [*mut PxActor; 2],
    pub extraDataStream: *const u8,
    pub extraDataStreamSize: u16,
    pub flags: PxContactPairHeaderFlags,
    pub structgen_pad0: [u8; 4],
    pub pairs: *const PxContactPair,
    pub nbPairs: u32,
    pub structgen_pad1: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPairPoint {
    pub position: PxVec3,
    pub separation: f32,
    pub normal: PxVec3,
    pub internalFaceIndex0: u32,
    pub impulse: PxVec3,
    pub internalFaceIndex1: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactPair {
    pub shapes: [*mut PxShape; 2],
    pub contactPatches: *const u8,
    pub contactPoints: *const u8,
    pub contactImpulses: *const f32,
    pub requiredBufferSize: u32,
    pub contactCount: u8,
    pub patchCount: u8,
    pub contactStreamSize: u16,
    pub flags: PxContactPairFlags,
    pub events: PxPairFlags,
    pub internalData: [u32; 2],
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTriggerPair {
    pub triggerShape: *mut PxShape,
    pub triggerActor: *mut PxActor,
    pub otherShape: *mut PxShape,
    pub otherActor: *mut PxActor,
    pub status: PxPairFlag,
    pub flags: PxTriggerPairFlags,
    pub structgen_pad0: [u8; 3],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConstraintInfo {
    pub constraint: *mut PxConstraint,
    pub externalReference: *mut std::ffi::c_void,
    pub type_: u32,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxFEMParameters {
    pub velocityDamping: f32,
    pub settlingThreshold: f32,
    pub sleepThreshold: f32,
    pub sleepDamping: f32,
    pub selfCollisionFilterDistance: f32,
    pub selfCollisionStressTolerance: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPruningStructure {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxExtendedVec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxObstacle {
    pub structgen_pad0: [u8; 8],
    pub mUserData: *mut std::ffi::c_void,
    pub mPos: PxExtendedVec3,
    pub mRot: PxQuat,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBoxObstacle {
    pub structgen_pad0: [u8; 8],
    pub mUserData: *mut std::ffi::c_void,
    pub mPos: PxExtendedVec3,
    pub mRot: PxQuat,
    pub mHalfExtents: PxVec3,
    pub structgen_pad1: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCapsuleObstacle {
    pub structgen_pad0: [u8; 8],
    pub mUserData: *mut std::ffi::c_void,
    pub mPos: PxExtendedVec3,
    pub mRot: PxQuat,
    pub mHalfHeight: f32,
    pub mRadius: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerState {
    pub deltaXP: PxVec3,
    pub structgen_pad0: [u8; 4],
    pub touchedShape: *mut PxShape,
    pub touchedActor: *mut PxRigidActor,
    pub touchedObstacleHandle: u32,
    pub collisionFlags: u32,
    pub standOnAnotherCCT: bool,
    pub standOnObstacle: bool,
    pub isMovingUp: bool,
    pub structgen_pad1: [u8; 5],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerStats {
    pub nbIterations: u16,
    pub nbFullUpdates: u16,
    pub nbPartialUpdates: u16,
    pub nbTessellation: u16,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerHit {
    pub controller: *mut PxController,
    pub worldPos: PxExtendedVec3,
    pub worldNormal: PxVec3,
    pub dir: PxVec3,
    pub length: f32,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerShapeHit {
    pub controller: *mut PxController,
    pub worldPos: PxExtendedVec3,
    pub worldNormal: PxVec3,
    pub dir: PxVec3,
    pub length: f32,
    pub structgen_pad0: [u8; 4],
    pub shape: *mut PxShape,
    pub actor: *mut PxRigidActor,
    pub triangleIndex: u32,
    pub structgen_pad1: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllersHit {
    pub controller: *mut PxController,
    pub worldPos: PxExtendedVec3,
    pub worldNormal: PxVec3,
    pub dir: PxVec3,
    pub length: f32,
    pub structgen_pad0: [u8; 4],
    pub other: *mut PxController,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerObstacleHit {
    pub controller: *mut PxController,
    pub worldPos: PxExtendedVec3,
    pub worldNormal: PxVec3,
    pub dir: PxVec3,
    pub length: f32,
    pub structgen_pad0: [u8; 4],
    pub userData: *const std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerFilters {
    pub mFilterData: *const PxFilterData,
    pub mFilterCallback: *mut PxQueryFilterCallback,
    pub mFilterFlags: PxQueryFlags,
    pub structgen_pad0: [u8; 6],
    pub mCCTFilterCallback: *mut PxControllerFilterCallback,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxControllerDesc {
    pub structgen_pad0: [u8; 8],
    pub position: PxExtendedVec3,
    pub upDirection: PxVec3,
    pub slopeLimit: f32,
    pub invisibleWallHeight: f32,
    pub maxJumpHeight: f32,
    pub contactOffset: f32,
    pub stepOffset: f32,
    pub density: f32,
    pub scaleCoeff: f32,
    pub volumeGrowth: f32,
    pub structgen_pad1: [u8; 4],
    pub reportCallback: *mut PxUserControllerHitReport,
    pub behaviorCallback: *mut PxControllerBehaviorCallback,
    pub nonWalkableMode: PxControllerNonWalkableMode,
    pub structgen_pad2: [u8; 4],
    pub material: *mut PxMaterial,
    pub registerDeletionListener: bool,
    pub clientID: u8,
    pub structgen_pad3: [u8; 6],
    pub userData: *mut std::ffi::c_void,
    pub structgen_pad4: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBoxControllerDesc {
    pub structgen_pad0: [u8; 8],
    pub position: PxExtendedVec3,
    pub upDirection: PxVec3,
    pub slopeLimit: f32,
    pub invisibleWallHeight: f32,
    pub maxJumpHeight: f32,
    pub contactOffset: f32,
    pub stepOffset: f32,
    pub density: f32,
    pub scaleCoeff: f32,
    pub volumeGrowth: f32,
    pub structgen_pad1: [u8; 4],
    pub reportCallback: *mut PxUserControllerHitReport,
    pub behaviorCallback: *mut PxControllerBehaviorCallback,
    pub nonWalkableMode: PxControllerNonWalkableMode,
    pub structgen_pad2: [u8; 4],
    pub material: *mut PxMaterial,
    pub registerDeletionListener: bool,
    pub clientID: u8,
    pub structgen_pad3: [u8; 6],
    pub userData: *mut std::ffi::c_void,
    pub structgen_pad4: [u8; 8],
    pub halfHeight: f32,
    pub halfSideExtent: f32,
    pub halfForwardExtent: f32,
    pub structgen_pad5: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCapsuleControllerDesc {
    pub structgen_pad0: [u8; 8],
    pub position: PxExtendedVec3,
    pub upDirection: PxVec3,
    pub slopeLimit: f32,
    pub invisibleWallHeight: f32,
    pub maxJumpHeight: f32,
    pub contactOffset: f32,
    pub stepOffset: f32,
    pub density: f32,
    pub scaleCoeff: f32,
    pub volumeGrowth: f32,
    pub structgen_pad1: [u8; 4],
    pub reportCallback: *mut PxUserControllerHitReport,
    pub behaviorCallback: *mut PxControllerBehaviorCallback,
    pub nonWalkableMode: PxControllerNonWalkableMode,
    pub structgen_pad2: [u8; 4],
    pub material: *mut PxMaterial,
    pub registerDeletionListener: bool,
    pub clientID: u8,
    pub structgen_pad3: [u8; 6],
    pub userData: *mut std::ffi::c_void,
    pub structgen_pad4: [u8; 8],
    pub radius: f32,
    pub height: f32,
    pub climbingMode: PxCapsuleClimbingMode,
    pub structgen_pad5: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDim3 {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSDFDesc {
    pub sdf: PxBoundedData,
    pub dims: PxDim3,
    pub meshLower: PxVec3,
    pub spacing: f32,
    pub subgridSize: u32,
    pub bitsPerSubgridPixel: PxSdfBitsPerSubgridPixel,
    pub sdfSubgrids3DTexBlockDim: PxDim3,
    pub sdfSubgrids: PxBoundedData,
    pub sdfStartSlots: PxBoundedData,
    pub subgridsMinSdfValue: f32,
    pub subgridsMaxSdfValue: f32,
    pub sdfBounds: PxBounds3,
    pub narrowBandThicknessRelativeToSdfBoundsDiagonal: f32,
    pub numThreadsForSdfConstruction: u32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxConvexMeshDesc {
    pub points: PxBoundedData,
    pub polygons: PxBoundedData,
    pub indices: PxBoundedData,
    pub flags: PxConvexFlags,
    pub vertexLimit: u16,
    pub polygonLimit: u16,
    pub quantizedCount: u16,
    pub sdfDesc: *mut PxSDFDesc,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTriangleMeshDesc {
    pub points: PxBoundedData,
    pub triangles: PxBoundedData,
    pub flags: PxMeshFlags,
    pub structgen_pad0: [u8; 22],
    pub sdfDesc: *mut PxSDFDesc,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTetrahedronMeshDesc {
    pub structgen_pad0: [u8; 16],
    pub points: PxBoundedData,
    pub tetrahedrons: PxBoundedData,
    pub flags: PxMeshFlags,
    pub tetsPerElement: u16,
    pub structgen_pad1: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSoftBodySimulationDataDesc {
    pub vertexToTet: PxBoundedData,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVH34MidphaseDesc {
    pub numPrimsPerLeaf: u32,
    pub buildStrategy: PxBVH34BuildStrategy,
    pub quantized: bool,
    pub structgen_pad0: [u8; 3],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMidphaseDesc {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBVHDesc {
    pub bounds: PxBoundedData,
    pub enlargement: f32,
    pub numPrimsPerLeaf: u32,
    pub buildStrategy: PxBVHBuildStrategy,
    pub structgen_pad0: [u8; 4],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxCookingParams {
    pub areaTestEpsilon: f32,
    pub planeTolerance: f32,
    pub convexMeshCookingType: PxConvexMeshCookingType,
    pub suppressTriangleMeshRemapTable: bool,
    pub buildTriangleAdjacencies: bool,
    pub buildGPUData: bool,
    pub structgen_pad0: [u8; 1],
    pub scale: PxTolerancesScale,
    pub meshPreprocessParams: PxMeshPreprocessingFlags,
    pub meshWeldTolerance: f32,
    pub midphaseDesc: PxMidphaseDesc,
    pub gaussMapLimit: u32,
    pub maxWeightRatioInTet: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultMemoryOutputStream {
    pub structgen_pad0: [u8; 32],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultMemoryInputData {
    pub structgen_pad0: [u8; 32],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultFileOutputStream {
    pub structgen_pad0: [u8; 16],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDefaultFileInputData {
    pub structgen_pad0: [u8; 24],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSpring {
    pub stiffness: f32,
    pub damping: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxDistanceJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJacobianRow {
    pub linear0: PxVec3,
    pub linear1: PxVec3,
    pub angular0: PxVec3,
    pub angular1: PxVec3,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxContactJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxFixedJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJointLimitParameters {
    pub restitution: f32,
    pub bounceThreshold: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub contactDistance_deprecated: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJointLinearLimit {
    pub restitution: f32,
    pub bounceThreshold: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub contactDistance_deprecated: f32,
    pub value: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJointLinearLimitPair {
    pub restitution: f32,
    pub bounceThreshold: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub contactDistance_deprecated: f32,
    pub upper: f32,
    pub lower: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJointAngularLimitPair {
    pub restitution: f32,
    pub bounceThreshold: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub contactDistance_deprecated: f32,
    pub upper: f32,
    pub lower: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJointLimitCone {
    pub restitution: f32,
    pub bounceThreshold: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub contactDistance_deprecated: f32,
    pub yAngle: f32,
    pub zAngle: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxJointLimitPyramid {
    pub restitution: f32,
    pub bounceThreshold: f32,
    pub stiffness: f32,
    pub damping: f32,
    pub contactDistance_deprecated: f32,
    pub yAngleMin: f32,
    pub yAngleMax: f32,
    pub zAngleMin: f32,
    pub zAngleMax: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPrismaticJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRevoluteJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSphericalJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxD6JointDrive {
    pub stiffness: f32,
    pub damping: f32,
    pub forceLimit: f32,
    pub flags: PxD6JointDriveFlags,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxD6Joint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGearJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRackAndPinionJoint {
    pub structgen_pad0: [u8; 16],
    pub userData: *mut std::ffi::c_void,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxGroupsMask {
    pub bits0: u16,
    pub bits1: u16,
    pub bits2: u16,
    pub bits3: u16,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRigidActorExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMassProperties {
    pub inertiaTensor: PxMat33,
    pub centerOfMass: PxVec3,
    pub mass: f32,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRigidBodyExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxShapeExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxMeshOverlapUtil {
    pub structgen_pad0: [u8; 1040],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxXmlMiscParameter {
    pub upVector: PxVec3,
    pub scale: PxTolerancesScale,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSerialization {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxStringTableExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxBroadPhaseExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSceneQueryExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxSamplingExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxPoissonSampler {
    pub structgen_pad0: [u8; 8],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTriangleMeshPoissonSampler {
    pub structgen_pad0: [u8; 24],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxTetrahedronMeshExt {
    pub structgen_pad0: [u8; 1],
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRepXObject {
    pub typeName: *const std::ffi::c_char,
    pub serializable: *const std::ffi::c_void,
    pub id: u64,
}
#[derive(Clone, Copy)]
#[cfg_attr(feature = "debug-structs", derive(Debug))]
#[repr(C)]
pub struct PxRepXInstantiationArgs {
    pub structgen_pad0: [u8; 8],
    pub cooker: *mut PxCooking,
    pub stringTable: *mut PxStringTable,
}
#[cfg(test)]
mod sizes {
    use super::*;
    use std::mem::size_of;
    #[test]
    fn check_sizes() {
        assert_eq!(size_of::<PxAllocator>(), 1);
        assert_eq!(size_of::<PxRawAllocator>(), 1);
        assert_eq!(size_of::<PxVirtualAllocator>(), 16);
        assert_eq!(size_of::<PxUserAllocated>(), 1);
        assert_eq!(size_of::<PxTempAllocator>(), 1);
        assert_eq!(size_of::<PxBitAndByte>(), 1);
        assert_eq!(size_of::<PxBitMap>(), 16);
        assert_eq!(size_of::<PxVec3>(), 12);
        assert_eq!(size_of::<PxVec3Padded>(), 16);
        assert_eq!(size_of::<PxQuat>(), 16);
        assert_eq!(size_of::<PxTransform>(), 28);
        assert_eq!(size_of::<PxTransformPadded>(), 32);
        assert_eq!(size_of::<PxMat33>(), 36);
        assert_eq!(size_of::<PxBounds3>(), 24);
        assert_eq!(size_of::<PxBroadcastingAllocator>(), 176);
        assert_eq!(size_of::<PxBroadcastingErrorCallback>(), 160);
        assert_eq!(size_of::<PxVec4>(), 16);
        assert_eq!(size_of::<PxMat44>(), 64);
        assert_eq!(size_of::<PxPlane>(), 16);
        assert_eq!(size_of::<Interpolation>(), 1);
        assert_eq!(size_of::<PxMutexImpl>(), 1);
        assert_eq!(size_of::<PxReadWriteLock>(), 8);
        assert_eq!(size_of::<PxProfileScoped>(), 40);
        assert_eq!(size_of::<PxSListEntry>(), 16);
        assert_eq!(size_of::<PxSListImpl>(), 1);
        assert_eq!(size_of::<PxSyncImpl>(), 1);
        assert_eq!(size_of::<PxCounterFrequencyToTensOfNanos>(), 16);
        assert_eq!(size_of::<PxTime>(), 8);
        assert_eq!(size_of::<PxVec2>(), 8);
        assert_eq!(size_of::<PxStridedData>(), 16);
        assert_eq!(size_of::<PxBoundedData>(), 24);
        assert_eq!(size_of::<PxDebugPoint>(), 16);
        assert_eq!(size_of::<PxDebugLine>(), 32);
        assert_eq!(size_of::<PxDebugTriangle>(), 48);
        assert_eq!(size_of::<PxDebugText>(), 32);
        assert_eq!(size_of::<PxDeserializationContext>(), 16);
        assert_eq!(size_of::<PxBase>(), 16);
        assert_eq!(size_of::<PxRefCounted>(), 16);
        assert_eq!(size_of::<PxTolerancesScale>(), 8);
        assert_eq!(size_of::<PxMetaDataEntry>(), 40);
        assert_eq!(size_of::<PxBaseTask>(), 24);
        assert_eq!(size_of::<PxTask>(), 32);
        assert_eq!(size_of::<PxLightCpuTask>(), 40);
        assert_eq!(size_of::<PxGeometry>(), 8);
        assert_eq!(size_of::<PxBoxGeometry>(), 20);
        assert_eq!(size_of::<PxBVH>(), 16);
        assert_eq!(size_of::<PxCapsuleGeometry>(), 16);
        assert_eq!(size_of::<PxHullPolygon>(), 20);
        assert_eq!(size_of::<PxConvexMesh>(), 16);
        assert_eq!(size_of::<PxMeshScale>(), 28);
        assert_eq!(size_of::<PxConvexMeshGeometry>(), 56);
        assert_eq!(size_of::<PxSphereGeometry>(), 12);
        assert_eq!(size_of::<PxPlaneGeometry>(), 8);
        assert_eq!(size_of::<PxTriangleMeshGeometry>(), 48);
        assert_eq!(size_of::<PxHeightFieldGeometry>(), 32);
        assert_eq!(size_of::<PxParticleSystemGeometry>(), 12);
        assert_eq!(size_of::<PxHairSystemGeometry>(), 8);
        assert_eq!(size_of::<PxTetrahedronMeshGeometry>(), 16);
        assert_eq!(size_of::<PxQueryHit>(), 4);
        assert_eq!(size_of::<PxLocationHit>(), 36);
        assert_eq!(size_of::<PxGeomRaycastHit>(), 44);
        assert_eq!(size_of::<PxGeomOverlapHit>(), 4);
        assert_eq!(size_of::<PxGeomSweepHit>(), 36);
        assert_eq!(size_of::<PxGeomIndexPair>(), 8);
        assert_eq!(size_of::<PxQueryThreadContext>(), 1);
        assert_eq!(size_of::<PxCustomGeometryType>(), 4);
        assert_eq!(size_of::<PxCustomGeometry>(), 16);
        assert_eq!(size_of::<PxGeometryHolder>(), 56);
        assert_eq!(size_of::<PxGeometryQuery>(), 1);
        assert_eq!(size_of::<PxHeightFieldSample>(), 4);
        assert_eq!(size_of::<PxHeightField>(), 16);
        assert_eq!(size_of::<PxHeightFieldDesc>(), 40);
        assert_eq!(size_of::<PxMeshQuery>(), 1);
        assert_eq!(size_of::<PxSimpleTriangleMesh>(), 56);
        assert_eq!(size_of::<PxTriangle>(), 36);
        assert_eq!(size_of::<PxTrianglePadded>(), 40);
        assert_eq!(size_of::<PxTriangleMesh>(), 16);
        assert_eq!(size_of::<PxBVH34TriangleMesh>(), 16);
        assert_eq!(size_of::<PxTetrahedron>(), 48);
        assert_eq!(size_of::<PxSoftBodyAuxData>(), 16);
        assert_eq!(size_of::<PxTetrahedronMesh>(), 16);
        assert_eq!(size_of::<PxSoftBodyMesh>(), 16);
        assert_eq!(size_of::<PxCollisionMeshMappingData>(), 8);
        assert_eq!(size_of::<PxSoftBodyCollisionData>(), 1);
        assert_eq!(size_of::<PxTetrahedronMeshData>(), 1);
        assert_eq!(size_of::<PxSoftBodySimulationData>(), 1);
        assert_eq!(size_of::<PxCollisionTetrahedronMeshData>(), 8);
        assert_eq!(size_of::<PxSimulationTetrahedronMeshData>(), 8);
        assert_eq!(size_of::<PxActor>(), 24);
        assert_eq!(size_of::<PxAggregate>(), 24);
        assert_eq!(size_of::<PxSpringModifiers>(), 16);
        assert_eq!(size_of::<PxRestitutionModifiers>(), 16);
        assert_eq!(size_of::<Px1DConstraint>(), 96);
        assert_eq!(size_of::<PxConstraintInvMassScale>(), 16);
        assert_eq!(size_of::<PxContactPoint>(), 80);
        assert_eq!(size_of::<PxSolverBody>(), 32);
        assert_eq!(size_of::<PxSolverBodyData>(), 112);
        assert_eq!(size_of::<PxConstraintBatchHeader>(), 8);
        assert_eq!(size_of::<PxSolverConstraintDesc>(), 64);
        assert_eq!(size_of::<PxSolverConstraintPrepDescBase>(), 128);
        assert_eq!(size_of::<PxSolverConstraintPrepDesc>(), 192);
        assert_eq!(size_of::<PxSolverContactDesc>(), 208);
        assert_eq!(size_of::<PxArticulationLimit>(), 8);
        assert_eq!(size_of::<PxArticulationDrive>(), 16);
        assert_eq!(size_of::<PxTGSSolverBodyVel>(), 64);
        assert_eq!(size_of::<PxTGSSolverBodyTxInertia>(), 64);
        assert_eq!(size_of::<PxTGSSolverBodyData>(), 48);
        assert_eq!(size_of::<PxTGSSolverConstraintPrepDescBase>(), 144);
        assert_eq!(size_of::<PxTGSSolverConstraintPrepDesc>(), 240);
        assert_eq!(size_of::<PxTGSSolverContactDesc>(), 240);
        assert_eq!(size_of::<PxArticulationTendonLimit>(), 8);
        assert_eq!(size_of::<PxArticulationAttachment>(), 24);
        assert_eq!(size_of::<PxArticulationTendonJoint>(), 24);
        assert_eq!(size_of::<PxArticulationTendon>(), 24);
        assert_eq!(size_of::<PxArticulationSpatialTendon>(), 24);
        assert_eq!(size_of::<PxArticulationFixedTendon>(), 24);
        assert_eq!(size_of::<PxSpatialForce>(), 32);
        assert_eq!(size_of::<PxSpatialVelocity>(), 32);
        assert_eq!(size_of::<PxArticulationRootLinkData>(), 76);
        assert_eq!(size_of::<PxArticulationCache>(), 136);
        assert_eq!(size_of::<PxArticulationSensor>(), 24);
        assert_eq!(size_of::<PxArticulationReducedCoordinate>(), 24);
        assert_eq!(size_of::<PxArticulationJointReducedCoordinate>(), 24);
        assert_eq!(size_of::<PxShape>(), 24);
        assert_eq!(size_of::<PxRigidActor>(), 24);
        assert_eq!(size_of::<PxNodeIndex>(), 8);
        assert_eq!(size_of::<PxRigidBody>(), 24);
        assert_eq!(size_of::<PxArticulationLink>(), 24);
        assert_eq!(size_of::<PxConeLimitedConstraint>(), 24);
        assert_eq!(size_of::<PxConeLimitParams>(), 32);
        assert_eq!(size_of::<PxConstraintShaderTable>(), 32);
        assert_eq!(size_of::<PxConstraint>(), 24);
        assert_eq!(size_of::<PxMassModificationProps>(), 16);
        assert_eq!(size_of::<PxContactPatch>(), 64);
        assert_eq!(size_of::<PxContact>(), 16);
        assert_eq!(size_of::<PxExtendedContact>(), 32);
        assert_eq!(size_of::<PxModifiableContact>(), 64);
        assert_eq!(size_of::<PxContactStreamIterator>(), 80);
        assert_eq!(size_of::<PxGpuContactPair>(), 72);
        assert_eq!(size_of::<PxContactSet>(), 16);
        assert_eq!(size_of::<PxContactModifyPair>(), 104);
        assert_eq!(size_of::<PxBaseMaterial>(), 24);
        assert_eq!(size_of::<PxFEMMaterial>(), 24);
        assert_eq!(size_of::<PxFilterData>(), 16);
        assert_eq!(size_of::<PxParticleRigidFilterPair>(), 16);
        assert_eq!(size_of::<PxMaterial>(), 24);
        assert_eq!(size_of::<PxGpuParticleBufferIndexPair>(), 8);
        assert_eq!(size_of::<PxParticleVolume>(), 32);
        assert_eq!(size_of::<PxDiffuseParticleParams>(), 40);
        assert_eq!(size_of::<PxParticleSpring>(), 24);
        assert_eq!(size_of::<PxParticleMaterial>(), 24);
        assert_eq!(size_of::<PxActorShape>(), 16);
        assert_eq!(size_of::<PxRaycastHit>(), 64);
        assert_eq!(size_of::<PxOverlapHit>(), 24);
        assert_eq!(size_of::<PxSweepHit>(), 56);
        assert_eq!(size_of::<PxRaycastCallback>(), 96);
        assert_eq!(size_of::<PxOverlapCallback>(), 56);
        assert_eq!(size_of::<PxSweepCallback>(), 88);
        assert_eq!(size_of::<PxRaycastBuffer>(), 96);
        assert_eq!(size_of::<PxOverlapBuffer>(), 56);
        assert_eq!(size_of::<PxSweepBuffer>(), 88);
        assert_eq!(size_of::<PxQueryCache>(), 24);
        assert_eq!(size_of::<PxQueryFilterData>(), 20);
        assert_eq!(size_of::<PxRigidDynamic>(), 24);
        assert_eq!(size_of::<PxRigidStatic>(), 24);
        assert_eq!(size_of::<PxSceneQueryDesc>(), 36);
        assert_eq!(size_of::<PxBroadPhaseRegion>(), 32);
        assert_eq!(size_of::<PxBroadPhaseRegionInfo>(), 48);
        assert_eq!(size_of::<PxBroadPhaseCaps>(), 4);
        assert_eq!(size_of::<PxBroadPhaseDesc>(), 32);
        assert_eq!(size_of::<PxBroadPhaseUpdateData>(), 80);
        assert_eq!(size_of::<PxBroadPhasePair>(), 8);
        assert_eq!(size_of::<PxBroadPhaseResults>(), 32);
        assert_eq!(size_of::<PxSceneLimits>(), 32);
        assert_eq!(size_of::<PxgDynamicsMemoryConfig>(), 48);
        assert_eq!(size_of::<PxSceneDesc>(), 352);
        assert_eq!(size_of::<PxSimulationStatistics>(), 2232);
        assert_eq!(size_of::<PxGpuBodyData>(), 64);
        assert_eq!(size_of::<PxGpuActorPair>(), 16);
        assert_eq!(size_of::<PxIndexDataPair>(), 16);
        assert_eq!(size_of::<PxDominanceGroupPair>(), 2);
        assert_eq!(size_of::<PxScene>(), 16);
        assert_eq!(size_of::<PxSceneReadLock>(), 8);
        assert_eq!(size_of::<PxSceneWriteLock>(), 8);
        assert_eq!(size_of::<PxContactPairExtraDataItem>(), 1);
        assert_eq!(size_of::<PxContactPairVelocity>(), 52);
        assert_eq!(size_of::<PxContactPairPose>(), 60);
        assert_eq!(size_of::<PxContactPairIndex>(), 4);
        assert_eq!(size_of::<PxContactPairExtraDataIterator>(), 48);
        assert_eq!(size_of::<PxContactPairHeader>(), 48);
        assert_eq!(size_of::<PxContactPairPoint>(), 48);
        assert_eq!(size_of::<PxContactPair>(), 64);
        assert_eq!(size_of::<PxTriggerPair>(), 40);
        assert_eq!(size_of::<PxConstraintInfo>(), 24);
        assert_eq!(size_of::<PxFEMParameters>(), 24);
        assert_eq!(size_of::<PxPruningStructure>(), 16);
        assert_eq!(size_of::<PxExtendedVec3>(), 24);
        assert_eq!(size_of::<PxObstacle>(), 56);
        assert_eq!(size_of::<PxBoxObstacle>(), 72);
        assert_eq!(size_of::<PxCapsuleObstacle>(), 64);
        assert_eq!(size_of::<PxControllerState>(), 48);
        assert_eq!(size_of::<PxControllerStats>(), 8);
        assert_eq!(size_of::<PxControllerHit>(), 64);
        assert_eq!(size_of::<PxControllerShapeHit>(), 88);
        assert_eq!(size_of::<PxControllersHit>(), 72);
        assert_eq!(size_of::<PxControllerObstacleHit>(), 72);
        assert_eq!(size_of::<PxControllerFilters>(), 32);
        assert_eq!(size_of::<PxControllerDesc>(), 136);
        assert_eq!(size_of::<PxBoxControllerDesc>(), 152);
        assert_eq!(size_of::<PxCapsuleControllerDesc>(), 152);
        assert_eq!(size_of::<PxDim3>(), 12);
        assert_eq!(size_of::<PxSDFDesc>(), 160);
        assert_eq!(size_of::<PxConvexMeshDesc>(), 88);
        assert_eq!(size_of::<PxTriangleMeshDesc>(), 80);
        assert_eq!(size_of::<PxTetrahedronMeshDesc>(), 72);
        assert_eq!(size_of::<PxSoftBodySimulationDataDesc>(), 24);
        assert_eq!(size_of::<PxBVH34MidphaseDesc>(), 12);
        assert_eq!(size_of::<PxMidphaseDesc>(), 16);
        assert_eq!(size_of::<PxBVHDesc>(), 40);
        assert_eq!(size_of::<PxCookingParams>(), 56);
        assert_eq!(size_of::<PxDefaultMemoryOutputStream>(), 32);
        assert_eq!(size_of::<PxDefaultMemoryInputData>(), 32);
        assert_eq!(size_of::<PxDefaultFileOutputStream>(), 16);
        assert_eq!(size_of::<PxDefaultFileInputData>(), 24);
        assert_eq!(size_of::<PxJoint>(), 24);
        assert_eq!(size_of::<PxSpring>(), 8);
        assert_eq!(size_of::<PxDistanceJoint>(), 24);
        assert_eq!(size_of::<PxJacobianRow>(), 48);
        assert_eq!(size_of::<PxContactJoint>(), 24);
        assert_eq!(size_of::<PxFixedJoint>(), 24);
        assert_eq!(size_of::<PxJointLimitParameters>(), 20);
        assert_eq!(size_of::<PxJointLinearLimit>(), 24);
        assert_eq!(size_of::<PxJointLinearLimitPair>(), 28);
        assert_eq!(size_of::<PxJointAngularLimitPair>(), 28);
        assert_eq!(size_of::<PxJointLimitCone>(), 28);
        assert_eq!(size_of::<PxJointLimitPyramid>(), 36);
        assert_eq!(size_of::<PxPrismaticJoint>(), 24);
        assert_eq!(size_of::<PxRevoluteJoint>(), 24);
        assert_eq!(size_of::<PxSphericalJoint>(), 24);
        assert_eq!(size_of::<PxD6JointDrive>(), 16);
        assert_eq!(size_of::<PxD6Joint>(), 24);
        assert_eq!(size_of::<PxGearJoint>(), 24);
        assert_eq!(size_of::<PxRackAndPinionJoint>(), 24);
        assert_eq!(size_of::<PxGroupsMask>(), 8);
        assert_eq!(size_of::<PxRigidActorExt>(), 1);
        assert_eq!(size_of::<PxMassProperties>(), 52);
        assert_eq!(size_of::<PxRigidBodyExt>(), 1);
        assert_eq!(size_of::<PxShapeExt>(), 1);
        assert_eq!(size_of::<PxMeshOverlapUtil>(), 1040);
        assert_eq!(size_of::<PxXmlMiscParameter>(), 20);
        assert_eq!(size_of::<PxSerialization>(), 1);
        assert_eq!(size_of::<PxStringTableExt>(), 1);
        assert_eq!(size_of::<PxBroadPhaseExt>(), 1);
        assert_eq!(size_of::<PxSceneQueryExt>(), 1);
        assert_eq!(size_of::<PxSamplingExt>(), 1);
        assert_eq!(size_of::<PxPoissonSampler>(), 8);
        assert_eq!(size_of::<PxTriangleMeshPoissonSampler>(), 24);
        assert_eq!(size_of::<PxTetrahedronMeshExt>(), 1);
        assert_eq!(size_of::<PxRepXObject>(), 24);
        assert_eq!(size_of::<PxRepXInstantiationArgs>(), 24);
    }
}
