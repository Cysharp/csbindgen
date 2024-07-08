use std::{
    collections::HashSet,
    ffi::{c_char, c_long, c_ulong, CString},
    num::*,
    ptr::NonNull,
};

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
mod lz4;

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
mod lz4_ffi;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// #[allow(non_upper_case_globals)]
// mod sqlite3;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// mod sqlite3_ffi;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// #[allow(non_upper_case_globals)]
// mod bullet3;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// mod bullet3_ffi;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// #[allow(non_upper_case_globals)]
// mod quiche;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// mod quiche_ffi;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// #[allow(non_upper_case_globals)]
// mod zstd;

// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[allow(non_camel_case_types)]
// mod zstd_ffi;

// mod others;
// pub use others::HogeMoge;

// #[no_mangle]
// pub extern "C" fn other_1(hoge: HogeMoge) {
//     println!("{:?}", hoge);
// }

// #[no_mangle]
// pub extern "C" fn string_char(str: char) {
//     println!("{}", str);
// }

#[allow(non_camel_case_types)]
pub type png_byte = ::std::os::raw::c_uchar;

#[allow(non_camel_case_types)]
pub type JPH_ContactPoints = [u128; 65usize];
#[allow(non_camel_case_types)]
pub type JPH_ContactPoints2 = [u128; 65];

#[repr(C)]
#[allow(unused)]
#[allow(non_snake_case)]
pub struct JPH_ContactManifold {
    pub mPenetrationDepth: f32,
    pub mWorldSpaceContactPointsOn1: JPH_ContactPoints,
    pub mWorldSpaceContactPointsOn2: JPH_ContactPoints,
    pub mWorldSpaceContactPointsOn3: JPH_ContactPoints2,
    pub mWorldSpaceContactPointsOn4: [u128; 65],
    pub mWorldSpaceContactPointsOn5: [u32; 65],
    pub png_name: [png_byte; 5usize],
}

#[no_mangle]
#[allow(unused)]
#[allow(non_snake_case)]
pub extern "C" fn JPH_PruneContactPoints(
    ioContactPointsOn1: *mut JPH_ContactPoints,
    ioContactPointsOn2: *mut JPH_ContactManifold,
) {
    todo!();
}

use bitflags::bitflags;

bitflags! {
    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct EnumFlags: u32 {
        const A = 0b00000001;
        const B = 0b00000010;
        const C = 0b00000100;
        const ABC = Self::A.bits() | Self::B.bits() | Self::C.bits();
    }
}

pub const FOO: i32 = 10;
pub const BAR: f32 = 120.432;
pub const BAR32: f32 = 120.431;
pub const BAR64: f64 = 120.432;
pub const STR: &str = "aiueo3";
pub const BSTR: &[u8] = b"kakikukeko"; // currently not supported.
pub const CBYTE: u8 = b'A';
pub const CCHAR: char = 'あ';
pub const BOOLCONST_T: bool = true;
pub const BOOLCONST_F: bool = false;

/// my comment!
#[no_mangle]
extern "C" fn comment_one(_flags: EnumFlags) {}

/// Multiline Comments
/// # GOTO
/// Here
/// Foo
/// Bar
///
/// TO
///
/// ZZZ
pub extern "C" fn long_jpn_comment() {}

#[repr(C)]
pub struct my_int_vec3(i32, i32, i32);

pub extern "C" fn use_vec3(_v3: my_int_vec3) {}

#[repr(C)]
pub struct NfcCard {
    pub delegate: unsafe extern "C" fn(ByteArray) -> ByteArray,
}

#[no_mangle]
pub extern "C" fn other_2(_hoge: NfcCard) {}

#[repr(C)]
pub struct ByteArray {
    pub i: i32,
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct event {
    pub a: i32,
}

#[no_mangle]
pub extern "C" fn event(event: event) {
    println!("{:?}", event);
}

#[no_mangle]
pub extern "C" fn test_func_issue_39(_f: extern "C" fn(i32)) {}

#[no_mangle]
pub extern "C" fn test_func_issue_39_variation1(_f: extern "C" fn(i32, i32, i32)) {}

#[no_mangle]
pub extern "C" fn nonnull_parameter(_output_word_uuid: NonNull<[u8; 16]>) {}

// #[no_mangle]
// pub extern "C" fn non_nonnull_parameter(_output_word_uuid: [u8; 16]){
// }

#[no_mangle]
pub extern "C" fn non_nonnull_parameter2(_output_word_uuid: NonNull<u8>) {}

#[no_mangle]
pub extern "C" fn nonzero_test(_a: NonZeroI8, _b: NonZeroU8, _c: NonZeroU32) {}

#[no_mangle]
pub extern "C" fn ge(_f: extern "C" fn(i32, i32, i32)) {}

#[no_mangle]
pub extern "C" fn nest_test(
    _f: ::std::option::Option<
        unsafe extern "C" fn(
            pxFunc: *mut ::std::option::Option<unsafe extern "C" fn(arg2: ::std::os::raw::c_int)>,
        ) -> ::std::os::raw::c_int,
    >,
) {
}

#[allow(non_camel_case_types)]
pub type LONG_PTR = ::std::os::raw::c_longlong;
#[allow(non_camel_case_types)]
pub type PSSIZE_T = *mut LONG_PTR;

#[no_mangle]
pub extern "C" fn alias_test1(_a: PSSIZE_T) {}

#[no_mangle]
pub extern "C" fn alias_test2(_b: LONG_PTR) {}

#[no_mangle]
pub unsafe extern "C" fn nullpointer_test(p: *const u8) {
    let ptr = unsafe { p.as_ref() };
    match ptr {
        Some(p2) => print!("pointer address: {}", *p2),
        None => println!("null pointer!"),
    };
}

#[no_mangle]
pub unsafe extern "C" fn csharp_to_rust_string(utf16_str: *const u16, utf16_len: i32) {
    let slice = std::slice::from_raw_parts(utf16_str, utf16_len as usize);

    let str = String::from_utf16(slice).unwrap();

    println!("{}", str);
}

#[no_mangle]
pub unsafe extern "C" fn csharp_to_rust_utf8(utf8_str: *const u8, utf8_len: i32) {
    let slice = std::slice::from_raw_parts(utf8_str, utf8_len as usize);
    let str = String::from_utf8_unchecked(slice.to_vec());
    println!("{}", str);
}

#[no_mangle]
pub unsafe extern "C" fn csharp_to_rust_bytes(bytes: *const u8, len: i32) {
    let slice = std::slice::from_raw_parts(bytes, len as usize);
    let vec = slice.to_vec();
    println!("{:?}", vec);
}

#[no_mangle]
pub extern "C" fn callback_test(cb: extern "C" fn(a: i32) -> i32) -> i32 {
    cb(100)
}

#[no_mangle]
pub extern "C" fn csharp_to_rust(cb: extern "C" fn(x: i32, y: i32) -> i32) {
    let sum = cb(10, 20); // invoke C# method
    println!("{sum}");
}

#[no_mangle]
pub extern "C" fn rust_to_csharp() -> extern "C" fn(x: i32, y: i32) -> i32 {
    sum // return rust method
}

extern "C" fn sum(x: i32, y: i32) -> i32 {
    x + y
}

#[no_mangle]
pub extern "C" fn cbt(_cb: CallbackTable) {}

#[no_mangle]
pub extern "C" fn nullable_callback_test(cb: Option<extern "C" fn(a: i32) -> i32>) -> i32 {
    match cb {
        Some(f) => f(100),
        None => -1,
    }
}

#[no_mangle]
pub extern "C" fn types_iroiro(_i: isize, _u: usize, _cl: c_long, _cul: c_ulong) {}

#[no_mangle]
pub extern "C" fn callback_test2() -> extern "C" fn(a: i32) -> i32 {
    callback
}

extern "C" fn callback(a: i32) -> i32 {
    a * a
}

#[no_mangle]
pub extern "C" fn enum_test(i: IntEnumTest) -> i32 {
    i as i32
}

#[repr(i8)]
pub enum IntEnumTest {
    A = 1,
    B = 2,
    C = 10,
}

#[no_mangle]
#[allow(improper_ctypes_definitions)]
pub extern "C" fn ignore_nop() -> (i32, i32) {
    println!("hello ignore!");
    (1, 2)
}

#[no_mangle]
pub extern "C" fn nop() -> () {
    println!("hello nop!");
}

#[no_mangle]
pub extern "C" fn my_add(x: i32, y: i32) -> i32 {
    x + y
}

#[repr(C)]
pub struct CounterContext;

#[no_mangle]
pub extern "C" fn create_counter_context() -> *mut CounterContext {
    let ctx = Box::new(InternalCounterContext {
        set: HashSet::new(),
    });
    Box::into_raw(ctx) as *mut CounterContext
}

#[no_mangle]
pub unsafe extern "C" fn counter_context_insert(context: *mut CounterContext, value: i32) {
    let mut counter = Box::from_raw(context as *mut InternalCounterContext);
    counter.set.insert(value);
    Box::into_raw(counter);
}

#[no_mangle]
pub unsafe extern "C" fn destroy_counter_context(context: *mut CounterContext) {
    _ = Box::from_raw(context as *mut InternalCounterContext);
    // for value in counter.set.iter() {
    //     println!("counter value: {}", value)
    // }
}

#[no_mangle]
pub extern "C" fn pass_vector3(v3: MyVector3) {
    println!("{}, {}, {}", v3.x, v3.y, v3.z);
}

#[no_mangle]
pub extern "C" fn return_union() -> MyUnion {
    MyUnion { bar: 53 }
}

#[repr(C)]
pub union MyUnion {
    pub foo: i32,
    pub bar: i64,
}

#[repr(C)]
pub struct MyVector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

// not repr(C)
pub struct InternalCounterContext {
    pub set: HashSet<i32>,
}

#[no_mangle]
pub unsafe extern "C" fn my_bool(
    x: bool,
    y: bool,
    z: bool,
    xr: *mut bool,
    yr: *mut bool,
    zr: *mut bool,
) -> bool {
    unsafe {
        *xr = x;
        *yr = y;
        *zr = z;
    }

    true
}

#[no_mangle]
pub extern "C" fn alloc_c_string() -> *mut c_char {
    let str = CString::new("foo bar baz").unwrap();
    str.into_raw()
}

#[no_mangle]
pub unsafe extern "C" fn free_c_string(str: *mut c_char) {
    unsafe { CString::from_raw(str) };
}

#[no_mangle]
pub extern "C" fn alloc_u8_string() -> *mut ByteBuffer {
    let str = format!("foo bar baz");
    let buf = ByteBuffer::from_vec(str.into_bytes());
    Box::into_raw(Box::new(buf))
}

#[no_mangle]
pub unsafe extern "C" fn free_u8_string(buffer: *mut ByteBuffer) {
    let buf = Box::from_raw(buffer);
    // drop inner buffer, if you need String, use String::from_utf8_unchecked(buf.destroy_into_vec()) instead.
    buf.destroy();
}

#[no_mangle]
pub extern "C" fn alloc_u8_buffer() -> *mut ByteBuffer {
    let vec: Vec<u8> = vec![1, 10, 100];
    let buf = ByteBuffer::from_vec(vec);
    Box::into_raw(Box::new(buf))
}

#[no_mangle]
pub unsafe extern "C" fn free_u8_buffer(buffer: *mut ByteBuffer) {
    let buf = Box::from_raw(buffer);
    // drop inner buffer, if you need Vec<u8>, use buf.destroy_into_vec() instead.
    buf.destroy();
}

#[no_mangle]
pub extern "C" fn alloc_i32_buffer() -> *mut ByteBuffer {
    let vec: Vec<i32> = vec![1, 10, 100, 1000, 10000];
    let buf = ByteBuffer::from_vec_struct(vec);
    Box::into_raw(Box::new(buf))
}

#[no_mangle]
pub unsafe extern "C" fn free_i32_buffer(buffer: *mut ByteBuffer) {
    let buf = Box::from_raw(buffer);
    // drop inner buffer, if you need Vec<i32>, use buf.destroy_into_vec_struct::<i32>() instead.
    buf.destroy();
}

#[no_mangle]
pub extern "C" fn create_context() -> *mut Context {
    let ctx = Box::new(Context { foo: true });
    Box::into_raw(ctx)
}

#[no_mangle]
pub unsafe extern "C" fn delete_context(context: *mut Context) {
    unsafe { Box::from_raw(context) };
}

#[no_mangle]
pub extern "C" fn call_bindgen() {
    let path = std::env::current_dir().unwrap();
    println!("starting dir: {}", path.display()); // csbindgen/csbindgen-tests

    csbindgen::Builder::default()
        .input_extern_file("../../../../csbindgen-tests/src/lib.rs")
        .csharp_class_name("LibRust")
        .csharp_dll_name("csbindgen_tests")
        .generate_csharp_file("../../../../dotnet-sandbox/method_call.cs")
        .unwrap();
}

#[no_mangle]
pub extern "C" fn call_bindgen_lz4() {
    let path = std::env::current_dir().unwrap();
    println!("starting dir: {}", path.display()); // csbindgen/csbindgen-tests

    csbindgen::Builder::default()
        .input_bindgen_file("../../../../csbindgen-tests/src/lz4.rs")
        .method_filter(|x| x.starts_with("LZ4"))
        .rust_method_prefix("csbindgen_")
        .rust_file_header("use super::lz4;")
        .rust_method_type_path("lz4")
        .csharp_class_name("LibLz4")
        .csharp_dll_name("csbindgen_tests")
        .generate_to_file(
            "../../../../csbindgen-tests/src/lz4_ffi.cs",
            "../../../../dotnet-sandbox/lz4_bindgen.cs",
        )
        .unwrap();
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Context {
    pub foo: bool,
}

#[test]
fn build_test() {
    let path = std::env::current_dir().unwrap();
    println!("starting dir: {}", path.display()); // csbindgen/csbindgen-tests
    std::env::set_current_dir(path.parent().unwrap()).unwrap();

    // // unsafe {
    // //     let num = lz4::LZ4_versionNumber();
    // //     println!("lz4 num: {}", num);
    // // }

    // csbindgen::Builder::default()
    //     .input_extern_file("csbindgen-tests/src/lib.rs")
    //     .csharp_class_name("NativeMethods")
    //     .csharp_dll_name("csbindgen_tests")
    //     .generate_csharp_file("dotnet-sandbox/NativeMethods.cs")
    //     .unwrap();

    csbindgen::Builder::new()
        .input_bindgen_file("csbindgen-tests/src/physx/physx_generated.rs")
        .input_bindgen_file("csbindgen-tests/src/physx/x86_64-pc-windows-msvc/structgen.rs")
        .csharp_namespace("Physx")
        .csharp_class_name("LibPhysxd")
        .csharp_dll_name("libphysx")
        .generate_csharp_file("dotnet-sandbox/libphysx_csbindgen.cs")
        .unwrap();
}

#[repr(C)]
pub struct ByteBuffer {
    ptr: *mut u8,
    length: i32,
    capacity: i32,
}

impl ByteBuffer {
    pub fn len(&self) -> usize {
        self.length
            .try_into()
            .expect("buffer length negative or overflowed")
    }

    pub fn from_vec(bytes: Vec<u8>) -> Self {
        let length = i32::try_from(bytes.len()).expect("buffer length cannot fit into a i32.");
        let capacity =
            i32::try_from(bytes.capacity()).expect("buffer capacity cannot fit into a i32.");

        // keep memory until call delete
        let mut v = std::mem::ManuallyDrop::new(bytes);

        Self {
            ptr: v.as_mut_ptr(),
            length,
            capacity,
        }
    }

    pub fn from_vec_struct<T: Sized>(bytes: Vec<T>) -> Self {
        let element_size = std::mem::size_of::<T>() as i32;

        let length = (bytes.len() as i32) * element_size;
        let capacity = (bytes.capacity() as i32) * element_size;

        let mut v = std::mem::ManuallyDrop::new(bytes);

        Self {
            ptr: v.as_mut_ptr() as *mut u8,
            length,
            capacity,
        }
    }

    pub fn destroy_into_vec(self) -> Vec<u8> {
        if self.ptr.is_null() {
            vec![]
        } else {
            let capacity: usize = self
                .capacity
                .try_into()
                .expect("buffer capacity negative or overflowed");
            let length: usize = self
                .length
                .try_into()
                .expect("buffer length negative or overflowed");

            unsafe { Vec::from_raw_parts(self.ptr, length, capacity) }
        }
    }

    pub fn destroy_into_vec_struct<T: Sized>(self) -> Vec<T> {
        if self.ptr.is_null() {
            vec![]
        } else {
            let element_size = std::mem::size_of::<T>() as i32;
            let length = (self.length * element_size) as usize;
            let capacity = (self.capacity * element_size) as usize;

            unsafe { Vec::from_raw_parts(self.ptr as *mut T, length, capacity) }
        }
    }

    pub fn destroy(self) {
        drop(self.destroy_into_vec());
    }
}

#[repr(C)]
pub struct CallbackTable {
    pub foo: extern "C" fn(),
    pub foobar: extern "C" fn(i: i32) -> i32,
}

pub extern "C" fn reference_type(_a: &i32, _b: &*mut i32, _c: &[u8; 16], _d: &Context) {}

pub extern "C" fn reference_hogemoge1(_a: &mut i32, _b: &&i32) {}
pub extern "C" fn reference_hogemoge2(_a: &mut i32, _b: &*mut i32) {}

#[no_mangle]
pub extern "C" fn create_counter_context2() -> *mut CounterContext2 {
    let ctx = Box::new(CounterContext2 {
        set: HashSet::new(),
    });
    Box::into_raw(ctx)
}

#[no_mangle]
pub unsafe extern "C" fn insert_counter_context2(context: *mut CounterContext2, value: i32) {
    let mut counter = Box::from_raw(context);
    counter.set.insert(value);
    Box::into_raw(counter);
}

#[no_mangle]
pub unsafe extern "C" fn delete_counter_context2(context: *mut CounterContext2) {
    let counter = Box::from_raw(context);
    for value in counter.set.iter() {
        println!("counter value: {}", value)
    }
}

// no repr(C)
pub struct CounterContext2 {
    pub set: HashSet<i32>,
}

pub struct InternalHiddenContext {
    pub a: i32,
}

pub struct TreatAsEmptyStruct {
    internal: std::sync::Arc<InternalHiddenContext>,
}

#[no_mangle]
pub unsafe extern "C" fn init_treat_as_empty_struct_context(
    _out: NonNull<Box<TreatAsEmptyStruct>>,
) {
}

#[no_mangle]
pub unsafe extern "C" fn free_treat_as_empty_struct_context(_src: *mut TreatAsEmptyStruct) {}

// fn run_physix(){
//     unsafe {
//         let foundation = physx_create_foundation();
//         let physics = physx_create_physics(foundation);

//         let mut scene_desc = PxSceneDesc_new(PxPhysics_getTolerancesScale(physics));
//         scene_desc.gravity = PxVec3 {
//             x: 0.0,
//             y: -9.81,
//             z: 0.0,
//         };

//         let dispatcher = phys_PxDefaultCpuDispatcherCreate(
//             1,
//             null_mut(),
//             PxDefaultCpuDispatcherWaitForWorkMode::WaitForWork,
//             0,
//         );
//         scene_desc.cpuDispatcher = dispatcher.cast();
//         scene_desc.filterShader = get_default_simulation_filter_shader();

//         let scene = PxPhysics_createScene_mut(physics, &scene_desc);

//         // Your physics simulation goes here
//     }
// }
