use std::{
    collections::HashSet,
    ffi::{c_char, c_void, CString},
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

#[no_mangle]
pub extern "C" fn callback_test(cb: extern "C" fn(a: i32) -> i32) -> i32 {
    cb(100)
}

#[no_mangle]
pub extern "C" fn nullable_callback_test(cb: Option<extern "C" fn(a: i32) -> i32>) -> i32 {
    match cb {
        Some(f) => f(100),
        None => -1,
    }
}

#[no_mangle]
pub extern "C" fn callback_test2() -> extern "C" fn(a: i32) -> i32 {
    callback
}

extern "C" fn callback(a: i32) -> i32 {
    a * a
}

#[no_mangle]
pub extern "C" fn enum_test(i:IntEnumTest) -> i32 {
    i as i32
}

#[repr(u8)]
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

#[no_mangle]
pub extern "C" fn create_counter_context() -> *mut c_void {
    let ctx = Box::new(CounterContext {
        set: HashSet::new(),
    });
    Box::into_raw(ctx) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn insert_counter_context(context: *mut c_void, value: i32) {
    let mut counter = Box::from_raw(context as *mut CounterContext);
    counter.set.insert(value);
    Box::into_raw(counter);
}

#[no_mangle]
pub unsafe extern "C" fn delete_counter_context(context: *mut c_void) {
    let counter = Box::from_raw(context as *mut CounterContext);
    for value in counter.set.iter() {
        println!("counter value: {}", value)
    }
}

#[repr(C)]
pub struct CounterContext {
    pub set: HashSet<i32>,
}

#[no_mangle]
pub extern "C" fn my_bool(
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
pub extern "C" fn free_c_string(str: *mut c_char) {
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
pub extern "C" fn delete_context(context: *mut Context) {
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
        .input_extern_file("../../../../csbindgen-tests/src/lz4.rs")
        .csharp_class_name("LibLz4")
        .csharp_dll_name("csbindgen_tests")
        .generate_to_file("../../../../csbindgen-tests/src/lz4_ffi.cs", "../../../../dotnet-sandbox/lz4_bindgen.cs")
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

    // // unsafe {
    // //     let num = lz4::LZ4_versionNumber();
    // //     println!("lz4 num: {}", num);
    // // }

    csbindgen::Builder::default()
        .input_extern_file("csbindgen-tests/src/lib.rs")
        .csharp_class_name("LibRust")
        .csharp_dll_name("csbindgen_tests")
        .generate_csharp_file("dotnet-sandbox/method_call.cs")
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

trait Ex {}

impl Ex for i32{
}