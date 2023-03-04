use std::io::Bytes;

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
pub extern "C" fn my_add(x: i32, y: i32) -> i32 {
    x + y
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
pub unsafe extern "C" fn new(x: *mut *mut Vec<u8>) {
    let v = Box::new(Vec::new());
    *x = Box::into_raw(v);
}

#[no_mangle]
pub unsafe extern "C" fn unsafe_return_string() -> *const u8 {
todo!();
}

#[no_mangle]
pub unsafe extern "C" fn unsafe_return_string2() -> *const u8 {
    todo!();
}

#[no_mangle]
pub extern "C" fn unsafe_destroy_string(s: *mut String) {
    unsafe { Box::from_raw(s) };
}

#[test]
fn build_test() {
    // let path = std::env::current_dir().unwrap();
    // println!("starting dir: {}", path.display()); // csbindgen/csbindgen-tests

    // // unsafe {
    // //     let num = lz4::LZ4_versionNumber();
    // //     println!("lz4 num: {}", num);
    // // }

    // csbindgen::Builder::default()
    //     .input_bindgen_file("src/lz4.rs")
    //     .rust_method_prefix("csbindgen_")
    //     .rust_file_header("use super::lz4;")
    //     .rust_method_type_path("lz4")
    //     .csharp_class_name("LibLz4")
    //     .csharp_dll_name("csbindgen_tests")
    //     .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal")
    //     .csharp_entry_point_prefix("csbindgen_")
    //     .csharp_method_prefix("")
    //     .generate_to_file("src/lz4_ffi.rs", "../dotnet-sandbox/lz4_bindgen.cs")
    //     .unwrap();
}
