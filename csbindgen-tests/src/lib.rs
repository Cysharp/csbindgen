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
