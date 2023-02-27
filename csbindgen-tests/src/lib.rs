 #[allow(dead_code)]
 #[allow(non_snake_case)]
 #[allow(non_camel_case_types)]
 #[allow(non_upper_case_globals)]
 mod lz4;

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
mod lz4_ffi;

#[test]
fn build_test() {
    let path = std::env::current_dir().unwrap();
    println!("starting dir: {}", path.display()); // csbindgen/csbindgen-tests

    // unsafe {
    //     let num = lz4::LZ4_versionNumber();
    //     println!("lz4 num: {}", num);
    // }

    csbindgen::Builder::new()
        .input_bindgen_file("src/quiche.rs")
        .rust_method_prefix("csbindgen_quiche_")
        .csharp_class_name("LibQuiche")
        .csharp_dll_name("libquiche")
        .generate_to_file("src/quiche_ffi.rs", "../dotnet-sandbox/quiche_bindgen.cs")
        .unwrap();
}
