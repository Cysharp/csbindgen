// use std::error::Error;

// fn main() -> Result<(), Box<dyn Error>> {
fn main() {
    bindgen::Builder::default()
        .header("c/lz4/lz4.h")
        .header("c/lz4/lz4hc.h")
        .header("c/lz4/lz4frame.h")
        .header("c/lz4/xxhash.h")
        .generate()
        .unwrap()
        .write_to_file("src/lz4.rs")
        .unwrap();

    cc::Build::new()
        .files([
            "c/lz4/lz4.c",
            "c/lz4/lz4hc.c",
            "c/lz4/lz4frame.c",
            "c/lz4/xxhash.c",
        ])
        .compile("lz4");

    // bindgen::Builder::default()
    //     .header("c/zstd/zstd.h")
    //     .generate()?
    //     .write_to_file("src/zstd.rs")?;

    // bindgen::Builder::default()
    //     .header("c/quiche/quiche.h")
    //     .generate()?
    //     .write_to_file("src/quiche.rs")?;

    // bindgen::Builder::default()
    //     .header("c/bullet3/PhysicsClientC_API.h")
    //     .header("c/bullet3/PhysicsClientSharedMemory_C_API.h")
    //     .header("c/bullet3/PhysicsClientSharedMemory2_C_API.h")
    //     .header("c/bullet3/PhysicsDirectC_API.h")
    //     .header("c/bullet3/SharedMemoryPublic.h")
    //     .generate()?
    //     .write_to_file("src/bullet3.rs")?;

    csbindgen::Builder::default()
        .input_bindgen_file("src/lz4.rs")
        .method_filter(|x| { !x.starts_with("_") && !x.starts_with("XXH") } )
        .rust_method_prefix("csbindgen_")
        .rust_file_header("use super::lz4;")
        .rust_method_type_path("lz4")
        .csharp_class_name("LibLz4")
        .csharp_namespace("CsBindgen")
        .csharp_dll_name("csbindgen_tests")
        .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal")
        .csharp_entry_point_prefix("csbindgen_")
        .csharp_method_prefix("")
        .csharp_c_long_convert("int")
        .csharp_c_long_convert("uint")
        .generate_to_file("src/lz4_ffi.rs", "../dotnet-sandbox/lz4_bindgen.cs")
        .unwrap();

    csbindgen::Builder::default()
        .input_extern_file("src/lib.rs")
        .csharp_class_name("LibRust")
        .csharp_dll_name("csbindgen_tests")
        .generate_csharp_file("../dotnet-sandbox/method_call.cs")
        .unwrap();

    // csbindgen::Builder::new()
    //     .input_bindgen_file("src/zstd.rs")
    //     .rust_method_prefix("csbindgen_zstd_")
    //     .csharp_class_name("LibZstd")
    //     .csharp_dll_name("libzsd")
    //     .generate_to_file("src/zstd_ffi.rs", "../dotnet-sandbox/zstd_bindgen.cs")?;

    // csbindgen::Builder::new()
    //     .input_bindgen_file("src/quiche.rs")
    //     .rust_method_prefix("csbindgen_quiche_")
    //     .csharp_class_name("LibQuiche")
    //     .csharp_dll_name("libquiche")
    //     .generate_to_file("src/quiche_ffi.rs", "../dotnet-sandbox/quiche_bindgen.cs")?;

    // csbindgen::Builder::new()
    //     .input_bindgen_file("src/bullet3.rs")
    //     .rust_method_prefix("csbindgen_bullet3_")
    //     .csharp_class_name("LibBullet3")
    //     .csharp_dll_name("libbullet3")
    //     .generate_to_file("src/bullet3_ffi.rs", "../dotnet-sandbox/bullet3_bindgen.cs")?;

    // Ok(())
}
