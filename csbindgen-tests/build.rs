use std::error::Error;

fn main() -> Result<(), Box<dyn Error>> {
    //fn main() {
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

    //  bindgen::Builder::default()
    //      .header("c/sqlite3/sqlite3.h")
    //      .generate()?
    //      .write_to_file("src/sqlite3.rs")?;

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
        .method_filter(|x| x.starts_with("LZ4"))
        .rust_method_prefix("csbindgen_")
        .rust_file_header("use super::lz4;")
        .rust_method_type_path("lz4")
        .csharp_class_name("LibLz4")
        .csharp_namespace("CsBindgen")
        .csharp_dll_name("csbindgen_tests")
        .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal")
        .csharp_entry_point_prefix("csbindgen_")
        .csharp_method_prefix("")
        .csharp_class_accessibility("public")
        //.csharp_c_long_convert("int")
        //.csharp_c_ulong_convert("uint")
        // .csharp_use_function_pointer(true)
        .csharp_generate_const_filter(|_| true)
        .generate_to_file("src/lz4_ffi.rs", "../dotnet-sandbox/lz4_bindgen.cs")
        .unwrap();

    // csbindgen::Builder::default()
    //     .input_bindgen_file("src/sqlite3.rs")
    //     .method_filter(|x| x.starts_with("sqlite3_"))
    //     .rust_method_prefix("csbindgen_")
    //     .rust_file_header("use super::sqlite3::*;")
    //     // .rust_method_type_path("sqlite3")
    //     .csharp_class_name("LibSqlite3")
    //     .csharp_namespace("CsBindgen")
    //     .csharp_dll_name("csbindgen_tests")
    //     .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal")
    //     .csharp_entry_point_prefix("csbindgen_")
    //     .csharp_method_prefix("")
    //     .csharp_class_accessibility("public")
    //     .generate_to_file("src/sqlite3_ffi.rs", "../dotnet-sandbox/sqlite3_bindgen.cs")
    //     .unwrap();

    csbindgen::Builder::default()
        .input_extern_file("src/lib.rs")
        .input_extern_file("src/others.rs")
        //.input_extern_files(&["src/lib.rs"])
        .csharp_class_name("NativeMethods")
        .csharp_dll_name("csbindgen_tests")
        .csharp_use_function_pointer(true)
        //.csharp_use_function_pointer(false)
        .csharp_generate_const_filter(|_| true)
        .generate_csharp_file("../dotnet-sandbox/NativeMethods.cs")
        .unwrap();

    csbindgen::Builder::default()
        .input_extern_file("src/nested_module_test.rs")
        .csharp_class_name("NestedModuleTests")
        .csharp_dll_name("csbindgen_tests_nested_module")
        .csharp_use_function_pointer(true)
        .generate_csharp_file("../dotnet-sandbox/NestedModuleTests.cs")
        .unwrap();

    csbindgen::Builder::new()
        .input_bindgen_file("src/zstd.rs")
        .method_filter(|x| x.starts_with("ZSTD_"))
        .rust_file_header("use super::zstd::*;")
        .csharp_class_name("LibZstd")
        .csharp_dll_name("libzsd")
        .generate_to_file("src/zstd_ffi.rs", "../dotnet-sandbox/zstd_bindgen.cs")?;

    csbindgen::Builder::new()
        .input_bindgen_file("src/quiche.rs")
        .method_filter(|x| x.starts_with("quiche_"))
        .rust_file_header("use super::quiche::*;")
        .csharp_class_name("LibQuiche")
        .csharp_dll_name("libquiche")
        .generate_to_file("src/quiche_ffi.rs", "../dotnet-sandbox/quiche_bindgen.cs")?;

    csbindgen::Builder::new()
        .input_bindgen_file("src/bullet3.rs")
        .method_filter(|x| x.starts_with("b3"))
        .rust_file_header("use super::bullet3::*;")
        .csharp_class_name("LibBullet3")
        .csharp_dll_name("libbullet3")
        .csharp_generate_const_filter(|_| true)
        .generate_to_file("src/bullet3_ffi.rs", "../dotnet-sandbox/bullet3_bindgen.cs")?;

    csbindgen::Builder::new()
        .input_bindgen_file("src/libpng16.rs")
        //.method_filter(|x| x.starts_with("png_"))
        .csharp_namespace("PixivApi.ImageFile")
        .csharp_class_name("LibPng16")
        .csharp_dll_name("libpng16")
        .generate_csharp_file("../dotnet-sandbox/libpng16_csbindgen.cs")?;

    csbindgen::Builder::new()
        .input_bindgen_file("src/physx/lib.rs")
        .input_bindgen_file("src/physx/physx_generated.rs")
        .input_bindgen_file("src/physx/x86_64-pc-windows-msvc/structgen.rs")
        // .input_bindgen_file("src/physx/unix/structgen.rs")
        // .csharp_disable_emit_dll_name(true)
        .csharp_namespace("Physx")
        .csharp_class_name("LibPhysx")
        .csharp_dll_name("libphys")
        .csharp_file_header("#if !FALSE")
        .csharp_file_footer("#endif")
        .generate_csharp_file("../dotnet-sandbox/libphysx_csbindgen.cs")?;

    Ok(())
}
