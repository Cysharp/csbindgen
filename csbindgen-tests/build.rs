use std::{
    error::Error,
};

fn main() -> Result<(), Box<dyn Error>> {
    bindgen::Builder::default()
        .header("c/lz4/lz4.h")
        //.header("c/lz4/lz4hc.h")
        //.header("c/lz4/lz4frame.h")
        //.header("c/lz4/xxhash.h")
        .generate()?
        .write_to_file("src/lz4/mod.rs")?;

    cc::Build::new().file("c/lz4/lz4.c").compile("csharp_lz4");

    bindgen::Builder::default()
        .header("c/zstd/zstd.h")
        .generate()?
        .write_to_file("src/zstd.rs")?;

    csbindgen::Builder::new()
        .input_bindgen_file("src/lz4/mod.rs")
        .rust_method_prefix("csbindgen_")
        .generate_to_file("src/ffi.rs", "../dotnet-sandbox/bindgen.cs")
        .unwrap();

        csbindgen::Builder::new()
        .input_bindgen_file("src/zstd.rs")
        .rust_method_prefix("csbindgen_")
        .generate_to_file("src/zstd_ffi.rs", "../dotnet-sandbox/zstd_bindgen.cs")
        .unwrap();

    Ok(())
}
