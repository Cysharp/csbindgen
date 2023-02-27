fn main() {
    let bindings = bindgen::Builder::default()
        .header("c/lz4/lz4.h")
        //.header("c/lz4/lz4hc.h")
        //.header("c/lz4/lz4frame.h")
        //.header("c/lz4/xxhash.h")
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file("src/lz4/mod.rs")
        .expect("Couldn't write bindings!");

    cc::Build::new().file("c/lz4/lz4.c").compile("csharp_lz4");

    csbindgen::Builder::new()
        .input_bindgen_file("src/lz4/mod.cs")
        .rust_method_prefix("csbindgen_")
        .generate_to_file("src/ffi.rs", "../dotnet-sandbox/bindgen.cs")
        .unwrap();
}
