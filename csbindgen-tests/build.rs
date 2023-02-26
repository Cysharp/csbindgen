#![feature(core_intrinsics)]
fn main() {
    let bindings = bindgen::Builder::default()
        .header("c/lz4/lz4.h")
        .header("c/lz4/lz4hc.h")
        .header("c/lz4/lz4frame.h")
        .header("c/lz4/xxhash.h")
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file("src/lz4/mod.rs")
        .expect("Couldn't write bindings!");

    cc::Build::new().file("c/lz4/lz4.c").compile("lz4");

    // TODO:write test
    csbindgen::run(
        "src/lz4/mod.rs",
        "src/ffi.rs",
        "../dotnet-sandbox/bindgen.cs",
    )
}
