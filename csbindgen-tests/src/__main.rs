use std::env;

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
mod lz4;

fn main() {
    let path = env::current_dir().unwrap();
    println!("starting dir: {}", path.display());

    unsafe {
        let num = lz4::LZ4_versionNumber();
        println!("lz4 num: {}", num);
    }

    csbindgen::run(
        "csbindgen-tests/src/lz4/mod.rs",
        "csbindgen-tests/src/ffi.rs",
        "dotnet-sandbox/bindgen.cs",
    );
}
