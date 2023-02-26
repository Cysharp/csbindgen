use std::env;

fn main() {

    let path = env::current_dir().unwrap();
    println!("starting dir: {}", path.display());
    
     csbindgen::run(
         "csbindgen-tests/src/lz4/mod.rs",
         "csbindgen-tests/src/ffi.rs",
         "dotnet-sandbox/bindgen.cs",
     );
}
