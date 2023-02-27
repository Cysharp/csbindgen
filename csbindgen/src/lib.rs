mod builder;
mod emitter;
mod parser;
mod type_meta;
mod util;

pub use builder::Builder;

use builder::BindgenOptions;
use emitter::*;
use parser::*;
use std::{collections::HashSet, error::Error};

pub(crate) fn generate(options: &BindgenOptions) -> Result<(String, String), Box<dyn Error>> {
    let path = &options.input_bindgen_file;
    let file_content = std::fs::read_to_string(path)?;
    let file_ast = syn::parse_file(file_content.as_str())?;

    let methods = collect_method(&file_ast);
    let aliases = collect_type_alias(&file_ast);
    let structs = collect_struct(&file_ast);

    let mut using_types = HashSet::new();
    for method in &methods {
        if let Some(x) = &method.return_type {
            using_types.insert(x.type_name.clone());
        }
        for p in &method.parameters {
            using_types.insert(p.rust_type.type_name.clone());
        }
    }
    for item in &structs {
        for item in &item.fields {
            using_types.insert(item.rust_type.type_name.clone());
        }
    }

    let aliases = reduce_type_alias(&aliases, &using_types);
    for alias in &aliases {
        using_types.insert(alias.1.type_name.clone());
    }

    let structs = reduce_struct(&structs, &using_types);

    let rust = emit_rust_method(&methods, &options);
    let csharp = emit_csharp(&methods, &aliases, &structs, &options);

    return Ok((rust, csharp));
}

// #[test]
// fn test() {
//     let path = std::env::current_dir().unwrap();
//     println!("starting dir: {}", path.display()); // csbindgen/csbindgen

//     Builder::new()
//         .input_bindgen_file("../csbindgen-tests/src/quiche.rs")
//         .rust_method_prefix("csbindgen_quiche_")
//         .csharp_class_name("LibQuiche")
//         .csharp_dll_name("libquiche")
//         .generate_to_file("../csbindgen-tests/src/quiche_ffi.rs", "../csbindgen-tests/../dotnet-sandbox/quiche_bindgen.cs")
//         .unwrap();
// }
