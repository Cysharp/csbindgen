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

enum GenerateKind {
    InputBindgen,
    InputExtern,
}

pub(crate) fn generate(
    generate_kind: GenerateKind,
    options: &BindgenOptions,
) -> Result<(Option<String>, String), Box<dyn Error>> {
    let path = match generate_kind {
        GenerateKind::InputBindgen => &options.input_bindgen_file,
        GenerateKind::InputExtern => &options.input_extern_file,
    };
    let file_content = std::fs::read_to_string(path)
        .expect(("input file not found, path:".to_string() + path).as_str());
    let file_ast = syn::parse_file(file_content.as_str())?;

    let (methods, generate_rust) = match generate_kind {
        GenerateKind::InputBindgen => (collect_foreign_method(&file_ast, options), true),
        GenerateKind::InputExtern => (collect_extern_method(&file_ast, options), false),
    };
    let aliases = collect_type_alias(&file_ast);
    let structs = collect_struct(&file_ast);
    let enums = collect_enum(&file_ast);

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
    let enums = reduce_enum(&enums, &using_types);

    let rust = if generate_rust {
        Some(emit_rust_method(&methods, options))
    } else {
        None
    };
    let csharp = emit_csharp(&methods, &aliases, &structs, &enums, options);

    Ok((rust, csharp))
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
