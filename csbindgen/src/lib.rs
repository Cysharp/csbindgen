mod alias_map;
mod builder;
mod emitter;
mod field_map;
mod parser;
mod type_meta;
mod util;

use alias_map::AliasMap;
pub use builder::Builder;

use builder::BindgenOptions;
use emitter::*;
use field_map::FieldMap;
use parser::*;
use std::{collections::HashSet, error::Error};
use type_meta::RustType;

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

    // collect using_types
    let mut using_types = HashSet::new();
    for method in &methods {
        // add to using_types with normalize
        if let Some(x) = &method.return_type {
            collect_using_types(&mut using_types, &aliases, x);
        }
        for p in &method.parameters {
            collect_using_types(&mut using_types, &aliases, &p.rust_type);
        }
    }

    let mut field_map = FieldMap::new();
    for struct_type in &structs {
        for field in &struct_type.fields {
            let struct_type_normalized = aliases.normalize(&struct_type.struct_name);
            collect_field_types(
                &mut field_map,
                &aliases,
                &struct_type_normalized,
                &field.rust_type,
            );
        }
    }

    let structs = reduce_struct(&structs, &field_map, &using_types);
    let enums = reduce_enum(&enums, &field_map, &using_types);

    let rust = if generate_rust {
        Some(emit_rust_method(&methods, options))
    } else {
        None
    };
    let csharp = emit_csharp(&methods, &aliases, &structs, &enums, options);

    Ok((rust, csharp))
}

fn collect_using_types(
    using_types: &mut HashSet<String>,
    aliases: &AliasMap,
    rust_type: &RustType,
) {
    if let type_meta::TypeKind::Option(o) = &rust_type.type_kind {
        collect_using_types(using_types, aliases, o);
    } else if let type_meta::TypeKind::Function(parameters, return_type) = &rust_type.type_kind {
        if let Some(x) = &return_type {
            collect_using_types(using_types, aliases, x);
        }
        for p in parameters {
            collect_using_types(using_types, aliases, &p.rust_type);
        }
    } else {
        let normalized = aliases.normalize(&rust_type.type_name);
        using_types.insert(normalized.clone());
    }
}

fn collect_field_types(
    field_map: &mut FieldMap,
    aliases: &AliasMap,
    struct_type_normalized: &String,
    rust_type: &RustType,
) {
    if let type_meta::TypeKind::Option(o) = &rust_type.type_kind {
        collect_field_types(field_map, aliases, struct_type_normalized, o);
    } else if let type_meta::TypeKind::Function(parameters, return_type) = &rust_type.type_kind {
        if let Some(x) = &return_type {
            collect_field_types(field_map, aliases, struct_type_normalized, x);
        }
        for p in parameters {
            collect_field_types(field_map, aliases, struct_type_normalized, &p.rust_type);
        }
    } else {
        let normalized = aliases.normalize(&rust_type.type_name);
        field_map.insert(struct_type_normalized, &normalized);
    }
}

// #[test]
// fn test() {
//     let path = std::env::current_dir().unwrap();
//     println!("starting dir: {}", path.display()); // csbindgen/csbindgen

//     Builder::new()
//         .input_bindgen_file("csbindgen-tests/src/lz4.rs")
//         .csharp_class_name("LibLz4")
//         .csharp_dll_name("csbindgen_tests")
//         .generate_to_file(
//             "csbindgen-tests/src/lz4_ffi.rs",
//             "dotnet-sandbox/lz4_bindgen.cs",
//         )
//         .unwrap();
// }
