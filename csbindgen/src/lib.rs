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
use type_meta::{ExternMethod, RustEnum, RustStruct, RustType};

enum GenerateKind {
    InputBindgen,
    InputExtern,
}

pub(crate) fn generate(
    generate_kind: GenerateKind,
    options: &BindgenOptions,
) -> Result<(Option<String>, String), Box<dyn Error>> {
    let (paths, generate_rust) = match generate_kind {
        GenerateKind::InputBindgen => (options.input_bindgen_files.as_slice(), true),
        GenerateKind::InputExtern => (options.input_extern_files.as_slice(), false),
    };

    let mut methods: Vec<ExternMethod> = vec![];
    let mut aliases = AliasMap::new();
    let mut structs: Vec<RustStruct> = vec![];
    let mut enums: Vec<RustEnum> = vec![];

    for path in paths {
        let file_content = std::fs::read_to_string(path)
            .expect(&format!("input file not found, path: {}", path.display()));
        let file_ast = syn::parse_file(file_content.as_str())?;

        match generate_kind {
            GenerateKind::InputBindgen => collect_foreign_method(&file_ast, options, &mut methods),
            GenerateKind::InputExtern => collect_extern_method(&file_ast, options, &mut methods),
        };
        collect_type_alias(&file_ast, &mut aliases);
        collect_struct(&file_ast, &mut structs);
        collect_enum(&file_ast, &mut enums);
    }

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
            let (struct_type_normalized, _) = aliases.normalize(&struct_type.struct_name);
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
        let (normalized, normalized_rust_type) = aliases.normalize(&rust_type.type_name);
        if let Some(x) = normalized_rust_type {
            collect_using_types(using_types, aliases, &x);
        } else {
            using_types.insert(normalized.clone());
        }
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
        let (normalized, normalized_rust_type) = aliases.normalize(&rust_type.type_name);
        if let Some(x) = normalized_rust_type {
            collect_field_types(field_map, aliases, struct_type_normalized, &x);
        } else {
            field_map.insert(struct_type_normalized, &normalized);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::{
        env,
        fs::{self},
        io::Write,
    };

    use regex::Regex;

    use super::*;

    #[test]
    fn test() {
        let path = std::env::current_dir().unwrap();
        println!("starting dir: {}", path.display()); // csbindgen/csbindgen

        Builder::new()
            .input_bindgen_file("csbindgen-tests/src/liblz4.rs")
            .csharp_class_name("LibLz4")
            .csharp_dll_name("csbindgen_tests")
            .generate_to_file(
                "csbindgen-tests/src/lz4_ffi.rs",
                "dotnet-sandbox/lz4_bindgen.cs",
            )
            .unwrap();
    }

    // cargo test update_package_version -- 1.0.0 --nocapture
    #[test]
    fn update_package_version() {
        let args: Vec<String> = env::args().collect();
        // 0: exe path
        // 1: update_package_version
        // 2: 1.0.0
        // 3: --nocapture

        if args[1] != "update_package_version" {
            return;
        }

        println!("version: {}", args[2]);
        let mut path = std::env::current_dir().unwrap();
        println!("current_dir: {}", path.display());

        path.push("Cargo.toml");
        let toml = fs::read_to_string(path.clone()).unwrap();

        // replace only first-match
        let regex = Regex::new("version = \".+\"").unwrap();

        let new_toml = regex.replace(toml.as_str(), format!("version = \"{}\"", args[2]));

        let mut file = fs::File::create(path.clone()).unwrap();
        file.write_all(new_toml.as_bytes()).unwrap();
        file.flush().unwrap();
    }
}
