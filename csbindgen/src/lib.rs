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
use type_meta::{ExternMethod, RustConst, RustEnum, RustStruct, RustType};

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
    let mut consts: Vec<RustConst> = vec![];

    for path in paths {
        let file_content = std::fs::read_to_string(path)
            .unwrap_or_else(|_| panic!("input file not found, path: {}", path.display()));
        let file_ast = syn::parse_file(file_content.as_str())?;

        match generate_kind {
            GenerateKind::InputBindgen => collect_foreign_method(&file_ast, options, &mut methods),
            GenerateKind::InputExtern => collect_extern_method(&file_ast, options, &mut methods),
        };
        collect_type_alias(&file_ast, &mut aliases);
        collect_struct(&file_ast, &mut structs);
        collect_enum(&file_ast, &mut enums);

        collect_const(&file_ast, &mut consts, options.csharp_generate_const_filter);
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

    using_types.extend(options.always_included_types.iter().cloned());

    let structs = reduce_struct(&structs, &field_map, &using_types);
    let enums = reduce_enum(&enums, &field_map, &using_types);

    let rust = if generate_rust {
        Some(emit_rust_method(&methods, options))
    } else {
        None
    };
    let csharp = emit_csharp(&methods, &aliases, &structs, &enums, &consts, options);

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

        std::env::set_current_dir(path.parent().unwrap()).unwrap();

        Builder::new()
            .input_bindgen_file("csbindgen-tests/src/lz4.rs")
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

        if args.len() < 2 || args[1] != "update_package_version" {
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

    fn compare_and_delete_files(original_file_path: &str, generated_file_path: &str) {
        let original = fs::read_to_string(original_file_path)
            .expect("Should have been able to read original file");

        let generated = fs::read_to_string(generated_file_path)
            .expect("Should have been able to read generated file");

        assert_eq!(original, generated);

        fs::remove_file(generated_file_path).unwrap();
    }

    // #[test]
    // fn test_emit_without_class() {
    //     let generated_file_path = "dotnet-sandbox/only_enums_and_structs_bindgen.cs";
    //     Builder::new()
    //         .always_included_types(["Vec3", "Foo"])
    //         .input_bindgen_file("csbindgen-tests/src/only_enums_and_structs.rs")
    //         .generate_csharp_file(generated_file_path)
    //         .unwrap();

    //     compare_and_delete_files(
    //         "dotnet-sandbox/only_enums_and_structs_original.cs",
    //         generated_file_path,
    //     );
    // }
}
