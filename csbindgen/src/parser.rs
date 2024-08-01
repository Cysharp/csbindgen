use crate::type_meta::ExportSymbolNaming::{ExportName, NoMangle};
use crate::util::get_str_from_meta;
use crate::{alias_map::AliasMap, builder::BindgenOptions, field_map::FieldMap, type_meta::*};
use regex::Regex;
use std::collections::HashSet;
use syn::{ForeignItem, Item, Pat, ReturnType};

enum FnItem {
    ForeignItem(syn::ForeignItemFn),
    Item(syn::ItemFn),
}

/// build a Vec of all Items, unless the Item is a Item::Mod, then append the Item contents of the vect
/// Do this recursively.
/// This is not memory-efficient, would work better with an iterator, but does not seem performance critical.
fn depth_first_module_walk(ast: &Vec<Item>) -> Vec<&syn::Item> {
    let mut unwrapped_items: Vec<&syn::Item> = vec![];
    for item in ast {
        if let Item::Mod(m) = item {
            if let Some((_, items)) = &m.content {
                unwrapped_items.extend(depth_first_module_walk(items));
            }
        } else {
            unwrapped_items.push(item);
        }
    }

    unwrapped_items
}

pub fn collect_foreign_method(
    ast: &syn::File,
    options: &BindgenOptions,
    list: &mut Vec<ExternMethod>,
) {
    for item in depth_first_module_walk(&ast.items) {
        if let Item::ForeignMod(m) = item {
            for item in m.items.iter() {
                if let ForeignItem::Fn(m) = item {
                    let method = parse_method(FnItem::ForeignItem(m.clone()), options);
                    if let Some(x) = method {
                        list.push(x);
                    }
                }
            }
        }
    }
}

pub fn collect_extern_method(
    ast: &syn::File,
    options: &BindgenOptions,
    list: &mut Vec<ExternMethod>,
) {
    for item in depth_first_module_walk(&ast.items) {
        if let Item::Fn(m) = item {
            // has extern
            if m.sig.abi.is_some() {
                let method = parse_method(FnItem::Item(m.clone()), options);
                if let Some(x) = method {
                    list.push(x);
                }
            }
        }
    }
}

fn parse_method(item: FnItem, options: &BindgenOptions) -> Option<ExternMethod> {
    let (sig, attrs, is_foreign_item) = match item {
        FnItem::ForeignItem(x) => (x.sig, x.attrs, true),
        FnItem::Item(x) => (x.sig, x.attrs, false),
    };

    let method_name = sig.ident.to_string();

    let mut parameters: Vec<Parameter> = Vec::new();
    let mut return_type: Option<RustType> = None;

    // argument
    for arg in sig.inputs.iter() {
        if let syn::FnArg::Typed(t) = arg {
            let mut parameter_name: String = "".to_string();

            if let Pat::Ident(ident) = &*t.pat {
                parameter_name = ident.ident.to_string();
            }

            let rust_type = parse_type(&t.ty);
            if rust_type.type_name.is_empty() {
                println!("csbindgen can't handle this parameter type so ignore generate, method_name: {} parameter_name: {}", method_name, parameter_name);
                return None;
            }

            parameters.push(Parameter {
                name: parameter_name,
                rust_type,
            });
        }
    }

    // return
    if let ReturnType::Type(_, b) = &sig.output {
        let rust_type = parse_type(b);
        if rust_type.type_name.is_empty() {
            println!(
                "csbindgen can't handle this return type so ignore generate, method_name: {}",
                method_name
            );
            return None;
        }

        return_type = Some(rust_type);
    }

    // check attrs
    let mut export_naming = NoMangle;
    if !is_foreign_item {
        let found = attrs
            .iter()
            .map(|attr| {
                let name = &attr.path().segments.last().unwrap().ident;
                if name == "no_mangle" {
                    return Some(NoMangle);
                } else if name == "export_name" {
                    if let Some(x) = get_str_from_meta(&attr.meta) {
                        return Some(ExportName(x));
                    }
                }
                None
            })
            .next();

        if let Some(x) = found.flatten() {
            export_naming = x;
        } else {
            println!(
                "csbindgen can't handle this function because there is neither #[no_mangle] nor #[export_name] so ignore generate, method_name: {}",
                method_name
            );
            return None;
        }
    }

    // doc
    let doc_comment = attrs
        .iter()
        .filter(|x| x.path().is_ident("doc"))
        .filter_map(|x| get_str_from_meta(&x.meta))
        .collect::<Vec<_>>();

    if !method_name.is_empty() && (options.method_filter)(method_name.clone()) {
        return Some(ExternMethod {
            method_name,
            export_naming,
            parameters,
            return_type,
            doc_comment,
        });
    }

    None
}

pub fn collect_type_alias(ast: &syn::File, result: &mut AliasMap) {
    for item in depth_first_module_walk(&ast.items) {
        if let Item::Type(t) = item {
            let name = t.ident.to_string();
            let alias = parse_type(&t.ty);
            result.insert(&name, &alias);
        } else if let Item::Use(t) = item {
            if let syn::UseTree::Path(t) = &t.tree {
                if let syn::UseTree::Rename(t) = &*t.tree {
                    let name = t.rename.to_string();
                    let alias = t.ident.to_string();
                    result.insert(
                        &name,
                        &RustType {
                            type_name: alias,
                            type_kind: TypeKind::Normal,
                        },
                    );
                }
            }
        }
    }
}

pub fn collect_struct(ast: &syn::File, result: &mut Vec<RustStruct>) {
    // collect union or struct
    for item in depth_first_module_walk(&ast.items) {
        if let Item::Union(t) = item {
            let struct_name = t.ident.to_string();
            let fields = collect_fields(&t.fields);

            result.push(RustStruct {
                struct_name,
                fields,
                is_union: true,
            });
        } else if let Item::Struct(t) = item {
            let mut repr = false;
            for attr in &t.attrs {
                let last_segment = attr.path().segments.last().unwrap();
                if last_segment.ident == "repr" {
                    repr = true;
                }
            }

            if repr {
                if let syn::Fields::Named(f) = &t.fields {
                    let struct_name = t.ident.to_string();
                    let fields = collect_fields(f);
                    result.push(RustStruct {
                        struct_name,
                        fields,
                        is_union: false,
                    });
                } else if let syn::Fields::Unnamed(f) = &t.fields {
                    let struct_name = t.ident.to_string();
                    let fields = collect_fields_unnamed(f);
                    result.push(RustStruct {
                        struct_name,
                        fields,
                        is_union: false,
                    });
                } else if let syn::Fields::Unit = &t.fields {
                    let struct_name = t.ident.to_string();
                    let fields: Vec<FieldMember> = Vec::new();
                    result.push(RustStruct {
                        struct_name,
                        fields,
                        is_union: false,
                    });
                }
            } else {
                // non #[repr(?)] struct, treat as Unit struct
                let struct_name = t.ident.to_string();
                let fields: Vec<FieldMember> = Vec::new();
                result.push(RustStruct {
                    struct_name,
                    fields,
                    is_union: false,
                });
            }
        }
    }
}

fn collect_fields(fields: &syn::FieldsNamed) -> Vec<FieldMember> {
    let mut result = Vec::new();

    for field in &fields.named {
        if let Some(x) = &field.ident {
            let t = parse_type(&field.ty);
            result.push(FieldMember {
                name: x.to_string(),
                rust_type: t,
            });
        }
    }

    result
}

fn collect_fields_unnamed(fields: &syn::FieldsUnnamed) -> Vec<FieldMember> {
    let mut result = Vec::new();

    let mut i = 0;
    for field in &fields.unnamed {
        i += 1;
        let name = format!("Item{i}");
        let t = parse_type(&field.ty);
        result.push(FieldMember { name, rust_type: t });
    }

    result
}

pub fn collect_const(
    ast: &syn::File,
    result: &mut Vec<RustConst>,
    filter: fn(const_name: &str) -> bool,
) {
    for item in depth_first_module_walk(&ast.items) {
        if let Item::Const(ct) = item {
            // pub const Ident: ty = expr
            let const_name = ct.ident.to_string();
            if filter(const_name.as_str()) {
                let t = parse_type(&ct.ty);

                if let syn::Expr::Lit(lit_expr) = &*ct.expr {
                    let value = match &lit_expr.lit {
                        syn::Lit::Str(s) => {
                            format!("\"{}\"", s.value())
                        }
                        syn::Lit::ByteStr(bs) => {
                            format!("{:?}", bs.value())
                        }
                        syn::Lit::Byte(b) => {
                            format!("{}", b.value())
                        }
                        syn::Lit::Char(c) => {
                            format!("'{}'", c.value())
                        }
                        syn::Lit::Int(i) => {
                            format!("{}", i.base10_parse::<i64>().unwrap())
                        }
                        syn::Lit::Float(f) => {
                            format!("{}", f.base10_parse::<f64>().unwrap())
                        }
                        syn::Lit::Bool(b) => {
                            format!("{}", b.value)
                        }
                        _ => String::new(),
                    };

                    result.push(RustConst {
                        const_name,
                        rust_type: t,
                        value,
                    });
                }
            }
        }
    }
}

pub fn collect_enum(ast: &syn::File, result: &mut Vec<RustEnum>) {
    for item in depth_first_module_walk(&ast.items) {
        if let Item::Enum(t) = item {
            let mut repr = None;
            for attr in &t.attrs {
                let last_segment = attr.path().segments.last().unwrap();
                if last_segment.ident == "repr" {
                    attr.parse_nested_meta(|meta| {
                        repr = meta.path.get_ident().map(|ident| ident.to_string());
                        Ok(())
                    })
                    .unwrap();
                }
            }

            let enum_name = t.ident.to_string();
            let mut fields = Vec::new(); // Vec<(String, Option<String>)>

            if t.variants
                .iter()
                .any(|x| !matches!(x.fields, syn::Fields::Unit))
            {
                println!("csbindgen can't handle Enum containing any variable with field, so ignore generate, enum_name: {enum_name}");
                continue;
            }

            for v in &t.variants {
                let name = v.ident.to_string();
                let mut value = None;

                match &v.discriminant {
                    Some((_, syn::Expr::Lit(x))) => {
                        if let syn::Lit::Int(x) = &x.lit {
                            let digits = x.base10_digits().to_string();
                            value = Some(digits);
                        }
                    }
                    Some((_, syn::Expr::Unary(u))) if matches!(u.op, syn::UnOp::Neg(_)) => {
                        if let syn::Expr::Lit(x) = &*u.expr {
                            if let syn::Lit::Int(x) = &x.lit {
                                value = Some(format!("-{}", x.base10_digits()));
                            }
                        }
                    }
                    _ => (),
                }

                fields.push((name, value));
            }

            result.push(RustEnum {
                enum_name,
                fields,
                repr,
                is_flags: false,
            });
        } else if let Item::Macro(t) = item {
            let last_segment = t.mac.path.segments.last().unwrap();
            if last_segment.ident == "bitflags" {
                // bitflags parsing template:
                // $(#[$outer:meta])*
                // $vis:vis struct $BitFlags:ident: $T:ty {
                //     $(
                //         $(#[$inner:ident $($args:tt)*])*
                //         const $Flag:ident = $value:expr;
                //     )*
                // }

                let token_string = t.mac.tokens.to_string();

                let match1 = Regex::new("struct ([^ ]+) : ([^ ]+)")
                    .unwrap()
                    .captures(token_string.as_str())
                    .unwrap();

                let enum_name = match1.get(1).unwrap().as_str().to_string();
                let repr = Some(match1.get(2).unwrap().as_str().to_string());

                let fields = Regex::new("const ([^ ]+) = ([^;]+)[ ]*;")
                    .unwrap()
                    .captures_iter(token_string.as_str())
                    .map(|x| {
                        (
                            x.get(1).unwrap().as_str().to_string(),
                            Some(
                                x.get(2)
                                    .unwrap()
                                    .as_str()
                                    .to_string()
                                    .replace("Self :: ", "")
                                    .replace(" . bits ()", "")
                                    .replace(" . bits", "")
                                    .trim()
                                    .to_string(),
                            ),
                        )
                    })
                    .collect::<Vec<_>>();

                result.push(RustEnum {
                    enum_name,
                    fields,
                    repr,
                    is_flags: true,
                });
            }
        }
    }
}

pub fn reduce_struct(
    structs: &Vec<RustStruct>,
    field_map: &FieldMap,
    using_types: &HashSet<String>,
) -> Vec<RustStruct> {
    let mut result = Vec::new();
    for item in structs {
        if field_map.exists_in_using_types(&item.struct_name, using_types, 0) {
            result.push(item.clone());
        }
    }

    result
}

pub fn reduce_enum(
    enums: &Vec<RustEnum>,
    field_map: &FieldMap,
    using_types: &HashSet<String>,
) -> Vec<RustEnum> {
    let mut result = Vec::new();
    for item in enums {
        if field_map.exists_in_using_types(&item.enum_name, using_types, 0) {
            result.push(item.clone());
        }
    }

    result
}

fn parse_type(t: &syn::Type) -> RustType {
    match t {
        syn::Type::Ptr(t) => {
            let has_const = t.const_token.is_some(); // not is has_mut

            if let syn::Type::Path(path) = &*t.elem {
                return RustType {
                    type_name: path.path.segments.last().unwrap().ident.to_string(),
                    type_kind: TypeKind::Pointer(
                        if has_const {
                            PointerType::ConstPointer
                        } else {
                            PointerType::MutPointer
                        },
                        Box::new(parse_type_path(path)),
                    ),
                };
            } else if let syn::Type::Ptr(t) = &*t.elem {
                if let syn::Type::Path(path) = &*t.elem {
                    let has_const2 = t.const_token.is_some();

                    let pointer_type = match (has_const, has_const2) {
                        (true, true) => PointerType::ConstPointerPointer,
                        (true, false) => PointerType::ConstMutPointerPointer,
                        (false, true) => PointerType::MutConstPointerPointer,
                        (false, false) => PointerType::MutPointerPointer,
                    };

                    return RustType {
                        type_name: path.path.segments.last().unwrap().ident.to_string(),
                        type_kind: TypeKind::Pointer(pointer_type, Box::new(parse_type_path(path))),
                    };
                }
            }
        }
        syn::Type::Path(t) => {
            return parse_type_path(t);
        }
        syn::Type::Array(t) => {
            let mut digits = "".to_string();
            if let syn::Expr::Lit(x) = &t.len {
                if let syn::Lit::Int(x) = &x.lit {
                    digits = x.base10_digits().to_string();
                }
            };

            let type_name = parse_type(&t.elem).type_name; // maybe ok, only retrieve type_name
            return RustType {
                type_name,
                type_kind: TypeKind::FixedArray(digits, None),
            };
        }
        syn::Type::Tuple(t) => {
            if t.elems.is_empty() {
                return RustType {
                    type_name: "()".to_string(),
                    type_kind: TypeKind::Normal,
                };
            };
        }
        syn::Type::BareFn(t) => {
            let mut parameters = Vec::new();

            for arg in t.inputs.iter() {
                let rust_type = parse_type(&arg.ty);

                let name = if let Some((ident, _)) = &arg.name {
                    ident.to_string()
                } else {
                    "".to_string()
                };
                parameters.push(Parameter { name, rust_type });
            }

            let ret = match &t.output {
                syn::ReturnType::Default => None,
                syn::ReturnType::Type(_, t) => Some(Box::new(parse_type(t))),
            };

            return RustType {
                type_name: "unsafe extern \"C\" fn".to_string(),
                type_kind: TypeKind::Function(parameters, ret),
            };
        }
        syn::Type::Reference(t) => {
            let result = parse_type(&t.elem);
            let is_mut = t.mutability.is_some();

            match result {
                RustType {
                    type_kind: TypeKind::Pointer(pt, _),
                    ..
                } => match pt {
                    PointerType::ConstPointer | PointerType::MutPointer => {
                        return RustType {
                            type_name: result.type_name,
                            type_kind: TypeKind::Pointer(
                                if is_mut {
                                    PointerType::MutPointer
                                } else {
                                    PointerType::ConstPointer
                                },
                                Box::new(parse_type(&t.elem)),
                            ),
                        };
                    }
                    PointerType::ConstPointerPointer | PointerType::MutConstPointerPointer => {
                        return RustType {
                            type_name: result.type_name,
                            type_kind: TypeKind::Pointer(
                                if is_mut {
                                    PointerType::MutConstPointerPointer
                                } else {
                                    PointerType::ConstPointerPointer
                                },
                                Box::new(parse_type(&t.elem)),
                            ),
                        };
                    }
                    PointerType::ConstMutPointerPointer | PointerType::MutPointerPointer => {
                        return RustType {
                            type_name: result.type_name,
                            type_kind: TypeKind::Pointer(
                                if is_mut {
                                    PointerType::MutPointerPointer
                                } else {
                                    PointerType::ConstMutPointerPointer
                                },
                                Box::new(parse_type(&t.elem)),
                            ),
                        };
                    }
                    _ => {}
                },
                _ => {
                    // &*t.elem is not pointer
                    return RustType {
                        type_name: result.type_name,
                        type_kind: TypeKind::Pointer(
                            PointerType::ConstPointer,
                            Box::new(parse_type(&t.elem)),
                        ),
                    };
                }
            }
        }
        _ => {}
    };

    // type_name = "" will ignore in collect method
    RustType {
        type_name: "".to_string(),
        type_kind: TypeKind::Normal,
    }
}

fn parse_type_path(t: &syn::TypePath) -> RustType {
    let last_segment = t.path.segments.last().unwrap();
    if let syn::PathArguments::AngleBracketed(x) = &last_segment.arguments {
        // generics
        if let Some(syn::GenericArgument::Type(t)) = x.args.first() {
            let rust_type = parse_type(t);
            if last_segment.ident == "Option" {
                return RustType {
                    type_name: "Option".to_string(),
                    type_kind: TypeKind::Option(Box::new(rust_type)),
                };
            } else if last_segment.ident == "NonNull" {
                return RustType {
                    type_name: "NonNull".to_string(),
                    type_kind: TypeKind::Pointer(PointerType::NonNull, Box::new(rust_type)),
                };
            } else if last_segment.ident == "Box" {
                return RustType {
                    type_name: "Box".to_string(),
                    type_kind: TypeKind::Pointer(PointerType::Box, Box::new(rust_type)),
                };
            }
        }
    }

    RustType {
        type_name: last_segment.ident.to_string(),
        type_kind: TypeKind::Normal,
    }
}
