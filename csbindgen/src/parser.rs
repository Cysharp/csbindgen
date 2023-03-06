use crate::{alias_map::AliasMap, builder::BindgenOptions, field_map::FieldMap, type_meta::*};
use std::collections::HashSet;
use syn::{ForeignItem, Item, Pat, ReturnType};

enum FnItem {
    ForeignItem(syn::ForeignItemFn),
    Item(syn::ItemFn),
}

pub fn collect_foreign_method(ast: &syn::File, options: &BindgenOptions) -> Vec<ExternMethod> {
    let mut list: Vec<ExternMethod> = Vec::new();

    for item in ast.items.iter() {
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

    list
}

pub fn collect_extern_method(ast: &syn::File, options: &BindgenOptions) -> Vec<ExternMethod> {
    let mut list: Vec<ExternMethod> = Vec::new();

    for item in ast.items.iter() {
        if let Item::Fn(m) = item {
            if let Some(_) = &m.sig.abi {
                // has extern
                let method = parse_method(FnItem::Item(m.clone()), options);
                if let Some(x) = method {
                    list.push(x);
                }
            }
        }
    }

    list
}

fn parse_method(item: FnItem, options: &BindgenOptions) -> Option<ExternMethod> {
    let (sig, attrs) = match item {
        FnItem::ForeignItem(x) => (x.sig, x.attrs),
        FnItem::Item(x) => (x.sig, x.attrs),
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
                println!("Csbindgen can't handle this parameter type so ignore generate, method_name: {} parameter_name: {}", method_name, parameter_name);
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
                "Csbindgen can't handle this return type so ignore generate, method_name: {}",
                method_name
            );
            return None;
        }

        return_type = Some(rust_type);
    }

    // doc
    let mut doc_comment = None;
    for attr in attrs {
        let last_segment = attr.path.segments.last().unwrap();
        if last_segment.ident.to_string() == "doc" {
            doc_comment = Some(attr.tokens.to_string());
        }
    }

    if !method_name.is_empty() && (options.method_filter)(method_name.clone()) {
        return Some(ExternMethod {
            method_name,
            parameters,
            return_type,
            doc_comment,
        });
    }

    None
}

pub fn collect_type_alias(ast: &syn::File) -> AliasMap {
    let mut result = AliasMap::new();
    for item in ast.items.iter() {
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

    result
}

pub fn collect_struct(ast: &syn::File) -> Vec<RustStruct> {
    // collect union or struct
    let mut result = Vec::new();

    for item in ast.items.iter() {
        if let Item::Union(t) = item {
            let struct_name = t.ident.to_string();
            let fields = collect_fields(&t.fields);

            result.push(RustStruct {
                struct_name,
                fields,
                is_union: true,
            });
        } else if let Item::Struct(t) = item {
            if let syn::Fields::Named(f) = &t.fields {
                let struct_name = t.ident.to_string();
                let fields = collect_fields(f);
                result.push(RustStruct {
                    struct_name,
                    fields,
                    is_union: false,
                });
            };
        }
    }

    result
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

pub fn collect_enum(ast: &syn::File) -> Vec<RustEnum> {
    let mut result = Vec::new();

    for item in ast.items.iter() {
        if let Item::Enum(t) = item {
            let mut repr = None;
            for attr in &t.attrs {
                let last_segment = attr.path.segments.last().unwrap();
                if last_segment.ident.to_string() == "repr" {
                    repr = Some(attr.tokens.to_string());
                }
            }

            let enum_name = t.ident.to_string();
            let mut fields = Vec::new(); // Vec<(String, Option<String>)>

            for v in &t.variants {
                let name = v.ident.to_string();
                let mut value = None;
                if let Some((_, expr)) = &v.discriminant {
                    if let syn::Expr::Lit(x) = expr {
                        if let syn::Lit::Int(x) = &x.lit {
                            let digits = x.base10_digits().to_string();
                            value = Some(digits);
                        }
                    };
                }

                fields.push((name, value));
            }

            result.push(RustEnum {
                enum_name,
                fields,
                repr,
            });
        }
    }

    result
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
            let has_const = t.const_token.is_some();
            // let has_mut = t.mutability.is_some();

            if let syn::Type::Path(path) = &*t.elem {
                return RustType {
                    type_name: path.path.segments.last().unwrap().ident.to_string(),
                    type_kind: TypeKind::Pointer(if has_const {
                        PointerType::ConstPointer
                    } else {
                        PointerType::MutPointer
                    }),
                };
            } else if let syn::Type::Ptr(t) = &*t.elem {
                if let syn::Type::Path(path) = &*t.elem {
                    return RustType {
                        type_name: path.path.segments.last().unwrap().ident.to_string(),
                        type_kind: TypeKind::Pointer(if has_const {
                            PointerType::ConstPointerPointer
                        } else {
                            PointerType::MutPointerPointer
                        }),
                    };
                }
            }
        }
        syn::Type::Path(t) => {
            let last_segment = t.path.segments.last().unwrap();
            if let syn::PathArguments::AngleBracketed(x) = &last_segment.arguments {
                // generics, only supports Option<> for null function pointer
                if last_segment.ident.to_string() == "Option" {
                    if let Some(x) = x.args.first() {
                        if let syn::GenericArgument::Type(t) = x {
                            let rust_type = parse_type(t);
                            return RustType {
                                type_name: "Option".to_string(),
                                type_kind: TypeKind::Option(Box::new(rust_type)),
                            };
                        }
                    }
                }
            } else {
                return RustType {
                    type_name: last_segment.ident.to_string(),
                    type_kind: TypeKind::Normal,
                };
            }
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
            if t.elems.len() == 0 {
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
                syn::ReturnType::Type(_, t) => Some(Box::new(parse_type(&t))),
            };

            return RustType {
                type_name: "extern \"C\" fn".to_string(),
                type_kind: TypeKind::Function(parameters, ret),
            };
        }
        _ => {}
    };

    // type_name = "" will ignore in collect method
    RustType {
        type_name: "".to_string(),
        type_kind: TypeKind::Normal,
    }
}
