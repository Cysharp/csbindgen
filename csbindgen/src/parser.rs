use crate::{builder::BindgenOptions, type_meta::*};
use std::collections::{HashMap, HashSet};
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
    let sig = match item {
        FnItem::ForeignItem(x) => x.sig,
        FnItem::Item(x) => x.sig,
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
            if  rust_type.type_name.is_empty(){
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
            println!("Csbindgen can't handle this return type so ignore generate, method_name: {}", method_name);
            return None;
        }

        return_type = Some(rust_type);
    }

    if !method_name.is_empty() && (options.method_filter)(method_name.clone()) {
        return Some(ExternMethod {
            method_name,
            parameters,
            return_type,
        });
    }

    None
}

pub fn collect_type_alias(ast: &syn::File) -> Vec<(String, RustType)> {
    let mut result = Vec::new();
    for item in ast.items.iter() {
        if let Item::Type(t) = item {
            let name = t.ident.to_string();
            let alias = parse_type(&t.ty);
            result.push((name, alias));
        } else if let Item::Use(t) = item {
            if let syn::UseTree::Path(t) = &t.tree {
                if let syn::UseTree::Rename(t) = &*t.tree {
                    let name = t.rename.to_string();
                    let alias = t.ident.to_string();
                    result.push((
                        name,
                        RustType {
                            is_const: false,
                            is_fixed_array: false,
                            is_mut: false,
                            is_pointer: false,
                            is_pointer_pointer: false,
                            fixed_array_digits: "".to_string(),
                            type_name: alias.to_string(),
                        },
                    ));
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

pub fn reduce_type_alias(
    aliases: &Vec<(String, RustType)>,
    using_types: &HashSet<String>,
) -> HashMap<String, RustType> {
    let mut map = HashMap::new();
    for (name, rust_type) in aliases {
        if using_types.contains(name) {
            map.insert(name.clone(), rust_type.clone());
        }
    }

    for (name, rust_type) in aliases {
        let pointed = map.get(rust_type.type_name.as_str());
        if let Some(x) = pointed {
            map.insert(name.to_string(), x.clone());
        }
    }

    map
}

pub fn reduce_struct(structs: &Vec<RustStruct>, using_types: &HashSet<String>) -> Vec<RustStruct> {
    let mut result = Vec::new();
    for item in structs {
        if using_types.contains(&item.struct_name) {
            result.push(item.clone());
        }
    }

    result
}

fn parse_type(t: &syn::Type) -> RustType {
    let mut has_star = false;
    let mut has_star_star = false;
    let mut has_const = false;
    let mut has_mut = false;
    let mut digits: String = "".to_string();

    let name = match t {
        syn::Type::Ptr(t) => {
            has_star = true;
            has_const = t.const_token.is_some();
            has_mut = t.mutability.is_some();

            if let syn::Type::Path(path) = &*t.elem {
                path.path.segments.last().unwrap().ident.to_string()
            } else if let syn::Type::Ptr(t) = &*t.elem {
                has_star = false;
                has_star_star = true;
                if let syn::Type::Path(path) = &*t.elem {
                    path.path.segments.last().unwrap().ident.to_string()
                } else {
                    "".to_string()
                }
            } else {
                "".to_string()
            }
        }
        syn::Type::Path(t) => t.path.segments.last().unwrap().ident.to_string(),
        syn::Type::Array(t) => {
            if let syn::Expr::Lit(x) = &t.len {
                if let syn::Lit::Int(x) = &x.lit {
                    digits = x.base10_digits().to_string();
                }
            };

            parse_type(&t.elem).type_name // maybe ok, only retrieve type_name
        }
        syn::Type::Tuple(t) => {
            if t.elems.len() == 0 {
                "()".to_string()
            } else {
                "".to_string()
            }
        }
        _ => "".to_string(),
    };

    RustType {
        is_const: has_const,
        is_mut: has_mut,
        is_pointer: has_star,
        is_pointer_pointer: has_star_star,
        is_fixed_array: !digits.is_empty(),
        type_name: name,
        fixed_array_digits: digits,
    }
}
