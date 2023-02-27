use std::collections::{HashSet, HashMap};
use crate::type_meta::*;
use syn::{ForeignItem, Item, Pat, ReturnType};

pub fn collect_method(ast: &syn::File) -> Vec<ExternMethod> {
    let mut list: Vec<ExternMethod> = Vec::new();

    for item in ast.items.iter() {
        if let Item::ForeignMod(m) = item {
            for item in m.items.iter() {
                if let ForeignItem::Fn(m) = item {
                    let method_name = m.sig.ident.to_string();

                    let mut parameters: Vec<Parameter> = Vec::new();
                    let mut retrun_type: Option<RustType> = None;

                    // argument
                    for arg in m.sig.inputs.iter() {
                        if let syn::FnArg::Typed(t) = arg {
                            let mut parameter_name: String = "".to_string();

                            if let Pat::Ident(ident) = &*t.pat {
                                parameter_name = ident.ident.to_string();
                            }

                            let rust_type = parse_type(&t.ty);

                            parameters.push(Parameter {
                                name: parameter_name,
                                rust_type: rust_type,
                            });
                        }
                    }

                    // return
                    if let ReturnType::Type(_, b) = &m.sig.output {
                        let rust_type = parse_type(&b);
                        if rust_type.type_name == "" {
                            continue;
                        }

                        retrun_type = Some(rust_type);
                    }

                    let t = ExternMethod {
                        method_name: method_name.clone(),
                        parameters: parameters,
                        return_type: retrun_type,
                    };

                    list.push(t.clone());
                }
            }
        }
    }

    return list;
}

pub fn collect_type_alias(ast: &syn::File) -> Vec<(String, RustType)> {
    let mut result = Vec::new();
    for item in ast.items.iter() {
        if let Item::Type(t) = item {
            let name = t.ident.to_string();
            let alias = parse_type(&t.ty);

            // pointer can not use alias.
            if !alias.is_pointer || !alias.is_pointer_pointer {
                result.push((name, alias));
            }
        }
    }
    return result;
}

pub fn collect_struct(ast: &syn::File) -> Vec<RustStruct> {
    // collect union or struct
    let mut result = Vec::new();

    for item in ast.items.iter() {
        if let Item::Union(t) = item {
            let name = t.ident.to_string();
            let fields = collect_fields(&t.fields);

            result.push(RustStruct {
                struct_name: name,
                fields: fields,
                is_union: true,
            });
        } else if let Item::Struct(t) = item {
            if let syn::Fields::Named(f) = &t.fields {
                let name = t.ident.to_string();
                let fields = collect_fields(&f);
                result.push(RustStruct {
                    struct_name: name,
                    fields: fields,
                    is_union: false,
                });
            };
        }
    }

    return result;
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

    return result;
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

    return map;
}

pub fn reduce_struct(structs: &Vec<RustStruct>, using_types: &HashSet<String>) -> Vec<RustStruct> {
    let mut result = Vec::new();
    for item in structs {
        if using_types.contains(&item.struct_name) {
            result.push(item.clone());
        }
    }

    return result;
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
        _ => "".to_string(),
    };

    return RustType {
        is_const: has_const,
        is_mut: has_mut,
        is_pointer: has_star,
        is_pointer_pointer: has_star_star,
        is_fixed_array: (digits != ""),
        type_name: name,
        fixed_array_digits: digits,
    };
}