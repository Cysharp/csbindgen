use std::{
    collections::{HashMap, HashSet},
    fmt::Display,
    fs::{self, File},
    io::Write,
};

use syn::{ForeignItem, Item, Pat, ReturnType};

// mod lz4;

pub fn run(rs_path: &str, output_path: &str, csharp_output_path: &str) {
    let file_content = fs::read_to_string(rs_path).unwrap();
    let file_ast = syn::parse_file(file_content.as_str()).unwrap();

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

    let vecs = emit_rust_method(&methods);

    // TODO: filter methods
    //if !(t.method_name.starts_with("_") || t.method_name == "") {

    let mut file = File::create(output_path).unwrap();
    {
        // TODO:modify here
        // TODO:modify here
        file.write_all("#[allow(unused)]\n".as_bytes()).unwrap();
        file.write_all("use ::std::os::raw::*;\n".as_bytes())
            .unwrap();
        file.write_all("use super::lz4;\n\n".as_bytes()).unwrap();
        for str in vecs {
            file.write_all(str.as_bytes()).unwrap();
        }

        file.flush().unwrap();
    }

    let csharp = emit_csharp(&methods, &aliases, &structs);
    let mut file = File::create(csharp_output_path).unwrap();
    file.write_all(csharp.as_bytes()).unwrap();
    file.flush().unwrap();
}

fn collect_method(ast: &syn::File) -> Vec<ExternMethod> {
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

fn collect_type_alias(ast: &syn::File) -> Vec<(String, RustType)> {
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

fn collect_struct(ast: &syn::File) -> Vec<RustStruct> {
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

fn reduce_type_alias(
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

fn reduce_struct(structs: &Vec<RustStruct>, using_types: &HashSet<String>) -> Vec<RustStruct> {
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

// TODO:rewrite
fn emit_rust_method(list: &Vec<ExternMethod>) -> Vec<String> {
    let mod_name = "lz4";

    let mut result = Vec::new();

    for item in list {
        let method_name = item.method_name.as_str();
        let parameters = item.parameters.iter().map(|p| {
            let mut sb = p.name.to_string();
            sb.push_str(": ");
            sb.push_str(p.rust_type.to_string(mod_name).as_str());
            sb
        });

        let parameter_only_names = item.parameters.iter().map(|p| p.name.as_str());

        let return_line = match &item.return_type {
            None => "".to_string(),
            Some(v) => {
                let mut sb = " -> ".to_string();
                sb.push_str(v.to_string(mod_name).as_str());
                sb
            }
        };

        let mut sb = String::new();
        sb.push_str("#[no_mangle]\n");
        sb.push_str(format!("pub extern \"C\" fn csbindgen_{method_name}(\n").as_str());
        for parameter in parameters {
            sb.push_str("    ");
            sb.push_str(parameter.as_str());
            sb.push_str(",\n");
        }
        sb.push_str(")");
        if return_line != "" {
            sb.push_str(return_line.as_str());
        }
        sb.push_str("\n");
        sb.push_str("{\n");
        sb.push_str("    unsafe {\n");
        sb.push_str(format!("        return {mod_name}::{method_name}(\n").as_str());
        for p in parameter_only_names {
            sb.push_str("            ");
            sb.push_str(p);
            sb.push_str(",\n");
        }
        sb.push_str("        )\n");
        sb.push_str("    }\n");
        sb.push_str_ln("}\n");

        result.push(sb);
    }

    return result;
}

fn emit_csharp(
    methods: &Vec<ExternMethod>,
    aliases: &HashMap<String, RustType>,
    structs: &Vec<RustStruct>,
) -> String {
    // TODO: options
    let namespace = "Csbindgen";
    let class_name = "NativeMethods";
    let dll_name = "csbindgen_tests";
    let method_prefix = "csbindgen_";

    let mut method_list_string = String::new();
    for item in methods {
        let method_name = &item.method_name;
        let return_type = match &item.return_type {
            Some(x) => x.to_csharp_string(),
            None => "void".to_string(),
        };

        let parameters = item
            .parameters
            .iter()
            .map(|p| format!("{} {}", p.rust_type.to_csharp_string(), p.name))
            .collect::<Vec<_>>()
            .join(", ");

        method_list_string.push_str_ln(
            "        [DllImport(__DllName, CallingConvention = CallingConvention.Cdecl)]",
        );
        method_list_string.push_str_ln(
            format!("        public static extern {return_type} {method_prefix}{method_name}({parameters});").as_str(),
        );
        method_list_string.push_str("\n");
    }

    let mut alias_string = String::new();
    let mut aliases: Vec<_> = aliases.iter().collect();
    aliases.sort_by_key(|x| x.0);
    for (name, rust_type) in aliases {
        alias_string.push_str_ln(
            format!("    using {} = {};", name, rust_type.to_csharp_string()).as_str(),
        );
    }

    let mut structs_string = String::new();
    for item in structs {
        let name = &item.struct_name;
        let layout_kind = if item.is_union {
            "Explicit"
        } else {
            "Sequential"
        };

        structs_string
            .push_str_ln(format!("    [StructLayout(LayoutKind.{layout_kind})]").as_str());
        structs_string.push_str_ln(format!("    public unsafe struct {name}").as_str());
        structs_string.push_str_ln("    {");
        for field in &item.fields {
            if item.is_union {
                structs_string.push_str_ln("        [FieldOffset(0)]");
            }
            structs_string.push_str(
                format!(
                    "        public {} {}",
                    field.rust_type.to_csharp_string(),
                    field.name
                )
                .as_str(),
            );
            if field.rust_type.is_fixed_array {
                let mut digits = field.rust_type.fixed_array_digits.clone();
                if digits == "0" {
                    digits = "1".to_string(); // 0 fixed array is not allowed in C#
                };

                structs_string.push_str(format!("[{}]", digits).as_str());
            }
            structs_string.push_str_ln(";");
        }
        structs_string.push_str_ln("    }");
        structs_string.push_str("\n");
    }

    let result = format!(
        "// <auto-generated>
// This code is generated via csbindgen.
// DON'T CHANGE THIS DIRECTLY.
// </auto-generated>
using System;
using System.Runtime.InteropServices;

namespace {namespace}
{{
{alias_string}

    public static unsafe partial class {class_name}
    {{
        const string __DllName = \"{dll_name}\";

{method_list_string}
    }}

{structs_string}    
}}
    "
    );

    return result;
}

trait PushStrLn {
    fn push_str_ln(&mut self, string: &str);
}

impl PushStrLn for String {
    fn push_str_ln(&mut self, string: &str) {
        self.push_str(string);
        self.push('\n');
    }
}

#[derive(Clone, Debug)]
pub struct Parameter {
    pub name: String,
    pub rust_type: RustType,
}

#[derive(Clone, Debug)]
pub struct FieldMember {
    pub name: String,
    pub rust_type: RustType,
}

#[derive(Clone, Debug)]
pub struct ExternMethod {
    pub method_name: String,
    pub parameters: Vec<Parameter>,
    pub return_type: Option<RustType>,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct RustType {
    pub type_name: String,
    pub is_pointer: bool,
    pub is_pointer_pointer: bool,
    pub is_const: bool,
    pub is_mut: bool,
    pub is_fixed_array: bool,
    pub fixed_array_digits: String,
}

#[derive(Clone, Debug)]
pub struct RustStruct {
    pub struct_name: String,
    pub fields: Vec<FieldMember>,
    pub is_union: bool,
}

impl RustType {
    pub fn to_string(&self, mod_prefix: &str) -> String {
        let mut sb = String::new();

        if self.is_pointer || self.is_pointer_pointer {
            sb.push_str("*");
        }
        if self.is_const {
            sb.push_str("const");
        }
        if self.is_mut {
            sb.push_str("mut");
        }
        if self.is_pointer_pointer {
            if self.is_const {
                sb.push_str(" *const");
            } else {
                sb.push_str(" *mut");
            }
        }

        sb.push_str(" ");

        if self.is_fixed_array {
            sb.push_str("[");
            sb.push_str(self.type_name.as_str());
            sb.push_str("; ");
            sb.push_str(self.fixed_array_digits.as_str());
            sb.push_str("]");
        } else {
            if !self.type_name.starts_with("c_")
                && !(self.type_name == "usize" || self.type_name == "isize")
            {
                sb.push_str(mod_prefix);
                sb.push_str("::");
            }
            sb.push_str(self.type_name.as_str());
        }

        sb
    }

    pub fn to_csharp_string(&self) -> String {
        fn convert_type_name(type_name: &str) -> &str {
            let name = match type_name {
                // std::os::raw https://doc.rust-lang.org/std/os/raw/index.html
                "c_char" => "Byte",
                "c_schar" => "SByte",
                "c_uchar" => "Byte",
                "c_short" => "Int16",
                "c_ushort" => "UInt16",
                "c_int" => "Int32",
                "c_uint" => "UInt32",
                "c_long" => "Int32",   // int? long? nint? TODO:configure
                "c_ulong" => "UInt32", // uint? ulong? nuint? TODO:configure
                "c_longlong" => "Int64",
                "c_ulonglong" => "UInt64",
                "c_float" => "Float",
                "c_double" => "Double",
                "c_void" => "void",
                // rust primitives
                "i8" => "SByte",
                "i16" => "Int16",
                "i32" => "Int32",
                "i64" => "Int64",
                "i128" => "Int128", // .NET 7
                "isize" => "IntPtr",
                "u8" => "Byte",
                "u16" => "UInt16",
                "u32" => "UInt32",
                "u64" => "UInt64",
                "u128" => "UInt128", // .NET 7
                "f32" => "Float",
                "f64" => "Double",
                "bool" => "Boolean",
                "usize" => "UIntPtr",
                "()" => "void", // if type is parameter, can't use in C#
                _ => type_name, // as is
            };
            name
        }

        let mut sb = String::new();

        if self.is_fixed_array {
            sb.push_str("fixed ");
            sb.push_str(convert_type_name(self.type_name.as_str()));
        } else {
            sb.push_str(convert_type_name(self.type_name.as_str()));
            if self.is_pointer {
                sb.push_str("*");
            }
            if self.is_pointer_pointer {
                sb.push_str("**");
            }
        }

        sb
    }
}

impl Display for RustType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string(""))
    }
}
