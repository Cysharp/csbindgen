use std::{collections::HashMap};

use crate::builder::BindgenOptions;

#[derive(Clone, Debug)]
pub struct Parameter {
    pub name: String,
    pub rust_type: RustType,
}

impl Parameter {
    pub fn escape_name(&self) -> String {
        match self.name.as_str() {
            // C# keywords: https://learn.microsoft.com/en-us/dotnet/csharp/language-reference/keywords/
            "abstract" | "as" | "base" | "bool" | "break" | "byte" | "case" | "catch" | "char"
            | "checked" | "class" | "const" | "continue" | "decimal" | "default" | "delegate"
            | "do" | "double" | "else" | "enum" | "event" | "explicit" | "extern" | "false"
            | "finally" | "fixed" | "float" | "for" | "foreach" | "goto" | "if" | "implicit"
            | "in" | "int" | "interface" | "internal" | "is" | "lock" | "long" | "namespace"
            | "new" | "null" | "object" | "operator" | "out" | "override" | "params"
            | "private" | "protected" | "public" | "readonly" | "ref" | "return" | "sbyte"
            | "sealed" | "short" | "sizeof" | "stackalloc" | "static" | "string" | "struct"
            | "switch" | "this" | "throw" | "true" | "try" | "typeof" | "uint" | "ulong"
            | "unchecked" | "unsafe" | "ushort" | "using" | "virtual" | "void" | "volatile"
            | "while" => "@".to_string() + self.name.as_str(),
            x => x.to_string(),
        }
    }
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

#[derive(Clone, Debug)]
pub struct RustType {
    pub type_name: String,
    pub type_kind: TypeKind,
}

#[derive(Clone, Debug)]
pub enum TypeKind {
    Normal,
    Pointer(PointerType),
    FixedArray(String, Option<PointerType>),         // digits
    Function(Vec<Parameter>, Option<Box<RustType>>), // parameter, return
    Option(Box<RustType>),
}

#[derive(Clone, Debug)]
pub enum PointerType {
    ConstPointer,
    MutPointer,
    ConstPointerPointer,
    MutPointerPointer,
}

#[derive(Clone, Debug)]
pub struct RustStruct {
    pub struct_name: String,
    pub fields: Vec<FieldMember>,
    pub is_union: bool,
}

impl RustType {
    pub fn to_rust_string(&self, type_path: &str) -> String {
        let mut sb = String::new();

        fn emit_pointer(sb: &mut String, p: &PointerType) {
            match p {
                ConstPointer => sb.push_str("*const"),
                MutPointer => sb.push_str("*mut"),
                ConstPointerPointer => sb.push_str("*const *const"),
                MutPointerPointer => sb.push_str("*mut *mut"),
            };
        }

        let emit_type_name = |sb: &mut String| {
            if !(self.type_name.starts_with("c_")
                || self.type_name == "usize"
                || self.type_name == "isize"
                || (type_path.is_empty()))
            {
                sb.push_str(type_path);
                sb.push_str("::");
            }
            sb.push_str(self.type_name.as_str());
        };

        use PointerType::*;
        use TypeKind::*;
        match &self.type_kind {
            Normal => {
                emit_type_name(&mut sb);
            }
            Pointer(p) => {
                emit_pointer(&mut sb, p);
                sb.push(' ');
                emit_type_name(&mut sb);
            }
            FixedArray(digits, pointer) => {
                if let Some(p) = pointer {
                    emit_pointer(&mut sb, p);
                    sb.push(' ');
                }

                sb.push('[');
                emit_type_name(&mut sb);
                sb.push_str("; ");
                sb.push_str(digits.as_str());
                sb.push(']');
            }
            Function(parameters, return_type) => {
                emit_type_name(&mut sb); // extern fn
                sb.push('(');
                let params = parameters
                    .iter()
                    .map(|x| format!("{}: {}", x.escape_name(), x.rust_type.to_rust_string(type_path)))
                    .collect::<Vec<_>>()
                    .join(", ");
                sb.push_str(params.as_str());
                sb.push(')');
                if let Some(t) = return_type {
                    sb.push_str(" -> ");
                    sb.push_str(t.to_rust_string(type_path).as_str());
                }
            },
            Option(inner) =>  {
                sb.push_str("Option<");
                sb.push_str(inner.to_rust_string(type_path).as_str());
                sb.push('>');
            }
        };

        sb
    }

    pub fn to_csharp_string(
        &self,
        options: &BindgenOptions,
        alias_map: &HashMap<String, RustType>,
    ) -> String {
        fn convert_type_name(type_name: &str, options: &BindgenOptions) -> String {
            let name = match type_name {
                // std::os::raw https://doc.rust-lang.org/std/os/raw/index.html
                // std::ffi::raw https://doc.rust-lang.org/core/ffi/index.html
                "c_char" => "byte",
                "c_schar" => "sbyte",
                "c_uchar" => "byte",
                "c_short" => "short",
                "c_ushort" => "ushort",
                "c_int" => "int",
                "c_uint" => "uint",
                "c_long" => &options.csharp_c_long_convert,
                "c_ulong" => &options.csharp_c_ulong_convert,
                "c_longlong" => "long",
                "c_ulonglong" => "ulong",
                "c_float" => "float",
                "c_double" => "double",
                "c_void" => "void",
                "CString" => "sbyte",
                // rust primitives
                "i8" => "sbyte",
                "i16" => "short",
                "i32" => "int",
                "i64" => "long",
                "i128" => "Int128", // .NET 7
                "isize" => "IntPtr",
                "u8" => "byte",
                "u16" => "ushort",
                "u32" => "uint",
                "u64" => "ulong",
                "u128" => "UInt128", // .NET 7
                "f32" => "float",
                "f64" => "double",
                "bool" => "bool",
                "usize" => "UIntPtr",
                "()" => "void",
                _ => type_name, // as is
            };
            name.to_string()
        }

        // resolve alias
        let (use_type, use_alias) = match alias_map.get(&self.type_name) {
            Some(x) => (x, true),
            None => (self, false),
        };

        let mut sb = String::new();

        match &self.type_kind {
            TypeKind::FixedArray(_, _) => {
                sb.push_str("fixed ");

                let type_name = convert_type_name(use_type.type_name.as_str(), options);
                let type_name = match type_name.as_str() {
                    // C# fixed allow types
                    "bool" | "byte" | "short" | "int" | "long" | "char" | "sbyte" | "ushort"
                    | "uint" | "ulong" | "float" | "double" => type_name,
                    _ => format!("byte/* {}, this length is invalid so must keep pointer and can't edit from C# */", type_name)
                };

                sb.push_str(type_name.as_str());
            }
            TypeKind::Function(parameters, return_type) => {
                sb.push_str("delegate* unmanaged[Cdecl]");
                sb.push('<');
                for p in parameters {
                    sb.push_str(&p.rust_type.to_csharp_string(options, alias_map));
                    sb.push_str(", ");
                }
                match return_type {
                    Some(x) => {
                        sb.push_str(&x.to_csharp_string(options, alias_map));
                    }
                    None => {
                        sb.push_str("void");
                    }
                };
                sb.push('>');
            },
            TypeKind::Option(inner) =>{
                // function pointer can not annotate ? so emit inner only
                sb.push_str(inner.to_csharp_string(options, alias_map).as_str());
            },
            _ => {
                sb.push_str(convert_type_name(use_type.type_name.as_str(), options).as_str());

                if use_alias {
                    if let TypeKind::Pointer(p) = &use_type.type_kind {
                        match p {
                            PointerType::MutPointer | PointerType::ConstPointer => {
                                sb.push('*');
                            }
                            PointerType::MutPointerPointer | PointerType::ConstPointerPointer => {
                                sb.push_str("**");
                            }
                        }
                    }
                }

                if let TypeKind::Pointer(p) = &self.type_kind {
                    match p {
                        PointerType::MutPointer | PointerType::ConstPointer => {
                            sb.push('*');
                        }
                        PointerType::MutPointerPointer | PointerType::ConstPointerPointer => {
                            sb.push_str("**");
                        }
                    }
                }
            }
        };

        sb
    }
}

impl std::fmt::Display for RustType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_rust_string(""))
    }
}
