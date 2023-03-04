use std::collections::HashMap;

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
    pub fn to_string(&self, type_path: &str) -> String {
        let mut sb = String::new();

        if self.is_pointer || self.is_pointer_pointer {
            sb.push('*');
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

        sb.push(' ');

        if self.is_fixed_array {
            sb.push('[');
            sb.push_str(self.type_name.as_str());
            sb.push_str("; ");
            sb.push_str(self.fixed_array_digits.as_str());
            sb.push(']');
        } else {
            if !(self.type_name.starts_with("c_")
                || self.type_name == "usize"
                || self.type_name == "isize"
                || (type_path.is_empty()))
            {
                sb.push_str(type_path);
                sb.push_str("::");
            }
            sb.push_str(self.type_name.as_str());
        }

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

        if self.is_fixed_array {
            sb.push_str("fixed ");

            let type_name = convert_type_name(use_type.type_name.as_str(), options);
            let type_name = match type_name.as_str() {
                // C# fixed allow types
                "bool" | "byte" | "short" | "int" | "long" | "char" | "sbyte" | "ushort"
                | "uint" | "ulong" | "float" | "double" => type_name,
                _ => format!("byte/* {}, this length is invalid so must keep pointer and can't edit from C# */", type_name)
            };

            sb.push_str(type_name.as_str());
        } else {
            sb.push_str(convert_type_name(use_type.type_name.as_str(), options).as_str());
            if use_alias {
                if use_type.is_pointer {
                    sb.push('*');
                }
                if use_type.is_pointer_pointer {
                    sb.push_str("**");
                }
            }

            if self.is_pointer {
                sb.push('*');
            }
            if self.is_pointer_pointer {
                sb.push_str("**");
            }
        }

        sb
    }
}

impl std::fmt::Display for RustType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string(""))
    }
}
