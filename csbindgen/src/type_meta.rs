use crate::builder::BindgenOptions;

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
    pub fn to_string(&self, type_path: &str) -> String {
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
                && !(type_path == "")
            {
                sb.push_str(type_path);
                sb.push_str("::");
            }
            sb.push_str(self.type_name.as_str());
        }

        sb
    }

    pub fn to_csharp_string(&self, options: &BindgenOptions) -> String {
        fn convert_type_name(type_name: &str, options: &BindgenOptions) -> String {
            let name = match type_name {
                // std::os::raw https://doc.rust-lang.org/std/os/raw/index.html
                "c_char" => "Byte",
                "c_schar" => "SByte",
                "c_uchar" => "Byte",
                "c_short" => "Int16",
                "c_ushort" => "UInt16",
                "c_int" => "Int32",
                "c_uint" => "UInt32",
                "c_long" => &options.csharp_c_long_convert,
                "c_ulong" => &options.csharp_c_ulong_convert,
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
            name.to_string()
        }

        let mut sb = String::new();

        if self.is_fixed_array {
            sb.push_str("fixed ");
            sb.push_str(convert_type_name(self.type_name.as_str(), &options).as_str());
        } else {
            sb.push_str(convert_type_name(self.type_name.as_str(), &options).as_str());
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

impl std::fmt::Display for RustType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.to_string(""))
    }
}
