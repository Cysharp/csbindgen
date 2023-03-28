use std::path::PathBuf;
use std::{
    error::Error,
    fs::{File, OpenOptions},
    io::{self, Write},
    path::Path,
};

use crate::{generate, GenerateKind};

pub struct Builder {
    options: BindgenOptions,
}

pub struct BindgenOptions {
    pub input_bindgen_file: PathBuf,
    pub input_extern_files: Vec<PathBuf>,
    pub method_filter: fn(method_name: String) -> bool,
    pub rust_method_type_path: String,
    pub rust_method_prefix: String,
    pub rust_file_header: String,
    pub csharp_namespace: String,
    pub csharp_class_name: String,
    pub csharp_dll_name: String,
    pub csharp_class_accessibility: String,
    pub csharp_entry_point_prefix: String,
    pub csharp_method_prefix: String,
    pub csharp_if_symbol: String,
    pub csharp_if_dll_name: String,
    pub csharp_use_function_pointer: bool,
}

impl Default for Builder {
    fn default() -> Self {
        Self {
            options: BindgenOptions {
                input_bindgen_file: PathBuf::new(),
                input_extern_files: vec![],
                method_filter: |x| !x.starts_with('_'),
                rust_method_type_path: "".to_string(),
                rust_method_prefix: "csbindgen_".to_string(),
                rust_file_header: "".to_string(),
                csharp_namespace: "CsBindgen".to_string(),
                csharp_class_name: "NativeMethods".to_string(),
                csharp_dll_name: "".to_string(),
                csharp_entry_point_prefix: "".to_string(),
                csharp_method_prefix: "".to_string(),
                csharp_class_accessibility: "internal".to_string(),
                csharp_if_symbol: "".to_string(),
                csharp_if_dll_name: "".to_string(),
                csharp_use_function_pointer: true,
            },
        }
    }
}

impl Builder {
    pub fn new() -> Self {
        Self::default()
    }

    /// Change an input .rs file(such as generated from bindgen) to generate binding.
    pub fn input_bindgen_file<T: AsRef<Path>>(mut self, input_bindgen_file: T) -> Builder {
        self.options.input_bindgen_file = input_bindgen_file.as_ref().to_path_buf();
        self
    }

    /// Add an input .rs file for collect extern methods to C# binding.
    pub fn input_extern_file<T: AsRef<Path>>(mut self, input_extern_file: T) -> Builder {
        self.options
            .input_extern_files
            .push(input_extern_file.as_ref().to_path_buf());
        self
    }

    /// Filter generate method callback, default is `!x.starts_with('_')`
    pub fn method_filter(mut self, method_filter: fn(method_name: String) -> bool) -> Builder {
        self.options.method_filter = method_filter;
        self
    }

    /// add original extern call type prefix to rust wrapper,
    /// `return {rust_method_type_path}::foo()`
    pub fn rust_method_type_path<T: Into<String>>(mut self, rust_method_type_path: T) -> Builder {
        self.options.rust_method_type_path = rust_method_type_path.into();
        self
    }

    /// add method prefix to rust wrapper, default is `csbindgen_`
    /// `pub extern "C" fn {rust_method_prefix}foo()`
    pub fn rust_method_prefix<T: Into<String>>(mut self, rust_method_prefix: T) -> Builder {
        self.options.rust_method_prefix = rust_method_prefix.into();
        self
    }

    /// add file header string to rust wrapper,
    /// `mod lz4;`, `use super::lz4;`
    pub fn rust_file_header<T: Into<String>>(mut self, rust_file_header: T) -> Builder {
        self.options.rust_file_header = rust_file_header.into();
        self
    }

    /// configure C# file namespace(default is `CsBindgen`),
    /// "namespace {csharp_namespace}"
    pub fn csharp_namespace<T: Into<String>>(mut self, csharp_namespace: T) -> Builder {
        self.options.csharp_namespace = csharp_namespace.into();
        self
    }

    /// configure C# class name(default is `NativeMethods`),
    /// `public static unsafe partial class {csharp_class_name}`
    pub fn csharp_class_name<T: Into<String>>(mut self, csharp_class_name: T) -> Builder {
        self.options.csharp_class_name = csharp_class_name.into();
        self
    }

    /// configure C# load dll name,
    /// `[DllImport({csharp_dll_name})]`
    pub fn csharp_dll_name<T: Into<String>>(mut self, csharp_dll_name: T) -> Builder {
        self.options.csharp_dll_name = csharp_dll_name.into();
        self
    }

    /// configure C# DllImport EntryPoint prefix,
    /// `[DllImport(, EntryPoint ="{csharp_entry_point_prefix}foo")]`
    pub fn csharp_entry_point_prefix<T: Into<String>>(
        mut self,
        csharp_entry_point_prefix: T,
    ) -> Builder {
        self.options.csharp_entry_point_prefix = csharp_entry_point_prefix.into();
        self
    }

    /// configure C# calling method name prefix,
    /// `public static extern void {csharp_method_prefix}foo()`
    pub fn csharp_method_prefix<T: Into<String>>(mut self, csharp_method_prefix: T) -> Builder {
        self.options.csharp_method_prefix = csharp_method_prefix.into();
        self
    }

    /// configure C# class accessibility, default is internal
    /// `{csharp_class_accessibility} static unsafe partial class NativeMethods`
    pub fn csharp_class_accessibility<T: Into<String>>(
        mut self,
        csharp_class_accessibility: T,
    ) -> Builder {
        self.options.csharp_class_accessibility = csharp_class_accessibility.into();
        self
    }

    /// configure add C# dll name if directive,
    /// #if {if_symbol} __DllName = {if_dll_name}
    pub fn csharp_dll_name_if<T: Into<String>>(mut self, if_symbol: T, if_dll_name: T) -> Builder {
        self.options.csharp_if_symbol = if_symbol.into();
        self.options.csharp_if_dll_name = if_dll_name.into();
        self
    }

    /// conifure C# generate function pointer as delegate* or Func/Action, default is true(generate delegate*)
    pub fn csharp_use_function_pointer(mut self, csharp_use_function_pointer: bool) -> Builder {
        self.options.csharp_use_function_pointer = csharp_use_function_pointer;
        self
    }

    pub fn generate_csharp_file<P: AsRef<Path>>(
        &self,
        csharp_output_path: P,
    ) -> Result<(), Box<dyn Error>> {
        if self.has_input_file() {
            let (_, csharp) = generate(GenerateKind::InputBindgen, &self.options)?;

            let mut csharp_file = make_file(csharp_output_path.as_ref())?;
            csharp_file.write_all(csharp.as_bytes())?;
            csharp_file.flush()?;
        }

        if self.has_input_externals() {
            let (_, csharp) = generate(GenerateKind::InputExtern, &self.options)?;

            let mut csharp_file = make_file(csharp_output_path.as_ref())?;
            csharp_file.write_all(csharp.as_bytes())?;
            csharp_file.flush()?;
        }

        Ok(())
    }

    fn has_input_file(&self) -> bool {
        !self.options.input_bindgen_file.to_string_lossy().is_empty()
    }
    fn has_input_externals(&self) -> bool {
        !self.options.input_extern_files.is_empty()
    }

    pub fn generate_to_file<P: AsRef<Path>>(
        &self,
        rust_output_path: P,
        csharp_output_path: P,
    ) -> Result<(), Box<dyn Error>> {
        if self.has_input_file() {
            let (rust, csharp) = generate(GenerateKind::InputBindgen, &self.options)?;

            if let Some(rust) = rust {
                let mut rust_file = make_file(rust_output_path.as_ref())?;

                rust_file.write_all(rust.as_bytes())?;
                rust_file.flush()?;
            }

            let mut csharp_file = make_file(csharp_output_path.as_ref())?;
            csharp_file.write_all(csharp.as_bytes())?;
            csharp_file.flush()?;
        }

        if self.has_input_externals() {
            let (rust, csharp) = generate(GenerateKind::InputExtern, &self.options)?;

            if let Some(rust) = rust {
                let mut rust_file = make_file(rust_output_path.as_ref())?;

                rust_file.write_all(rust.as_bytes())?;
                rust_file.flush()?;
            }

            let mut csharp_file = make_file(csharp_output_path.as_ref())?;
            csharp_file.write_all(csharp.as_bytes())?;
            csharp_file.flush()?;
        }

        Ok(())
    }
}

fn make_file<P: AsRef<Path>>(path: P) -> io::Result<File> {
    let path = path.as_ref();
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let file = OpenOptions::new()
        .write(true)
        .truncate(true)
        .create(true)
        .open(path)?;
    Ok(file)
}
