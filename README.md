# csbindgen
[![Crates](https://img.shields.io/crates/v/csbindgen.svg)](https://crates.io/crates/csbindgen) [![Api Rustdoc](https://img.shields.io/badge/api-rustdoc-blue)](https://docs.rs/csbindgen)

Generate C# FFI from Rust for automatically brings native code and C native library to .NET and Unity.

There are usually many pains involved in using the C Library with C#. Not only is it difficult to create bindings, but cross-platform builds are very difficult. In this day and age, you have to build for multiple platforms and architectures, windows, osx, linux, android, ios, each with x64, x86, arm.

[Rust](https://www.rust-lang.org/) has an excellent toolchain for cross-platform builds, as well as [cc crate](https://crates.io/crates/cc), [cmake crate](https://crates.io/crates/cmake) allow C source code to be integrated into the build. And [rust-bindgen](https://crates.io/crates/bindgen), which generates bindings from `.h`, is highly functional and very stable.

csbindgen can easily bring native C libraries into C# through Rust. csbindgen generates Rust extern code and C# DllImport code to work with C# from code generated from C by bindgen. With cc crate or cmake crate, C code is linked to the single rust native library.

Of course, you can also output pure FFI Rust code (or a wrapper layer to make it easier to bring C, C++ libraries into C#) to C#.

Getting Started
---
Install on `Cargo.toml` as `build-dependencies` and set up `bindgen::Builder` on `build.rs`.

```toml
[build-dependencies]
csbindgen = "0.1.1"
```

### C (to Rust) to C#

For example, build [lz4](https://github.com/lz4/lz4) compression library.

```rust
// using bindgen, generate binding code
bindgen::Builder::default()
    .header("c/lz4/lz4.h")
    .generate().unwrap()
    .write_to_file("lz4.rs").unwrap();

// using cc, build and link c code
cc::Build::new().file("lz4.c").compile("lz4");

// csbindgen code, generate both rust ffi and C# dll import
csbindgen::Builder::default()
    .input_bindgen_file("lz4.rs") // read from bindgen generated code
    .csharp_dll_name("liblz4")
    .generate_to_file("lz4_ffi.rs", "../dotnet/NativeMethods.lz4.g.cs")
    .unwrap();
```

It will generates like these code.

```rust
// lz4_ffi.rs

#[allow(unused)]
use ::std::os::raw::*;

use super::lz4;

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_default(src: *const c_char, dst: *mut c_char, srcSize:  c_int, dstCapacity:  c_int) ->  c_int
{
    unsafe {
        return lz4::LZ4_compress_default(src, dst, srcSize, dstCapacity);
    }
}
```

```csharp
// NativeMethods.lz4.g.cs

using System;
using System.Runtime.InteropServices;

namespace CsBindgen
{
    internal static unsafe partial class NativeMethods
    {
        const string __DllName = "liblz4";

        [DllImport(__DllName, EntryPoint = "csbindgen_LZ4_compress_default", CallingConvention = CallingConvention.Cdecl)]
        public static extern int LZ4_compress_default(byte* src, byte* dst, int srcSize, int dstCapacity);
    }
}
```

Finally import generated module on `lib.rs`.

```rust
// lib.rs, import generated codes.
#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
mod lz4;

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
mod lz4_ffi;
```

### Rust to C#.

You can bring simple Rust FFI code to C#.

```rust
// lib.rs, simple FFI code
#[no_mangle]
pub extern "C" fn my_add(x: i32, y: i32) -> i32 {
    x + y
}
```

Setup csbindgen code to `build.rs`.

```rust
csbindgen::Builder::default()
    .input_extern_file("lib.rs")
    .csharp_dll_name("nativelib")
    .generate_csharp_file("../dotnet/NativeMethods.g.cs")
    .unwrap();
```

It will generate this C# code.

```csharp
// NativeMethods.g.cs
using System;
using System.Runtime.InteropServices;

namespace CsBindgen
{
    internal static unsafe partial class NativeMethods
    {
        const string __DllName = "nativelib";

        [DllImport(__DllName, EntryPoint = "my_add", CallingConvention = CallingConvention.Cdecl)]
        public static extern int my_add(int x, int y);
    }
}
```

Builder options(configure template)
---
`input_bindgen_file` -> setup options -> `generate_to_file` to use C to C# workflow. Here are full option guide.

```rust
csbindgen::Builder::default()
    .input_bindgen_file("src/lz4.rs")
    .method_filter(|x| { x.starts_with("LZ4") } )
    .rust_method_prefix("csbindgen_")
    .rust_file_header("use super::lz4;")
    .rust_method_type_path("lz4")
    .csharp_class_name("LibLz4")
    .csharp_class_accessibility("public")
    .csharp_namespace("CsBindgen")
    .csharp_dll_name("csbindgen_tests")
    .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal")
    .csharp_entry_point_prefix("csbindgen_")
    .csharp_method_prefix("")
    .csharp_c_long_convert("int")
    .csharp_c_ulong_convert("uint")
    .generate_to_file("src/lz4_ffi.rs", "../dotnet-sandbox/lz4_bindgen.cs")
    .unwrap();
```

It will be embedded in the placeholder of the output file.

```rust
#[allow(unused)]
use ::std::os::raw::*;

{rust_file_header}

#[no_mangle]
pub extern "C" fn {rust_method_prefix}LZ4_versionNumber() ->  c_int
{
    unsafe {
        return {rust_method_type_path}::LZ4_versionNumber()
    }
}
```

```csharp
using System;
using System.Runtime.InteropServices;

namespace {csharp_namespace}
{
    {csharp_class_accessibility} static unsafe partial class {csharp_class_name}
    {
#if {csharp_dll_name_if(if_symbol,...)}
        const string __DllName = "{csharp_dll_name_if(...,if_dll_name)}";
#else
        const string __DllName = "{csharp_dll_name}";
#endif
    }

    [DllImport(__DllName, EntryPoint = "{csharp_entry_point_prefix}LZ4_versionNumber", CallingConvention = CallingConvention.Cdecl)]
    public static extern int {csharp_method_prefix}LZ4_versionNumber();
}
```

Adjust `rust_file_header` and `rust_method_type_path` to match your module configuration.

`method_filter` allows you to specify which methods to exclude; if unspecified, methods prefixed with `_` are excluded by default.

`rust_method_prefix` and `csharp_method_prefix` or `csharp_entry_point_prefix` must be adjusted to match the method name to be called.

`csharp_dll_name_if` is optional. If specified, `#if` allows two DllName to be specified, which is useful if the name must be `__Internal` at iOS build.

If the file path to be loaded needs to be changed depending on the operating system, the following load code can be used.

```csharp
internal static unsafe partial class NativeMethods
{
    // https://docs.microsoft.com/en-us/dotnet/standard/native-interop/cross-platform
    // Library path will search
    // win => __DllName, __DllName.dll
    // linux, osx => __DllName.so, __DllName.dylib
    // __DllName

    static NativeMethods()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeMethods).Assembly, DllImportResolver);
    }

    static IntPtr DllImportResolver(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (libraryName == __DllName)
        {
            var path = "runtimes/";
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                path += "win-";

            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
            {
                path += "osx-";
            }
            else
            {
                path += "linux-";
            }

            if (RuntimeInformation.OSArchitecture == Architecture.X86)
            {
                path += "x86";
            }
            else if (RuntimeInformation.OSArchitecture == Architecture.X64)
            {
                path += "x64";
            }
            else if (RuntimeInformation.OSArchitecture == Architecture.Arm64)
            {
                path += "arm64";
            }

            path += "/native/" + __DllName;

            return NativeLibrary.Load(path, assembly, searchPath);
        }

        return IntPtr.Zero;
    }
}
```

`csharp_c_long_convert` and `csharp_c_ulong_convert` configure how handles `c_long` and `c_ulong` to C# type. default is to `int` and `uint` because `LLP64` is 32bit representation but you can change it to 64bit.

## Builder options: Rust to C#

Rust to C# is similar workflow as C to C#, use the `input_extern_file` -> setup options -> `generate_csharp_file`.

```rust
csbindgen::Builder::default()
    .input_extern_file("src/lib.rs")
    .csharp_class_name("LibRust")
    .csharp_dll_name("csbindgen_tests")
    .generate_csharp_file("../dotnet-sandbox/NativeMethods.cs")
    .unwrap();
```

`generate_csharp_file` does not generate Rust file so no need to use `rust_` option.

License
---
This library is licensed under the MIT License.
