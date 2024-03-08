# csbindgen
[![Crates](https://img.shields.io/crates/v/csbindgen.svg)](https://crates.io/crates/csbindgen) [![Api Rustdoc](https://img.shields.io/badge/api-rustdoc-blue)](https://docs.rs/csbindgen)

Generate C# FFI from Rust for automatically brings native code and C native library to .NET and Unity.

Automatically generates C# `DllImport` code from Rust `extern "C" fn` code. Whereas DllImport defaults to the Windows calling convention and requires a lot of configuration for C calls, csbindgen generates code optimized for "Cdecl" calls. Also .NET and Unity have different callback invocation methods (.NET uses function pointers, while Unity uses MonoPInvokeCallback), but you can output code for either by configuration.

When used with Rust's excellent C integration, you can also bring C libraries into C#.

There are usually many pains involved in using the C Library with C#. Not only is it difficult to create bindings, but cross-platform builds are very difficult. In this day and age, you have to build for multiple platforms and architectures, windows, osx, linux, android, ios, each with x64, x86, arm.

[Rust](https://www.rust-lang.org/) has an excellent toolchain for cross-platform builds, as well as [cc crate](https://crates.io/crates/cc), [cmake crate](https://crates.io/crates/cmake) allow C source code to be integrated into the build. And [rust-bindgen](https://crates.io/crates/bindgen), which generates bindings from `.h`, is highly functional and very stable.

csbindgen can easily bring native C libraries into C# through Rust. csbindgen generates Rust extern code and C# DllImport code to work with C# from code generated from C by bindgen. With cc crate or cmake crate, C code is linked to the single rust native library.

showcase:
* [lz4_bindgen.cs](https://github.com/Cysharp/csbindgen/blob/47fd97eb379beeb278d7546c6d0b9a404b28fbd1/dotnet-sandbox/lz4_bindgen.cs) : [LZ4](https://github.com/lz4/lz4) compression library C# binding
* [zstd_bindgen.cs](https://github.com/Cysharp/csbindgen/blob/47fd97eb379beeb278d7546c6d0b9a404b28fbd1/dotnet-sandbox/zstd_bindgen.cs) : [Zstandard](https://github.com/facebook/zstd) compression library C# binding
* [quiche_bindgen.cs](https://github.com/Cysharp/csbindgen/blob/47fd97eb379beeb278d7546c6d0b9a404b28fbd1/dotnet-sandbox/quiche_bindgen.cs) : [cloudflare/quiche](https://github.com/cloudflare/quiche) QUIC and HTTP/3 library C# binding
* [bullet3_bindgen.cs](https://github.com/Cysharp/csbindgen/blob/47fd97eb379beeb278d7546c6d0b9a404b28fbd1/dotnet-sandbox/bullet3_bindgen.cs) : [Bullet Physics SDK](https://github.com/bulletphysics/bullet3) C# binding
* [sqlite3_bindgen.cs](https://github.com/Cysharp/csbindgen/blob/47fd97eb379beeb278d7546c6d0b9a404b28fbd1/dotnet-sandbox/sqlite3_bindgen.cs) : [SQLite](https://www.sqlite.org/index.html) C# binding
* [Cysharp/YetAnotherHttpHandler](https://github.com/Cysharp/YetAnotherHttpHandler) : brings the power of HTTP/2 (and gRPC) to Unity and .NET Standard
* [Cysharp/MagicPhysX](https://github.com/Cysharp/MagicPhysX) : .NET PhysX 5 binding to all platforms(win, osx, linux)

Getting Started
---
Install on `Cargo.toml` as `build-dependencies` and set up `bindgen::Builder` on `build.rs`.

```toml
[package]
name = "example"
version = "0.1.0"

[lib]
crate-type = ["cdylib"]

[build-dependencies]
csbindgen = "1.8.0"
```

### Rust to C#.

You can bring Rust FFI code to C#.

```rust
// lib.rs, simple FFI code
#[no_mangle]
pub extern "C" fn my_add(x: i32, y: i32) -> i32 {
    x + y
}
```

Setup csbindgen code to `build.rs`.

```rust
fn main() {
    csbindgen::Builder::default()
        .input_extern_file("lib.rs")
        .csharp_dll_name("example")
        .generate_csharp_file("../dotnet/NativeMethods.g.cs")
        .unwrap();
}
```

`csharp_dll_name` is for specifying `[DllImport({DLL_NAME}, ...)]` on the C# side, which should match the name of the dll binary.
See [#library-loading](#library-loading) section for how to resolve the dll file path.

> [!NOTE]
> In this example, the value of `csharp_dll_name` is output by the Rust project you set up. 
> In the above, `package.name` in the Cargo.toml is set to "example". By default, the following binaries should be output to the `target/` folder of the Rust project.
>  - Windows: example.dll
>  - Linux: libexample.so
>  - macOS: libexample.dylib
>
> The filename without the extension should be specified to DllImport. Be careful that by default, rust compiler prefixes filenames with "lib" in some environments.
> So if you want to try this example as is on macOS, `csharp_dll_name` would be "libexample".

Then, let's run `cargo build` it will generate this C# code.

```csharp
// NativeMethods.g.cs
using System;
using System.Runtime.InteropServices;

namespace CsBindgen
{
    internal static unsafe partial class NativeMethods
    {
        const string __DllName = "example";

        [DllImport(__DllName, EntryPoint = "my_add", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
        public static extern int my_add(int x, int y);
    }
}
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
    .input_bindgen_file("lz4.rs")            // read from bindgen generated code
    .rust_file_header("use super::lz4::*;")     // import bindgen generated modules(struct/method)
    .csharp_entry_point_prefix("csbindgen_") // adjust same signature of rust method and C# EntryPoint
    .csharp_dll_name("liblz4")
    .generate_to_file("lz4_ffi.rs", "../dotnet/NativeMethods.lz4.g.cs")
    .unwrap();
```

It will generates like these code.

```rust
// lz4_ffi.rs

#[allow(unused)]
use ::std::os::raw::*;

use super::lz4::*;

#[no_mangle]
pub unsafe extern "C" fn csbindgen_LZ4_compress_default(src: *const c_char, dst: *mut c_char, srcSize:  c_int, dstCapacity:  c_int) ->  c_int
{
    LZ4_compress_default(src, dst, srcSize, dstCapacity)
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

        [DllImport(__DllName, EntryPoint = "csbindgen_LZ4_compress_default", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
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



## Builder options(configure template)

### Builder options: Rust to C#

Rust to C#, use the `input_extern_file` -> setup options -> `generate_csharp_file`.

```rust
csbindgen::Builder::default()
    .input_extern_file("src/lib.rs")        // required
    .csharp_dll_name("mynativelib")         // required
    .csharp_class_name("NativeMethods")     // optional, default: NativeMethods
    .csharp_namespace("CsBindgen")          // optional, default: CsBindgen
    .csharp_class_accessibility("internal") // optional, default: internal
    .csharp_entry_point_prefix("")          // optional, default: ""
    .csharp_method_prefix("")               // optional, default: ""
    .csharp_use_function_pointer(true)      // optional, default: true
    .csharp_disable_emit_dll_name(false)    // optional, default: false
    .csharp_imported_namespaces("MyLib")    // optional, default: empty
    .csharp_generate_const_filter (|_|false) // optional, default: `|_|false`
    .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal") // optional, default: ""
    .csharp_type_rename(|rust_type_name| match rust_type_name {     // optional, default: `|x| x`
        "FfiConfiguration" => "Configuration".into(),
        _ => x,
    })
    .generate_csharp_file("../dotnet-sandbox/NativeMethods.cs")     // required
    .unwrap();
```

`csharp_*` configuration will be embedded in the placeholder of the output file.

```csharp
using System;
using System.Runtime.InteropServices;
using {csharp_imported_namespaces};

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

    {csharp_generate_const_filter}

    [DllImport(__DllName, EntryPoint = "{csharp_entry_point_prefix}LZ4_versionNumber", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    public static extern int {csharp_method_prefix}LZ4_versionNumber();
}
```

`csharp_dll_name_if` is optional. If specified, `#if` allows two DllName to be specified, which is useful if the name must be `__Internal` at iOS build.

`csharp_disable_emit_dll_name` is optional, if set to true then don't emit `const string __DllName`. It is useful for generate same class-name from different builder.

`csharp_generate_const_filter` is optional, if set a filter fun, then generate filter C# `const` field from Rust `const`.

`input_extern_file` and `input_bindgen_file` allow mulitple call, if you need to add dependent struct, use this.

```rust
csbindgen::Builder::default()
    .input_extern_file("src/lib.rs")
    .input_extern_file("src/struct_modules.rs")
    .generate_csharp_file("../dotnet-sandbox/NativeMethods.cs");
```

also `csharp_imported_namespaces` can call multiple times.

### Unity Callback

`csharp_use_function_pointer` configures how generate function pointer. The default is to generate a `delegate*`, but Unity does not support it; setting it to `false` will generate a `Func/Action` that can be used with `MonoPInvokeCallback`.

```csharp
// true(default) generates delegate*
[DllImport(__DllName, EntryPoint = "callback_test", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
public static extern int callback_test(delegate* unmanaged[Cdecl]<int, int> cb);

// You can define like this callback method.
[UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) })]
static int Method(int x) => x * x;

// And use it.
callback_test(&Method);

// ---

// false will generates {method_name}_{parameter_name}_delegate, it is useful for Unity
[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
public delegate int callback_test_cb_delegate(int a);

[DllImport(__DllName, EntryPoint = "callback_test", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
public static extern int callback_test(callback_test_cb_delegate cb);

// Unity can define callback method as MonoPInvokeCallback
[MonoPInvokeCallback(typeof(NativeMethods.callback_test_cb_delegate))]
static int Method(int x) => x * x;

// And use it.
callback_test(Method);
```

### Builder options: C (to Rust) to C#

`input_bindgen_file` -> setup options -> `generate_to_file` to use C to C# workflow.

```rust
csbindgen::Builder::default()
    .input_bindgen_file("src/lz4.rs")             // required
    .method_filter(|x| { x.starts_with("LZ4") } ) // optional, default: |x| !x.starts_with('_')
    .rust_method_prefix("csbindgen_")             // optional, default: "csbindgen_"
    .rust_file_header("use super::lz4::*;")       // optional, default: ""
    .rust_method_type_path("lz4")                 // optional, default: ""
    .csharp_dll_name("lz4")                       // required
    .csharp_class_name("NativeMethods")           // optional, default: NativeMethods
    .csharp_namespace("CsBindgen")                // optional, default: CsBindgen
    .csharp_class_accessibility("internal")       // optional, default: internal
    .csharp_entry_point_prefix("csbindgen_")      // required, you must set same as rust_method_prefix
    .csharp_method_prefix("")                     // optional, default: ""
    .csharp_use_function_pointer(true)            // optional, default: true
    .csharp_imported_namespaces("MyLib")          // optional, default: empty
    .csharp_generate_const_filter(|_|false)       // optional, default:|_|false
    .csharp_dll_name_if("UNITY_IOS && !UNITY_EDITOR", "__Internal")         // optional, default: ""
    .csharp_type_rename(|rust_type_name| match rust_type_name.as_str() {    // optional, default: `|x| x`
        "FfiConfiguration" => "Configuration".into(),
        _ => x,
    })
    .csharp_file_header("#if !UNITY_WEBGL")       // optional, default: ""
    .csharp_file_footer("#endif")                 // optional, default: ""
    .generate_to_file("src/lz4_ffi.rs", "../dotnet-sandbox/lz4_bindgen.cs") // required
    .unwrap();
```

It will be embedded in the placeholder of the output file.

```rust
#[allow(unused)]
use ::std::os::raw::*;

{rust_file_header}

#[no_mangle]
pub unsafe extern "C" fn {rust_method_prefix}LZ4_versionNumber() ->  c_int
{
    {rust_method_type_path}::LZ4_versionNumber()
}
```

`csharp_*` option template is same as Rust to C#, see above documentation.

Adjust `rust_file_header` for match your module configuration, recommend to use `::*`, also using `rust_method_type_path` that add explicitly resolve path.

`method_filter` allows you to specify which methods to exclude; if unspecified, methods prefixed with `_` are excluded by default. C libraries are usually published with a specific prefix. For example, [LZ4](https://github.com/lz4/lz4) is `LZ4`, [ZStandard](https://github.com/facebook/zstd) is `ZSTD_`, [quiche](https://github.com/cloudflare/quiche) is `quiche_`, [Bullet Physics SDK](https://github.com/bulletphysics/bullet3) is `b3`.

`rust_method_prefix` and `csharp_method_prefix` or `csharp_entry_point_prefix` must be adjusted to match the method name to be called.

Library Loading
---
If the file path to be loaded needs to be changed depending on the operating system, the following load code can be used.

```csharp
internal static unsafe partial class NativeMethods
{
    // https://docs.microsoft.com/en-us/dotnet/standard/native-interop/cross-platform
    // Library path will search
    // win => __DllName, __DllName.dll
    // linux, osx => __DllName.so, __DllName.dylib

    static NativeMethods()
    {
        NativeLibrary.SetDllImportResolver(typeof(NativeMethods).Assembly, DllImportResolver);
    }

    static IntPtr DllImportResolver(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (libraryName == __DllName)
        {
            var path = "runtimes/";
            var extension = "";

            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                path += "win-";
                extension = ".dll";
            }
            else if (RuntimeInformation.IsOSPlatform(OSPlatform.OSX))
            {
                path += "osx-";
                extension = ".dylib";
            }
            else
            {
                path += "linux-";
                extension = ".so";
            }

            if (RuntimeInformation.ProcessArchitecture == Architecture.X86)
            {
                path += "x86";
            }
            else if (RuntimeInformation.ProcessArchitecture == Architecture.X64)
            {
                path += "x64";
            }
            else if (RuntimeInformation.ProcessArchitecture == Architecture.Arm64)
            {
                path += "arm64";
            }

            path += "/native/" + __DllName + extension;

            return NativeLibrary.Load(Path.Combine(AppContext.BaseDirectory, path), assembly, searchPath);
        }

        return IntPtr.Zero;
    }
}
```

If Unity, configure Platform settings in each native library's inspector.

Grouping Extension Methods
---
In an object-oriented style, it is common to create methods that take a pointer to a state (this) as their first argument. With csbindgen, you can group these methods using extension methods by specifying a Source Generator on the C# side.

Install csbindgen from NuGet, and specify [GroupedNativeMethods] for the partial class of the generated extension methods.

> PM> Install-Package [csbindgen](https://www.nuget.org/packages/csbindgen)

```csharp
// create new file and write same type-name with same namespace
namespace CsBindgen
{
    // append `GroupedNativeMethods` attribute
    [GroupedNativeMethods]
    internal static unsafe partial class NativeMethods
    {
    }
}
```

```csharp
// original methods
[DllImport(__DllName, EntryPoint = "counter_context_insert", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
public static extern void counter_context_insert(counter_context* context, int value);

// generated methods
public static void Insert(this ref global::CsBindgen.counter_context @context, int @value)

// ----

counter_context* context = NativeMethods.create_counter_context();

// standard style
NativeMethods.counter_context_insert(context, 10);

// generated style
context->Insert(10);
```

`GroupedNativeMethods` has four configuration parameters.

```csharp
public GroupedNativeMethodsAttribute(
    string removePrefix = "",
    string removeSuffix = "",
    bool removeUntilTypeName = true,
    bool fixMethodName = true)
```

The convention for function names when using this feature is as follows:
- The first argument must be a pointer type.
- `removeUntilTypeName` will remove until find type-name in method-name. 
  - For example `foo_counter_context_insert(countext_context* foo)` -> `Insert`. 
  - As a result, it is recommended to use a naming convention where the same type name is placed immediately before the verb. 

Type Marshalling
---
Rust types will map these C# types.

| Rust | C# |
| ---- | -- |
| `i8` | `sbyte` |
| `i16` | `short` |
| `i32` | `int` |
| `i64` | `long` |
| `i128` | `Int128` |
| `isize` | `nint` |
| `u8` | `byte` |
| `u16` | `ushort` |
| `u32` | `uint` |
| `u64` | `ulong` |
| `u128` | `UInt128` |
| `usize` | `nuint` |
| `f32` | `float` |
| `f64` | `double` |
| `bool` | `[MarshalAs(UnmanagedType.U1)]bool` |
| `char` | `uint` |
| `()` | `void` |
| `c_char` | `byte` |
| `c_schar` | `sbyte` |
| `c_uchar` | `byte` |
| `c_short` | `short` |
| `c_ushort` | `ushort` |
| `c_int` | `int` |
| `c_uint` | `uint` |
| `c_long` | `CLong` |
| `c_ulong` | `CULong` |
| `c_longlong` | `long` |
| `c_ulonglong` | `ulong` |
| `c_float` | `float` |
| `c_double` | `double` |
| `c_void` | `void` |
| `CString` | `sbyte` |
| `NonZeroI8` | `sbyte` |
| `NonZeroI16` | `short` |
| `NonZeroI32` | `int` |
| `NonZeroI64` | `long` |
| `NonZeroI128` | `Int128` |
| `NonZeroIsize` | `nint` |
| `NonZeroU8` | `byte` |
| `NonZeroU16` | `ushort` |
| `NonZeroU32` | `uint` |
| `NonZeroU64` | `ulong` |
| `NonZeroU128` | `UInt128` |
| `NonZeroUsize` | `nuint` |
| `#[repr(C)]Struct` | `[StructLayout(LayoutKind.Sequential)]Struct` |
| `#[repr(C)]Union` | `[StructLayout(LayoutKind.Explicit)]Struct` |
| `#[repr(u*/i*)]Enum` | `Enum` |
| [bitflags!](https://crates.io/crates/bitflags) | `[Flags]Enum` |
| `extern "C" fn` | `delegate* unmanaged[Cdecl]<>` or `Func<>/Action<>` |
| `Option<extern "C" fn>` | `delegate* unmanaged[Cdecl]<>` or `Func<>/Action<>` |
| `*mut T` | `T*` |
| `*const T` | `T*` |
| `*mut *mut T` | `T**` |
| `*const *const T` | `T**` |
| `*mut *const T` | `T**` |
| `*const *mut T` | `T**` |
| `&T` | `T*` |
| `&mut T` | `T*` |
| `&&T` | `T**` |
| `&*mut T` | `T**` |
| `NonNull<T>` | `T*` |
| `Box<T>` | `T*` |

csbindgen is designed to return primitives that do not cause marshalling. It is better to convert from pointers to Span yourself than to do the conversion implicitly and in a black box. This is a recent trend, such as the addition of [DisableRuntimeMarshalling](https://learn.microsoft.com/en-us/dotnet/api/system.runtime.compilerservices.disableruntimemarshallingattribute) from .NET 7.

Older C# version do not support `nint` and `nuint`. You can use `csharp_use_nint_types` to use `IntPtr` and `UIntPtr` in their place:

```rust
    csbindgen::Builder::default()
        .input_extern_file("lib.rs")
        .csharp_dll_name("nativelib")
        .generate_csharp_file("../dotnet/NativeMethods.g.cs")
        .csharp_use_nint_types(false)
        .unwrap();
```

`c_long` and `c_ulong` will convert to [CLong](https://learn.microsoft.com/en-us/dotnet/api/system.runtime.interopservices.clong), [CULong](https://learn.microsoft.com/en-us/dotnet/api/system.runtime.interopservices.culong) struct after .NET 6. If you want to convert in Unity, you will need Shim.



```csharp
// Currently Unity is .NET Standard 2.1 so does not exist CLong and CULong
namespace System.Runtime.InteropServices
{
    internal struct CLong
    {
        public int Value; // #if Windows = int, Unix x32 = int, Unix x64 = long
    }

    internal struct CULong
    {
        public uint Value; // #if Windows = uint, Unix x32 = uint, Unix x64 = ulong
    }
}
```

### Struct

csbindgen supports `Struct`, you can define `#[repr(C)]` struct on method parameter or return value.

```rust
// If you define this struct...
#[repr(C)]
pub struct MyVector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[no_mangle]
pub extern "C" fn pass_vector3(v3: MyVector3) {
    println!("{}, {}, {}", v3.x, v3.y, v3.z);
}
```

```csharp
// csbindgen generates this C# struct
[StructLayout(LayoutKind.Sequential)]
internal unsafe partial struct MyVector3
{
    public float x;
    public float y;
    public float z;
}
```

Also supports tuple struct, it will generate `Item*` fields in C#.

```
#[repr(C)]
pub struct MyIntVec3(i32, i32, i32);
```

```csharp
[StructLayout(LayoutKind.Sequential)]
internal unsafe partial struct MyIntVec3
{
    public int Item1;
    public int Item2;
    public int Item3;
}
```

It also supports unit struct, but there is no C# struct that is synonymous with Rust's unit struct (0 byte), so it cannot be materialized. Instead of using void*, it is recommended to use typed pointers.

```
// 0-byte in Rust
#[repr(C)]
pub struct MyContext;
```

```csharp
// 1-byte in C#
[StructLayout(LayoutKind.Sequential)]
internal unsafe partial struct MyContext
{
}
```

### Union

`Union` will generate `[FieldOffset(0)]` struct.

```rust
#[repr(C)]
pub union MyUnion {
    pub foo: i32,
    pub bar: i64,
}

#[no_mangle]
pub extern "C" fn return_union() -> MyUnion {
    MyUnion { bar: 53 }
}
```

```csharp
[StructLayout(LayoutKind.Explicit)]
internal unsafe partial struct MyUnion
{
    [FieldOffset(0)]
    public int foo;
    [FieldOffset(0)]
    public long bar;
}
```

### Enum

`#[repr(i*)]` or `#[repr(u*)]` defined `Enum` is supported.

```rust
#[repr(u8)]
pub enum ByteEnum {
    A = 1,
    B = 2,
    C = 10,
}
```

```csharp
internal enum ByteTest : byte
{
    A = 1,
    B = 2,
    C = 10,
}
```

### bitflags Enum

csbindgen supports [bitflags](https://crates.io/crates/bitflags) crate.

```rust
bitflags! {
    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    struct EnumFlags: u32 {
        const A = 0b00000001;
        const B = 0b00000010;
        const C = 0b00000100;
        const ABC = Self::A.bits() | Self::B.bits() | Self::C.bits();
    }
}
```

```csharp
[Flags]
internal enum EnumFlags : uint
{
    A = 0b00000001,
    B = 0b00000010,
    C = 0b00000100,
    ABC = A | B | C,
}
```

### Function

You can receive, return function to/from C#.

```rust
#[no_mangle]
pub extern "C" fn csharp_to_rust(cb: extern "C" fn(x: i32, y: i32) -> i32) {
    let sum = cb(10, 20); // invoke C# method
    println!("{sum}");
}

#[no_mangle]
pub extern "C" fn rust_to_csharp() -> extern fn(x: i32, y: i32) -> i32 {
    sum // return rust method
}

extern "C" fn sum(x:i32, y:i32) -> i32 {
    x + y
}
```

In default, csbindgen generates `extern "C" fn` as `delegate* unmanaged[Cdecl]<>`.

```csharp
[DllImport(__DllName, EntryPoint = "csharp_to_rust", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
public static extern void csharp_to_rust(delegate* unmanaged[Cdecl]<int, int, int> cb);

[DllImport(__DllName, EntryPoint = "rust_to_csharp", CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
public static extern delegate* unmanaged[Cdecl]<int, int, int> rust_to_csharp();
```

It can use in C# like this.

```csharp
// C# -> Rust, pass static UnmanagedCallersOnly method with `&`
[UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) })]
static int Sum(int x, int y) => x + y;

NativeMethods.csharp_to_rust(&Sum);

// Rust -> C#, get typed delegate*
var f = NativeMethods.rust_to_csharp();

var v = f(20, 30);
Console.WriteLine(v); // 50
```

> Unity can not use C# 9.0 function pointer, csbindgen has to use `MonoPInvokeCallback` options. see: [Unity Callback](#unity-callback) section.

Rust FFI suppots `Option<fn>`, it can receive null pointer.

```rust
#[no_mangle]
pub extern "C" fn nullable_callback_test(cb: Option<extern "C" fn(a: i32) -> i32>) -> i32 {
    match cb {
        Some(f) => f(100),
        None => -1,
    }
}
```

```csharp
var v = NativeMethods.nullable_callback_test(null); // -1
```

### Pointer

Allocated Rust memory in heap can send to C# via pointer and `Box::into_raw` and `Box::from_raw`.

```rust
#[no_mangle]
pub extern "C" fn create_context() -> *mut Context {
    let ctx = Box::new(Context { foo: true });
    Box::into_raw(ctx)
}

#[no_mangle]
pub extern "C" fn delete_context(context: *mut Context) {
    unsafe { Box::from_raw(context) };
}

#[repr(C)]
pub struct Context {
    pub foo: bool,
    pub bar: i32,
    pub baz: u64
}
```

```csharp
var context = NativeMethods.create_context();

// do anything...

NativeMethods.delete_context(context);
```

You can also pass memory allocated by C# to Rust (use `fixed` or `GCHandle.Alloc(Pinned)`). The important thing is that memory allocated in Rust must release in Rust and memory allocated in C# must release in C#.

If you want to pass a non FFI Safe struct reference, csbindgen generates empty C# struct.

```rust
#[no_mangle]
pub extern "C" fn create_counter_context() -> *mut CounterContext {
    let ctx = Box::new(CounterContext {
        set: HashSet::new(),
    });
    Box::into_raw(ctx)
}

#[no_mangle]
pub unsafe extern "C" fn insert_counter_context(context: *mut CounterContext, value: i32) {
    let mut counter = Box::from_raw(context);
    counter.set.insert(value);
    Box::into_raw(counter);
}

#[no_mangle]
pub unsafe extern "C" fn delete_counter_context(context: *mut CounterContext) {
    let counter = Box::from_raw(context);
    for value in counter.set.iter() {
        println!("counter value: {}", value)
    }
}

// no repr(C)
pub struct CounterContext {
    pub set: HashSet<i32>,
}
```

```csharp
// csbindgen generates this handler type
[StructLayout(LayoutKind.Sequential)]
internal unsafe partial struct CounterContext
{
}

// You can hold pointer instance
CounterContext* ctx = NativeMethods.create_counter_context();
    
NativeMethods.insert_counter_context(ctx, 10);
NativeMethods.insert_counter_context(ctx, 20);

NativeMethods.delete_counter_context(ctx);
```

In this case, recommed to use with [Grouping Extension Methods](#grouping-extension-methods).

If you want to pass null-pointer, in rust side, convert to Option by `as_ref()`.

```rust
#[no_mangle]
pub unsafe extern "C" fn null_pointer_test(p: *const u8) {
    let ptr = unsafe { p.as_ref() };
    match ptr {
        Some(p2) => print!("pointer address: {}", *p2),
        None => println!("null pointer!"),
    };
}
```

```csharp
// in C#, invoke by null.
NativeMethods.null_pointer_test(null);
```

### String and Array(Span)

Rust's String, Array(Vec) and C#'s String, Array is different thing. Since it cannot be shared, pass it with a pointer and handle it with slice(Span) or materialize it if necessary.

`CString` is null-terminated string. It can send by `*mut c_char` and received as `byte*` in C#.

```rust
#[no_mangle]
pub extern "C" fn alloc_c_string() -> *mut c_char {
    let str = CString::new("foo bar baz").unwrap();
    str.into_raw()
}

#[no_mangle]
pub unsafe extern "C" fn free_c_string(str: *mut c_char) {
    unsafe { CString::from_raw(str) };
}
```

```csharp
// null-terminated `byte*` or sbyte* can materialize by new String()
var cString = NativeMethods.alloc_c_string();
var str = new String((sbyte*)cString);
NativeMethods.free_c_string(cString);
```

Rust's String is UTF-8(`Vec<u8>`) but C# String is UTF-16. Andalso, `Vec<>` can not send to C# so require to convert pointer and control memory manually. Here is the buffer manager for FFI.

```rust
#[repr(C)]
pub struct ByteBuffer {
    ptr: *mut u8,
    length: i32,
    capacity: i32,
}

impl ByteBuffer {
    pub fn len(&self) -> usize {
        self.length
            .try_into()
            .expect("buffer length negative or overflowed")
    }

    pub fn from_vec(bytes: Vec<u8>) -> Self {
        let length = i32::try_from(bytes.len()).expect("buffer length cannot fit into a i32.");
        let capacity =
            i32::try_from(bytes.capacity()).expect("buffer capacity cannot fit into a i32.");

        // keep memory until call delete
        let mut v = std::mem::ManuallyDrop::new(bytes);

        Self {
            ptr: v.as_mut_ptr(),
            length,
            capacity,
        }
    }

    pub fn from_vec_struct<T: Sized>(bytes: Vec<T>) -> Self {
        let element_size = std::mem::size_of::<T>() as i32;

        let length = (bytes.len() as i32) * element_size;
        let capacity = (bytes.capacity() as i32) * element_size;

        let mut v = std::mem::ManuallyDrop::new(bytes);

        Self {
            ptr: v.as_mut_ptr() as *mut u8,
            length,
            capacity,
        }
    }

    pub fn destroy_into_vec(self) -> Vec<u8> {
        if self.ptr.is_null() {
            vec![]
        } else {
            let capacity: usize = self
                .capacity
                .try_into()
                .expect("buffer capacity negative or overflowed");
            let length: usize = self
                .length
                .try_into()
                .expect("buffer length negative or overflowed");

            unsafe { Vec::from_raw_parts(self.ptr, length, capacity) }
        }
    }

    pub fn destroy_into_vec_struct<T: Sized>(self) -> Vec<T> {
        if self.ptr.is_null() {
            vec![]
        } else {
            let element_size = std::mem::size_of::<T>() as i32;
            let length = (self.length * element_size) as usize;
            let capacity = (self.capacity * element_size) as usize;

            unsafe { Vec::from_raw_parts(self.ptr as *mut T, length, capacity) }
        }
    }

    pub fn destroy(self) {
        drop(self.destroy_into_vec());
    }
}
```

```csharp
// C# side span utility
partial struct ByteBuffer
{
    public unsafe Span<byte> AsSpan()
    {
        return new Span<byte>(ptr, length);
    }

    public unsafe Span<T> AsSpan<T>()
    {
        return MemoryMarshal.CreateSpan(ref Unsafe.AsRef<T>(ptr), length / Unsafe.SizeOf<T>());
    }
}
```

With `ByteBuffer`, you can send `Vec<>` to C#. the pattern for `String`, `Vec<u8>`, `Vec<i32>`, Rust -> C# is as follows.

```rust
#[no_mangle]
pub extern "C" fn alloc_u8_string() -> *mut ByteBuffer {
    let str = format!("foo bar baz");
    let buf = ByteBuffer::from_vec(str.into_bytes());
    Box::into_raw(Box::new(buf))
}

#[no_mangle]
pub unsafe extern "C" fn free_u8_string(buffer: *mut ByteBuffer) {
    let buf = Box::from_raw(buffer);
    // drop inner buffer, if you need String, use String::from_utf8_unchecked(buf.destroy_into_vec()) instead.
    buf.destroy();
}

#[no_mangle]
pub extern "C" fn alloc_u8_buffer() -> *mut ByteBuffer {
    let vec: Vec<u8> = vec![1, 10, 100];
    let buf = ByteBuffer::from_vec(vec);
    Box::into_raw(Box::new(buf))
}

#[no_mangle]
pub unsafe extern "C" fn free_u8_buffer(buffer: *mut ByteBuffer) {
    let buf = Box::from_raw(buffer);
    // drop inner buffer, if you need Vec<u8>, use buf.destroy_into_vec() instead.
    buf.destroy();
}

#[no_mangle]
pub extern "C" fn alloc_i32_buffer() -> *mut ByteBuffer {
    let vec: Vec<i32> = vec![1, 10, 100, 1000, 10000];
    let buf = ByteBuffer::from_vec_struct(vec);
    Box::into_raw(Box::new(buf))
}

#[no_mangle]
pub unsafe extern "C" fn free_i32_buffer(buffer: *mut ByteBuffer) {
    let buf = Box::from_raw(buffer);
    // drop inner buffer, if you need Vec<i32>, use buf.destroy_into_vec_struct::<i32>() instead.
    buf.destroy();
}
```

```csharp
var u8String = NativeMethods.alloc_u8_string();
var u8Buffer = NativeMethods.alloc_u8_buffer();
var i32Buffer = NativeMethods.alloc_i32_buffer();
try
{
    var str = Encoding.UTF8.GetString(u8String->AsSpan());
    Console.WriteLine(str);

    Console.WriteLine("----");

    var buffer = u8Buffer->AsSpan();
    foreach (var item in buffer)
    {
        Console.WriteLine(item);
    }

    Console.WriteLine("----");

    var i32Span = i32Buffer->AsSpan<int>();
    foreach (var item in i32Span)
    {
        Console.WriteLine(item);
    }
}
finally
{
    NativeMethods.free_u8_string(u8String);
    NativeMethods.free_u8_buffer(u8Buffer);
    NativeMethods.free_i32_buffer(i32Buffer);
}
```

C# to Rust would be a bit simpler to send, just pass byte* and length. In Rust, use `std::slice::from_raw_parts` to create slice. 

```rust
#[no_mangle]
pub unsafe extern "C" fn csharp_to_rust_string(utf16_str: *const u16, utf16_len: i32) {
    let slice = std::slice::from_raw_parts(utf16_str, utf16_len as usize);
    let str = String::from_utf16(slice).unwrap();
    println!("{}", str);
}

#[no_mangle]
pub unsafe extern "C" fn csharp_to_rust_utf8(utf8_str: *const u8, utf8_len: i32) {
    let slice = std::slice::from_raw_parts(utf8_str, utf8_len as usize);
    let str = String::from_utf8_unchecked(slice.to_vec());
    println!("{}", str);
}


#[no_mangle]
pub unsafe extern "C" fn csharp_to_rust_bytes(bytes: *const u8, len: i32) {
    let slice = std::slice::from_raw_parts(bytes, len as usize);
    let vec = slice.to_vec();
    println!("{:?}", vec);
}
```

```csharp
var str = "foobarbaz:あいうえお"; // ENG:JPN(Unicode, testing for UTF16)
fixed (char* p = str)
{
    NativeMethods.csharp_to_rust_string((ushort*)p, str.Length);
}

var str2 = Encoding.UTF8.GetBytes("あいうえお:foobarbaz");
fixed (byte* p = str2)
{
    NativeMethods.csharp_to_rust_utf8(p, str2.Length);
}

var bytes = new byte[] { 1, 10, 100, 255 };
fixed (byte* p = bytes)
{
    NativeMethods.csharp_to_rust_bytes(p, bytes.Length);
}
```

Again, the important thing is that memory allocated in Rust must release in Rust and memory allocated in C# must release in C#.

Build Tracing
---
csbindgen silently skips over any method with a non-generatable type. If you build with `cargo build -vv`, you will get thse message if not geneated.

* `csbindgen can't handle this parameter type so ignore generate, method_name: {} parameter_name: {}`
* `csbindgen can't handle this return type so ignore generate, method_name: {}`


Non-Generatable method: C variadic/variable arguments method
---
csbindgen doesn't handle C's variadic arguments, which causes undefined behaviors, because this feature is not stable both in C# and Rust.
There is a `__arglist` keyword for C's variadic arguments in C#. [`__arglist` has many problems except Windows environment.](https://github.com/dotnet/runtime/issues/48796)
[There is an issue about C's variadic arguments in Rust.](https://github.com/rust-lang/rust/issues/44930)

License
---
This library is licensed under the MIT License.
