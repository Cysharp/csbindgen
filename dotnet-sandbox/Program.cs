

// See https://aka.ms/new-console-template for more information
//using Csbindgen;
using CsBindgen;
using System.Buffers.Text;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;



unsafe
{
    var a = false;
    var b = false;
    var c = false;
    Console.WriteLine(Encoding.Default);


    var p = LibRust.unsafe_return_string();

    var s = Encoding.UTF8.GetString(p, 5);
    Console.WriteLine(s);



    var z = LibRust.my_bool(true, false, true, &a, &b, &c);


    Console.WriteLine(a);
    Console.WriteLine(b);
    Console.WriteLine(c);
    Console.WriteLine(z);

}


public static unsafe partial class LibraryImportNativeMethods
{
    const string __DllName = "csbindgen_tests";


    [LibraryImport(__DllName, EntryPoint = "my_bool")]
    [return: MarshalAs(UnmanagedType.U1)]
    public static partial bool my_bool([MarshalAs(UnmanagedType.U1)] bool x, [MarshalAs(UnmanagedType.U1)] bool y, [MarshalAs(UnmanagedType.U1)] bool z, bool* xr, bool* yr, bool* zr);



    [LibraryImport(__DllName)]
    public static partial void foo(Foo f);
}

public struct Foo
{
    [MarshalAs(UnmanagedType.U1)] public bool A;
}