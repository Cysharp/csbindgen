// See https://aka.ms/new-console-template for more information
//using Csbindgen;
using CsBindgen;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
unsafe
{
    var a = false;
    var b = false;
    var c = false;



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


}