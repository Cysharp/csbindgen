// See https://aka.ms/new-console-template for more information
//using Csbindgen;
using CsBindgen;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
unsafe
{
    //var v = NativeMethods();

    var z = LibRust.my_add(100, 200);
    Console.WriteLine(z);

    var s = LibLz4.LZ4_versionString();
    var ss = new string((sbyte*)s);
    Console.WriteLine(ss);

    //var bytes = new byte[] { 1, 10, 100, 100, 100, 100, 100, 100 };
    //var dest = new byte[100];

    //fixed (byte* p = bytes)
    //fixed (byte* d = dest)
    //{

    //    var len = NativeMethods.csbindgen_LZ4_compress_default(p, d, bytes.Length, dest.Length);

    //}


    // var vvv = new string((sbyte*)v);

    // Console.WriteLine(vvv);

}
