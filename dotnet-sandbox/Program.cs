// See https://aka.ms/new-console-template for more information
//using Csbindgen;
using CsBindgen;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
unsafe
{
    //var v = NativeMethods();

    var z = LibRust.my_add(100, 200);
    Console.WriteLine(z);

    var s = CsBindgen.LibLz4.LZ4_versionString();
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

{
    public static unsafe partial class LibLz4
    {
        static LibLz4()
        {
            NativeLibrary.SetDllImportResolver(typeof(LibLz4).Assembly, DllImportResolver);
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
}