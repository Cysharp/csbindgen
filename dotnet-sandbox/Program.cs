

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
    LibRust.call_bindgen();
    


    var cString = LibRust.alloc_c_string();
    var u8String = LibRust.alloc_u8_string();
    var u8Buffer = LibRust.alloc_u8_buffer();
    var i32Buffer = LibRust.alloc_i32_buffer();
    try
    {
        var str = new String((sbyte*)cString);
        Console.WriteLine(str);

        Console.WriteLine("----");

        var str2 = Encoding.UTF8.GetString(u8String->AsSpan());
        Console.WriteLine(str2);

        Console.WriteLine("----");

        var buffer3 = u8Buffer->AsSpan();
        foreach (var item in buffer3)
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
        LibRust.free_c_string(cString);
        LibRust.free_u8_string(u8String);
        LibRust.free_u8_buffer(u8Buffer);
        LibRust.free_i32_buffer(i32Buffer);
    }


    //var buf = LibRust.return_raw_buffer();
    //try
    //{

    //    var span = buf->AsSpan();





    //    var str = Encoding.UTF8.GetString(span);
    //    Console.WriteLine(str);

    //    //foreach (var item in span)
    //    //{
    //    //    Console.WriteLine(item);
    //    //}

    //}
    //finally
    //{
    //    LibRust.delete_raw_buffer(buf);
    //}



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

namespace CsBindgen
{
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
}