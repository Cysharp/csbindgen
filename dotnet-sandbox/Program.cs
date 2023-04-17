

// See https://aka.ms/new-console-template for more information
//using Csbindgen;
using CsBindgen;
using Physx;
using System.Buffers.Text;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Text.RegularExpressions;

unsafe
{
    //var foundation = LibPhysx.physx_create_foundation();

    //foundation->ReleaseMut();


    //var vec3 = new PxVec3() { x = 10.0f };




    var handler = NativeMethods.create_counter_context();

    handler->Insert(10);
    handler->Insert(20);
    handler->Insert(30);
    
    NativeMethods.destroy_counter_context(handler);

    


    //ctx->DeleteContext2();




    //NativeMethods.call_bindgen_lz4();


    var str = "foobarbaz:あいうえお"; // JPN(Unicode)
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



    //NativeMethods.csharp_to_rust_utf8
    //NativeMethods.alias_test1(null);




    // C# -> Rust, pass static UnmanagedCallersOnly method with `&`
    [UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) })]
    static int Sum(int x, int y) => x + y;

    NativeMethods.csharp_to_rust(&Sum);

    // Rust -> C#, get typed delegate*
    var f = NativeMethods.rust_to_csharp();

    var v = f(20, 30);
    Console.WriteLine(v); // 50






    // var tako = LibRust.callback_test(&Method);

    //var tako = LibRust.callback_test(&Method);


    //Console.WriteLine(tako);


    //var cc = LibRust.enum_test(IntEnumTest.C);
    //Console.WriteLine(cc);


    //    var context = LibRust.create_context();

    //    // do anything...

    //    LibRust.delete_context(context);

    //var ctx = LibRust.create_counter_context(); // ctx = void*

    //LibRust.insert_counter_context(ctx, 10);
    //LibRust.insert_counter_context(ctx, 20);

    //LibRust.delete_counter_context(ctx);

    //LibRust.insert_counter_context(ctx, 20);
    //LibRust.insert_counter_context(ctx, 30);
    //LibRust.insert_counter_context(ctx, 99);
    //LibRust.delete_counter_context(ctx);

    // var cString = LibRust.alloc_c_string();
    //var u8String = LibRust.alloc_u8_string();
    //var u8Buffer = LibRust.alloc_u8_buffer();
    //var i32Buffer = LibRust.alloc_i32_buffer();
    //try
    //{
    //    var str = new String((sbyte*)cString);
    //    Console.WriteLine(str);

    //    Console.WriteLine("----");

    //    var str2 = Encoding.UTF8.GetString(u8String->AsSpan());
    //    Console.WriteLine(str2);

    //    Console.WriteLine("----");

    //    var buffer3 = u8Buffer->AsSpan();
    //    foreach (var item in buffer3)
    //    {
    //        Console.WriteLine(item);
    //    }

    //    Console.WriteLine("----");

    //    var i32Span = i32Buffer->AsSpan<int>();
    //    foreach (var item in i32Span)
    //    {
    //        Console.WriteLine(item);
    //    }
    //}
    //finally
    //{
    //    LibRust.free_c_string(cString);
    //    LibRust.free_u8_string(u8String);
    //    LibRust.free_u8_buffer(u8Buffer);
    //    LibRust.free_i32_buffer(i32Buffer);
    //}


    //var buf = LibRust.return_raw_buffer();
    //try
    //{

    //    var span = buf->AsSpan();

    // ExactSpelling = true



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


    //[UnmanagedCallConv(CallConvs = new[] { typeof(CallConvCdecl) })]
    //[LibraryImport(__DllName, EntryPoint = "my_bool")]
    //[return: MarshalAs(UnmanagedType.U1)]
    //public static partial bool my_bool([MarshalAs(UnmanagedType.U1)] bool x, [MarshalAs(UnmanagedType.U1)] bool y, [MarshalAs(UnmanagedType.U1)] bool z, bool* xr, bool* yr, bool* zr);



    ////[LibraryImport(__DllName)]
    ////public static partial void foo(Foo f);




    //[LibraryImport(__DllName, EntryPoint = "nullable_callback_test")]
    //public static partial int nullable_callback_test([MarshalAs(UnmanagedType.FunctionPtr)] Func<int, int> cb);

    //[LibraryImport(__DllName, EntryPoint = "nullable_callback_test")]
    //[UnmanagedCallConv(CallConvs = new[] { typeof(CallConvCdecl) })]
    //public static partial int nullable_callback_test2(delegate* unmanaged[Cdecl]<int, int> cb);

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

