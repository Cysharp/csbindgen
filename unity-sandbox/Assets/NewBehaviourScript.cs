using AOT;
using CsBindgen;
using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.UI;
using static CsBindgen.LibRust;

public class NewBehaviourScript : MonoBehaviour
{
    public Text text;

    // Start is called before the first frame update
    unsafe void Start()
    {
        try
        {
            var v = LibRust.my_add(10, 20);
            UnityEngine.Debug.Log("Native value: " + v);


            [MonoPInvokeCallback(typeof(Func<int, int>))]
            static int Method(int x) => x * x;

            // var tako = LibRust.callback_test(&Method);

            var a = LibRust.callback_test(Method);

            //var tako = LibRust.callback_test(&Method);



            //var a = LibRust.callback_test((delegate* unmanaged[Cdecl]<int, int>)callback);

            //var a = LibRust.nullable_callback_test((nullable_callback_test_cb_delegate)null);

            // var a = LibRust.callback_test(null);
            //var a = LibRust.callback_test((void*)callback);

            text.text = "Native Callback Return:" + a;
            UnityEngine.Debug.Log(a);
        }
        catch (Exception ex)
        {
            text.text = "Exception:" + ex.ToString();
        }
    }

    //[UnmanagedCallersOnly(CallConvs = new[] { typeof(CallConvCdecl) })]
    static int Method(int x) => x * x;




    [MonoPInvokeCallback(typeof(Func<int, int>))]
    static int Callback(int x)
    {
        Debug.Log("Callback!");
        return x * x;
    }



}

namespace System.Runtime.InteropServices
{
    internal struct CLong
    {
        public int Value; // Windows = int, Unix x32 = int, Unix x64 = long
    }

    internal struct CULong
    {
        public uint Value; // Windows = uint, Unix x32 = uint, Unix x64 = ulong
    }
}

namespace CsBindgen
{
    internal static unsafe partial class LibRust
    {
        const string __DllName = "csbindgen_tests";

        [DllImport(__DllName, EntryPoint = "alias_test1", CallingConvention = CallingConvention.Cdecl)]
        public static extern void alias_test1(long* _a);

        [DllImport(__DllName, EntryPoint = "alias_test2", CallingConvention = CallingConvention.Cdecl)]
        public static extern void alias_test2(long _b);

        [DllImport(__DllName, EntryPoint = "nullpointer_test", CallingConvention = CallingConvention.Cdecl)]
        public static extern void nullpointer_test(byte* p);

        [DllImport(__DllName, EntryPoint = "callback_test", CallingConvention = CallingConvention.Cdecl)]
        public static extern int callback_test(Func<int, int> cb);

        [DllImport(__DllName, EntryPoint = "nullable_callback_test", CallingConvention = CallingConvention.Cdecl)]
        public static extern int nullable_callback_test(Func<int, int> cb);

        [DllImport(__DllName, EntryPoint = "types_iroiro", CallingConvention = CallingConvention.Cdecl)]
        public static extern void types_iroiro(nint _i, nuint _u, CLong _cl, CULong _cul);

        [DllImport(__DllName, EntryPoint = "callback_test2", CallingConvention = CallingConvention.Cdecl)]
        public static extern Func<int, int> callback_test2();

        [DllImport(__DllName, EntryPoint = "callback", CallingConvention = CallingConvention.Cdecl)]
        public static extern int callback(int a);

        [DllImport(__DllName, EntryPoint = "enum_test", CallingConvention = CallingConvention.Cdecl)]
        public static extern int enum_test(IntEnumTest i);

        [DllImport(__DllName, EntryPoint = "nop", CallingConvention = CallingConvention.Cdecl)]
        public static extern void nop();

        [DllImport(__DllName, EntryPoint = "my_add", CallingConvention = CallingConvention.Cdecl)]
        public static extern int my_add(int x, int y);

        [DllImport(__DllName, EntryPoint = "create_counter_context", CallingConvention = CallingConvention.Cdecl)]
        public static extern void* create_counter_context();

        [DllImport(__DllName, EntryPoint = "insert_counter_context", CallingConvention = CallingConvention.Cdecl)]
        public static extern void insert_counter_context(void* context, int value);

        [DllImport(__DllName, EntryPoint = "delete_counter_context", CallingConvention = CallingConvention.Cdecl)]
        public static extern void delete_counter_context(void* context);

        [DllImport(__DllName, EntryPoint = "my_bool", CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.U1)]
        public static extern bool my_bool([MarshalAs(UnmanagedType.U1)] bool x, [MarshalAs(UnmanagedType.U1)] bool y, [MarshalAs(UnmanagedType.U1)] bool z, bool* xr, bool* yr, bool* zr);

        [DllImport(__DllName, EntryPoint = "alloc_c_string", CallingConvention = CallingConvention.Cdecl)]
        public static extern byte* alloc_c_string();

        [DllImport(__DllName, EntryPoint = "free_c_string", CallingConvention = CallingConvention.Cdecl)]
        public static extern void free_c_string(byte* str);

        [DllImport(__DllName, EntryPoint = "alloc_u8_string", CallingConvention = CallingConvention.Cdecl)]
        public static extern ByteBuffer* alloc_u8_string();

        [DllImport(__DllName, EntryPoint = "free_u8_string", CallingConvention = CallingConvention.Cdecl)]
        public static extern void free_u8_string(ByteBuffer* buffer);

        [DllImport(__DllName, EntryPoint = "alloc_u8_buffer", CallingConvention = CallingConvention.Cdecl)]
        public static extern ByteBuffer* alloc_u8_buffer();

        [DllImport(__DllName, EntryPoint = "free_u8_buffer", CallingConvention = CallingConvention.Cdecl)]
        public static extern void free_u8_buffer(ByteBuffer* buffer);

        [DllImport(__DllName, EntryPoint = "alloc_i32_buffer", CallingConvention = CallingConvention.Cdecl)]
        public static extern ByteBuffer* alloc_i32_buffer();

        [DllImport(__DllName, EntryPoint = "free_i32_buffer", CallingConvention = CallingConvention.Cdecl)]
        public static extern void free_i32_buffer(ByteBuffer* buffer);

        [DllImport(__DllName, EntryPoint = "create_context", CallingConvention = CallingConvention.Cdecl)]
        public static extern Context* create_context();

        [DllImport(__DllName, EntryPoint = "delete_context", CallingConvention = CallingConvention.Cdecl)]
        public static extern void delete_context(Context* context);

        [DllImport(__DllName, EntryPoint = "call_bindgen", CallingConvention = CallingConvention.Cdecl)]
        public static extern void call_bindgen();

        [DllImport(__DllName, EntryPoint = "call_bindgen_lz4", CallingConvention = CallingConvention.Cdecl)]
        public static extern void call_bindgen_lz4();


    }

    [StructLayout(LayoutKind.Sequential)]
    internal unsafe partial struct Context
    {
        [MarshalAs(UnmanagedType.U1)] public bool foo;
    }

    [StructLayout(LayoutKind.Sequential)]
    internal unsafe partial struct ByteBuffer
    {
        public byte* ptr;
        public int length;
        public int capacity;
    }


    internal enum IntEnumTest : byte
    {
        A = 1,
        B = 2,
        C = 10,
    }


}
