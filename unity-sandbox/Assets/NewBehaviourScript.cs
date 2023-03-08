using AOT;
using CsBindgen;
using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
public delegate int MyDelegate(int x, int y);

public class NewBehaviourScript : MonoBehaviour
{
    public Text text1;
    public Text text2;

    static Text test_s;



    // Start is called before the first frame update
    unsafe void Start()
    {
        try
        {
            test_s = text1;

            // var sum2 = NativeMethods.my_add(50, 49); // simple test


            
            [MonoPInvokeCallback(typeof(Func<int, int, int>))]
            static int MethodWithDebug(int x, int y)
            {
                try
                {
                    var sum = x + y;
                    UnityEngine.Debug.Log("Call from Rust:" + sum);
                    test_s.text = "Call from Rust:" + sum.ToString();
                    return sum;
                }
                catch (Exception ex)
                {
                    test_s.text = "Exception from Rust:" + ex.ToString();
                    Debug.LogError(ex.ToString());
                    return -1;
                }
            }

            NativeMethods.csharp_to_rust(MethodWithDebug);


            var f = NativeMethods.rust_to_csharp();


            var sum2 = f(1000, 2000);
            UnityEngine.Debug.Log("Invoke Rust Method:" + sum2);
            text2.text = "Invoke Rust method:" + sum2;
        }
        catch (Exception ex)
        {
            text2.text = "Exceptio invoke rust:" + ex.ToString();
            Debug.LogError(ex.ToString());
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
