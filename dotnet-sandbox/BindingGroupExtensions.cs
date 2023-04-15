

using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Physx
{
    [GroupedNativeMethodsGenerator.GroupedNativeMethods(removePrefix: "Px")]
    internal static unsafe partial class LibPhysx
    {
    }
}

namespace CsBindgen
{
    [GroupedNativeMethodsGenerator.GroupedNativeMethods(removePrefix: "b3")]
    internal static unsafe partial class LibBullet3
    {
    }

    [GroupedNativeMethodsGenerator.GroupedNativeMethods(removePrefix: "quiche_")]
    internal static unsafe partial class LibQuiche
    {
    }

    [GroupedNativeMethodsGenerator.GroupedNativeMethods(removePrefix: "sqlite3_")]
    public static unsafe partial class LibSqlite3
    {
    }

    [GroupedNativeMethodsGenerator.GroupedNativeMethods()]
    internal static unsafe partial class NativeMethods
    {
    }
}


namespace PixivApi.ImageFile
{
    [GroupedNativeMethodsGenerator.GroupedNativeMethods(removePrefix: "png_")]
    internal static unsafe partial class LibPng16
    {
    }
}

namespace Jolt
{
    [GroupedNativeMethodsGenerator.GroupedNativeMethods(removePrefix: "JPH_")]
    internal static unsafe partial class NativeMethods
    {
    }
}

namespace CsBindgen
{

    internal static unsafe class NativeMethodsGroupingExtensions2
    {
        [DebuggerStepThrough]
        public static void DeleteContext2(this ref global::CsBindgen.Context @context)
        {
            NativeMethods.delete_context((global::CsBindgen.Context*)Unsafe.AsPointer(ref @context));
        }
    }
}