using CsBindgen;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace CsbindgenDotnetConsoleApp
{
    // Concept for grouping extension

    internal static unsafe class BindingGroupExtensions
    {
        // public static extern bool quiche_conn_set_keylog_path(quiche_conn* conn, byte* path);
        // public static extern nuint quiche_conn_max_send_udp_payload_size(quiche_conn* conn);
        // public static extern int quiche_conn_close(quiche_conn* conn, [MarshalAs(UnmanagedType.U1)] bool app, ulong err, byte* reason, nuint reason_len);

        public static bool SetKeylogPath(this ref quiche_conn conn, byte* path)
        {
            return LibQuiche.quiche_conn_set_keylog_path((quiche_conn*)Unsafe.AsPointer(ref conn), path);
        }

        public static nuint MaxSendUdpPayloadSize(this ref quiche_conn conn)
        {
            return LibQuiche.quiche_conn_max_send_udp_payload_size((quiche_conn*)Unsafe.AsPointer(ref conn));
        }

        public static int MaxSendUdpPayloadSize(this ref quiche_conn conn, bool app, ulong err, byte* reason, nuint reason_len)
        {
            return LibQuiche.quiche_conn_close((quiche_conn*)Unsafe.AsPointer(ref conn), app, err, reason, reason_len);
        }

        public static bool Hoge(quiche_conn* conn, byte* path)
        {
            return conn->SetKeylogPath(path);
        }

    }
}


namespace Physx
{
    [GroupedNativeMethodsGenerator.GroupedNativeMethods]
    internal static unsafe partial class LibPhysxd
    {
    }
}