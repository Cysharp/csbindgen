// auto-generated via csbindgen

#[allow(unused)]
use ::std::os::raw::*;




#[no_mangle]
pub extern "C" fn csbindgen_zstd___va_start(
    arg1: *mut *mut c_char    
)
{
    unsafe {
        return __va_start(
            arg1
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd___security_init_cookie(
    
)
{
    unsafe {
        return __security_init_cookie(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd___security_check_cookie(
    _StackCookie:  usize    
)
{
    unsafe {
        return __security_check_cookie(
            _StackCookie
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_versionNumber(
    
) ->  c_uint
{
    unsafe {
        return ZSTD_versionNumber(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_versionString(
    
) -> *const c_char
{
    unsafe {
        return ZSTD_versionString(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compress(
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize,
    compressionLevel:  c_int    
) ->  usize
{
    unsafe {
        return ZSTD_compress(
            dst,
            dstCapacity,
            src,
            srcSize,
            compressionLevel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_decompress(
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    compressedSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_decompress(
            dst,
            dstCapacity,
            src,
            compressedSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getFrameContentSize(
    src: *const c_void,
    srcSize:  usize    
) ->  c_ulonglong
{
    unsafe {
        return ZSTD_getFrameContentSize(
            src,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getDecompressedSize(
    src: *const c_void,
    srcSize:  usize    
) ->  c_ulonglong
{
    unsafe {
        return ZSTD_getDecompressedSize(
            src,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_findFrameCompressedSize(
    src: *const c_void,
    srcSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_findFrameCompressedSize(
            src,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compressBound(
    srcSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_compressBound(
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_isError(
    code:  usize    
) ->  c_uint
{
    unsafe {
        return ZSTD_isError(
            code
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getErrorName(
    code:  usize    
) -> *const c_char
{
    unsafe {
        return ZSTD_getErrorName(
            code
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_minCLevel(
    
) ->  c_int
{
    unsafe {
        return ZSTD_minCLevel(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_maxCLevel(
    
) ->  c_int
{
    unsafe {
        return ZSTD_maxCLevel(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_defaultCLevel(
    
) ->  c_int
{
    unsafe {
        return ZSTD_defaultCLevel(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_createCCtx(
    
) -> *mut ZSTD_CCtx
{
    unsafe {
        return ZSTD_createCCtx(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_freeCCtx(
    cctx: *mut ZSTD_CCtx    
) ->  usize
{
    unsafe {
        return ZSTD_freeCCtx(
            cctx
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compressCCtx(
    cctx: *mut ZSTD_CCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize,
    compressionLevel:  c_int    
) ->  usize
{
    unsafe {
        return ZSTD_compressCCtx(
            cctx,
            dst,
            dstCapacity,
            src,
            srcSize,
            compressionLevel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_createDCtx(
    
) -> *mut ZSTD_DCtx
{
    unsafe {
        return ZSTD_createDCtx(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_freeDCtx(
    dctx: *mut ZSTD_DCtx    
) ->  usize
{
    unsafe {
        return ZSTD_freeDCtx(
            dctx
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_decompressDCtx(
    dctx: *mut ZSTD_DCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_decompressDCtx(
            dctx,
            dst,
            dstCapacity,
            src,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_cParam_getBounds(
    cParam:  ZSTD_cParameter    
) ->  ZSTD_bounds
{
    unsafe {
        return ZSTD_cParam_getBounds(
            cParam
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CCtx_setParameter(
    cctx: *mut ZSTD_CCtx,
    param:  ZSTD_cParameter,
    value:  c_int    
) ->  usize
{
    unsafe {
        return ZSTD_CCtx_setParameter(
            cctx,
            param,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CCtx_setPledgedSrcSize(
    cctx: *mut ZSTD_CCtx,
    pledgedSrcSize:  c_ulonglong    
) ->  usize
{
    unsafe {
        return ZSTD_CCtx_setPledgedSrcSize(
            cctx,
            pledgedSrcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CCtx_reset(
    cctx: *mut ZSTD_CCtx,
    reset:  ZSTD_ResetDirective    
) ->  usize
{
    unsafe {
        return ZSTD_CCtx_reset(
            cctx,
            reset
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compress2(
    cctx: *mut ZSTD_CCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_compress2(
            cctx,
            dst,
            dstCapacity,
            src,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_dParam_getBounds(
    dParam:  ZSTD_dParameter    
) ->  ZSTD_bounds
{
    unsafe {
        return ZSTD_dParam_getBounds(
            dParam
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DCtx_setParameter(
    dctx: *mut ZSTD_DCtx,
    param:  ZSTD_dParameter,
    value:  c_int    
) ->  usize
{
    unsafe {
        return ZSTD_DCtx_setParameter(
            dctx,
            param,
            value
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DCtx_reset(
    dctx: *mut ZSTD_DCtx,
    reset:  ZSTD_ResetDirective    
) ->  usize
{
    unsafe {
        return ZSTD_DCtx_reset(
            dctx,
            reset
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_createCStream(
    
) -> *mut ZSTD_CStream
{
    unsafe {
        return ZSTD_createCStream(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_freeCStream(
    zcs: *mut ZSTD_CStream    
) ->  usize
{
    unsafe {
        return ZSTD_freeCStream(
            zcs
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compressStream2(
    cctx: *mut ZSTD_CCtx,
    output: *mut ZSTD_outBuffer,
    input: *mut ZSTD_inBuffer,
    endOp:  ZSTD_EndDirective    
) ->  usize
{
    unsafe {
        return ZSTD_compressStream2(
            cctx,
            output,
            input,
            endOp
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CStreamInSize(
    
) ->  usize
{
    unsafe {
        return ZSTD_CStreamInSize(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CStreamOutSize(
    
) ->  usize
{
    unsafe {
        return ZSTD_CStreamOutSize(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_initCStream(
    zcs: *mut ZSTD_CStream,
    compressionLevel:  c_int    
) ->  usize
{
    unsafe {
        return ZSTD_initCStream(
            zcs,
            compressionLevel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compressStream(
    zcs: *mut ZSTD_CStream,
    output: *mut ZSTD_outBuffer,
    input: *mut ZSTD_inBuffer    
) ->  usize
{
    unsafe {
        return ZSTD_compressStream(
            zcs,
            output,
            input
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_flushStream(
    zcs: *mut ZSTD_CStream,
    output: *mut ZSTD_outBuffer    
) ->  usize
{
    unsafe {
        return ZSTD_flushStream(
            zcs,
            output
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_endStream(
    zcs: *mut ZSTD_CStream,
    output: *mut ZSTD_outBuffer    
) ->  usize
{
    unsafe {
        return ZSTD_endStream(
            zcs,
            output
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_createDStream(
    
) -> *mut ZSTD_DStream
{
    unsafe {
        return ZSTD_createDStream(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_freeDStream(
    zds: *mut ZSTD_DStream    
) ->  usize
{
    unsafe {
        return ZSTD_freeDStream(
            zds
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_initDStream(
    zds: *mut ZSTD_DStream    
) ->  usize
{
    unsafe {
        return ZSTD_initDStream(
            zds
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_decompressStream(
    zds: *mut ZSTD_DStream,
    output: *mut ZSTD_outBuffer,
    input: *mut ZSTD_inBuffer    
) ->  usize
{
    unsafe {
        return ZSTD_decompressStream(
            zds,
            output,
            input
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DStreamInSize(
    
) ->  usize
{
    unsafe {
        return ZSTD_DStreamInSize(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DStreamOutSize(
    
) ->  usize
{
    unsafe {
        return ZSTD_DStreamOutSize(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compress_usingDict(
    ctx: *mut ZSTD_CCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize,
    dict: *const c_void,
    dictSize:  usize,
    compressionLevel:  c_int    
) ->  usize
{
    unsafe {
        return ZSTD_compress_usingDict(
            ctx,
            dst,
            dstCapacity,
            src,
            srcSize,
            dict,
            dictSize,
            compressionLevel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_decompress_usingDict(
    dctx: *mut ZSTD_DCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize,
    dict: *const c_void,
    dictSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_decompress_usingDict(
            dctx,
            dst,
            dstCapacity,
            src,
            srcSize,
            dict,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_createCDict(
    dictBuffer: *const c_void,
    dictSize:  usize,
    compressionLevel:  c_int    
) -> *mut ZSTD_CDict
{
    unsafe {
        return ZSTD_createCDict(
            dictBuffer,
            dictSize,
            compressionLevel
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_freeCDict(
    CDict: *mut ZSTD_CDict    
) ->  usize
{
    unsafe {
        return ZSTD_freeCDict(
            CDict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_compress_usingCDict(
    cctx: *mut ZSTD_CCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize,
    cdict: *const ZSTD_CDict    
) ->  usize
{
    unsafe {
        return ZSTD_compress_usingCDict(
            cctx,
            dst,
            dstCapacity,
            src,
            srcSize,
            cdict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_createDDict(
    dictBuffer: *const c_void,
    dictSize:  usize    
) -> *mut ZSTD_DDict
{
    unsafe {
        return ZSTD_createDDict(
            dictBuffer,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_freeDDict(
    ddict: *mut ZSTD_DDict    
) ->  usize
{
    unsafe {
        return ZSTD_freeDDict(
            ddict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_decompress_usingDDict(
    dctx: *mut ZSTD_DCtx,
    dst: *mut c_void,
    dstCapacity:  usize,
    src: *const c_void,
    srcSize:  usize,
    ddict: *const ZSTD_DDict    
) ->  usize
{
    unsafe {
        return ZSTD_decompress_usingDDict(
            dctx,
            dst,
            dstCapacity,
            src,
            srcSize,
            ddict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getDictID_fromDict(
    dict: *const c_void,
    dictSize:  usize    
) ->  c_uint
{
    unsafe {
        return ZSTD_getDictID_fromDict(
            dict,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getDictID_fromCDict(
    cdict: *const ZSTD_CDict    
) ->  c_uint
{
    unsafe {
        return ZSTD_getDictID_fromCDict(
            cdict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getDictID_fromDDict(
    ddict: *const ZSTD_DDict    
) ->  c_uint
{
    unsafe {
        return ZSTD_getDictID_fromDDict(
            ddict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_getDictID_fromFrame(
    src: *const c_void,
    srcSize:  usize    
) ->  c_uint
{
    unsafe {
        return ZSTD_getDictID_fromFrame(
            src,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CCtx_loadDictionary(
    cctx: *mut ZSTD_CCtx,
    dict: *const c_void,
    dictSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_CCtx_loadDictionary(
            cctx,
            dict,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CCtx_refCDict(
    cctx: *mut ZSTD_CCtx,
    cdict: *const ZSTD_CDict    
) ->  usize
{
    unsafe {
        return ZSTD_CCtx_refCDict(
            cctx,
            cdict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_CCtx_refPrefix(
    cctx: *mut ZSTD_CCtx,
    prefix: *const c_void,
    prefixSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_CCtx_refPrefix(
            cctx,
            prefix,
            prefixSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DCtx_loadDictionary(
    dctx: *mut ZSTD_DCtx,
    dict: *const c_void,
    dictSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_DCtx_loadDictionary(
            dctx,
            dict,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DCtx_refDDict(
    dctx: *mut ZSTD_DCtx,
    ddict: *const ZSTD_DDict    
) ->  usize
{
    unsafe {
        return ZSTD_DCtx_refDDict(
            dctx,
            ddict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_DCtx_refPrefix(
    dctx: *mut ZSTD_DCtx,
    prefix: *const c_void,
    prefixSize:  usize    
) ->  usize
{
    unsafe {
        return ZSTD_DCtx_refPrefix(
            dctx,
            prefix,
            prefixSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_sizeof_CCtx(
    cctx: *const ZSTD_CCtx    
) ->  usize
{
    unsafe {
        return ZSTD_sizeof_CCtx(
            cctx
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_sizeof_DCtx(
    dctx: *const ZSTD_DCtx    
) ->  usize
{
    unsafe {
        return ZSTD_sizeof_DCtx(
            dctx
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_sizeof_CStream(
    zcs: *const ZSTD_CStream    
) ->  usize
{
    unsafe {
        return ZSTD_sizeof_CStream(
            zcs
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_sizeof_DStream(
    zds: *const ZSTD_DStream    
) ->  usize
{
    unsafe {
        return ZSTD_sizeof_DStream(
            zds
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_sizeof_CDict(
    cdict: *const ZSTD_CDict    
) ->  usize
{
    unsafe {
        return ZSTD_sizeof_CDict(
            cdict
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_zstd_ZSTD_sizeof_DDict(
    ddict: *const ZSTD_DDict    
) ->  usize
{
    unsafe {
        return ZSTD_sizeof_DDict(
            ddict
        )
    }
}

    