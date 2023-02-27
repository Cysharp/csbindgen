// auto-generated via csbindgen

#[allow(unused)]
use ::std::os::raw::*;




#[no_mangle]
pub extern "C" fn csbindgen_LZ4_versionNumber(
    
) ->  c_int
{
    unsafe {
        return LZ4_versionNumber(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_versionString(
    
) -> *const c_char
{
    unsafe {
        return LZ4_versionString(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_default(
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    dstCapacity:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_default(
            src,
            dst,
            srcSize,
            dstCapacity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_safe(
    src: *const c_char,
    dst: *mut c_char,
    compressedSize:  c_int,
    dstCapacity:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_safe(
            src,
            dst,
            compressedSize,
            dstCapacity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compressBound(
    inputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compressBound(
            inputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_fast(
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    dstCapacity:  c_int,
    acceleration:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_fast(
            src,
            dst,
            srcSize,
            dstCapacity,
            acceleration
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_sizeofState(
    
) ->  c_int
{
    unsafe {
        return LZ4_sizeofState(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_fast_extState(
    state: *mut c_void,
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    dstCapacity:  c_int,
    acceleration:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_fast_extState(
            state,
            src,
            dst,
            srcSize,
            dstCapacity,
            acceleration
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_destSize(
    src: *const c_char,
    dst: *mut c_char,
    srcSizePtr: *mut c_int,
    targetDstSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_destSize(
            src,
            dst,
            srcSizePtr,
            targetDstSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_safe_partial(
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    targetOutputSize:  c_int,
    dstCapacity:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_safe_partial(
            src,
            dst,
            srcSize,
            targetOutputSize,
            dstCapacity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_createStream(
    
) -> *mut LZ4_stream_t
{
    unsafe {
        return LZ4_createStream(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_freeStream(
    streamPtr: *mut LZ4_stream_t    
) ->  c_int
{
    unsafe {
        return LZ4_freeStream(
            streamPtr
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_resetStream_fast(
    streamPtr: *mut LZ4_stream_t    
)
{
    unsafe {
        return LZ4_resetStream_fast(
            streamPtr
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_loadDict(
    streamPtr: *mut LZ4_stream_t,
    dictionary: *const c_char,
    dictSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_loadDict(
            streamPtr,
            dictionary,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_fast_continue(
    streamPtr: *mut LZ4_stream_t,
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    dstCapacity:  c_int,
    acceleration:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_fast_continue(
            streamPtr,
            src,
            dst,
            srcSize,
            dstCapacity,
            acceleration
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_saveDict(
    streamPtr: *mut LZ4_stream_t,
    safeBuffer: *mut c_char,
    maxDictSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_saveDict(
            streamPtr,
            safeBuffer,
            maxDictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_createStreamDecode(
    
) -> *mut LZ4_streamDecode_t
{
    unsafe {
        return LZ4_createStreamDecode(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_freeStreamDecode(
    LZ4_stream: *mut LZ4_streamDecode_t    
) ->  c_int
{
    unsafe {
        return LZ4_freeStreamDecode(
            LZ4_stream
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_setStreamDecode(
    LZ4_streamDecode: *mut LZ4_streamDecode_t,
    dictionary: *const c_char,
    dictSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_setStreamDecode(
            LZ4_streamDecode,
            dictionary,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decoderRingBufferSize(
    maxBlockSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decoderRingBufferSize(
            maxBlockSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_safe_continue(
    LZ4_streamDecode: *mut LZ4_streamDecode_t,
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    dstCapacity:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_safe_continue(
            LZ4_streamDecode,
            src,
            dst,
            srcSize,
            dstCapacity
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_safe_usingDict(
    src: *const c_char,
    dst: *mut c_char,
    srcSize:  c_int,
    dstCapacity:  c_int,
    dictStart: *const c_char,
    dictSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_safe_usingDict(
            src,
            dst,
            srcSize,
            dstCapacity,
            dictStart,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_safe_partial_usingDict(
    src: *const c_char,
    dst: *mut c_char,
    compressedSize:  c_int,
    targetOutputSize:  c_int,
    maxOutputSize:  c_int,
    dictStart: *const c_char,
    dictSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_safe_partial_usingDict(
            src,
            dst,
            compressedSize,
            targetOutputSize,
            maxOutputSize,
            dictStart,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen___va_start(
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
pub extern "C" fn csbindgen___security_init_cookie(
    
)
{
    unsafe {
        return __security_init_cookie(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen___security_check_cookie(
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
pub extern "C" fn csbindgen_LZ4_initStream(
    buffer: *mut c_void,
    size:  usize    
) -> *mut LZ4_stream_t
{
    unsafe {
        return LZ4_initStream(
            buffer,
            size
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress(
    src: *const c_char,
    dest: *mut c_char,
    srcSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress(
            src,
            dest,
            srcSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_limitedOutput(
    src: *const c_char,
    dest: *mut c_char,
    srcSize:  c_int,
    maxOutputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_limitedOutput(
            src,
            dest,
            srcSize,
            maxOutputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_withState(
    state: *mut c_void,
    source: *const c_char,
    dest: *mut c_char,
    inputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_withState(
            state,
            source,
            dest,
            inputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_limitedOutput_withState(
    state: *mut c_void,
    source: *const c_char,
    dest: *mut c_char,
    inputSize:  c_int,
    maxOutputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_limitedOutput_withState(
            state,
            source,
            dest,
            inputSize,
            maxOutputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_continue(
    LZ4_streamPtr: *mut LZ4_stream_t,
    source: *const c_char,
    dest: *mut c_char,
    inputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_continue(
            LZ4_streamPtr,
            source,
            dest,
            inputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_compress_limitedOutput_continue(
    LZ4_streamPtr: *mut LZ4_stream_t,
    source: *const c_char,
    dest: *mut c_char,
    inputSize:  c_int,
    maxOutputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_compress_limitedOutput_continue(
            LZ4_streamPtr,
            source,
            dest,
            inputSize,
            maxOutputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_uncompress(
    source: *const c_char,
    dest: *mut c_char,
    outputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_uncompress(
            source,
            dest,
            outputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_uncompress_unknownOutputSize(
    source: *const c_char,
    dest: *mut c_char,
    isize_:  c_int,
    maxOutputSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_uncompress_unknownOutputSize(
            source,
            dest,
            isize_,
            maxOutputSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_create(
    inputBuffer: *mut c_char    
) -> *mut c_void
{
    unsafe {
        return LZ4_create(
            inputBuffer
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_sizeofStreamState(
    
) ->  c_int
{
    unsafe {
        return LZ4_sizeofStreamState(

        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_resetStreamState(
    state: *mut c_void,
    inputBuffer: *mut c_char    
) ->  c_int
{
    unsafe {
        return LZ4_resetStreamState(
            state,
            inputBuffer
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_slideInputBuffer(
    state: *mut c_void    
) -> *mut c_char
{
    unsafe {
        return LZ4_slideInputBuffer(
            state
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_safe_withPrefix64k(
    src: *const c_char,
    dst: *mut c_char,
    compressedSize:  c_int,
    maxDstSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_safe_withPrefix64k(
            src,
            dst,
            compressedSize,
            maxDstSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_fast_withPrefix64k(
    src: *const c_char,
    dst: *mut c_char,
    originalSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_fast_withPrefix64k(
            src,
            dst,
            originalSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_fast(
    src: *const c_char,
    dst: *mut c_char,
    originalSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_fast(
            src,
            dst,
            originalSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_fast_continue(
    LZ4_streamDecode: *mut LZ4_streamDecode_t,
    src: *const c_char,
    dst: *mut c_char,
    originalSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_fast_continue(
            LZ4_streamDecode,
            src,
            dst,
            originalSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_decompress_fast_usingDict(
    src: *const c_char,
    dst: *mut c_char,
    originalSize:  c_int,
    dictStart: *const c_char,
    dictSize:  c_int    
) ->  c_int
{
    unsafe {
        return LZ4_decompress_fast_usingDict(
            src,
            dst,
            originalSize,
            dictStart,
            dictSize
        )
    }
}

#[no_mangle]
pub extern "C" fn csbindgen_LZ4_resetStream(
    streamPtr: *mut LZ4_stream_t    
)
{
    unsafe {
        return LZ4_resetStream(
            streamPtr
        )
    }
}

    