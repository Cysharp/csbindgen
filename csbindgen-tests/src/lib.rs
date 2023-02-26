#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
mod lz4;

#[allow(dead_code)]
#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
mod ffi;

pub fn add(left: usize, right: usize) -> usize {
    left + right
}


#[test]
fn it_works() {
    //let n = ffi::Cysharp_LZ4_versionNumber();
    //println!("{}", n);
}
