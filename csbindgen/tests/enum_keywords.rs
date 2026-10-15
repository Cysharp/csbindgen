use std::fs;

#[test]
fn enum_variants_escape_csharp_keywords() {
    let directory =
        std::env::temp_dir().join(format!("csbindgen-enum-keywords-{}", std::process::id()));
    fs::create_dir(&directory).unwrap();
    let input = directory.join("input.rs");
    let output = directory.join("NativeMethods.cs");
    fs::write(
        &input,
        r#"
#[repr(i32)]
pub enum Status {
    event = 7,
    class = 9,
    delegate = 11,
    Ready = 13,
}

#[no_mangle]
pub extern "C" fn status_value(value: Status) -> i32 {
    value as i32
}
"#,
    )
    .unwrap();

    csbindgen::Builder::default()
        .input_extern_file(&input)
        .csharp_dll_name("enum_keywords")
        .generate_csharp_file(&output)
        .unwrap();

    let generated = fs::read_to_string(output).unwrap();
    fs::remove_dir_all(directory).unwrap();
    assert!(generated.contains("@event = 7,"));
    assert!(generated.contains("@class = 9,"));
    assert!(generated.contains("@delegate = 11,"));
    assert!(generated.contains("Ready = 13,"));
}
