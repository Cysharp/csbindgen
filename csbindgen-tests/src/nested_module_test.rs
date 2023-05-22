mod nested_mod {
    #[repr(C)]
    pub struct NumberStruct {
        pub num: i32,
    }
    type NumberStructAlias = NumberStruct;
    #[no_mangle]
    pub extern "C" fn triple_input(input: NumberStruct) -> i32 {
        input.num * 3
    }
    #[no_mangle]
    pub extern "C" fn triple_input_aliased(input: NumberStructAlias) -> i32 {
        input.num * 3
    }
    
    #[repr(u8)]
    pub enum NumberEnum {
        One,
        Two,
        Three,
    }
    #[no_mangle]
    pub extern "C" fn number_map(input: NumberEnum) -> i32 {
        match input { 
            NumberEnum::One => 1,
            NumberEnum::Two => 2,
            NumberEnum::Three => 3,
            _ => -1
        }
    }
}