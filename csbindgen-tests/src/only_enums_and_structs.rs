#[repr(C)]
pub enum Foo {
    Bar = 1,
    Baz = 2,
}

#[repr(C)]
pub struct Vec3 {
    x: f32,
    y: f32,
    z: f32,
}
