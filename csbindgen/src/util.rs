pub trait PushStrLn {
    fn push_str_ln(&mut self, string: &str);
}

impl PushStrLn for String {
    fn push_str_ln(&mut self, string: &str) {
        self.push_str(string);
        self.push('\n');
    }
}
