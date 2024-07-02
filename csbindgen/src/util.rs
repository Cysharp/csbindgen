use syn::Meta;

pub trait PushStrLn {
    fn push_str_ln(&mut self, string: &str);
}

impl PushStrLn for String {
    fn push_str_ln(&mut self, string: &str) {
        self.push_str(string);
        self.push('\n');
    }
}

pub fn get_str_from_meta(meta: &Meta) -> Option<String> {
    match meta {
        Meta::NameValue(nv) => match &nv.value {
            syn::Expr::Lit(l) => match &l.lit {
                syn::Lit::Str(s) => Some(s.value()),
                _ => None,
            },
            _ => None,
        },
        _ => None,
    }
}
