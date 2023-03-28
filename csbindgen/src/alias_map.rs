use std::collections::HashMap;

use crate::type_meta::{RustType, TypeKind};

#[derive(Clone, Debug)]
pub struct AliasMap {
    // type FOO = ::std::os::raw::c_longlong;
    type_aliases: HashMap<String, RustType>,
}

impl AliasMap {
    pub fn new() -> Self {
        Self {
            type_aliases: HashMap::new(),
        }
    }

    pub fn insert(&mut self, name: &String, alias: &RustType) {
        self.type_aliases.insert(name.to_owned(), alias.clone());
    }

    pub fn get_mapped_value(&self, name: &String) -> Option<RustType> {
        match self.type_aliases.get(name) {
            Some(x) => {
                if let Some(x2) = self.get_mapped_value(&x.type_name) {
                    // currently multiple pointer alias not supported, only one layer.
                    if let TypeKind::Pointer(_, _) = &x.type_kind {
                        return Some(RustType {
                            type_name: x2.type_name,
                            type_kind: x.type_kind.clone(),
                        });
                    }
                    return Some(x2);
                }
                Some(x.clone())
            }
            None => None,
        }
    }

    pub fn normalize(&self, name: &String) -> (String, Option<RustType>) {
        match self.type_aliases.get(name) {
            Some(x) => {
                let d = self.normalize_deep(x);
                (d.type_name.clone(), Some(d))
            }
            None => (name.to_owned(), None),
        }
    }

    fn normalize_deep(&self, name: &RustType) -> RustType {
        match self.type_aliases.get(&name.type_name) {
            Some(x) => self.normalize_deep(x),
            None => name.clone(),
        }
    }
}

#[cfg(test)]
mod test {
    use crate::type_meta::TypeKind;

    use super::*;

    #[test]
    fn alias_map_test() {
        // SSIZE_T  -> LONG_PTR -> c_longlong
        // SSIZE_T2 -> LONG_PTR -> c_longlong

        let mut map = AliasMap::new();
        map.insert(
            &"SSIZE_T".to_string(),
            &RustType {
                type_name: "LONG_PTR".to_string(),
                type_kind: TypeKind::Normal,
            },
        );
        map.insert(
            &"LONG_PTR".to_string(),
            &RustType {
                type_name: "c_longlong".to_string(),
                type_kind: TypeKind::Normal,
            },
        );
        map.insert(
            &"SSIZE_T2".to_string(),
            &RustType {
                type_name: "LONG_PTR".to_string(),
                type_kind: TypeKind::Normal,
            },
        );

        assert_eq!(map.normalize(&"SSIZE_T".to_string()).0, "c_longlong");
        assert_eq!(map.normalize(&"SSIZE_T2".to_string()).0, "c_longlong");
        assert_eq!(map.normalize(&"c_longlong".to_string()).0, "c_longlong");
        assert_eq!(map.normalize(&"c_longlong".to_string()).0, "c_longlong");

        assert_eq!(
            map.get_mapped_value(&"SSIZE_T".to_string())
                .unwrap()
                .type_name,
            "c_longlong"
        );
        assert_eq!(
            map.get_mapped_value(&"SSIZE_T2".to_string())
                .unwrap()
                .type_name,
            "c_longlong"
        );
        assert_eq!(
            map.get_mapped_value(&"LONG_PTR".to_string())
                .unwrap()
                .type_name,
            "c_longlong"
        );
        assert!(map.get_mapped_value(&"c_longlong".to_string()).is_none());
    }
}
