use std::{cell::RefCell, collections::HashMap, rc::Rc};

use crate::type_meta::{RustType, TypeKind};

#[derive(Clone, Debug)]
pub struct AliasMap {
    nodes: HashMap<String, Rc<RefCell<Node>>>,
}

impl AliasMap {
    pub fn new() -> Self {
        Self {
            nodes: HashMap::new(),
        }
    }
}

#[derive(Clone, Debug)]
struct Node {
    pub value: RustType,
    pub next: Option<Rc<RefCell<Node>>>,
}

impl Node {
    pub fn get_last_value(&self) -> RustType {
        match &self.next {
            Some(x) => x.borrow().get_last_value(),
            None => self.value.clone(),
        }
    }
}

impl AliasMap {
    pub fn insert(&mut self, name: &String, alias: &RustType) {
        match (self.nodes.get(name), self.nodes.get(&alias.type_name)) {
            (Some(_), Some(_)) => {} // duplicate is not allowed in system
            (Some(left), None) => {
                let right_node = Rc::new(RefCell::new(Node {
                    value: alias.clone(),
                    next: None,
                }));

                left.borrow_mut().next = Some(right_node.clone());
                self.nodes.insert(alias.type_name.to_owned(), right_node);
            }
            (None, Some(right)) => {
                let left_node = Rc::new(RefCell::new(Node {
                    value: RustType {
                        type_name: name.to_owned(),
                        type_kind: TypeKind::Normal,
                    },
                    next: Some(right.clone()),
                }));
                self.nodes.insert(name.to_owned(), left_node);
            }
            (None, None) => {
                let right_node = Rc::new(RefCell::new(Node {
                    value: alias.clone(),
                    next: None,
                }));

                let left_node = Rc::new(RefCell::new(Node {
                    value: RustType {
                        type_name: name.to_owned(),
                        type_kind: TypeKind::Normal,
                    },
                    next: Some(right_node.clone()),
                }));

                self.nodes.insert(name.to_owned(), left_node);
                self.nodes.insert(alias.type_name.to_owned(), right_node);
            }
        }
    }

    #[allow(dead_code)]
    pub fn contains(&self, name: &String) -> bool {
        self.nodes.contains_key(name)
    }

    pub fn get_mapped_value(&self, name: &String) -> Option<RustType> {
        match self.nodes.get(name) {
            Some(x) => Some(x.borrow().get_last_value()),
            None => None,
        }
    }
}

#[cfg(test)]
mod test {
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

        assert!(map.contains(&"LONG_PTR".to_string()));
        assert!(map.contains(&"c_longlong".to_string()));
        assert!(map.contains(&"SSIZE_T".to_string()));
        assert!(map.contains(&"SSIZE_T2".to_string()));

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
        assert_eq!(
            map.get_mapped_value(&"c_longlong".to_string())
                .unwrap()
                .type_name,
            "c_longlong"
        );
    }
}
