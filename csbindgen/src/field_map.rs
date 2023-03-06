use std::{
    cell::RefCell,
    collections::{HashMap, HashSet},
};

#[derive(Clone, Debug)]
pub struct FieldMap {
    fields: HashMap<String, RefCell<HashSet<String>>>, // field_type_name -> DeclaringType(s)
}

impl FieldMap {
    pub fn new() -> Self {
        Self {
            fields: HashMap::new(),
        }
    }

    // type_name must be normalized
    pub fn insert(&mut self, type_name: &String, field_type: &String) {
        match self.fields.get(field_type) {
            Some(x) => {
                x.borrow_mut().insert(type_name.to_owned());
            }
            None => {
                let map = RefCell::new(HashSet::new());
                map.borrow_mut().insert(type_name.to_owned());

                self.fields.insert(field_type.to_owned(), map);
            }
        }
    }

    pub fn exists_in_using_types(
        &self,
        struct_name: &String,
        using_types: &HashSet<String>,
        recursive_count: i32, // detect recrusive reference
    ) -> bool {
        if recursive_count >= 10 {
            return false;
        }

        if using_types.contains(struct_name) {
            return true;
        }

        // try to find declaring types
        if let Some(x) = self.fields.get(struct_name) {
            for name in x.borrow().iter() {
                if self.exists_in_using_types(name, using_types, recursive_count + 1) {
                    return true;
                }
            }
        }

        false
    }
}
