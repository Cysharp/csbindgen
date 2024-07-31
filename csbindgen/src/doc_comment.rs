use syn::Attribute;

use crate::util::get_str_from_meta;

pub fn gather_docs(attrs: &[Attribute]) -> Vec<String> {
    attrs
        .iter()
        .filter(|x| x.path().is_ident("doc"))
        .filter_map(|x| get_str_from_meta(&x.meta))
        .collect::<Vec<_>>()
}

pub fn escape_doc_comment(doc_comment: &[String], indent: &str) -> Option<String> {
    if doc_comment.is_empty() {
        return None;
    }

    let mut lines = Vec::with_capacity(doc_comment.len() + 2);

    lines.push(format!("{}/// <summary>", indent));

    for comment in doc_comment.iter() {
        if comment.trim().is_empty() {
            lines.push(format!("{}///", indent));
        } else {
            for line in comment.lines() {
                lines.push(format!(
                    "{}/// {}",
                    indent,
                    line.replace("&", "&amp;")
                        .replace("<", "&lt;")
                        .replace(">", "&gt;"),
                ));
            }
        }
    }

    lines.push(format!("{}/// </summary>", indent));

    Some(lines.join("\n"))
}
