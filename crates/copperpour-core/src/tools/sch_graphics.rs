//! Drawing-only schematic items: polylines, rectangles, circles and text.
//!
//! These carry no electrical meaning, so they are written as targeted edits and
//! leave every other byte of the file as it was.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::{get_path, invalid_arg, require_array, require_str, ToolDef};
use copperpour_sexp::{
    geometry::round6,
    writer::{new_uuid, read_consistent, write_atomic_if_unchanged},
};
use serde_json::{json, Value};

use super::sch_batch::{schematic_text_font, schematic_text_justify, DEFAULT_TEXT_JUSTIFY};
use super::sch_wiring::insert_before_close;

pub fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "add_schematic_graphics",
            "Add graphic polylines, rectangles, circles and text, which have no electrical \
             meaning, in one write. Each item has kind polyline {points: [{x, y}, ...]}, \
             rectangle {x1, y1, x2, y2}, circle {x, y, radius} or text {text, x, y, size, \
             justify, rotation}. Optional on a shape: width (mm, 0 = default), line_type \
             (default, solid, dash, dot, dash_dot, dash_dot_dot) and color ([r, g, b, a]; r, g, \
             b are 0-255, a is 0-1). One bad item refuses the whole call and writes nothing.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "items": {
                        "type": "array",
                        "items": {
                            "type": "object",
                            "properties": {
                                "kind": {
                                    "type": "string",
                                    "enum": ["polyline", "rectangle", "circle", "text"]
                                },
                                "points": {
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "properties": {
                                            "x": { "type": "number" },
                                            "y": { "type": "number" }
                                        },
                                        "required": ["x", "y"]
                                    }
                                },
                                "x1": { "type": "number" },
                                "y1": { "type": "number" },
                                "x2": { "type": "number" },
                                "y2": { "type": "number" },
                                "x": { "type": "number" },
                                "y": { "type": "number" },
                                "radius": { "type": "number" },
                                "text": { "type": "string" },
                                "size": { "type": "number" },
                                "justify": { "type": "string" },
                                "rotation": { "type": "number" },
                                "bold": { "type": "boolean" },
                                "italic": { "type": "boolean" },
                                "thickness": { "type": "number" },
                                "width": { "type": "number" },
                                "line_type": { "type": "string" },
                                "color": {
                                    "type": "array",
                                    "items": { "type": "number" },
                                    "minItems": 4,
                                    "maxItems": 4
                                }
                            },
                            "required": ["kind"]
                        },
                        "description": "Items to add"
                    }
                },
                "required": ["schematic", "items"]
            }),
            |args, ctx| async move { handle_add_graphics(args, ctx).await }
        ),
        tool!(
            "list_schematic_graphics",
            "List top-level text, polylines, rectangles, circles and arcs with their UUIDs. \
             Delete them with batch_delete, edit text with edit_schematic_text.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" }
                },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_list_graphics(args, ctx).await }
        ),
        tool!(
            "edit_schematic_text",
            "Change a top-level text item's content, position or size, found by UUID.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "uuid": { "type": "string", "description": "UUID from list_schematic_graphics" },
                    "text": { "type": "string", "description": "New content" },
                    "x": { "type": "number", "description": "New X in mm (with y)" },
                    "y": { "type": "number", "description": "New Y in mm (with x)" },
                    "size": { "type": "number", "description": "New font size in mm" }
                },
                "required": ["schematic", "uuid"]
            }),
            |args, ctx| async move { handle_edit_text(args, ctx).await }
        ),
    ]
}

/// Byte ranges of the root's direct children, whatever the file's indentation.
/// Tracks strings, so parentheses in text don't count.
fn top_level_items(content: &str) -> Vec<(usize, usize)> {
    let (mut depth, mut in_str, mut esc, mut start) = (0usize, false, false, 0usize);
    let mut out = Vec::new();
    for (i, c) in content.char_indices() {
        if in_str {
            match (esc, c) {
                (true, _) => esc = false,
                (false, '\\') => esc = true,
                (false, '"') => in_str = false,
                _ => {}
            }
            continue;
        }
        match c {
            '"' => in_str = true,
            '(' => {
                depth += 1;
                if depth == 2 {
                    start = i;
                }
            }
            ')' => {
                if depth == 2 {
                    out.push((start, i + 1));
                }
                depth = depth.saturating_sub(1);
            }
            _ => {}
        }
    }
    out
}

fn item_kind(block: &str) -> &str {
    block[1..]
        .split(|c: char| c.is_whitespace() || c == '(' || c == ')')
        .next()
        .unwrap_or("")
}

fn sexp_str(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\r', "")
        .replace('\n', "\\n")
        .replace('\t', "\\t")
}

/// A finite coordinate rounded to the six decimals KiCad writes.
fn coord(item: &Value, key: &str) -> Result<f64, String> {
    let value = item[key]
        .as_f64()
        .ok_or_else(|| format!("item needs number '{key}'"))?;
    let rounded = round6(value);
    if rounded.is_finite() {
        Ok(rounded)
    } else {
        Err(format!("'{key}' must be a finite number of millimetres"))
    }
}

fn stroke_sexp(item: &Value) -> Result<String, String> {
    let width = match item.get("width") {
        Some(_) => coord(item, "width")?,
        None => 0.0,
    };
    let line_type = item["line_type"].as_str().unwrap_or("default");
    if !matches!(
        line_type,
        "default" | "solid" | "dash" | "dot" | "dash_dot" | "dash_dot_dot"
    ) {
        return Err(format!("unknown line_type '{line_type}'"));
    }
    let color = match item.get("color") {
        Some(_) => {
            let c = item["color"]
                .as_array()
                .filter(|c| c.len() == 4)
                .ok_or("color must be [r, g, b, a]")?;
            let rgb = |v: &Value| v.as_u64().filter(|n| *n <= 255);
            let (Some(r), Some(g), Some(b), Some(a)) = (
                rgb(&c[0]),
                rgb(&c[1]),
                rgb(&c[2]),
                c[3].as_f64().filter(|a| (0.0..=1.0).contains(a)),
            ) else {
                return Err("color needs r, g, b of 0-255 and a of 0-1".into());
            };
            format!(" (color {r} {g} {b} {a})")
        }
        None => String::new(),
    };
    Ok(format!(
        "(stroke (width {width}) (type {line_type}){color}) (fill (type none))"
    ))
}

fn graphic_sexp(item: &Value, uuid: &str) -> Result<String, String> {
    match item["kind"].as_str().unwrap_or("") {
        "polyline" => {
            let points = item["points"].as_array().map(Vec::as_slice).unwrap_or(&[]);
            if points.len() < 2 {
                return Err("polyline needs at least 2 points".into());
            }
            let mut xy = String::new();
            for point in points {
                let (x, y) = (coord(point, "x")?, coord(point, "y")?);
                xy.push_str(&format!(" (xy {x} {y})"));
            }
            Ok(format!(
                "\n  (polyline (pts{xy}) {} (uuid \"{uuid}\"))",
                stroke_sexp(item)?
            ))
        }
        "rectangle" => Ok(format!(
            "\n  (rectangle (start {} {}) (end {} {}) {} (uuid \"{uuid}\"))",
            coord(item, "x1")?,
            coord(item, "y1")?,
            coord(item, "x2")?,
            coord(item, "y2")?,
            stroke_sexp(item)?
        )),
        "circle" => {
            let radius = coord(item, "radius")?;
            if radius <= 0.0 {
                return Err("circle needs a positive 'radius'".into());
            }
            Ok(format!(
                "\n  (circle (center {} {}) (radius {radius}) {} (uuid \"{uuid}\"))",
                coord(item, "x")?,
                coord(item, "y")?,
                stroke_sexp(item)?
            ))
        }
        "text" => {
            let text = item["text"].as_str().ok_or("text item needs 'text'")?;
            let size = item["size"].as_f64().unwrap_or(1.27);
            let rotation = match item.get("rotation") {
                Some(_) => coord(item, "rotation")?,
                None => 0.0,
            };
            let justify = item["justify"].as_str().unwrap_or(DEFAULT_TEXT_JUSTIFY);
            Ok(format!(
                "\n  (text \"{}\" (exclude_from_sim no) (at {} {} {rotation}) (effects {}{}) \
                 (uuid \"{uuid}\"))",
                sexp_str(text),
                coord(item, "x")?,
                coord(item, "y")?,
                schematic_text_font(item, size)?,
                schematic_text_justify(justify)?,
            ))
        }
        other => Err(format!("unknown graphic kind '{other}'")),
    }
}

async fn handle_add_graphics(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let items = match require_array(args, "items") {
        Ok(items) => items,
        Err(e) => return Ok(e),
    };
    let mut block = String::new();
    let mut uuids = Vec::new();
    for (i, item) in items.iter().enumerate() {
        let uuid = new_uuid();
        match graphic_sexp(item, &uuid) {
            Ok(s) => block.push_str(&s),
            // All or nothing: a half-drawn sheet is worse than an error.
            Err(e) => return Ok(invalid_arg("items", &format!("item {i}: {e}"))),
        }
        uuids.push(uuid);
    }
    let content = read_consistent(&sch_path)?;
    let new_content = insert_before_close(&content, &block);
    write_atomic_if_unchanged(&sch_path, &content, &new_content)?;
    Ok(CallToolResult::json(&json!({
        "added_count": uuids.len(),
        "uuids": uuids
    })))
}

async fn handle_list_graphics(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let content = read_consistent(&sch_path)?;
    let mut out = Vec::new();
    for (s, e) in top_level_items(&content) {
        let block = &content[s..e];
        let kind = item_kind(block);
        if !matches!(kind, "text" | "polyline" | "rectangle" | "circle" | "arc") {
            continue;
        }
        let uuid = block
            .rfind("(uuid \"")
            .and_then(|i| block[i + 7..].split('"').next())
            .unwrap_or("");
        let mut entry = json!({ "kind": kind, "uuid": uuid });
        let mut search_from = 0;
        if kind == "text" {
            let (text, (_, end)) = first_string(block);
            entry["text"] = json!(text);
            search_from = end;
        }
        let rest = &block[search_from..];
        for key in ["(at ", "(start ", "(center ", "(xy "] {
            if let Some(i) = rest.find(key) {
                let nums: Vec<f64> = rest[i + key.len()..]
                    .split(')')
                    .next()
                    .unwrap_or("")
                    .split_whitespace()
                    .filter_map(|n| n.parse().ok())
                    .collect();
                if nums.len() >= 2 {
                    entry["x"] = json!(nums[0]);
                    entry["y"] = json!(nums[1]);
                    break;
                }
            }
        }
        out.push(entry);
    }
    Ok(CallToolResult::json(&json!({ "items": out })))
}

/// First quoted string in a block: (unescaped text, byte range of the raw string with quotes).
fn first_string(block: &str) -> (String, (usize, usize)) {
    let Some(start) = block.find('"') else {
        return (String::new(), (0, 0));
    };
    let (mut text, mut esc) = (String::new(), false);
    for (i, c) in block[start + 1..].char_indices() {
        match (esc, c) {
            (true, 'n') => {
                text.push('\n');
                esc = false;
            }
            (true, 't') => {
                text.push('\t');
                esc = false;
            }
            (true, _) => {
                text.push(c);
                esc = false;
            }
            (false, '\\') => esc = true,
            (false, '"') => return (text, (start, start + 1 + i + 1)),
            _ => text.push(c),
        }
    }
    (text, (start, block.len()))
}

async fn handle_edit_text(
    args: &Value,
    _ctx: &crate::tools::ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let uuid = match require_str(args, "uuid") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let (new_x, new_y) = (args["x"].as_f64(), args["y"].as_f64());
    if new_x.is_some() != new_y.is_some() {
        return Ok(invalid_arg("x", "x and y must be given together"));
    }
    if args["text"].is_null() && new_x.is_none() && args["size"].is_null() {
        return Ok(CallToolResult::error(
            "No fields to change: provide at least one of text, x+y, size",
        ));
    }
    let content = read_consistent(&sch_path)?;
    let needle = format!(r#"(uuid "{uuid}")"#);
    let Some(uuid_pos) = content.find(&needle) else {
        return Ok(CallToolResult::error(format!("UUID '{uuid}' not found")));
    };
    let Some((s, e)) = top_level_items(&content)
        .into_iter()
        .find(|&(s, e)| s <= uuid_pos && uuid_pos < e)
    else {
        return Ok(CallToolResult::error(format!(
            "UUID '{uuid}' is not in an item"
        )));
    };
    if item_kind(&content[s..e]) != "text" {
        return Ok(CallToolResult::error(format!(
            "UUID '{uuid}' is not a text item"
        )));
    }
    let mut block = content[s..e].to_string();
    if let Some(text) = args["text"].as_str() {
        let (_, (a, b)) = first_string(&block);
        block.replace_range(a..b, &format!("\"{}\"", sexp_str(text)));
    }
    if let (Some(x), Some(y)) = (new_x, new_y) {
        let (x, y) = (round6(x), round6(y));
        if !x.is_finite() || !y.is_finite() {
            return Ok(invalid_arg("x", "must be a finite number of millimetres"));
        }
        // Search after the string, whose content may itself contain "(at ".
        let from = first_string(&block).1 .1;
        if let Some(i) = block[from..].find("(at ").map(|i| i + from) {
            let end = i + block[i..].find(')').unwrap_or(0);
            let rotation = block[i + 4..end]
                .split_whitespace()
                .nth(2)
                .unwrap_or("0")
                .to_string();
            block.replace_range(i..end, &format!("(at {x} {y} {rotation}"));
        }
    }
    if let Some(size) = args["size"].as_f64() {
        let from = first_string(&block).1 .1;
        if let Some(i) = block[from..].find("(size ").map(|i| i + from) {
            let end = i + block[i..].find(')').unwrap_or(0);
            block.replace_range(i..end, &format!("(size {size} {size}"));
        }
    }
    let new_content = format!("{}{}{}", &content[..s], block, &content[e..]);
    write_atomic_if_unchanged(&sch_path, &content, &new_content)?;
    Ok(CallToolResult::json(&json!({ "edited": uuid })))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::router::ToolRouter;
    use crate::tools::{ServerConfig, ToolContext};
    use std::path::PathBuf;
    use std::sync::Arc;
    use tempfile::TempDir;

    const TABBED: &str = "(kicad_sch\n\t(version 20250114)\n\t(paper \"A4\")\n\t\
        (text \"a (b) \\\"c\\\"\"\n\t\t(at 1 2 0)\n\t\t(effects\n\t\t\t(font\n\t\t\t\t(size 1.27 1.27)\n\t\t\t)\n\t\t)\n\t\t\
        (uuid \"11111111-1111-1111-1111-111111111111\")\n\t)\n\t\
        (sheet_instances\n\t\t(path \"/\"\n\t\t\t(page \"1\")\n\t\t)\n\t)\n)\n";

    fn ctx() -> ToolContext {
        ToolContext::new(
            ServerConfig {
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: String::new(),
                project_dir: None,
                jlcpcb_db_path: None,
                auto_load_toolsets: false,
                eager_toolsets: false,
            },
            Arc::new(ToolRouter::new()),
        )
    }

    fn tabbed_file(dir: &TempDir) -> PathBuf {
        let path = dir.path().join("t.kicad_sch");
        std::fs::write(&path, TABBED).unwrap();
        path
    }

    #[tokio::test]
    async fn graphics_land_before_the_end_and_list_back() {
        let dir = TempDir::new().unwrap();
        let path = tabbed_file(&dir);
        let schematic = path.display().to_string();
        let added = handle_add_graphics(
            &json!({ "schematic": schematic, "items": [
                { "kind": "polyline", "points": [{"x": 0, "y": 0}, {"x": 5, "y": 0}],
                  "color": [200, 0, 0, 1] },
                { "kind": "rectangle", "x1": 1, "y1": 1, "x2": 2, "y2": 2, "line_type": "dash" },
                { "kind": "circle", "x": 3, "y": 3, "radius": 1 },
                { "kind": "text", "text": "say \"hi\"", "x": 4, "y": 4, "justify": "left" }
            ]}),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(!added.is_error, "{:?}", added.content);

        let out = std::fs::read_to_string(&path).unwrap();
        assert!(copperpour_sexp::parser::parse_sexp(&out).is_ok(), "{out}");
        assert!(out.contains("(color 200 0 0 1)"));
        let kinds: Vec<&str> = top_level_items(&out)
            .iter()
            .map(|&(s, e)| item_kind(&out[s..e]))
            .filter(|k| matches!(*k, "polyline" | "rectangle" | "circle" | "text"))
            .collect();
        assert_eq!(kinds.len(), 5, "{kinds:?}");
    }

    #[tokio::test]
    async fn a_bad_item_writes_nothing() {
        for bad in [
            json!({ "kind": "blob" }),
            json!({ "kind": "polyline", "points": [{"x": 0, "y": 0}] }),
            json!({ "kind": "circle", "x": 0, "y": 0, "radius": 0 }),
            json!({ "kind": "rectangle", "x1": 1, "y1": 1, "x2": 2, "y2": 2, "line_type": "wavy" }),
            json!({ "kind": "rectangle", "x1": 1, "y1": 1, "x2": 2, "y2": 2, "color": [999, 0, 0, 1] }),
        ] {
            let dir = TempDir::new().unwrap();
            let path = tabbed_file(&dir);
            let result = handle_add_graphics(
                &json!({ "schematic": path.display().to_string(), "items": [
                    { "kind": "rectangle", "x1": 1, "y1": 1, "x2": 2, "y2": 2 }, bad
                ]}),
                &ctx(),
            )
            .await
            .unwrap();
            assert!(result.is_error);
            assert_eq!(std::fs::read_to_string(&path).unwrap(), TABBED);
        }
    }

    #[tokio::test]
    async fn edit_text_keeps_parens_in_content_apart_from_fields() {
        let dir = TempDir::new().unwrap();
        let path = tabbed_file(&dir);
        handle_edit_text(
            &json!({ "schematic": path.display().to_string(),
                     "uuid": "11111111-1111-1111-1111-111111111111",
                     "text": "new (at 9 9 0) text", "x": 7, "y": 8, "size": 2 }),
            &ctx(),
        )
        .await
        .unwrap();
        let out = std::fs::read_to_string(&path).unwrap();
        assert!(out.contains("(text \"new (at 9 9 0) text\""), "{out}");
        assert!(out.contains("(at 7 8 0)"), "{out}");
        assert!(out.contains("(size 2 2)"), "{out}");
    }

    #[tokio::test]
    async fn edit_text_refuses_a_uuid_that_is_not_text() {
        let dir = TempDir::new().unwrap();
        let path = tabbed_file(&dir);
        let schematic = path.display().to_string();
        handle_add_graphics(
            &json!({ "schematic": schematic, "items": [
                { "kind": "rectangle", "x1": 1, "y1": 1, "x2": 2, "y2": 2 } ] }),
            &ctx(),
        )
        .await
        .unwrap();
        let out = std::fs::read_to_string(&path).unwrap();
        let (s, e) = top_level_items(&out)
            .into_iter()
            .find(|&(s, e)| item_kind(&out[s..e]) == "rectangle")
            .unwrap();
        let uuid = out[s..e].rsplit('"').nth(1).unwrap().to_string();
        let result = handle_edit_text(
            &json!({ "schematic": schematic, "uuid": uuid, "text": "x" }),
            &ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error);
    }
}
