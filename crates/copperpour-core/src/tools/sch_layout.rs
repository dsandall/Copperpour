//! `sch_layout` toolset — placement-first tooling.
//!
//! Doctrine: placement is more than half the battle. The model decides *what*
//! goes *where* deliberately; these tools assign and validate the geometry —
//! pin-collinear rows, array placement transfer, and a static placement
//! oracle. Routing (in `sch_wiring`) is a thin consumer of good placement: it
//! never avoids obstacles, it reports interference so placement gets fixed.
//!
//! All placement is translation-only (the caller pre-sets rotation); pins are
//! kept on the 1.27 mm grid.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::{find_instance_index_unique, get_path, opt_f64, opt_str, ToolContext, ToolDef};
use copperpour_schematic_editor as cse;
use copperpour_sexp::{
    bbox::{field_text_bbox_estimate, instance_extent, symbol_bbox_for_instance, Rect},
    geometry::{points_coincident, snap_point, snap_to_grid},
    parser::SexpNode,
    schematic::{
        extract_lib_pins_resolved, extract_symbol_instances_with_nodes, extract_wires,
        pin_endpoint, read_schematic, LibPin, SymbolInstance,
    },
};
use serde_json::json;

pub fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "place_row",
            "Place a chain of components in a row so that each component's in-flow pin is \
             COLLINEAR with the previous component's out-flow pin (same Y for direction \
             'right', same X for 'down'), with at least min_gap of free space between the \
             occupied extents (symbol body + visible field text). Components are moved by \
             translation only — set rotations first. All pin positions land on the 1.27mm \
             grid. Returns each component's final position and its in/out pin coordinates \
             so you can route or chain further rows.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "chain": {
                        "type": "array",
                        "minItems": 1,
                        "description": "Components in flow order. in_pin/out_pin are pin NUMBERS; when omitted the symbol origin is used as the connection point.",
                        "items": {
                            "type": "object",
                            "properties": {
                                "ref": { "type": "string", "description": "Reference designator (e.g. 'R1')" },
                                "uuid": { "type": "string", "description": "Symbol UUID — required only when the reference is ambiguous" },
                                "in_pin": { "type": "string", "description": "Pin number that receives the incoming signal" },
                                "out_pin": { "type": "string", "description": "Pin number that feeds the next component" }
                            },
                            "required": ["ref"]
                        }
                    },
                    "start": {
                        "type": "object",
                        "description": "Where the first component's in-flow pin (or origin) goes. Snapped to the 1.27mm grid.",
                        "properties": {
                            "x": { "type": "number" },
                            "y": { "type": "number" }
                        },
                        "required": ["x", "y"]
                    },
                    "direction": { "type": "string", "enum": ["right", "down"], "default": "right" },
                    "min_gap": {
                        "type": "number",
                        "default": 2.54,
                        "description": "Minimum free space in mm between adjacent components' extents (floor 2.54)"
                    }
                },
                "required": ["schematic", "chain", "start"]
            }),
            |args, ctx| async move { handle_place_row(args, ctx).await }
        ),
        tool!(
            "repeat_placement",
            "Transfer a placement pattern: compute each prototype component's offset from \
             the prototype group origin, then place every instance group with the same \
             offsets at origin + k*pitch along the direction axis. pitch 'auto' = prototype \
             extent size along the direction + 2x2.54mm clearance, snapped UP to a 2.54 \
             multiple. With order_from, instance groups are assigned to slots in the row \
             order of the listed hub pins (one pin per instance group, positionally) — the \
             crossing-minimization hook. Placement only: never creates or deletes \
             components; rotation/mirror are copied from the prototype counterpart.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "prototype": {
                        "type": "array",
                        "minItems": 1,
                        "items": { "type": "string" },
                        "description": "Reference designators of the already-placed prototype group. The first ref's position is the group origin."
                    },
                    "instances": {
                        "type": "array",
                        "minItems": 1,
                        "items": { "type": "array", "items": { "type": "string" } },
                        "description": "Instance groups; each inner array corresponds 1:1 positionally to `prototype`."
                    },
                    "pitch": {
                        "description": "Spacing in mm between consecutive group origins, or 'auto'.",
                        "anyOf": [
                            { "type": "number", "exclusiveMinimum": 0 },
                            { "type": "string", "enum": ["auto"] }
                        ],
                        "default": "auto"
                    },
                    "direction": { "type": "string", "enum": ["down", "right"], "default": "down" },
                    "order_from": {
                        "type": "object",
                        "description": "Hub component whose pin row order assigns instance groups to slots (minimizes wire crossings). pins[k] belongs to instances[k]; groups are sorted into the pins' coordinate order along the direction axis.",
                        "properties": {
                            "ref": { "type": "string" },
                            "pins": { "type": "array", "items": { "type": "string" }, "description": "One pin number per instance group" }
                        },
                        "required": ["ref", "pins"]
                    }
                },
                "required": ["schematic", "prototype", "instances"]
            }),
            |args, ctx| async move { handle_repeat_placement(args, ctx).await }
        ),
        tool!(
            "check_placement",
            "Static placement oracle (read-only, never modifies the file). Reports: \
             'overlap' — two occupied extents intersect (symbol-body against symbol-body is \
             an error; field text against anything is a warning), with the intersection \
             rectangle; 'off_grid_pin' — a pin endpoint not on the grid within 0.01mm; \
             'misaligned_pair' — a non-axis-aligned wire directly joining two component \
             pins (their shared axis coordinate differs, i.e. the placement needs a nudge). \
             Fix placement until this is clean before routing.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "include_text": {
                        "type": "boolean",
                        "default": true,
                        "description": "Also check estimated Reference/Value text boxes for overlap"
                    },
                    "grid": { "type": "number", "default": 1.27, "description": "Grid pitch in mm for the pin check" }
                },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_check_placement(args, ctx).await }
        ),
    ]
}

// ─── Shared helpers ──────────────────────────────────────────────────────────

fn rect_json(r: &Rect) -> serde_json::Value {
    json!({ "min_x": r.min_x, "min_y": r.min_y, "max_x": r.max_x, "max_y": r.max_y })
}

/// Snap a value UP to the next multiple of `grid` (values already on the grid
/// stay put).
fn snap_up(v: f64, grid: f64) -> f64 {
    ((v - 1e-9) / grid).ceil() * grid
}

/// Resolved pins (with world endpoints) of an instance, via its embedded
/// library symbol and the extends chain.
#[allow(clippy::type_complexity)]
fn pins_with_endpoints(
    lib_syms: &[&SexpNode],
    inst: &SymbolInstance,
) -> Result<Vec<(LibPin, (f64, f64))>, String> {
    let sym = lib_syms
        .iter()
        .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(inst.lib_id.as_str()))
        .ok_or_else(|| {
            format!(
                "Library symbol '{}' (for '{}') is not embedded in this schematic",
                inst.lib_id, inst.reference
            )
        })?;
    let t = inst.pin_transform();
    Ok(extract_lib_pins_resolved(lib_syms, sym)
        .into_iter()
        .map(|p| {
            let ep = pin_endpoint(&p, t);
            (p, ep)
        })
        .collect())
}

/// Occupied extent of an instance, falling back to a point at its origin when
/// the library symbol can't be resolved (conservative enough for spacing).
fn extent_or_origin(
    lib_syms: &[&SexpNode],
    inst: &SymbolInstance,
    node: &SexpNode,
    include_text: bool,
) -> Rect {
    instance_extent(lib_syms, inst, node, include_text)
        .unwrap_or_else(|| Rect::from_point(inst.x, inst.y))
}

/// Mutable handle to the cse symbol matching a parsed instance (by uuid when
/// available, else by unique reference).
fn cse_symbol_mut<'a>(
    sch: &'a mut cse::Schematic,
    inst: &SymbolInstance,
) -> Result<&'a mut cse::Symbol, String> {
    if let Some(u) = &inst.uuid {
        return sch
            .symbols
            .iter_mut()
            .find(|s| &s.uuid == u)
            .ok_or_else(|| format!("Component uuid '{u}' not found on write-back"));
    }
    let mut matches = sch
        .symbols
        .iter_mut()
        .filter(|s| s.reference() == Some(inst.reference.as_str()));
    match (matches.next(), matches.next()) {
        (Some(s), None) => Ok(s),
        (None, _) => Err(format!("Component '{}' not found", inst.reference)),
        (Some(_), Some(_)) => Err(format!(
            "more than one component shares reference '{}': pass a uuid",
            inst.reference
        )),
    }
}

const GRID: f64 = 1.27;

// ─── place_row ───────────────────────────────────────────────────────────────

struct RowItem {
    inst: SymbolInstance,
    in_pin: Option<String>,
    out_pin: Option<String>,
    /// World coords at the CURRENT (pre-move) position.
    in_pt: (f64, f64),
    out_pt: (f64, f64),
    extent: Rect,
    delta: (f64, f64),
}

async fn handle_place_row(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let chain = match args["chain"].as_array() {
        Some(a) if !a.is_empty() => a.clone(),
        _ => return Ok(CallToolResult::error("'chain' must be a non-empty array")),
    };
    let (start_x, start_y) = match (args["start"]["x"].as_f64(), args["start"]["y"].as_f64()) {
        (Some(x), Some(y)) => (x, y),
        _ => {
            return Ok(CallToolResult::error(
                "'start' must provide numeric x and y",
            ))
        }
    };
    let direction = opt_str(args, "direction").unwrap_or("right");
    let horizontal = match direction {
        "right" => true,
        "down" => false,
        other => {
            return Ok(CallToolResult::error(format!(
                "direction must be 'right' or 'down', got '{other}'"
            )))
        }
    };
    let gap = opt_f64(args, "min_gap").unwrap_or(2.54).max(2.54);

    let (_, tree) = read_schematic(&sch_path)?;
    let lib_syms = tree
        .find("lib_symbols")
        .map(|n| n.find_all("symbol"))
        .unwrap_or_default();
    let pairs = extract_symbol_instances_with_nodes(&tree);
    let instances: Vec<SymbolInstance> = pairs.iter().map(|(i, _)| i.clone()).collect();

    // Resolve every chain item BEFORE moving anything.
    let mut items: Vec<RowItem> = Vec::new();
    for el in &chain {
        let reference = match el["ref"].as_str() {
            Some(r) => r,
            None => return Ok(CallToolResult::error("chain item missing 'ref'")),
        };
        let idx = match find_instance_index_unique(&instances, reference, el["uuid"].as_str()) {
            Ok(i) => i,
            Err(e) => return Ok(CallToolResult::error(e)),
        };
        let (inst, node) = (&pairs[idx].0, pairs[idx].1);

        let pin_pt = |pin_arg: &serde_json::Value| -> Result<((f64, f64), Option<String>), String> {
            match pin_arg.as_str() {
                Some(num) => {
                    let pins = pins_with_endpoints(&lib_syms, inst)?;
                    let (_, ep) = pins
                        .iter()
                        .find(|(p, _)| p.number == num)
                        .ok_or_else(|| format!("Pin '{num}' not found on '{reference}'"))?;
                    Ok((*ep, Some(num.to_string())))
                }
                None => Ok(((inst.x, inst.y), None)),
            }
        };
        let (in_pt, in_pin) = match pin_pt(&el["in_pin"]) {
            Ok(v) => v,
            Err(e) => return Ok(CallToolResult::error(e)),
        };
        let (out_pt, out_pin) = match pin_pt(&el["out_pin"]) {
            Ok(v) => v,
            Err(e) => return Ok(CallToolResult::error(e)),
        };

        items.push(RowItem {
            inst: inst.clone(),
            in_pin,
            out_pin,
            in_pt,
            out_pt,
            extent: extent_or_origin(&lib_syms, inst, node, true),
            delta: (0.0, 0.0),
        });
    }

    // Compute translations. Everything downstream of `start` stays on-grid
    // because pin offsets from the snapped anchors are library-grid multiples.
    let (sx, sy) = snap_point(start_x, start_y, GRID);
    let main = |p: (f64, f64)| if horizontal { p.0 } else { p.1 };
    let cross = |p: (f64, f64)| if horizontal { p.1 } else { p.0 };
    let mk_delta = |dm: f64, dc: f64| if horizontal { (dm, dc) } else { (dc, dm) };
    let extent_main = |r: &Rect| {
        if horizontal {
            (r.min_x, r.max_x)
        } else {
            (r.min_y, r.max_y)
        }
    };
    let shift = |r: &Rect, d: (f64, f64)| Rect {
        min_x: r.min_x + d.0,
        min_y: r.min_y + d.1,
        max_x: r.max_x + d.0,
        max_y: r.max_y + d.1,
    };

    items[0].delta = (sx - items[0].in_pt.0, sy - items[0].in_pt.1);
    for k in 1..items.len() {
        let prev = &items[k - 1];
        let prev_extent = shift(&prev.extent, prev.delta);
        let prev_out = (prev.out_pt.0 + prev.delta.0, prev.out_pt.1 + prev.delta.1);

        let it = &items[k];
        let dc = cross(prev_out) - cross(it.in_pt);
        // Along the flow axis: closest position honoring the free-space gap,
        // then snapped UP so the pin stays on the grid.
        let (ext_min, _) = extent_main(&it.extent);
        let dm_gap = extent_main(&prev_extent).1 + gap - ext_min;
        let in_main_raw = (main(it.in_pt) + dm_gap).max(main(prev_out) + GRID);
        let in_main = snap_up(in_main_raw, GRID);
        items[k].delta = mk_delta(in_main - main(it.in_pt), dc);
    }

    // Apply.
    let mut sch = cse::Schematic::load(&sch_path)?;
    for it in &items {
        let sym = match cse_symbol_mut(&mut sch, &it.inst) {
            Ok(s) => s,
            Err(e) => return Ok(CallToolResult::error(e)),
        };
        sym.translate(it.delta.0, it.delta.1);
    }
    sch.overwrite()?;

    let placements: Vec<serde_json::Value> = items
        .iter()
        .map(|it| {
            let pin_json = |pt: (f64, f64), num: &Option<String>| {
                json!({
                    "pin": num,
                    "x": pt.0 + it.delta.0,
                    "y": pt.1 + it.delta.1
                })
            };
            json!({
                "ref": it.inst.reference,
                "uuid": it.inst.uuid,
                "position": { "x": it.inst.x + it.delta.0, "y": it.inst.y + it.delta.1 },
                "in": pin_json(it.in_pt, &it.in_pin),
                "out": pin_json(it.out_pt, &it.out_pin)
            })
        })
        .collect();

    Ok(CallToolResult::json(&json!({
        "direction": direction,
        "start": { "x": sx, "y": sy },
        "gap": gap,
        "placements": placements
    })))
}

// ─── repeat_placement ────────────────────────────────────────────────────────

async fn handle_repeat_placement(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let proto_refs: Vec<String> = match args["prototype"].as_array() {
        Some(a) if !a.is_empty() => a
            .iter()
            .filter_map(|v| v.as_str().map(String::from))
            .collect(),
        _ => {
            return Ok(CallToolResult::error(
                "'prototype' must be a non-empty array of references",
            ))
        }
    };
    let groups_raw = match args["instances"].as_array() {
        Some(a) if !a.is_empty() => a.clone(),
        _ => {
            return Ok(CallToolResult::error(
                "'instances' must be a non-empty array of reference arrays",
            ))
        }
    };
    let direction = opt_str(args, "direction").unwrap_or("down");
    let vertical = match direction {
        "down" => true,
        "right" => false,
        other => {
            return Ok(CallToolResult::error(format!(
                "direction must be 'down' or 'right', got '{other}'"
            )))
        }
    };

    let (_, tree) = read_schematic(&sch_path)?;
    let lib_syms = tree
        .find("lib_symbols")
        .map(|n| n.find_all("symbol"))
        .unwrap_or_default();
    let pairs = extract_symbol_instances_with_nodes(&tree);
    let instances: Vec<SymbolInstance> = pairs.iter().map(|(i, _)| i.clone()).collect();

    let resolve = |r: &str| find_instance_index_unique(&instances, r, None);

    // Prototype geometry.
    let mut proto_idx: Vec<usize> = Vec::new();
    for r in &proto_refs {
        match resolve(r) {
            Ok(i) => proto_idx.push(i),
            Err(e) => return Ok(CallToolResult::error(e)),
        }
    }
    let origin = (instances[proto_idx[0]].x, instances[proto_idx[0]].y);
    let offsets: Vec<(f64, f64)> = proto_idx
        .iter()
        .map(|&i| (instances[i].x - origin.0, instances[i].y - origin.1))
        .collect();
    let proto_extent = proto_idx
        .iter()
        .map(|&i| extent_or_origin(&lib_syms, &instances[i], pairs[i].1, true))
        .reduce(|a, b| a.union(&b))
        .unwrap_or_else(|| Rect::from_point(origin.0, origin.1));

    // Pitch.
    let (pitch, pitch_mode) = match &args["pitch"] {
        v if v.is_number() => {
            let p = v.as_f64().unwrap_or(0.0);
            if p <= 0.0 {
                return Ok(CallToolResult::error("'pitch' must be > 0"));
            }
            (p, "explicit")
        }
        v if v.is_null() || v.as_str() == Some("auto") => {
            let size = if vertical {
                proto_extent.height()
            } else {
                proto_extent.width()
            };
            (snap_up(size + 2.0 * 2.54, 2.54), "auto")
        }
        other => {
            return Ok(CallToolResult::error(format!(
                "'pitch' must be a positive number or \"auto\", got {other}"
            )))
        }
    };

    // Instance groups.
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for (gi, g) in groups_raw.iter().enumerate() {
        let refs: Vec<&str> = g
            .as_array()
            .map(|a| a.iter().filter_map(|v| v.as_str()).collect())
            .unwrap_or_default();
        if refs.len() != proto_refs.len() {
            return Ok(CallToolResult::error(format!(
                "instance group {gi} has {} refs but the prototype has {} — groups must \
                 correspond 1:1 positionally",
                refs.len(),
                proto_refs.len()
            )));
        }
        let mut idxs = Vec::new();
        for r in refs {
            match resolve(r) {
                Ok(i) => idxs.push(i),
                Err(e) => return Ok(CallToolResult::error(e)),
            }
        }
        groups.push(idxs);
    }

    // Slot assignment (crossing minimization via order_from).
    let mut slots: Vec<usize> = (0..groups.len()).collect();
    let mut hub_pin_info: Vec<Option<serde_json::Value>> = vec![None; groups.len()];
    if let Some(of) = args["order_from"].as_object() {
        let hub_ref = match of.get("ref").and_then(|v| v.as_str()) {
            Some(r) => r,
            None => return Ok(CallToolResult::error("'order_from.ref' is required")),
        };
        let pin_nums: Vec<&str> = of
            .get("pins")
            .and_then(|v| v.as_array())
            .map(|a| a.iter().filter_map(|v| v.as_str()).collect())
            .unwrap_or_default();
        if pin_nums.len() != groups.len() {
            return Ok(CallToolResult::error(format!(
                "'order_from.pins' has {} pins but there are {} instance groups — list one \
                 hub pin per group, positionally",
                pin_nums.len(),
                groups.len()
            )));
        }
        let hub_idx = match resolve(hub_ref) {
            Ok(i) => i,
            Err(e) => return Ok(CallToolResult::error(e)),
        };
        let hub_pins = match pins_with_endpoints(&lib_syms, &instances[hub_idx]) {
            Ok(p) => p,
            Err(e) => return Ok(CallToolResult::error(e)),
        };
        let mut coords: Vec<f64> = Vec::new();
        for num in &pin_nums {
            let Some((_, ep)) = hub_pins.iter().find(|(p, _)| p.number == *num) else {
                return Ok(CallToolResult::error(format!(
                    "Pin '{num}' not found on hub '{hub_ref}'"
                )));
            };
            coords.push(if vertical { ep.1 } else { ep.0 });
        }
        // Rank each group by its hub pin's coordinate along the direction axis:
        // the group whose pin comes first gets the nearest slot.
        let mut order: Vec<usize> = (0..groups.len()).collect();
        order.sort_by(|&a, &b| coords[a].partial_cmp(&coords[b]).unwrap());
        for (rank, &gi) in order.iter().enumerate() {
            slots[gi] = rank;
        }
        for (gi, num) in pin_nums.iter().enumerate() {
            hub_pin_info[gi] = Some(json!({ "pin": num, "coord": coords[gi] }));
        }
    }

    // Apply.
    let mut sch = cse::Schematic::load(&sch_path)?;
    let mut group_reports: Vec<serde_json::Value> = Vec::new();
    for (gi, group) in groups.iter().enumerate() {
        let step = ((slots[gi] + 1) as f64) * pitch;
        let raw_origin = if vertical {
            (origin.0, origin.1 + step)
        } else {
            (origin.0 + step, origin.1)
        };
        let group_origin = snap_point(raw_origin.0, raw_origin.1, GRID);

        let mut members: Vec<serde_json::Value> = Vec::new();
        for (j, &idx) in group.iter().enumerate() {
            let inst = &instances[idx];
            let target = (group_origin.0 + offsets[j].0, group_origin.1 + offsets[j].1);
            let proto_inst = &instances[proto_idx[j]];
            let proto_mirror = pairs[proto_idx[j]]
                .1
                .find("mirror")
                .and_then(|m| m.get(1))
                .and_then(|m| m.as_str())
                .map(String::from);
            let sym = match cse_symbol_mut(&mut sch, inst) {
                Ok(s) => s,
                Err(e) => return Ok(CallToolResult::error(e)),
            };
            let (cx, cy) = sym.position();
            sym.translate(target.0 - cx, target.1 - cy);
            sym.set_rotation(proto_inst.rotation);
            sym.mirror = proto_mirror;
            members.push(json!({
                "ref": inst.reference,
                "maps_to": proto_inst.reference,
                "x": target.0,
                "y": target.1
            }));
        }
        group_reports.push(json!({
            "group": gi,
            "slot": slots[gi],
            "origin": { "x": group_origin.0, "y": group_origin.1 },
            "hub_pin": hub_pin_info[gi],
            "members": members
        }));
    }
    sch.overwrite()?;

    Ok(CallToolResult::json(&json!({
        "pitch": pitch,
        "pitch_mode": pitch_mode,
        "direction": direction,
        "prototype_origin": { "x": origin.0, "y": origin.1 },
        "prototype_extent": rect_json(&proto_extent),
        "groups": group_reports
    })))
}

// ─── check_placement ─────────────────────────────────────────────────────────

async fn handle_check_placement(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let include_text = args["include_text"].as_bool().unwrap_or(true);
    let grid = opt_f64(args, "grid").unwrap_or(GRID);
    let tol = 0.01;

    let (_, tree) = read_schematic(&sch_path)?;
    let lib_syms = tree
        .find("lib_symbols")
        .map(|n| n.find_all("symbol"))
        .unwrap_or_default();
    let pairs = extract_symbol_instances_with_nodes(&tree);

    let bodies: Vec<Option<Rect>> = pairs
        .iter()
        .map(|(inst, _)| symbol_bbox_for_instance(&lib_syms, inst))
        .collect();
    let texts: Vec<Vec<Rect>> = pairs
        .iter()
        .map(|(_, node)| {
            if include_text {
                field_text_bbox_estimate(node)
            } else {
                Vec::new()
            }
        })
        .collect();

    let mut violations: Vec<serde_json::Value> = Vec::new();

    // Overlaps.
    for i in 0..pairs.len() {
        for j in (i + 1)..pairs.len() {
            let (ri, rj) = (&pairs[i].0.reference, &pairs[j].0.reference);
            if let (Some(a), Some(b)) = (&bodies[i], &bodies[j]) {
                if let Some(ix) = a.intersection(b) {
                    violations.push(json!({
                        "type": "overlap",
                        "severity": "error",
                        "a": { "ref": ri, "part": "body" },
                        "b": { "ref": rj, "part": "body" },
                        "intersection": rect_json(&ix)
                    }));
                }
            }
            let mut text_overlap = |ta: &Rect, ra: &str, other: &Rect, rb: &str, part_b: &str| {
                if let Some(ix) = ta.intersection(other) {
                    violations.push(json!({
                        "type": "overlap",
                        "severity": "warning",
                        "a": { "ref": ra, "part": "text" },
                        "b": { "ref": rb, "part": part_b },
                        "intersection": rect_json(&ix)
                    }));
                }
            };
            for ta in &texts[i] {
                if let Some(b) = &bodies[j] {
                    text_overlap(ta, ri, b, rj, "body");
                }
                for tb in &texts[j] {
                    text_overlap(ta, ri, tb, rj, "text");
                }
            }
            for tb in &texts[j] {
                if let Some(a) = &bodies[i] {
                    text_overlap(tb, rj, a, ri, "body");
                }
            }
        }
    }

    // Off-grid pins.
    for (inst, _) in &pairs {
        let Ok(pins) = pins_with_endpoints(&lib_syms, inst) else {
            continue; // no embedded definition — nothing to check
        };
        for (pin, (px, py)) in pins {
            let off_x = (px - snap_to_grid(px, grid)).abs() > tol;
            let off_y = (py - snap_to_grid(py, grid)).abs() > tol;
            if off_x || off_y {
                violations.push(json!({
                    "type": "off_grid_pin",
                    "severity": "error",
                    "ref": inst.reference,
                    "pin": pin.number,
                    "at": { "x": px, "y": py },
                    "grid": grid
                }));
            }
        }
    }

    // Misaligned pairs: a wire whose two endpoints are pins of two different
    // components but which is not axis-aligned — the pins were meant to line
    // up and don't.
    let mut all_pins: Vec<(String, String, f64, f64)> = Vec::new();
    for (inst, _) in &pairs {
        if let Ok(pins) = pins_with_endpoints(&lib_syms, inst) {
            for (pin, (px, py)) in pins {
                all_pins.push((inst.reference.clone(), pin.number, px, py));
            }
        }
    }
    for w in extract_wires(&tree) {
        if (w.x1 - w.x2).abs() <= tol || (w.y1 - w.y2).abs() <= tol {
            continue; // axis-aligned — fine
        }
        let pin_at = |x: f64, y: f64| {
            all_pins
                .iter()
                .find(|(_, _, px, py)| points_coincident(x, y, *px, *py, tol))
        };
        if let (Some(a), Some(b)) = (pin_at(w.x1, w.y1), pin_at(w.x2, w.y2)) {
            if a.0 != b.0 {
                violations.push(json!({
                    "type": "misaligned_pair",
                    "severity": "warning",
                    "a": { "ref": a.0, "pin": a.1, "x": a.2, "y": a.3 },
                    "b": { "ref": b.0, "pin": b.1, "x": b.2, "y": b.3 },
                    "delta": { "dx": b.2 - a.2, "dy": b.3 - a.3 }
                }));
            }
        }
    }

    let errors = violations
        .iter()
        .filter(|v| v["severity"] == "error")
        .count();
    Ok(CallToolResult::json(&json!({
        "clean": violations.is_empty(),
        "error_count": errors,
        "warning_count": violations.len() - errors,
        "violations": violations
    })))
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::router::ToolRouter;
    use crate::tools::ServerConfig;
    use copperpour_sexp::schematic::extract_symbol_instances;
    use std::sync::Arc;

    fn test_ctx() -> ToolContext {
        ToolContext::new(
            ServerConfig {
                auto_load_toolsets: false,
                eager_toolsets: false,
                kicad_cli: String::new(),
                kicad_binary: String::new(),
                ipc_address: String::new(),
                project_dir: None,
                jlcpcb_db_path: None,
            },
            Arc::new(ToolRouter::new()),
        )
    }

    /// Library section: a Device:R-shaped part (body ±1.016 × ±2.54, pin tips
    /// at (0, ±3.81)) and a 4-pin hub whose pins sit on the right side at
    /// distinct heights.
    const LIB: &str = r#"  (lib_symbols
    (symbol "Dev:R"
      (property "Reference" "R" (at 2.032 0 90))
      (symbol "R_1_1"
        (rectangle (start -1.016 2.54) (end 1.016 -2.54))
        (pin passive line (at 0 3.81 270) (length 1.27) (name "~") (number "1"))
        (pin passive line (at 0 -3.81 90) (length 1.27) (name "~") (number "2"))
      )
    )
    (symbol "Dev:Hub"
      (property "Reference" "U" (at 0 7.62 0))
      (symbol "Hub_1_1"
        (rectangle (start -3.81 6.35) (end 3.81 -6.35))
        (pin output line (at 6.35 5.08 180) (length 2.54) (name "A") (number "1"))
        (pin output line (at 6.35 0 180) (length 2.54) (name "B") (number "2"))
        (pin output line (at 6.35 -5.08 180) (length 2.54) (name "C") (number "3"))
      )
    )
  )"#;

    fn placed(lib: &str, reference: &str, x: f64, y: f64, rot: f64) -> String {
        format!(
            "  (symbol\n    (lib_id \"{lib}\")\n    (at {x} {y} {rot})\n    (unit 1)\n    (uuid \"u-{reference}\")\n    (property \"Reference\" \"{reference}\"\n      (at {rx} {ry} 0)\n    )\n    (property \"Value\" \"10k\"\n      (at {rx} {vy} 0)\n    )\n  )",
            rx = x + 2.54,
            ry = y - 1.27,
            vy = y + 1.27,
        )
    }

    fn write_sch(body: &str) -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("layout.kicad_sch");
        std::fs::write(
            &path,
            format!(
                "(kicad_sch\n  (version 20250610)\n  (generator \"konnect\")\n  (uuid \"3af69a4c-1faa-40bd-91dc-c4fc245c4cbd\")\n  (paper \"A4\")\n{LIB}\n{body}\n)\n"
            ),
        )
        .unwrap();
        (dir, path)
    }

    fn parse_result(result: &CallToolResult) -> serde_json::Value {
        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text content");
        };
        serde_json::from_str(text).unwrap_or_else(|_| panic!("non-JSON result: {text}"))
    }

    fn on_grid(v: f64) -> bool {
        (v - snap_to_grid(v, 1.27)).abs() < 1e-6
    }

    // ─── place_row ─────────────────────────────────────────────────────────

    #[tokio::test]
    async fn place_row_aligns_pins_collinear_snapped_and_gapped() {
        // Three resistors rotated 90° (pins on the X axis), scattered anywhere.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}",
            placed("Dev:R", "R1", 50.0, 50.0, 90.0),
            placed("Dev:R", "R2", 90.17, 63.5, 90.0),
            placed("Dev:R", "R3", 30.48, 88.9, 90.0),
        ));

        let result = handle_place_row(
            &json!({
                "schematic": path.display().to_string(),
                "chain": [
                    { "ref": "R1", "in_pin": "1", "out_pin": "2" },
                    { "ref": "R2", "in_pin": "1", "out_pin": "2" },
                    { "ref": "R3", "in_pin": "1", "out_pin": "2" }
                ],
                "start": { "x": 101.0, "y": 101.0 }
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);

        let placements = out["placements"].as_array().unwrap();
        assert_eq!(placements.len(), 3);
        // Start snapped to the grid: 101.0 → 101.6.
        assert_eq!(out["start"]["x"], 101.6);
        let flow_y = placements[0]["in"]["y"].as_f64().unwrap();
        for p in placements {
            for end in ["in", "out"] {
                let x = p[end]["x"].as_f64().unwrap();
                let y = p[end]["y"].as_f64().unwrap();
                assert!((y - flow_y).abs() < 1e-6, "pins must be collinear: {out}");
                assert!(on_grid(x), "pin x {x} must be on the 1.27 grid");
                assert!(on_grid(y), "pin y {y} must be on the 1.27 grid");
            }
        }
        assert_eq!(placements[0]["in"]["x"], 101.6);

        // Free-space gap between adjacent extents ≥ 2.54, recomputed from the
        // written file.
        let (_, tree) = read_schematic(&path).unwrap();
        let lib_syms = tree
            .find("lib_symbols")
            .map(|n| n.find_all("symbol"))
            .unwrap_or_default();
        let pairs = extract_symbol_instances_with_nodes(&tree);
        let mut extents: Vec<(String, Rect)> = pairs
            .iter()
            .map(|(i, n)| (i.reference.clone(), extent_or_origin(&lib_syms, i, n, true)))
            .collect();
        extents.sort_by(|a, b| a.1.min_x.partial_cmp(&b.1.min_x).unwrap());
        for w in extents.windows(2) {
            let gap = w[1].1.min_x - w[0].1.max_x;
            assert!(
                gap >= 2.54 - 1e-6,
                "free space between {} and {} is {gap}",
                w[0].0,
                w[1].0
            );
        }

        // Written positions match the returned ones.
        let instances = extract_symbol_instances(&tree);
        for p in placements {
            let inst = instances
                .iter()
                .find(|i| i.reference == p["ref"].as_str().unwrap())
                .unwrap();
            assert!((inst.x - p["position"]["x"].as_f64().unwrap()).abs() < 1e-6);
            assert!((inst.y - p["position"]["y"].as_f64().unwrap()).abs() < 1e-6);
        }
    }

    #[tokio::test]
    async fn place_row_errors_on_ambiguous_reference() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("Dev:R", "R1", 50.0, 50.0, 0.0),
            // Same reference, different uuid — placed() bakes uuid from ref,
            // so patch one in manually.
            placed("Dev:R", "R1", 80.0, 50.0, 0.0).replace("u-R1", "u-R1-dup"),
        ));
        let result = handle_place_row(
            &json!({
                "schematic": path.display().to_string(),
                "chain": [{ "ref": "R1" }],
                "start": { "x": 100, "y": 100 }
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error, "ambiguous ref must error");
        let msg = format!("{:?}", result.content);
        assert!(msg.contains("share reference"), "{msg}");
    }

    // ─── repeat_placement ──────────────────────────────────────────────────

    #[tokio::test]
    async fn repeat_placement_auto_pitch_covers_extent_and_applies_offsets() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}\n{}\n{}",
            placed("Dev:R", "R1", 101.6, 101.6, 0.0),
            placed("Dev:R", "C1", 111.76, 101.6, 0.0),
            placed("Dev:R", "R2", 30.48, 30.48, 90.0),
            placed("Dev:R", "C2", 35.56, 35.56, 90.0),
            placed("Dev:R", "R3", 40.64, 40.64, 90.0),
            placed("Dev:R", "C3", 45.72, 45.72, 90.0),
        ));

        let result = handle_repeat_placement(
            &json!({
                "schematic": path.display().to_string(),
                "prototype": ["R1", "C1"],
                "instances": [["R2", "C2"], ["R3", "C3"]],
                "direction": "down"
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);

        let pitch = out["pitch"].as_f64().unwrap();
        let extent_h = out["prototype_extent"]["max_y"].as_f64().unwrap()
            - out["prototype_extent"]["min_y"].as_f64().unwrap();
        assert!(
            pitch >= extent_h + 2.0 * 2.54 - 1e-6,
            "auto pitch {pitch} must cover the extent {extent_h} + 2×2.54 clearance"
        );
        assert!(
            (pitch / 2.54 - (pitch / 2.54).round()).abs() < 1e-6,
            "auto pitch {pitch} must be a 2.54 multiple"
        );

        let (_, tree) = read_schematic(&path).unwrap();
        let instances = extract_symbol_instances(&tree);
        let pos = |r: &str| {
            let i = instances.iter().find(|i| i.reference == r).unwrap();
            (i.x, i.y, i.rotation)
        };
        // Group 1 sits one pitch below the prototype, group 2 two pitches.
        assert_eq!(pos("R2").0, 101.6);
        assert!(
            (pos("R2").1 - (101.6 + pitch)).abs() < 1e-6,
            "{:?}",
            pos("R2")
        );
        assert!((pos("R3").1 - (101.6 + 2.0 * pitch)).abs() < 1e-6);
        // Offsets within the group replicate the prototype's.
        assert!((pos("C2").0 - (101.6 + 10.16)).abs() < 1e-6);
        assert!((pos("C2").1 - pos("R2").1).abs() < 1e-6);
        // Rotation copied from the prototype counterpart (0°, not the old 90°).
        assert_eq!(pos("R2").2, 0.0);
        assert_eq!(pos("C3").2, 0.0);
    }

    #[tokio::test]
    async fn repeat_placement_order_from_assigns_slots_by_hub_pin_row() {
        // Hub pin 1 (top), 2 (middle), 3 (bottom). Groups listed with pins
        // ["3", "1"]: group 0's hub pin is at the BOTTOM, so it takes the far
        // slot; group 1 (pin 1, top) takes the near slot.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}",
            placed("Dev:Hub", "U1", 50.8, 101.6, 0.0),
            placed("Dev:R", "R1", 101.6, 101.6, 0.0),
            placed("Dev:R", "R2", 30.48, 30.48, 0.0),
            placed("Dev:R", "R3", 35.56, 35.56, 0.0),
        ));

        let result = handle_repeat_placement(
            &json!({
                "schematic": path.display().to_string(),
                "prototype": ["R1"],
                "instances": [["R2"], ["R3"]],
                "direction": "down",
                "order_from": { "ref": "U1", "pins": ["3", "1"] }
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);
        assert_eq!(out["groups"][0]["slot"], 1, "pin 3 is lower → far slot");
        assert_eq!(out["groups"][1]["slot"], 0, "pin 1 is upper → near slot");

        let (_, tree) = read_schematic(&path).unwrap();
        let instances = extract_symbol_instances(&tree);
        let y = |r: &str| instances.iter().find(|i| i.reference == r).unwrap().y;
        assert!(
            y("R3") < y("R2"),
            "R3 (hub pin 1, top) must land above R2 (hub pin 3, bottom): R3.y={} R2.y={}",
            y("R3"),
            y("R2")
        );
    }

    #[tokio::test]
    async fn repeat_placement_rejects_mismatched_group_size() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}",
            placed("Dev:R", "R1", 101.6, 101.6, 0.0),
            placed("Dev:R", "C1", 111.76, 101.6, 0.0),
            placed("Dev:R", "R2", 30.48, 30.48, 0.0),
        ));
        let before = std::fs::read_to_string(&path).unwrap();
        let result = handle_repeat_placement(
            &json!({
                "schematic": path.display().to_string(),
                "prototype": ["R1", "C1"],
                "instances": [["R2"]]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error);
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            before,
            "nothing may move on a rejected call"
        );
    }

    // ─── check_placement ───────────────────────────────────────────────────

    #[tokio::test]
    async fn check_placement_catches_overlap_and_off_grid_pin() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}",
            // R1 and R2 bodies overlap (1.27 apart, bodies are 2.032 wide).
            placed("Dev:R", "R1", 101.6, 101.6, 0.0),
            placed("Dev:R", "R2", 102.87, 101.6, 0.0),
            // R3 far away and off-grid: pins at y = 50.05 ± 3.81.
            placed("Dev:R", "R3", 203.2, 50.05, 0.0),
            // R4 far away, clean.
            placed("Dev:R", "R4", 254.0, 101.6, 0.0),
        ));

        let result = handle_check_placement(
            &json!({ "schematic": path.display().to_string() }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);
        let violations = out["violations"].as_array().unwrap();

        let body_overlaps: Vec<_> = violations
            .iter()
            .filter(|v| {
                v["type"] == "overlap" && v["a"]["part"] == "body" && v["b"]["part"] == "body"
            })
            .collect();
        assert_eq!(body_overlaps.len(), 1, "{out}");
        assert_eq!(body_overlaps[0]["severity"], "error");
        let pair = format!(
            "{}{}",
            body_overlaps[0]["a"]["ref"], body_overlaps[0]["b"]["ref"]
        );
        assert!(pair.contains("R1") && pair.contains("R2"), "{out}");
        assert!(
            body_overlaps[0]["intersection"]["min_x"].is_number(),
            "intersection rect must be reported: {out}"
        );

        let off_grid: Vec<_> = violations
            .iter()
            .filter(|v| v["type"] == "off_grid_pin")
            .collect();
        assert_eq!(off_grid.len(), 2, "both R3 pins are off grid: {out}");
        assert!(off_grid.iter().all(|v| v["ref"] == "R3"));
        assert!(
            !violations
                .iter()
                .any(|v| v["type"] == "off_grid_pin" && v["ref"] == "R4"),
            "R4 is clean"
        );
        assert_eq!(out["clean"], false);
    }

    #[tokio::test]
    async fn check_placement_flags_misaligned_wired_pair_and_respects_include_text() {
        // R1 pin2 at (101.6, 105.41); R2 pin1 at (127.0, 106.68) — 1.27 off in
        // Y, joined by a diagonal wire.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n  (wire\n    (pts (xy 101.6 105.41) (xy 127 106.68))\n    (stroke (width 0) (type default))\n    (uuid \"w1\")\n  )",
            placed("Dev:R", "R1", 101.6, 101.6, 0.0),
            placed("Dev:R", "R2", 127.0, 110.49, 0.0),
        ));

        let result = handle_check_placement(
            &json!({ "schematic": path.display().to_string(), "include_text": false }),
            &test_ctx(),
        )
        .await
        .unwrap();
        let out = parse_result(&result);
        let mis: Vec<_> = out["violations"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|v| v["type"] == "misaligned_pair")
            .collect();
        assert_eq!(mis.len(), 1, "{out}");
        assert_eq!(mis[0]["severity"], "warning");
        assert!((mis[0]["delta"]["dy"].as_f64().unwrap().abs() - 1.27).abs() < 1e-6);
        assert!(
            !out["violations"]
                .as_array()
                .unwrap()
                .iter()
                .any(|v| v["a"]["part"] == "text" || v["b"]["part"] == "text"),
            "include_text=false must suppress text checks: {out}"
        );

        // The file was not modified — read-only oracle.
        let content = std::fs::read_to_string(&path).unwrap();
        assert!(content.contains("(uuid \"w1\")"));
    }
}
