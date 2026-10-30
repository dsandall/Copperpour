//! Schematic routing and net tools built on placed pins.
//!
//! `route_connection` draws a wire only where it joins nothing else. A route
//! that would touch a foreign pin or another net's wire is refused with the
//! list of blockers, so placement gets fixed rather than a short drawn.

use crate::mcp::protocol::CallToolResult;
use crate::tool;
use crate::tools::{
    find_instance_index_unique, get_path, opt_str, require_str, ToolContext, ToolDef,
};
use copperpour_schematic_editor as cse;
use copperpour_sexp::{
    bbox::{symbol_bbox_for_instance, Rect},
    geometry::{point_on_segment, points_coincident, snap_to_grid, transform_pin},
    parser::SexpNode,
    schematic::{
        extract_junctions, extract_labels, extract_lib_pins_resolved, extract_symbol_instances,
        extract_wires, find_t_junctions_excluding, pin_endpoint, read_schematic, Label, LabelKind,
        LibPin, SymbolInstance, Wire,
    },
};
use serde_json::json;

use super::sch_connectivity::ConnectivityIndex;
use super::sch_wiring::cse_wires_to_sexp;
use std::collections::{HashMap, HashSet};

pub fn tools() -> Vec<ToolDef> {
    vec![
        tool!(
            "cleanup_junctions",
            "Remove duplicate junction dots (several at the same coordinate — one is kept) \
             and orphan junctions that don't sit on any wire. Use dry_run to preview \
             removals without writing.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string" },
                    "dry_run": { "type": "boolean", "description": "Report removals without applying them", "default": false }
                },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_cleanup_junctions(args, ctx).await }
        ),
        tool!(
            "route_connection",
            "Engine-computed wiring between two pins (or a pin and a net). Never asks for \
             wire coordinates: endpoints are resolved from ref+pin, the path is chosen as \
             straight if collinear, else an L, else a Z with the jog placed per jog_bias. \
             This is NOT an obstacle-avoiding router — when every candidate crosses a \
             symbol body or overlaps a different net's wire, the best candidate is still \
             placed and the crossings are returned as `interference` so you can FIX THE \
             PLACEMENT (bad routes signal bad placement; see check_placement/place_row in \
             the sch_layout toolset). Exception: a candidate that would pass through a \
             foreign pin, or touch/overlap a wire of another net (decided by real \
             connectivity, not just net names), is a hard short and is never selected — \
             if EVERY candidate is hard-blocked, an error is returned and nothing is \
             placed. When no candidate is fully clean, the Z jog lane is slid outward \
             in 2.54mm steps to find a clean lane automatically (the chosen lane is \
             returned as jog_at). Junction dots are added automatically; created wire \
             and junction uuids are returned as wire_uuids/junction_uuids.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "from": {
                        "type": "object",
                        "properties": {
                            "ref": { "type": "string", "description": "Reference designator" },
                            "pin": { "type": "string", "description": "Pin number" },
                            "uuid": { "type": "string", "description": "Symbol UUID, only needed for ambiguous refs" }
                        },
                        "required": ["ref", "pin"]
                    },
                    "to": {
                        "type": "object",
                        "description": "Either {ref, pin[, uuid]} for a pin, or {net} to route to the nearest point already on that net.",
                        "properties": {
                            "ref": { "type": "string" },
                            "pin": { "type": "string" },
                            "uuid": { "type": "string" },
                            "net": { "type": "string", "description": "Net name — routes to the nearest wire endpoint/pin/label on this net" }
                        }
                    },
                    "jog_bias": {
                        "description": "Where the Z-path's jog leg goes: 'early' (2.54mm after start), 'late' (2.54mm before end), or a number = mm after the start along the first axis. Default: midpoint, grid-snapped. This is only the search START — if that lane is blocked, adjacent lanes are tried automatically.",
                        "anyOf": [
                            { "type": "string", "enum": ["early", "late"] },
                            { "type": "number" }
                        ]
                    }
                },
                "required": ["schematic", "from", "to"]
            }),
            |args, ctx| async move { handle_route_connection(args, ctx).await }
        ),
        tool!(
            "apply_net",
            "Declarative net materialization: state which pins belong to a net and let the \
             engine decide how to realize it. style 'wire' routes pin-to-pin in a \
             nearest-neighbor chain (via the route_connection engine); 'label' adds a \
             2.54mm stub pointing away from each symbol body plus an oriented net label; \
             'auto' (default) picks wire for 2 pins within 50mm whose route is \
             interference-free, wire-trunk for 3+ collinear pins, labels otherwise. \
             Returns what was chosen and why.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "net": { "type": "string", "description": "Net name" },
                    "pins": {
                        "type": "array",
                        "minItems": 1,
                        "items": {
                            "type": "object",
                            "properties": {
                                "ref": { "type": "string" },
                                "pin": { "type": "string" },
                                "uuid": { "type": "string" }
                            },
                            "required": ["ref", "pin"]
                        }
                    },
                    "style": { "type": "string", "enum": ["wire", "label", "auto"], "default": "auto" },
                    "label_type": {
                        "type": "string",
                        "enum": ["net_label", "global_label", "hierarchical_label"],
                        "default": "net_label"
                    },
                    "shape": {
                        "type": "string",
                        "description": "Shape for global/hierarchical labels (input/output/bidirectional/tri_state/passive)",
                        "default": "input"
                    }
                },
                "required": ["schematic", "net", "pins"]
            }),
            |args, ctx| async move { handle_apply_net(args, ctx).await }
        ),
        tool!(
            "mark_unused_pins",
            "Add a no-connect flag to every genuinely unconnected pin. A pin counts as \
             connected when it coincides with a wire endpoint, lies anywhere ON a wire \
             segment (mid-wire tap), touches another symbol's pin directly, or has a label \
             at it. Pins that already carry a no-connect are skipped, never duplicated.",
            json!({
                "type": "object",
                "properties": {
                    "schematic": { "type": "string", "description": "Path to .kicad_sch file" },
                    "references": {
                        "type": "array",
                        "items": { "type": "string" },
                        "description": "Limit to these reference designators (default: all components; #PWR-style power symbols are always skipped)"
                    }
                },
                "required": ["schematic"]
            }),
            |args, ctx| async move { handle_mark_unused_pins(args, ctx).await }
        ),
    ]
}

/// Positions of the junctions already in the document — every T-detection
/// pass must exclude these, or each wire operation re-inserts dots the file
/// already has.
fn cse_junction_positions(sch: &cse::Schematic) -> Vec<(f64, f64)> {
    sch.junctions.iter().map(|j| j.position()).collect()
}

async fn handle_cleanup_junctions(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let dry_run = args["dry_run"].as_bool().unwrap_or(false);
    let tol = 0.01;

    let mut sch = cse::Schematic::load(&sch_path)?;
    let wires = cse_wires_to_sexp(&sch);

    let mut kept: Vec<cse::Junction> = Vec::new();
    let mut removed_duplicates: Vec<serde_json::Value> = Vec::new();
    let mut removed_orphans: Vec<serde_json::Value> = Vec::new();

    for j in &sch.junctions {
        let (x, y) = j.position();
        let entry = json!({ "x": x, "y": y, "uuid": j.uuid });
        let on_wire = wires.iter().any(|w| {
            copperpour_sexp::geometry::point_on_segment(x, y, w.x1, w.y1, w.x2, w.y2, tol)
        });
        if !on_wire {
            removed_orphans.push(entry);
        } else if kept
            .iter()
            .any(|k| copperpour_sexp::geometry::points_coincident(x, y, k.x, k.y, tol))
        {
            removed_duplicates.push(entry);
        } else {
            kept.push(j.clone());
        }
    }

    let removed = removed_duplicates.len() + removed_orphans.len();
    if !dry_run && removed > 0 {
        sch.junctions = kept;
        sch.overwrite()?;
    }

    Ok(CallToolResult::json(&json!({
        "removed": removed,
        "applied": !dry_run && removed > 0,
        "dry_run": dry_run,
        "duplicates_removed": removed_duplicates,
        "orphans_removed": removed_orphans
    })))
}

const GRID: f64 = 1.27;

const TOL: f64 = 0.01;

/// How close to a pin a wire may run inside that pin's own symbol bbox — the
/// pin stub allowance.
const PIN_STUB: f64 = GRID;

/// Resolve a `{ref, pin[, uuid]}` object to (instance index, pin, endpoint).
fn resolve_pin_spec(
    spec: &serde_json::Value,
    instances: &[SymbolInstance],
    lib_syms: &[&SexpNode],
) -> Result<(usize, LibPin, (f64, f64)), String> {
    let reference = spec["ref"]
        .as_str()
        .ok_or_else(|| "pin spec missing 'ref'".to_string())?;
    let pin_number = spec["pin"]
        .as_str()
        .ok_or_else(|| format!("pin spec for '{reference}' missing 'pin'"))?;
    let idx = find_instance_index_unique(instances, reference, spec["uuid"].as_str())?;
    let inst = &instances[idx];
    let sym = lib_syms
        .iter()
        .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(&inst.lib_id))
        .ok_or_else(|| {
            format!(
                "Library symbol '{}' (for '{}') is not embedded in this schematic",
                inst.lib_id, reference
            )
        })?;
    let pin = extract_lib_pins_resolved(lib_syms, sym)
        .into_iter()
        .find(|p| p.number == pin_number)
        .ok_or_else(|| format!("Pin '{pin_number}' not found on '{reference}'"))?;
    let ep = pin_endpoint(&pin, inst.pin_transform());
    Ok((idx, pin, ep))
}

/// Clip an axis-aligned segment against the strict interior of a rect.
/// Returns the clipped sub-segment when the segment genuinely penetrates —
/// running exactly along an edge or touching a corner does not count.
fn clip_segment_to_rect(
    a: (f64, f64),
    b: (f64, f64),
    r: &Rect,
) -> Option<((f64, f64), (f64, f64))> {
    const EPS: f64 = 1e-6;
    let shrunk = r.expand(-EPS);
    if shrunk.max_x <= shrunk.min_x || shrunk.max_y <= shrunk.min_y {
        return None;
    }
    if (a.1 - b.1).abs() < EPS {
        // Horizontal.
        let y = a.1;
        if y < shrunk.min_y || y > shrunk.max_y {
            return None;
        }
        let (lo, hi) = (a.0.min(b.0), a.0.max(b.0));
        let (cl, ch) = (lo.max(shrunk.min_x), hi.min(shrunk.max_x));
        (ch - cl > EPS).then_some(((cl, y), (ch, y)))
    } else if (a.0 - b.0).abs() < EPS {
        // Vertical.
        let x = a.0;
        if x < shrunk.min_x || x > shrunk.max_x {
            return None;
        }
        let (lo, hi) = (a.1.min(b.1), a.1.max(b.1));
        let (cl, ch) = (lo.max(shrunk.min_y), hi.min(shrunk.max_y));
        (ch - cl > EPS).then_some(((x, cl), (x, ch)))
    } else {
        None // only axis-aligned candidates are generated
    }
}

/// Overlap of two collinear axis-aligned segments (more than a shared point).
/// Returns the overlap midpoint.
fn collinear_overlap(a: (f64, f64), b: (f64, f64), w: &Wire) -> Option<(f64, f64)> {
    let horiz_seg = (a.1 - b.1).abs() < TOL;
    let horiz_wire = (w.y1 - w.y2).abs() < TOL;
    if horiz_seg && horiz_wire && (a.1 - w.y1).abs() < TOL {
        let (lo, hi) = (a.0.min(b.0), a.0.max(b.0));
        let (wl, wh) = (w.x1.min(w.x2), w.x1.max(w.x2));
        let (cl, ch) = (lo.max(wl), hi.min(wh));
        return (ch - cl > TOL).then_some(((cl + ch) / 2.0, a.1));
    }
    let vert_seg = (a.0 - b.0).abs() < TOL;
    let vert_wire = (w.x1 - w.x2).abs() < TOL;
    if vert_seg && vert_wire && (a.0 - w.x1).abs() < TOL {
        let (lo, hi) = (a.1.min(b.1), a.1.max(b.1));
        let (wl, wh) = (w.y1.min(w.y2), w.y1.max(w.y2));
        let (cl, ch) = (lo.max(wl), hi.min(wh));
        return (ch - cl > TOL).then_some((a.0, (cl + ch) / 2.0));
    }
    None
}

#[derive(Clone)]
struct RouteCandidate {
    kind: &'static str, // "straight" | "L" | "Z"
    points: Vec<(f64, f64)>,
    /// For Z paths: the jog lane — the x column (x-jog) or y row (y-jog) the
    /// middle segment runs on. `None` for straight/L.
    jog_at: Option<f64>,
}

/// Jog coordinate for the Z path between `from` and `to` along one axis.
fn jog_coord(from: f64, to: f64, bias: &serde_json::Value) -> f64 {
    let sign = if to >= from { 1.0 } else { -1.0 };
    let raw = match bias {
        v if v.is_number() => from + sign * v.as_f64().unwrap_or(0.0),
        v if v.as_str() == Some("early") => from + sign * 2.54,
        v if v.as_str() == Some("late") => to - sign * 2.54,
        _ => (from + to) / 2.0,
    };
    let snapped = snap_to_grid(raw, GRID);
    // Keep the jog strictly between the endpoints; degenerate bias falls back
    // to the midpoint.
    let (lo, hi) = (from.min(to), from.max(to));
    if snapped <= lo + TOL || snapped >= hi - TOL {
        snap_to_grid((from + to) / 2.0, GRID)
    } else {
        snapped
    }
}

/// Candidate paths in preference order: straight, the two L orientations, the
/// two Z orientations. No other shapes — obstacle avoidance is placement's job.
fn build_candidates(
    p1: (f64, f64),
    p2: (f64, f64),
    jog_bias: &serde_json::Value,
) -> Vec<RouteCandidate> {
    if points_coincident(p1.0, p1.1, p2.0, p2.1, TOL) {
        return vec![];
    }
    if (p1.0 - p2.0).abs() < TOL || (p1.1 - p2.1).abs() < TOL {
        return vec![RouteCandidate {
            kind: "straight",
            points: vec![p1, p2],
            jog_at: None,
        }];
    }
    let mx = jog_coord(p1.0, p2.0, jog_bias);
    let my = jog_coord(p1.1, p2.1, jog_bias);
    vec![
        RouteCandidate {
            kind: "L",
            points: vec![p1, (p2.0, p1.1), p2],
            jog_at: None,
        },
        RouteCandidate {
            kind: "L",
            points: vec![p1, (p1.0, p2.1), p2],
            jog_at: None,
        },
        z_at_lane(p1, p2, mx, true),
        z_at_lane(p1, p2, my, false),
    ]
}

/// One Z candidate with its jog on `lane`: an x column when `x_jog`, else a
/// y row.
fn z_at_lane(p1: (f64, f64), p2: (f64, f64), lane: f64, x_jog: bool) -> RouteCandidate {
    let points = if x_jog {
        vec![p1, (lane, p1.1), (lane, p2.1), p2]
    } else {
        vec![p1, (p1.0, lane), (p2.0, lane), p2]
    };
    RouteCandidate {
        kind: "Z",
        points,
        jog_at: Some(lane),
    }
}

/// Alternate jog lanes for a Z whose own lane is dirty (jog lane search).
/// From the base lane — the caller's jog_bias is the search START, never a
/// constraint — step ±2.54mm alternating outward (toward `p2` first), up to
/// 12 lanes each direction, grid-snapped and strictly between the endpoints'
/// coordinates on the jog axis.
fn z_lane_alternatives(
    base: &RouteCandidate,
    p1: (f64, f64),
    p2: (f64, f64),
) -> Vec<RouteCandidate> {
    let Some(start) = base.jog_at else {
        return vec![];
    };
    // A vertical middle segment means the jog picks an x column.
    let x_jog = (base.points[1].0 - base.points[2].0).abs() < TOL;
    let (from, to) = if x_jog { (p1.0, p2.0) } else { (p1.1, p2.1) };
    let (lo, hi) = (from.min(to), from.max(to));
    let dir = if to >= from { 1.0 } else { -1.0 };
    let mut out = Vec::new();
    for k in 1..=12 {
        for sign in [dir, -dir] {
            let lane = snap_to_grid(start + sign * k as f64 * 2.54, GRID);
            if lane <= lo + TOL || lane >= hi - TOL || (lane - start).abs() < TOL {
                continue;
            }
            out.push(z_at_lane(p1, p2, lane, x_jog));
        }
    }
    out
}

/// Synthetic net labels for power-symbol pins. A power symbol (#PWR
/// reference / `power:` lib) names its net via its Value ("+24V", "GND", …),
/// and its single pin anchors that net at the pin's endpoint — so a net that
/// exists only through power symbols is still resolvable and routable-to.
fn power_pin_labels(instances: &[SymbolInstance], lib_syms: &[&SexpNode]) -> Vec<Label> {
    let mut out = Vec::new();
    for inst in instances {
        let is_power = inst.reference.starts_with("#PWR") || inst.lib_id.starts_with("power:");
        if !is_power || inst.value.is_empty() {
            continue;
        }
        let Some(sym) = lib_syms
            .iter()
            .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(&inst.lib_id))
        else {
            continue;
        };
        let t = inst.pin_transform();
        for pin in extract_lib_pins_resolved(lib_syms, sym) {
            let (px, py) = pin_endpoint(&pin, t);
            out.push(Label {
                kind: LabelKind::PowerSymbol,
                net: inst.value.clone(),
                x: px,
                y: py,
                rotation: 0.0,
                uuid: None,
            });
        }
    }
    out
}

/// One pin endpoint the router must never run a wire through (unless it is
/// the route's own from/to point).
struct PinPt {
    x: f64,
    y: f64,
    inst_idx: usize,
    number: String,
}

/// Static route context built once per call.
struct RouteEnv {
    instances: Vec<SymbolInstance>,
    bodies: Vec<Option<Rect>>,
    wires: Vec<Wire>,
    /// Real labels plus synthetic power-pin labels — see [`power_pin_labels`].
    labels: Vec<Label>,
    /// Pre-resolved net of every wire (by its first endpoint).
    wire_nets: Vec<Option<String>>,
    /// Every pin endpoint on the sheet, for the foreign-pin hard-block check.
    pin_pts: Vec<PinPt>,
    /// Physical-connectivity union-find over wires, junctions, labels, and
    /// pins — see [`RouteEnv::build`]. RefCell because path compression in
    /// `find` mutates; `evaluate` only ever queries it.
    conn: std::cell::RefCell<NetGraph>,
}

impl RouteEnv {
    fn build(tree: &SexpNode) -> Self {
        let lib_syms = tree
            .find("lib_symbols")
            .map(|n| n.find_all("symbol"))
            .unwrap_or_default();
        let instances = extract_symbol_instances(tree);
        let bodies = instances
            .iter()
            .map(|i| symbol_bbox_for_instance(&lib_syms, i))
            .collect();
        let mut pin_pts = Vec::new();
        for (i, inst) in instances.iter().enumerate() {
            let Some(sym) = lib_syms
                .iter()
                .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(&inst.lib_id))
            else {
                continue;
            };
            let t = inst.pin_transform();
            for pin in extract_lib_pins_resolved(&lib_syms, sym) {
                let (px, py) = pin_endpoint(&pin, t);
                pin_pts.push(PinPt {
                    x: px,
                    y: py,
                    inst_idx: i,
                    number: pin.number.clone(),
                });
            }
        }
        let wires = extract_wires(tree);
        let mut labels = extract_labels(tree);
        labels.extend(power_pin_labels(&instances, &lib_syms));
        let mut g = build_net_graph(&wires, &labels);
        let wire_nets = wires.iter().map(|w| g.net_at(w.x1, w.y1)).collect();

        // Physical-connectivity union-find (touch classification): wires
        // union at shared endpoints, and every point of interest — wire
        // endpoint, junction dot, label anchor, pin tip — unions with every
        // wire segment it lies ON (KiCad connects an endpoint touching
        // mid-segment via auto-junction, and a junction dot connects all
        // wires through its point). Same-named labels union too: they are
        // one net even with no wire between them.
        let junctions = extract_junctions(tree);
        let mut conn = build_net_graph(&wires, &labels);
        let touch_pts: Vec<(f64, f64)> = wires
            .iter()
            .flat_map(|w| [(w.x1, w.y1), (w.x2, w.y2)])
            .chain(junctions.iter().copied())
            .chain(labels.iter().map(|l| (l.x, l.y)))
            .chain(pin_pts.iter().map(|p| (p.x, p.y)))
            .collect();
        for (x, y) in touch_pts {
            let k = pt_key(x, y);
            conn.ensure(k);
            for w in &wires {
                if point_on_segment(x, y, w.x1, w.y1, w.x2, w.y2, TOL) {
                    conn.union(k, pt_key(w.x1, w.y1));
                }
            }
        }
        let mut first_label_of: std::collections::HashMap<&str, (i64, i64)> =
            std::collections::HashMap::new();
        for l in &labels {
            let k = pt_key(l.x, l.y);
            match first_label_of.get(l.net.as_str()) {
                Some(&k0) => conn.union(k0, k),
                None => {
                    first_label_of.insert(&l.net, k);
                }
            }
        }

        RouteEnv {
            instances,
            bodies,
            wires,
            labels,
            wire_nets,
            pin_pts,
            conn: std::cell::RefCell::new(conn),
        }
    }

    fn net_at(&self, x: f64, y: f64) -> Option<String> {
        let mut g = build_net_graph(&self.wires, &self.labels);
        if let Some(net) = g.net_at(x, y) {
            return Some(net);
        }
        // A point mid-wire is not a graph node — inherit that wire's net.
        for (w, net) in self.wires.iter().zip(&self.wire_nets) {
            if net.is_some() && point_on_segment(x, y, w.x1, w.y1, w.x2, w.y2, TOL) {
                return net.clone();
            }
        }
        None
    }

    /// Evaluate one candidate path. Returns `(hard_blocks, interference)`.
    ///
    /// Hard blocks would place a silent short and DISQUALIFY the candidate:
    ///   - a segment passing through (or ending on) any pin endpoint other
    ///     than the route's own two endpoints ("pin"), and
    ///   - any touch of a FOREIGN wire ("wire"): a path point (start, corner,
    ///     or end — each becomes a wire endpoint and thus a junction site)
    ///     landing on it, one of its endpoints landing on a path segment, or
    ///     a collinear overlap with it. All of these merge nets, via
    ///     auto-junction or fused segments.
    ///
    /// Foreign is decided by CONNECTIVITY, not name: a wire union-find-
    /// connected (through wires, junctions, labels, pins) to either route
    /// endpoint is the route's own net, and touching or riding it is a legal
    /// bus-in/tap that scores nothing. Equal net NAMES short-circuit to
    /// "same net" as well (routing to a named net may ride any island of
    /// it), but a name mismatch never overrides union-find truth. Plain X
    /// crossings — an intersection with no endpoint of either party at it —
    /// create no junction and stay legal and un-scored.
    ///
    /// Ordinary interference is still placeable best-effort: symbol bodies
    /// the path penetrates (except the endpoint symbols' own bodies within
    /// PIN_STUB of their pin).
    fn evaluate(
        &self,
        cand: &RouteCandidate,
        from_idx: Option<usize>,
        from_pt: (f64, f64),
        to_idx: Option<usize>,
        to_pt: (f64, f64),
        route_net: &Option<String>,
    ) -> (Vec<serde_json::Value>, Vec<serde_json::Value>) {
        let near = |p: (f64, f64), q: (f64, f64)| {
            (p.0 - q.0).abs() <= PIN_STUB + TOL && (p.1 - q.1).abs() <= PIN_STUB + TOL
        };
        let own_pt = |x: f64, y: f64| {
            points_coincident(x, y, from_pt.0, from_pt.1, TOL)
                || points_coincident(x, y, to_pt.0, to_pt.1, TOL)
        };

        let mut hard: Vec<serde_json::Value> = Vec::new();
        let mut soft: Vec<serde_json::Value> = Vec::new();
        let mut seen_pins: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut seen_symbols: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut hard_wires: std::collections::HashSet<usize> = std::collections::HashSet::new();

        // Per-wire touch classification (see the method docs): same-net by
        // union-find connectivity to either route endpoint, or by an equal
        // resolved net name.
        let same_net: Vec<bool> = {
            let mut conn = self.conn.borrow_mut();
            let from_root = conn.find(pt_key(from_pt.0, from_pt.1));
            let to_root = conn.find(pt_key(to_pt.0, to_pt.1));
            (0..self.wires.len())
                .map(|wi| {
                    if route_net.is_some() && self.wire_nets[wi] == *route_net {
                        return true;
                    }
                    let w = &self.wires[wi];
                    let r = conn.find(pt_key(w.x1, w.y1));
                    r == from_root || r == to_root
                })
                .collect()
        };

        // Foreign pins on any segment — hard.
        for seg in cand.points.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            if points_coincident(a.0, a.1, b.0, b.1, TOL) {
                continue;
            }
            for (pi, p) in self.pin_pts.iter().enumerate() {
                if own_pt(p.x, p.y) {
                    continue;
                }
                if point_on_segment(p.x, p.y, a.0, a.1, b.0, b.1, TOL) && seen_pins.insert(pi) {
                    hard.push(json!({
                        "kind": "pin",
                        "ref": self.instances[p.inst_idx].reference,
                        "pin": p.number,
                        "at": { "x": p.x, "y": p.y }
                    }));
                }
            }
        }

        // Path points landing ON existing wires. Every path point becomes a
        // wire endpoint, so a landing T-taps that wire: legal on the route's
        // own net, a hard short on any foreign wire.
        for q in &cand.points {
            for (wi, w) in self.wires.iter().enumerate() {
                if same_net[wi] {
                    continue;
                }
                if point_on_segment(q.0, q.1, w.x1, w.y1, w.x2, w.y2, TOL) && hard_wires.insert(wi)
                {
                    hard.push(json!({
                        "kind": "wire",
                        "uuid": w.uuid,
                        "net": self.wire_nets[wi],
                        "at": { "x": q.0, "y": q.1 }
                    }));
                }
            }
        }

        for seg in cand.points.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            if points_coincident(a.0, a.1, b.0, b.1, TOL) {
                continue;
            }
            for (i, body) in self.bodies.iter().enumerate() {
                let Some(body) = body else { continue };
                let Some((ca, cb)) = clip_segment_to_rect(a, b, body) else {
                    continue;
                };
                // Endpoint symbols get a pin-stub allowance.
                if Some(i) == from_idx && near(ca, from_pt) && near(cb, from_pt) {
                    continue;
                }
                if Some(i) == to_idx && near(ca, to_pt) && near(cb, to_pt) {
                    continue;
                }
                if seen_symbols.insert(i) {
                    soft.push(json!({
                        "kind": "symbol",
                        "ref": self.instances[i].reference,
                        "uuid": self.instances[i].uuid,
                        "at": { "x": (ca.0 + cb.0) / 2.0, "y": (ca.1 + cb.1) / 2.0 }
                    }));
                }
            }
            for (wi, w) in self.wires.iter().enumerate() {
                if same_net[wi] {
                    // Same net — riding or touching the wire is a legal bus-in.
                    continue;
                }
                // A foreign wire's endpoint on this segment T-taps us the
                // moment auto-junctioning runs — as hard as landing on it.
                let endpoint_touch = [(w.x1, w.y1), (w.x2, w.y2)]
                    .into_iter()
                    .find(|(ex, ey)| point_on_segment(*ex, *ey, a.0, a.1, b.0, b.1, TOL));
                let Some(at) = collinear_overlap(a, b, w).or(endpoint_touch) else {
                    continue;
                };
                if hard_wires.insert(wi) {
                    hard.push(json!({
                        "kind": "wire",
                        "uuid": w.uuid,
                        "net": self.wire_nets[wi],
                        "at": { "x": at.0, "y": at.1 }
                    }));
                }
            }
        }
        (hard, soft)
    }
}

/// Outcome of candidate selection.
enum RouteChoice {
    /// A hard-block-free candidate exists; `interference` is its (possibly
    /// empty) ordinary interference.
    Chosen {
        cand: RouteCandidate,
        interference: Vec<serde_json::Value>,
    },
    /// EVERY candidate would short through a foreign pin or a differing net.
    /// Placing any of them is worse than placing nothing — the caller must
    /// error out without writing wires. `hard` is the least-bad candidate's
    /// hard-block list, for the error message.
    Blocked {
        hard: Vec<serde_json::Value>,
    },
    NoCandidates,
}

/// Evaluate ALL candidates and pick the first (in preference order:
/// straight → L → Z) with zero hard blocks and zero interference; else the
/// hard-block-free one with the fewest interferences; `Blocked` only when
/// every candidate is hard-blocked.
///
/// Jog lane search: when NO candidate is fully clean, each dirty Z gets its
/// jog lane slid outward (see [`z_lane_alternatives`]) and is replaced by
/// the first fully-clean lane found — so consecutive same-span fan-outs land
/// in adjacent clean lanes without caller-supplied biases.
fn choose_candidate(
    cands: Vec<RouteCandidate>,
    env: &RouteEnv,
    from_idx: Option<usize>,
    from_pt: (f64, f64),
    to_idx: Option<usize>,
    to_pt: (f64, f64),
    route_net: &Option<String>,
) -> RouteChoice {
    type Eval = (
        RouteCandidate,
        Vec<serde_json::Value>,
        Vec<serde_json::Value>,
    );
    let mut evals: Vec<Eval> = cands
        .into_iter()
        .map(|c| {
            let (hard, soft) = env.evaluate(&c, from_idx, from_pt, to_idx, to_pt, route_net);
            (c, hard, soft)
        })
        .collect();
    if evals.is_empty() {
        return RouteChoice::NoCandidates;
    }
    // Jog lane search (both Z orientations).
    if !evals.iter().any(|(_, h, s)| h.is_empty() && s.is_empty()) {
        for eval in &mut evals {
            if eval.0.kind != "Z" {
                continue;
            }
            let base = eval.0.clone();
            for alt in z_lane_alternatives(&base, from_pt, to_pt) {
                let (h, s) = env.evaluate(&alt, from_idx, from_pt, to_idx, to_pt, route_net);
                if h.is_empty() && s.is_empty() {
                    *eval = (alt, h, s);
                    break;
                }
            }
        }
    }
    let best_clean = evals
        .iter()
        .enumerate()
        .filter(|(_, (_, hard, _))| hard.is_empty())
        .min_by_key(|(i, (_, _, soft))| (soft.len(), *i))
        .map(|(i, _)| i);
    match best_clean {
        Some(i) => {
            let (cand, _, soft) = evals.swap_remove(i);
            RouteChoice::Chosen {
                cand,
                interference: soft,
            }
        }
        None => {
            let least_bad = evals
                .iter()
                .enumerate()
                .min_by_key(|(i, (_, hard, _))| (hard.len(), *i))
                .map(|(i, _)| i)
                .unwrap();
            let (_, hard, _) = evals.swap_remove(least_bad);
            RouteChoice::Blocked { hard }
        }
    }
}

/// Write a chosen path into the schematic: wires plus auto-detected junction
/// dots, dedupe-safe. Returns the uuids of the wires and junctions it
/// created (counts fall out as the lengths).
fn apply_path(sch: &mut cse::Schematic, points: &[(f64, f64)]) -> (Vec<String>, Vec<String>) {
    let mut wire_uuids: Vec<String> = Vec::new();
    let junctions_before = sch.junctions.len();
    for seg in points.windows(2) {
        let (a, b) = (seg[0], seg[1]);
        if points_coincident(a.0, a.1, b.0, b.1, TOL) {
            continue;
        }
        let mut existing = cse_wires_to_sexp(sch);
        existing.push(Wire {
            x1: a.0,
            y1: a.1,
            x2: b.0,
            y2: b.1,
            uuid: None,
        });
        let junctions = find_t_junctions_excluding(&existing, TOL, &cse_junction_positions(sch));
        wire_uuids.push(sch.add_wire(a.0, a.1, b.0, b.1).uuid.clone());
        for (jx, jy) in &junctions {
            sch.add_junction(*jx, *jy);
        }
    }
    let junction_uuids = sch.junctions[junctions_before..]
        .iter()
        .map(|j| j.uuid.clone())
        .collect();
    (wire_uuids, junction_uuids)
}

async fn handle_route_connection(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let jog_bias = args["jog_bias"].clone();

    let (_, tree) = read_schematic(&sch_path)?;
    let lib_syms = tree
        .find("lib_symbols")
        .map(|n| n.find_all("symbol"))
        .unwrap_or_default();
    let env = RouteEnv::build(&tree);

    let (from_idx, _, from_pt) = match resolve_pin_spec(&args["from"], &env.instances, &lib_syms) {
        Ok(v) => v,
        Err(e) => return Ok(CallToolResult::error(e)),
    };

    // Resolve the destination: a pin, or the nearest point already on a net.
    let (to_idx, to_pt, to_net): (Option<usize>, (f64, f64), Option<String>) =
        if let Some(net) = args["to"]["net"].as_str() {
            // env.labels includes synthetic power-pin labels, so a net that
            // exists only via power symbols (GND, +24V, …) is targetable.
            let mut g = build_net_graph(&env.wires, &env.labels);
            let pts = g.points_on_net(net);
            let target = pts
                .iter()
                .map(|k| (k.0 as f64 / 1000.0, k.1 as f64 / 1000.0))
                .filter(|p| !points_coincident(p.0, p.1, from_pt.0, from_pt.1, TOL))
                .min_by(|a, b| {
                    let da = (a.0 - from_pt.0).abs() + (a.1 - from_pt.1).abs();
                    let db = (b.0 - from_pt.0).abs() + (b.1 - from_pt.1).abs();
                    da.partial_cmp(&db).unwrap()
                });
            let Some(target) = target else {
                return Ok(CallToolResult::error(format!(
                    "Net '{net}' has no existing point (wire endpoint, pin, or label) to \
                     route to"
                )));
            };
            // If the nearest net point is some symbol's pin, that symbol gets
            // the endpoint pin-stub allowance too.
            let owner = env.instances.iter().enumerate().find_map(|(i, inst)| {
                let sym = lib_syms
                    .iter()
                    .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(&inst.lib_id))?;
                let t = inst.pin_transform();
                extract_lib_pins_resolved(&lib_syms, sym)
                    .iter()
                    .any(|p| {
                        let (px, py) = pin_endpoint(p, t);
                        points_coincident(px, py, target.0, target.1, TOL)
                    })
                    .then_some(i)
            });
            (owner, target, Some(net.to_string()))
        } else {
            match resolve_pin_spec(&args["to"], &env.instances, &lib_syms) {
                Ok((idx, _, ep)) => (Some(idx), ep, None),
                Err(e) => return Ok(CallToolResult::error(e)),
            }
        };

    if points_coincident(from_pt.0, from_pt.1, to_pt.0, to_pt.1, TOL) {
        return Ok(CallToolResult::json(&json!({
            "route": "none",
            "reason": "endpoints coincide — already connected",
            "path": [{ "x": from_pt.0, "y": from_pt.1 }],
            "wires_added": 0,
            "junctions_added": 0,
            "wire_uuids": [],
            "junction_uuids": [],
            "interference": []
        })));
    }

    let route_net = to_net
        .clone()
        .or_else(|| env.net_at(from_pt.0, from_pt.1))
        .or_else(|| env.net_at(to_pt.0, to_pt.1));

    let cands = build_candidates(from_pt, to_pt, &jog_bias);
    let (chosen, interference) = match choose_candidate(
        cands,
        &env,
        Some(from_idx),
        from_pt,
        to_idx,
        to_pt,
        &route_net,
    ) {
        RouteChoice::Chosen { cand, interference } => (cand, interference),
        RouteChoice::Blocked { hard } => {
            // Structured refusal: MCP clients (Codex among them) mangle long plain-text
            // error bodies, and the caller needs the blocker list machine-readable.
            return Ok(CallToolResult::json(&json!({
                "refused": true,
                "reason": "every route candidate would create a short (foreign pin touch or foreign-net wire touch/overlap)",
                "hard_blocks": hard,
                "wires_added": 0,
                "note": "No wires were placed — a placed short is worse than no route. Fix the placement with the sch_layout tools so a clean corridor exists, then re-route."
            })));
        }
        RouteChoice::NoCandidates => {
            return Ok(CallToolResult::error("No route candidate could be built"));
        }
    };

    let mut sch = cse::Schematic::load(&sch_path)?;
    let (wire_uuids, junction_uuids) = apply_path(&mut sch, &chosen.points);
    sch.overwrite()?;

    Ok(CallToolResult::json(&json!({
        "route": chosen.kind,
        "jog_at": chosen.jog_at,
        "path": chosen.points.iter().map(|p| json!({ "x": p.0, "y": p.1 })).collect::<Vec<_>>(),
        "wires_added": wire_uuids.len(),
        "junctions_added": junction_uuids.len(),
        "wire_uuids": wire_uuids,
        "junction_uuids": junction_uuids,
        "interference": interference,
        "note": if interference.is_empty() {
            serde_json::Value::Null
        } else {
            json!("route placed WITH interference — this signals bad placement; adjust it \
                   with sch_layout tools and re-route")
        }
    })))
}

/// A resolved pin for apply_net: where it is and which way its stub points.
struct NetPin {
    inst_idx: usize,
    reference: String,
    number: String,
    pt: (f64, f64),
    /// Unit vector pointing AWAY from the symbol body in schematic space.
    away: (f64, f64),
}

fn stub_direction(pin: &LibPin, inst: &SymbolInstance) -> (f64, f64) {
    // The pin angle points from the tip TOWARD the body; away is the opposite,
    // pushed through the instance rotation/mirror (direction-only transform).
    let rad = pin.rotation.to_radians();
    let t0 = copperpour_sexp::geometry::PinTransform {
        comp_x: 0.0,
        comp_y: 0.0,
        ..inst.pin_transform()
    };
    let (dx, dy) = transform_pin(-rad.cos(), -rad.sin(), t0);
    // Snap to the dominant axis.
    if dx.abs() >= dy.abs() {
        (dx.signum(), 0.0)
    } else {
        (0.0, dy.signum())
    }
}

async fn handle_apply_net(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let net = match require_str(args, "net") {
        Ok(v) => v.to_string(),
        Err(e) => return Ok(e),
    };
    let pin_specs = match args["pins"].as_array() {
        Some(a) if !a.is_empty() => a.clone(),
        _ => return Ok(CallToolResult::error("'pins' must be a non-empty array")),
    };
    let style = opt_str(args, "style").unwrap_or("auto");
    let label_type = opt_str(args, "label_type").unwrap_or("net_label");
    let shape = opt_str(args, "shape").unwrap_or("input");

    let (_, tree) = read_schematic(&sch_path)?;
    let lib_syms = tree
        .find("lib_symbols")
        .map(|n| n.find_all("symbol"))
        .unwrap_or_default();
    let env = RouteEnv::build(&tree);

    let mut pins: Vec<NetPin> = Vec::new();
    for spec in &pin_specs {
        match resolve_pin_spec(spec, &env.instances, &lib_syms) {
            Ok((idx, pin, ep)) => {
                let away = stub_direction(&pin, &env.instances[idx]);
                pins.push(NetPin {
                    inst_idx: idx,
                    reference: env.instances[idx].reference.clone(),
                    number: pin.number.clone(),
                    pt: ep,
                    away,
                });
            }
            Err(e) => return Ok(CallToolResult::error(e)),
        }
    }

    // Decide the realization.
    let route_net = Some(net.clone());
    let (chosen_style, reason) = match style {
        "wire" => ("wire", "explicitly requested".to_string()),
        "label" => ("label", "explicitly requested".to_string()),
        _ => {
            if pins.len() < 2 {
                (
                    "label",
                    "fewer than 2 pins — labels join by name".to_string(),
                )
            } else if pins.len() == 2 {
                let d = (pins[0].pt.0 - pins[1].pt.0).abs() + (pins[0].pt.1 - pins[1].pt.1).abs();
                if d > 50.0 {
                    (
                        "label",
                        format!("2 pins {d:.2}mm apart (>50mm) — too far for a direct wire"),
                    )
                } else {
                    // Dry candidate check — no writes. Hard-blocked candidates
                    // (foreign-pin / differing-net touches) never count as clean.
                    let cands = build_candidates(pins[0].pt, pins[1].pt, &serde_json::Value::Null);
                    let clean = matches!(
                        choose_candidate(
                            cands,
                            &env,
                            Some(pins[0].inst_idx),
                            pins[0].pt,
                            Some(pins[1].inst_idx),
                            pins[1].pt,
                            &route_net,
                        ),
                        RouteChoice::Chosen { ref interference, .. } if interference.is_empty()
                    );
                    if clean {
                        (
                            "wire",
                            format!("2 pins {d:.2}mm apart with an interference-free route"),
                        )
                    } else {
                        (
                            "label",
                            "direct route would cross a symbol or foreign wire — labels \
                             instead; fix placement to enable wiring"
                                .to_string(),
                        )
                    }
                }
            } else {
                let all_x = pins.iter().all(|p| (p.pt.0 - pins[0].pt.0).abs() < TOL);
                let all_y = pins.iter().all(|p| (p.pt.1 - pins[0].pt.1).abs() < TOL);
                if all_x || all_y {
                    (
                        "wire",
                        format!("{} pins collinear on one axis — wire trunk", pins.len()),
                    )
                } else {
                    (
                        "label",
                        format!("{} pins not collinear — labels join by name", pins.len()),
                    )
                }
            }
        }
    };

    let mut sch = cse::Schematic::load(&sch_path)?;
    let mut actions: Vec<serde_json::Value> = Vec::new();
    let mut all_interference: Vec<serde_json::Value> = Vec::new();
    let mut all_wire_uuids: Vec<String> = Vec::new();
    let mut all_junction_uuids: Vec<String> = Vec::new();

    if chosen_style == "wire" {
        // Nearest-neighbor chain order.
        let mut remaining: Vec<usize> = (1..pins.len()).collect();
        let mut order: Vec<usize> = vec![0];
        while !remaining.is_empty() {
            let cur = *order.last().unwrap();
            let (pos, _) = remaining
                .iter()
                .enumerate()
                .min_by(|(_, &a), (_, &b)| {
                    let da = (pins[a].pt.0 - pins[cur].pt.0).abs()
                        + (pins[a].pt.1 - pins[cur].pt.1).abs();
                    let db = (pins[b].pt.0 - pins[cur].pt.0).abs()
                        + (pins[b].pt.1 - pins[cur].pt.1).abs();
                    da.partial_cmp(&db).unwrap()
                })
                .unwrap();
            order.push(remaining.remove(pos));
        }
        for pair in order.windows(2) {
            let (a, b) = (&pins[pair[0]], &pins[pair[1]]);
            let cands = build_candidates(a.pt, b.pt, &serde_json::Value::Null);
            let (chosen, inter) = match choose_candidate(
                cands,
                &env,
                Some(a.inst_idx),
                a.pt,
                Some(b.inst_idx),
                b.pt,
                &route_net,
            ) {
                RouteChoice::Chosen { cand, interference } => (cand, interference),
                RouteChoice::Blocked { hard } => {
                    // Returning before sch.overwrite(): NOTHING is written —
                    // a placed pin-short is worse than no route.
                    return Ok(CallToolResult::error(format!(
                        "apply_net '{}': every route candidate for {}.{} → {}.{} would \
                         create a short (foreign pin / foreign wire): {}. No wires were \
                         written. Fix the placement or use style 'label'.",
                        net,
                        a.reference,
                        a.number,
                        b.reference,
                        b.number,
                        serde_json::to_string(&hard).unwrap_or_default()
                    )));
                }
                RouteChoice::NoCandidates => continue, // coincident pins — nothing to draw
            };
            let (wire_uuids, junction_uuids) = apply_path(&mut sch, &chosen.points);
            actions.push(json!({
                "from": { "ref": a.reference, "pin": a.number },
                "to": { "ref": b.reference, "pin": b.number },
                "route": chosen.kind,
                "jog_at": chosen.jog_at,
                "wires_added": wire_uuids.len(),
                "junctions_added": junction_uuids.len(),
                "wire_uuids": wire_uuids.clone(),
                "junction_uuids": junction_uuids.clone()
            }));
            all_wire_uuids.extend(wire_uuids);
            all_junction_uuids.extend(junction_uuids);
            all_interference.extend(inter);
        }
    } else {
        for p in &pins {
            let end = (p.pt.0 + p.away.0 * 2.54, p.pt.1 + p.away.1 * 2.54);
            let rotation = match (p.away.0 as i64, p.away.1 as i64) {
                (-1, _) => 180.0,
                (_, -1) => 90.0,
                (_, 1) => 270.0,
                _ => 0.0,
            };
            let stub_uuid = sch.add_wire(p.pt.0, p.pt.1, end.0, end.1).uuid.clone();
            all_wire_uuids.push(stub_uuid.clone());
            match label_type {
                "global_label" => {
                    sch.add_global_label(&net, shape, end.0, end.1);
                    let idx = sch.global_labels.len() - 1;
                    if let Some(gl) = sch.global_labels.get_mut(idx) {
                        gl.set_rotation(rotation);
                    }
                }
                "hierarchical_label" => {
                    sch.add_hierarchical_label(&net, shape, end.0, end.1);
                    let idx = sch.hierarchical_labels.len() - 1;
                    if let Some(hl) = sch.hierarchical_labels.get_mut(idx) {
                        hl.set_rotation(rotation);
                    }
                }
                _ => {
                    let label = sch.add_label(&net, end.0, end.1);
                    label.set_rotation(rotation);
                }
            }
            actions.push(json!({
                "ref": p.reference,
                "pin": p.number,
                "stub": { "from": { "x": p.pt.0, "y": p.pt.1 }, "to": { "x": end.0, "y": end.1 } },
                "wire_uuids": [stub_uuid],
                "label": { "x": end.0, "y": end.1, "rotation": rotation, "type": label_type }
            }));
        }
    }
    sch.overwrite()?;

    Ok(CallToolResult::json(&json!({
        "net": net,
        "style": chosen_style,
        "reason": reason,
        "actions": actions,
        "wire_uuids": all_wire_uuids,
        "junction_uuids": all_junction_uuids,
        "interference": all_interference
    })))
}

async fn handle_mark_unused_pins(
    args: &serde_json::Value,
    _ctx: &ToolContext,
) -> anyhow::Result<CallToolResult> {
    let sch_path = get_path(args, "schematic")?;
    let filter_refs: Vec<String> = args["references"]
        .as_array()
        .map(|a| {
            a.iter()
                .filter_map(|v| v.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();

    let (_, tree) = read_schematic(&sch_path)?;
    let lib_syms = tree
        .find("lib_symbols")
        .map(|n| n.find_all("symbol"))
        .unwrap_or_default();
    let instances = extract_symbol_instances(&tree);
    let wires = extract_wires(&tree);
    let buses = copperpour_sexp::schematic::extract_buses(&tree);
    let labels = copperpour_sexp::schematic::extract_all_net_labels(&tree);
    let index = ConnectivityIndex::build(
        &tree,
        &wires,
        &buses,
        &labels,
        crate::tools::sch_connectivity::COINCIDENT_TOLERANCE,
    );

    let mut to_add: Vec<(f64, f64)> = Vec::new();
    let mut per_ref: std::collections::BTreeMap<String, Vec<String>> =
        std::collections::BTreeMap::new();
    let mut skipped_existing = 0usize;

    for inst in &instances {
        if !filter_refs.is_empty() && !filter_refs.contains(&inst.reference) {
            continue;
        }
        // Power symbols connect by name; an X on a #PWR pin is never wanted.
        if inst.reference.starts_with('#') {
            continue;
        }
        let Some(sym) = lib_syms
            .iter()
            .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(&inst.lib_id))
        else {
            continue;
        };
        let t = inst.pin_transform();
        for pin in extract_lib_pins_resolved(&lib_syms, sym) {
            let (px, py) = pin_endpoint(&pin, t);
            if index.has_no_connect(px, py) {
                skipped_existing += 1;
                continue;
            }
            // A wire through the pin is a connection someone drew, even without
            // the dot KiCad needs: an X there would hide that, not mark it unused.
            if index.attaches_pin(px, py) || index.on_wire(px, py) {
                continue;
            }
            // A pin already queued (two unconnected pins can't share a point
            // unless they touch — and touching counts as connected).
            to_add.push((px, py));
            per_ref
                .entry(inst.reference.clone())
                .or_default()
                .push(pin.number.clone());
        }
    }

    if !to_add.is_empty() {
        let mut sch = cse::Schematic::load(&sch_path)?;
        for (x, y) in &to_add {
            sch.add_no_connect(*x, *y);
        }
        sch.overwrite()?;
    }

    Ok(CallToolResult::json(&json!({
        "added_count": to_add.len(),
        "skipped_existing": skipped_existing,
        "per_ref": per_ref.iter().map(|(r, pins)| json!({ "ref": r, "pins": pins })).collect::<Vec<_>>()
    })))
}

pub(crate) fn pt_key(x: f64, y: f64) -> (i64, i64) {
    ((x * 1000.0).round() as i64, (y * 1000.0).round() as i64)
}

pub(crate) struct NetGraph {
    pub(crate) point_nets: HashMap<(i64, i64), String>,
    pub(crate) parent: HashMap<(i64, i64), (i64, i64)>,
}

impl NetGraph {
    pub(crate) fn new() -> Self {
        NetGraph {
            point_nets: HashMap::new(),
            parent: HashMap::new(),
        }
    }

    pub(crate) fn ensure(&mut self, k: (i64, i64)) {
        self.parent.entry(k).or_insert(k);
    }

    pub(crate) fn find(&mut self, k: (i64, i64)) -> (i64, i64) {
        self.ensure(k);
        let p = self.parent[&k];
        if p == k {
            return k;
        }
        let root = self.find(p);
        self.parent.insert(k, root);
        root
    }

    pub(crate) fn union(&mut self, a: (i64, i64), b: (i64, i64)) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra != rb {
            self.parent.insert(rb, ra);
        }
    }

    pub(crate) fn add_wire(&mut self, w: &Wire) {
        let a = pt_key(w.x1, w.y1);
        let b = pt_key(w.x2, w.y2);
        self.ensure(a);
        self.ensure(b);
        self.union(a, b);
    }

    pub(crate) fn add_label(&mut self, x: f64, y: f64, net: &str) {
        let k = pt_key(x, y);
        self.ensure(k);
        self.point_nets.insert(k, net.to_string());
    }

    pub(crate) fn net_at(&mut self, x: f64, y: f64) -> Option<String> {
        let k = pt_key(x, y);
        self.ensure(k);
        let root = self.find(k);
        let labels: Vec<_> = self.point_nets.clone().into_iter().collect();
        for (lk, net) in labels {
            if self.find(lk) == root {
                return Some(net);
            }
        }
        None
    }

    pub(crate) fn points_on_net(&mut self, net: &str) -> Vec<(i64, i64)> {
        // Collect keys first to avoid simultaneous borrow of point_nets and self.find()
        let net_keys: Vec<(i64, i64)> = self
            .point_nets
            .iter()
            .filter(|(_, n)| n.as_str() == net)
            .map(|(k, _)| *k)
            .collect();
        let net_roots: HashSet<(i64, i64)> = net_keys.iter().map(|k| self.find(*k)).collect();
        let all_keys: Vec<(i64, i64)> = self.parent.keys().cloned().collect();
        all_keys
            .into_iter()
            .filter(|k| net_roots.contains(&self.find(*k)))
            .collect()
    }
}

pub(crate) fn build_net_graph(
    wires: &[Wire],
    labels: &[copperpour_sexp::schematic::Label],
) -> NetGraph {
    let mut g = NetGraph::new();
    for w in wires {
        g.add_wire(w);
    }
    for l in labels {
        g.add_label(l.x, l.y, &l.net);
    }
    g
}

#[cfg(test)]
mod routing_tests {
    use super::*;
    use crate::router::ToolRouter;
    use crate::tools::ServerConfig;
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

    /// Device:R-shaped part: body rectangle ±1.016 × ±2.54, pin tips at
    /// (0, ±3.81). At rotation 90 the pins land on the X axis (pin 1 left,
    /// pin 2 right).
    const LIB: &str = r##"  (lib_symbols
    (symbol "Dev:R"
      (property "Reference" "R" (at 2.032 0 90))
      (symbol "R_1_1"
        (rectangle (start -1.016 2.54) (end 1.016 -2.54))
        (pin passive line (at 0 3.81 270) (length 1.27) (name "~") (number "1"))
        (pin passive line (at 0 -3.81 90) (length 1.27) (name "~") (number "2"))
      )
    )
    (symbol "power:+24V"
      (power)
      (property "Reference" "#PWR" (at 0 0 0))
      (property "Value" "+24V" (at 0 0 0))
      (symbol "+24V_1_1"
        (pin power_in line (at 0 0 270) (length 0) (name "+24V") (number "1"))
      )
    )
  )"##;

    fn placed(reference: &str, x: f64, y: f64, rot: f64) -> String {
        format!(
            "  (symbol\n    (lib_id \"Dev:R\")\n    (at {x} {y} {rot})\n    (unit 1)\n    (uuid \"u-{reference}\")\n    (property \"Reference\" \"{reference}\"\n      (at {x} {y} 0)\n      (effects (font (size 1.27 1.27)) (hide yes))\n    )\n  )"
        )
    }

    /// A power:+24V symbol instance whose pin (length 0) sits exactly at (x, y).
    fn placed_power(reference: &str, x: f64, y: f64) -> String {
        format!(
            "  (symbol\n    (lib_id \"power:+24V\")\n    (at {x} {y} 0)\n    (unit 1)\n    (uuid \"u-{reference}\")\n    (property \"Reference\" \"{reference}\"\n      (at {x} {y} 0)\n      (effects (font (size 1.27 1.27)) (hide yes))\n    )\n    (property \"Value\" \"+24V\"\n      (at {x} {y} 0)\n    )\n  )"
        )
    }

    fn write_sch(body: &str) -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("routing.kicad_sch");
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

    async fn route(
        path: &std::path::Path,
        from: serde_json::Value,
        to: serde_json::Value,
    ) -> serde_json::Value {
        let result = handle_route_connection(
            &json!({ "schematic": path.display().to_string(), "from": from, "to": to }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        parse_result(&result)
    }

    // ─── route_connection ──────────────────────────────────────────────────

    #[tokio::test]
    async fn route_collinear_pins_with_one_straight_wire() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 101.6, 90.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        assert_eq!(out["route"], "straight", "{out}");
        assert_eq!(out["wires_added"], 1);
        assert!(out["interference"].as_array().unwrap().is_empty(), "{out}");

        let sch = cse::Schematic::load(&path).unwrap();
        assert_eq!(sch.wires.len(), 1);
        let w = sch.wires.iter().next().unwrap();
        let xs = [w.start.0, w.end.0];
        assert!(xs.contains(&105.41) && xs.contains(&118.11), "{xs:?}");
        assert_eq!(w.start.1, 101.6);
        assert_eq!(w.end.1, 101.6);
    }

    #[tokio::test]
    async fn route_offset_pins_with_an_l() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 116.84, 90.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        assert_eq!(out["route"], "L", "{out}");
        assert_eq!(out["wires_added"], 2);
        assert!(out["interference"].as_array().unwrap().is_empty(), "{out}");
        assert_eq!(out["path"].as_array().unwrap().len(), 3);
        // Endpoints are the resolved pin tips — the engine computed them.
        assert_eq!(out["path"][0]["x"], 105.41);
        assert_eq!(out["path"][2]["x"], 118.11);
        assert_eq!(out["path"][2]["y"], 116.84);
    }

    #[tokio::test]
    async fn route_falls_back_to_z_when_both_l_corners_are_blocked() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 116.84, 90.0),
            // Bodies sitting exactly on the two L corners.
            placed("R3", 118.11, 101.6, 0.0),
            placed("R4", 105.41, 116.84, 0.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        assert_eq!(out["route"], "Z", "{out}");
        assert_eq!(out["wires_added"], 3);
        assert!(
            out["interference"].as_array().unwrap().is_empty(),
            "the Z jog must clear both blockers: {out}"
        );
        // Default jog = grid-snapped midpoint of the X span.
        assert_eq!(out["path"][1]["x"], 111.76, "{out}");
    }

    #[tokio::test]
    async fn blocked_route_is_still_placed_and_reports_interference() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 101.6, 90.0),
            // Body squarely on the only (straight) path.
            placed("R5", 111.76, 101.6, 0.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        let inter = out["interference"].as_array().unwrap();
        assert_eq!(inter.len(), 1, "{out}");
        assert_eq!(inter[0]["kind"], "symbol");
        assert_eq!(inter[0]["ref"], "R5");
        assert!(inter[0]["at"]["x"].is_number());
        // The router does NOT avoid obstacles — the wire is still placed.
        assert_eq!(out["wires_added"], 1);
        assert!(
            out["note"].as_str().unwrap().contains("placement"),
            "the interference note must steer toward fixing placement: {out}"
        );
        let sch = cse::Schematic::load(&path).unwrap();
        assert_eq!(sch.wires.len(), 1);
    }

    #[tokio::test]
    async fn route_to_net_targets_the_nearest_point_on_that_net() {
        let (_d, path) = write_sch(&format!(
            "{}\n  (wire\n    (pts (xy 203.2 101.6) (xy 215.9 101.6))\n    (stroke (width 0) (type default))\n    (uuid \"w-vbus\")\n  )\n  (label \"VBUS\"\n    (at 215.9 101.6 0)\n    (effects (font (size 1.27 1.27)) (justify left bottom))\n    (uuid \"l-vbus\")\n  )",
            placed("R1", 121.92, 101.6, 90.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "net": "VBUS" }),
        )
        .await;

        assert_eq!(out["route"], "straight", "{out}");
        let last = out["path"].as_array().unwrap().last().unwrap().clone();
        assert_eq!(last["x"], 203.2, "must hit the NEAREST net point: {out}");
        assert_eq!(last["y"], 101.6);
    }

    #[tokio::test]
    async fn route_to_unknown_net_errors() {
        let (_d, path) = write_sch(&placed("R1", 101.6, 101.6, 90.0));
        let result = handle_route_connection(
            &json!({
                "schematic": path.display().to_string(),
                "from": { "ref": "R1", "pin": "2" },
                "to": { "net": "NOPE" }
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(result.is_error);
    }

    // ─── Fix A: foreign pins are hard blockers ─────────────────────────────

    #[tokio::test]
    async fn pin_touching_ls_are_rejected_for_a_clean_z() {
        // Both L candidates run straight through a third component's pins;
        // the x-jog Z is completely clean and MUST win despite L preference.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}",
            placed("R1", 101.6, 101.6, 90.0), // from: pin 2 at (105.41, 101.6)
            placed("R2", 121.92, 116.84, 90.0), // to:   pin 1 at (118.11, 116.84)
            // RA pins at (118.11, 101.6) and (118.11, 109.22) — on L1's corner/leg.
            placed("RA", 118.11, 105.41, 0.0),
            // RB pins at (105.41, 104.14) and (105.41, 111.76) — on L2's leg.
            placed("RB", 105.41, 107.95, 0.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        assert_eq!(
            out["route"], "Z",
            "pin-touching Ls must never be selected: {out}"
        );
        assert!(
            out["interference"].as_array().unwrap().is_empty(),
            "the clean Z has no interference: {out}"
        );
        // No placed wire may pass through a foreign pin endpoint.
        let sch = cse::Schematic::load(&path).unwrap();
        for w in sch.wires.iter() {
            for (px, py) in [
                (118.11, 101.6),
                (118.11, 109.22),
                (105.41, 104.14),
                (105.41, 111.76),
            ] {
                assert!(
                    !copperpour_sexp::geometry::point_on_segment(
                        px, py, w.start.0, w.start.1, w.end.0, w.end.1, 0.01
                    ),
                    "wire {:?}-{:?} passes through foreign pin ({px}, {py})",
                    w.start,
                    w.end
                );
            }
        }
    }

    #[tokio::test]
    async fn all_candidates_pin_blocked_errors_and_writes_nothing() {
        // Collinear pins → the straight wire is the ONLY candidate, and a
        // third component's pin sits exactly on it. Routing must refuse: a
        // placed pin-short is worse than no route.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 101.6, 90.0),
            // RX pin 1 at (111.76, 101.6) — squarely on the straight path.
            placed("RX", 111.76, 105.41, 0.0),
        ));
        let before = std::fs::read_to_string(&path).unwrap();

        let result = handle_route_connection(
            &json!({
                "schematic": path.display().to_string(),
                "from": { "ref": "R1", "pin": "2" },
                "to": { "ref": "R2", "pin": "1" }
            }),
            &test_ctx(),
        )
        .await
        .unwrap();

        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text content");
        };
        assert!(
            text.contains("\"refused\":true") || text.contains("\"refused\": true"),
            "an unavoidable pin-short must be refused: {text}"
        );
        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text content");
        };
        assert!(
            text.contains("RX") && text.contains("pin"),
            "error must name the blocking pin: {text}"
        );
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            before,
            "no wires may be written when every candidate is hard-blocked"
        );
    }

    // ─── Fix B: power symbols anchor route-to-net ──────────────────────────

    #[tokio::test]
    async fn route_to_net_reaches_a_net_named_only_by_a_power_symbol() {
        // The +24V net exists ONLY via a power symbol pin plus its wire —
        // no labels anywhere. route-to-net must still find it and T onto the
        // wire with a junction.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n  (wire\n    (pts (xy 152.4 96.52) (xy 152.4 109.22))\n    (stroke (width 0) (type default))\n    (uuid \"w-24v\")\n  )",
            placed("R1", 114.3, 101.6, 90.0), // pin 2 at (118.11, 101.6)
            placed_power("#PWR01", 152.4, 96.52),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "net": "+24V" }),
        )
        .await;

        assert_eq!(out["route"], "L", "{out}");
        // The L corner lands mid-wire on the +24V wire → T + junction dot.
        let sch = cse::Schematic::load(&path).unwrap();
        let t_junctions = sch
            .junctions
            .iter()
            .filter(|j| (j.x - 152.4).abs() < 0.01 && (j.y - 101.6).abs() < 0.01)
            .count();
        assert_eq!(
            t_junctions, 1,
            "T onto the existing wire needs a junction: {out}"
        );
        assert_eq!(out["junction_uuids"].as_array().unwrap().len(), 1, "{out}");
    }

    // ─── Connectivity-classified wire touches + jog lane search ────────────

    #[tokio::test]
    async fn unnamed_same_net_trunk_overlap_is_legal_and_unscored() {
        // A bare, UNNAMED trunk wire starts at R1 pin 2. The straight route
        // to R2 pin 1 rides the trunk collinearly — same net by union-find
        // connectivity (the trunk touches the route's from endpoint), so the
        // clean candidate is chosen with ZERO interference. Name comparison
        // would have called this foreign (both nets are null).
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n  (wire\n    (pts (xy 105.41 101.6) (xy 127 101.6))\n    (stroke (width 0) (type default))\n    (uuid \"w-trunk\")\n  )",
            placed("R1", 101.6, 101.6, 90.0),  // pin 2 at (105.41, 101.6) = trunk start
            placed("R2", 121.92, 101.6, 90.0), // pin 1 at (118.11, 101.6), mid-trunk
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;
        assert_eq!(out["route"], "straight", "{out}");
        assert!(
            out["interference"].as_array().unwrap().is_empty(),
            "riding a same-net trunk must not be scored as interference: {out}"
        );
    }

    #[tokio::test]
    async fn unnamed_foreign_collinear_overlap_is_a_hard_block() {
        // A floating, unnamed wire lies exactly on the only (straight)
        // candidate's span. It is connected to neither route endpoint —
        // overlapping it would fuse two unrelated nets in a way
        // find_shorted_nets cannot see (both nets unnamed). Hard block:
        // error out, write nothing.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n  (wire\n    (pts (xy 109.22 101.6) (xy 114.3 101.6))\n    (stroke (width 0) (type default))\n    (uuid \"w-foreign\")\n  )",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 101.6, 90.0),
        ));
        let before = std::fs::read_to_string(&path).unwrap();
        let result = handle_route_connection(
            &json!({
                "schematic": path.display().to_string(),
                "from": { "ref": "R1", "pin": "2" },
                "to": { "ref": "R2", "pin": "1" }
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text content");
        };
        assert!(
            text.contains("\"refused\":true") || text.contains("\"refused\": true"),
            "an unnamed foreign collinear overlap must be a hard block: {text}"
        );
        assert!(
            text.contains("w-foreign"),
            "refusal must name the wire: {text}"
        );
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            before,
            "no wires may be written on a hard block"
        );
    }

    #[tokio::test]
    async fn foreign_vertical_on_the_jog_lane_shifts_the_z_one_lane() {
        // Both L verticals run through blocker pins (hard), the y-jog Z is
        // body-blocked (soft), and an unnamed foreign vertical wire sits
        // exactly on the default x-jog lane (111.76) — a hard collinear
        // overlap. The lane search must slide the Z one lane over (114.3)
        // and route fully clean.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}\n{}\n  (wire\n    (pts (xy 111.76 93.98) (xy 111.76 124.46))\n    (stroke (width 0) (type default))\n    (uuid \"w-foreign-v\")\n  )",
            placed("R1", 101.6, 101.6, 90.0),   // from: pin 2 at (105.41, 101.6)
            placed("R2", 121.92, 116.84, 90.0), // to: pin 1 at (118.11, 116.84)
            placed("R3", 118.11, 101.6, 0.0),   // pin (118.11, 105.41) kills L1
            placed("R4", 105.41, 116.84, 0.0),  // pin (105.41, 113.03) kills L2
            placed("R5", 111.76, 109.22, 0.0),  // body across the y-jog Z's middle leg
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        assert_eq!(out["route"], "Z", "{out}");
        let jog = out["jog_at"].as_f64().expect("Z result must report jog_at");
        assert!(
            (jog - 114.3).abs() < 0.01,
            "the jog must shift one clean lane over: {out}"
        );
        assert!(
            out["interference"].as_array().unwrap().is_empty(),
            "the shifted lane must be fully clean: {out}"
        );
        assert_eq!(
            out["junctions_added"], 0,
            "plain X crossings must not grow junctions: {out}"
        );
        // No placed wire may ride the foreign wire's column.
        let sch = cse::Schematic::load(&path).unwrap();
        for w in sch.wires.iter().filter(|w| w.uuid != "w-foreign-v") {
            assert!(
                !((w.start.0 - 111.76).abs() < 0.01 && (w.end.0 - 111.76).abs() < 0.01),
                "placed wire {:?}-{:?} rides the foreign column",
                w.start,
                w.end
            );
        }
    }

    #[tokio::test]
    async fn successive_fanouts_land_in_adjacent_lanes_without_bias() {
        // Two identical-span routes, NO jog_bias on either. Route 1 takes the
        // default mid lane; route 2's default lane would collinear-overlap
        // route 1's vertical (hard by connectivity — both nets unnamed), so
        // it must auto-pick the adjacent clean lane.
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}",
            placed("R1", 101.6, 101.6, 90.0), // from 1: pin 2 (105.41, 101.6)
            placed("R2", 121.92, 116.84, 90.0), // to 1:   pin 1 (118.11, 116.84)
            placed("R5", 101.6, 106.68, 90.0), // from 2: pin 2 (105.41, 106.68)
            placed("R6", 121.92, 121.92, 90.0), // to 2:   pin 1 (118.11, 121.92)
            // Pin blockers on the L corridors of BOTH routes.
            placed("R3", 118.11, 101.6, 0.0),
            placed("R4", 105.41, 116.84, 0.0),
            placed("R7", 118.11, 106.68, 0.0),
            placed("R8", 105.41, 121.92, 0.0),
        ));
        let out1 = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;
        let out2 = route(
            &path,
            json!({ "ref": "R5", "pin": "2" }),
            json!({ "ref": "R6", "pin": "1" }),
        )
        .await;

        assert_eq!(out1["route"], "Z", "{out1}");
        assert_eq!(out2["route"], "Z", "{out2}");
        assert!(
            out1["interference"].as_array().unwrap().is_empty(),
            "{out1}"
        );
        assert!(
            out2["interference"].as_array().unwrap().is_empty(),
            "no hard blocks and no interference on the second fan-out: {out2}"
        );
        let jog1 = out1["jog_at"].as_f64().unwrap();
        let jog2 = out2["jog_at"].as_f64().unwrap();
        assert!((jog1 - 111.76).abs() < 0.01, "{out1}");
        assert!(
            (jog2 - 114.3).abs() < 0.01,
            "second fan-out must slide to the adjacent clean lane: {out2}"
        );
        // The two Z verticals must not share a column.
        let sch = cse::Schematic::load(&path).unwrap();
        let verticals: Vec<f64> = sch
            .wires
            .iter()
            .filter(|w| (w.start.0 - w.end.0).abs() < 0.01)
            .map(|w| w.start.0)
            .collect();
        assert_eq!(verticals.len(), 2, "one vertical per Z: {verticals:?}");
        assert!(
            (verticals[0] - verticals[1]).abs() > 0.01,
            "fan-out verticals share a column: {verticals:?}"
        );
    }

    // ─── Fix C: created uuids are returned and resolvable ──────────────────

    #[tokio::test]
    async fn route_connection_returns_resolvable_wire_uuids() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 116.84, 90.0),
        ));
        let out = route(
            &path,
            json!({ "ref": "R1", "pin": "2" }),
            json!({ "ref": "R2", "pin": "1" }),
        )
        .await;

        let wire_uuids = out["wire_uuids"].as_array().unwrap();
        assert_eq!(
            wire_uuids.len(),
            out["wires_added"].as_u64().unwrap() as usize
        );
        assert!(!wire_uuids.is_empty(), "{out}");
        assert!(out["junction_uuids"].is_array(), "{out}");

        let sch = cse::Schematic::load(&path).unwrap();
        for u in wire_uuids {
            let u = u.as_str().unwrap();
            assert!(
                sch.wires.iter().any(|w| w.uuid == u),
                "wire uuid {u} must resolve in the file"
            );
        }
    }

    #[tokio::test]
    async fn apply_net_returns_resolvable_wire_uuids_for_both_styles() {
        // Wire style.
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 101.6, 90.0),
        ));
        let result = handle_apply_net(
            &json!({
                "schematic": path.display().to_string(),
                "net": "SIG",
                "pins": [ { "ref": "R1", "pin": "2" }, { "ref": "R2", "pin": "1" } ]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);
        assert_eq!(out["style"], "wire", "{out}");
        let wire_uuids = out["wire_uuids"].as_array().unwrap();
        assert_eq!(wire_uuids.len(), 1, "{out}");
        assert!(out["junction_uuids"].is_array(), "{out}");
        let sch = cse::Schematic::load(&path).unwrap();
        assert!(sch
            .wires
            .iter()
            .any(|w| w.uuid == wire_uuids[0].as_str().unwrap()));

        // Label style: one stub wire per pin, all uuids resolvable.
        let (_d2, path2) = write_sch(&format!(
            "{}\n{}",
            placed("R3", 101.6, 101.6, 0.0),
            placed("R4", 203.2, 203.2, 0.0),
        ));
        let result2 = handle_apply_net(
            &json!({
                "schematic": path2.display().to_string(),
                "net": "SIG",
                "pins": [ { "ref": "R3", "pin": "2" }, { "ref": "R4", "pin": "1" } ],
                "style": "label"
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result2.is_error, "{:?}", result2.content);
        let out2 = parse_result(&result2);
        let stub_uuids = out2["wire_uuids"].as_array().unwrap();
        assert_eq!(stub_uuids.len(), 2, "one stub per pin: {out2}");
        let sch2 = cse::Schematic::load(&path2).unwrap();
        for u in stub_uuids {
            assert!(sch2.wires.iter().any(|w| w.uuid == u.as_str().unwrap()));
        }
    }

    // ─── apply_net ─────────────────────────────────────────────────────────

    #[tokio::test]
    async fn apply_net_auto_picks_labels_for_a_far_pair() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 0.0),
            placed("R2", 203.2, 203.2, 0.0),
        ));
        let result = handle_apply_net(
            &json!({
                "schematic": path.display().to_string(),
                "net": "SIG",
                "pins": [ { "ref": "R1", "pin": "2" }, { "ref": "R2", "pin": "1" } ]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);
        assert_eq!(out["style"], "label", "{out}");
        assert!(out["reason"].as_str().unwrap().contains("50"), "{out}");

        let sch = cse::Schematic::load(&path).unwrap();
        assert_eq!(sch.labels.len(), 2, "one label per pin");
        assert!(sch.labels.iter().all(|l| l.text == "SIG"));
        assert_eq!(sch.wires.len(), 2, "one 2.54mm stub per pin");
        // Stubs point away from the body: R1 pin 2 is at the bottom (y+3.81),
        // so its stub runs further down.
        let stub = sch
            .wires
            .iter()
            .find(|w| (w.start.1 - 105.41).abs() < 0.01 || (w.end.1 - 105.41).abs() < 0.01)
            .expect("stub at R1 pin 2");
        let far_y = stub.start.1.max(stub.end.1);
        assert!(
            (far_y - 107.95).abs() < 0.01,
            "stub must extend AWAY from the body"
        );
    }

    #[tokio::test]
    async fn apply_net_auto_picks_wire_for_a_near_clear_pair() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 90.0),
            placed("R2", 121.92, 101.6, 90.0),
        ));
        let result = handle_apply_net(
            &json!({
                "schematic": path.display().to_string(),
                "net": "SIG",
                "pins": [ { "ref": "R1", "pin": "2" }, { "ref": "R2", "pin": "1" } ]
            }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);
        assert_eq!(out["style"], "wire", "{out}");
        assert!(out["interference"].as_array().unwrap().is_empty());

        let sch = cse::Schematic::load(&path).unwrap();
        assert_eq!(sch.labels.len(), 0, "wire style adds no labels");
        assert_eq!(sch.wires.len(), 1);
    }

    // ─── mark_unused_pins ──────────────────────────────────────────────────

    #[tokio::test]
    async fn mark_unused_pins_ncs_exactly_the_unconnected_pins() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}\n{}\n  (wire\n    (pts (xy 101.6 105.41) (xy 120.65 105.41))\n    (stroke (width 0) (type default))\n    (uuid \"w1\")\n  )\n  (no_connect (at 127 97.79) (uuid \"nc1\"))",
            // R1: pin 2 (101.6, 105.41) is a wire ENDPOINT; pin 1 unconnected.
            placed("R1", 101.6, 101.6, 0.0),
            // R2: pin 1 (127, 97.79) already has a no-connect; pin 2 unconnected.
            placed("R2", 127.0, 101.6, 0.0),
            // R3: pin 1 (114.3, 105.41) taps the MIDDLE of the wire; pin 2 unconnected.
            placed("R3", 114.3, 109.22, 0.0),
        ));

        let result = handle_mark_unused_pins(
            &json!({ "schematic": path.display().to_string() }),
            &test_ctx(),
        )
        .await
        .unwrap();
        assert!(!result.is_error, "{:?}", result.content);
        let out = parse_result(&result);

        assert_eq!(out["added_count"], 3, "{out}");
        assert_eq!(out["skipped_existing"], 1, "R2 pin 1 already had one");
        let per_ref = out["per_ref"].as_array().unwrap();
        let pins_of = |r: &str| {
            per_ref
                .iter()
                .find(|e| e["ref"] == r)
                .map(|e| e["pins"].clone())
        };
        assert_eq!(pins_of("R1").unwrap(), json!(["1"]));
        assert_eq!(pins_of("R2").unwrap(), json!(["2"]));
        assert_eq!(
            pins_of("R3").unwrap(),
            json!(["2"]),
            "the mid-wire tap pin 1 is CONNECTED and must not be NC'd: {out}"
        );

        let sch = cse::Schematic::load(&path).unwrap();
        assert_eq!(sch.no_connects.len(), 4, "1 existing + 3 added");
        // Second run is a no-op — no duplicates.
        let result2 = handle_mark_unused_pins(
            &json!({ "schematic": path.display().to_string() }),
            &test_ctx(),
        )
        .await
        .unwrap();
        let out2 = parse_result(&result2);
        assert_eq!(out2["added_count"], 0, "{out2}");
        assert_eq!(
            cse::Schematic::load(&path).unwrap().no_connects.len(),
            4,
            "no duplicated no-connects"
        );
    }

    #[tokio::test]
    async fn mark_unused_pins_honors_reference_filter() {
        let (_d, path) = write_sch(&format!(
            "{}\n{}",
            placed("R1", 101.6, 101.6, 0.0),
            placed("R2", 127.0, 101.6, 0.0),
        ));
        let result = handle_mark_unused_pins(
            &json!({ "schematic": path.display().to_string(), "references": ["R1"] }),
            &test_ctx(),
        )
        .await
        .unwrap();
        let out = parse_result(&result);
        assert_eq!(out["added_count"], 2, "both R1 pins, nothing of R2: {out}");
        assert!(out["per_ref"]
            .as_array()
            .unwrap()
            .iter()
            .all(|e| e["ref"] == "R1"));
    }
}

#[cfg(test)]
mod junction_tests {
    use super::super::sch_wiring::{
        handle_add_junction, handle_batch_add_junction, handle_batch_add_wire,
    };
    use super::*;
    use crate::router::ToolRouter;
    use crate::tools::ServerConfig;
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

    fn blank_sch() -> (tempfile::TempDir, std::path::PathBuf) {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("wires.kicad_sch");
        std::fs::write(&path, crate::tools::blank_schematic_template()).unwrap();
        (dir, path)
    }

    fn junctions_at(path: &std::path::Path, x: f64, y: f64) -> usize {
        let sch = cse::Schematic::load(path).unwrap();
        sch.junctions
            .iter()
            .filter(|j| (j.x - x).abs() < 0.01 && (j.y - y).abs() < 0.01)
            .count()
    }

    fn junction_count(path: &std::path::Path) -> usize {
        cse::Schematic::load(path).unwrap().junctions.len()
    }

    // All coordinates are 1.27 mm multiples so grid snapping is a no-op.

    #[tokio::test]
    async fn repeated_batch_add_wire_does_not_duplicate_junctions() {
        let (_d, path) = blank_sch();
        let ctx = test_ctx();
        let sch = path.display().to_string();

        // Main run plus a branch dropping from its middle: T at (114.3, 101.6).
        let result = handle_batch_add_wire(
            &json!({ "schematic": sch, "wires": [
                { "x1": 101.6, "y1": 101.6, "x2": 127.0, "y2": 101.6 },
                { "x1": 114.3, "y1": 101.6, "x2": 114.3, "y2": 88.9 }
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert!(!result.is_error);
        assert_eq!(junctions_at(&path, 114.3, 101.6), 1);

        // A second batch with another branch must add ONE junction at the new
        // T and none at the pre-existing one.
        handle_batch_add_wire(
            &json!({ "schematic": sch, "wires": [
                { "x1": 119.38, "y1": 101.6, "x2": 119.38, "y2": 88.9 }
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert_eq!(junctions_at(&path, 114.3, 101.6), 1, "no re-inserted dot");
        assert_eq!(junctions_at(&path, 119.38, 101.6), 1);
        assert_eq!(junction_count(&path), 2);

        // Re-adding the same branch wire again still must not duplicate dots.
        handle_batch_add_wire(
            &json!({ "schematic": sch, "wires": [
                { "x1": 114.3, "y1": 101.6, "x2": 114.3, "y2": 88.9 }
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert_eq!(junctions_at(&path, 114.3, 101.6), 1);
        assert_eq!(junction_count(&path), 2);
    }

    #[tokio::test]
    async fn add_junction_twice_at_same_coord_yields_one() {
        let (_d, path) = blank_sch();
        let ctx = test_ctx();
        let args = json!({ "schematic": path.display().to_string(), "x": 101.6, "y": 101.6 });

        assert!(
            !handle_add_junction(&args, &test_ctx())
                .await
                .unwrap()
                .is_error
        );
        assert!(!handle_add_junction(&args, &ctx).await.unwrap().is_error);
        assert_eq!(junctions_at(&path, 101.6, 101.6), 1);

        // Batch path dedupes too, including within one call.
        handle_batch_add_junction(
            &json!({ "schematic": path.display().to_string(), "positions": [
                { "x": 101.6, "y": 101.6 },
                { "x": 101.6, "y": 101.6 },
                { "x": 127.0, "y": 101.6 }
            ]}),
            &ctx,
        )
        .await
        .unwrap();
        assert_eq!(junctions_at(&path, 101.6, 101.6), 1);
        assert_eq!(junction_count(&path), 2);
    }

    async fn cleanup(path: &std::path::Path, dry_run: bool) -> CallToolResult {
        handle_cleanup_junctions(
            &json!({ "schematic": path.display().to_string(), "dry_run": dry_run }),
            &test_ctx(),
        )
        .await
        .unwrap()
    }

    /// A wire with three junctions stacked on one mid-wire point plus an
    /// orphan dot floating in empty space — the observed real-file damage.
    fn damaged_sch() -> (tempfile::TempDir, std::path::PathBuf) {
        let (dir, path) = blank_sch();
        let mut sch = cse::Schematic::load(&path).unwrap();
        sch.add_wire(101.6, 101.6, 127.0, 101.6);
        for _ in 0..3 {
            // Push directly — add_junction itself dedupes now.
            sch.junctions.push(cse::Junction::new(114.3, 101.6));
        }
        sch.junctions.push(cse::Junction::new(50.8, 50.8)); // orphan
        sch.overwrite().unwrap();
        (dir, path)
    }

    #[tokio::test]
    async fn cleanup_junctions_dedupes_and_removes_orphans() {
        let (_d, path) = damaged_sch();
        let result = cleanup(&path, false).await;
        assert!(!result.is_error);

        assert_eq!(
            junctions_at(&path, 114.3, 101.6),
            1,
            "one dot kept per coord"
        );
        assert_eq!(junctions_at(&path, 50.8, 50.8), 0, "orphan removed");
        assert_eq!(junction_count(&path), 1);

        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text result");
        };
        let out: serde_json::Value = serde_json::from_str(text).unwrap();
        assert_eq!(out["removed"], 3);
        assert_eq!(out["duplicates_removed"].as_array().unwrap().len(), 2);
        assert_eq!(out["orphans_removed"].as_array().unwrap().len(), 1);
    }

    #[tokio::test]
    async fn cleanup_junctions_dry_run_leaves_file_untouched() {
        let (_d, path) = damaged_sch();
        let before = std::fs::read_to_string(&path).unwrap();

        let result = cleanup(&path, true).await;
        assert!(!result.is_error);
        assert_eq!(
            std::fs::read_to_string(&path).unwrap(),
            before,
            "dry_run must not write"
        );

        let crate::mcp::protocol::ToolContent::Text { text } = &result.content[0] else {
            panic!("expected text result");
        };
        let out: serde_json::Value = serde_json::from_str(text).unwrap();
        assert_eq!(out["removed"], 3, "dry run still reports what it would do");
        assert_eq!(out["applied"], false);
    }
}
