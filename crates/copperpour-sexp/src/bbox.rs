//! Conservative bounding-box estimation for placed schematic symbols.
//!
//! Placement is more than half the battle: these boxes power gap/pitch
//! computation in the placement tools, overlap detection in the placement
//! oracle, and obstacle *reporting* (never avoidance) in the router. The
//! estimates are approximate but deliberately conservative — a box may be a
//! little larger than the drawn symbol, never smaller.
//!
//! Sources of extent, resolved through the `(extends …)` chain:
//! - drawing primitives: `rectangle`, `polyline`, `circle`, `arc`
//! - pin tips and pin body-attachment ends
//! - (via [`field_text_bbox_estimate`]) visible Reference/Value field text,
//!   estimated as chars × font_size × 0.7 wide, font_size × 1.3 tall.

use crate::geometry::{transform_pin, PinTransform};
use crate::parser::SexpNode;
use crate::schematic::{parse_at, resolve_extends_chain, SymbolInstance};

// ─── Rect ─────────────────────────────────────────────────────────────────────

/// Axis-aligned rectangle in schematic (Y-down) space, millimetres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

impl Rect {
    pub fn from_point(x: f64, y: f64) -> Rect {
        Rect {
            min_x: x,
            min_y: y,
            max_x: x,
            max_y: y,
        }
    }

    pub fn include(&mut self, x: f64, y: f64) {
        self.min_x = self.min_x.min(x);
        self.min_y = self.min_y.min(y);
        self.max_x = self.max_x.max(x);
        self.max_y = self.max_y.max(y);
    }

    pub fn union(&self, other: &Rect) -> Rect {
        Rect {
            min_x: self.min_x.min(other.min_x),
            min_y: self.min_y.min(other.min_y),
            max_x: self.max_x.max(other.max_x),
            max_y: self.max_y.max(other.max_y),
        }
    }

    pub fn width(&self) -> f64 {
        self.max_x - self.min_x
    }

    pub fn height(&self) -> f64 {
        self.max_y - self.min_y
    }

    /// Grow (or shrink, with negative margin) on all four sides.
    pub fn expand(&self, margin: f64) -> Rect {
        Rect {
            min_x: self.min_x - margin,
            min_y: self.min_y - margin,
            max_x: self.max_x + margin,
            max_y: self.max_y + margin,
        }
    }

    /// True overlap with positive area — boxes that merely share an edge or a
    /// corner (a pin tip landing exactly on a neighbour's box) do NOT count.
    pub fn intersects(&self, other: &Rect) -> bool {
        const EPS: f64 = 1e-6;
        self.min_x < other.max_x - EPS
            && other.min_x < self.max_x - EPS
            && self.min_y < other.max_y - EPS
            && other.min_y < self.max_y - EPS
    }

    /// The overlap region, when [`Rect::intersects`] is true.
    pub fn intersection(&self, other: &Rect) -> Option<Rect> {
        if !self.intersects(other) {
            return None;
        }
        Some(Rect {
            min_x: self.min_x.max(other.min_x),
            min_y: self.min_y.max(other.min_y),
            max_x: self.max_x.min(other.max_x),
            max_y: self.max_y.min(other.max_y),
        })
    }

    pub fn contains(&self, x: f64, y: f64) -> bool {
        x >= self.min_x && x <= self.max_x && y >= self.min_y && y <= self.max_y
    }
}

// ─── Symbol body bbox ─────────────────────────────────────────────────────────

/// Bounding box of a library symbol's drawn body — graphics primitives plus
/// pin endpoints — resolved through the extends chain and transformed by the
/// instance placement `t` into schematic space. `None` when the definition
/// draws nothing at all.
pub fn symbol_bbox(lib_syms: &[&SexpNode], sym_node: &SexpNode, t: PinTransform) -> Option<Rect> {
    let mut local_pts: Vec<(f64, f64)> = Vec::new();
    for sym in resolve_extends_chain(lib_syms, sym_node) {
        collect_graphic_points(sym, &mut local_pts);
    }
    let mut rect: Option<Rect> = None;
    for (lx, ly) in local_pts {
        let (x, y) = transform_pin(lx, ly, t);
        match &mut rect {
            Some(r) => r.include(x, y),
            None => rect = Some(Rect::from_point(x, y)),
        }
    }
    rect
}

/// [`symbol_bbox`] looked up by a placed instance's `lib_id`.
pub fn symbol_bbox_for_instance(lib_syms: &[&SexpNode], inst: &SymbolInstance) -> Option<Rect> {
    let sym = lib_syms
        .iter()
        .find(|n| n.get(1).and_then(|c| c.as_str()) == Some(inst.lib_id.as_str()))?;
    symbol_bbox(lib_syms, sym, inst.pin_transform())
}

/// Collect extreme points (in symbol-local, Y-up space) of every drawing
/// primitive, recursing into unit sub-symbols ("R_1_1", …) exactly like pin
/// extraction does.
fn collect_graphic_points(node: &SexpNode, out: &mut Vec<(f64, f64)>) {
    for rect in node.find_all("rectangle") {
        if let (Some(s), Some(e)) = (rect.find("start"), rect.find("end")) {
            if let (Some(sx), Some(sy), Some(ex), Some(ey)) =
                (s.get_f64(1), s.get_f64(2), e.get_f64(1), e.get_f64(2))
            {
                out.push((sx, sy));
                out.push((ex, ey));
            }
        }
    }
    for poly in node.find_all("polyline") {
        if let Some(pts) = poly.find("pts") {
            for xy in pts.find_all("xy") {
                if let (Some(x), Some(y)) = (xy.get_f64(1), xy.get_f64(2)) {
                    out.push((x, y));
                }
            }
        }
    }
    for circle in node.find_all("circle") {
        if let Some(c) = circle.find("center") {
            if let (Some(cx), Some(cy)) = (c.get_f64(1), c.get_f64(2)) {
                let r = circle
                    .find("radius")
                    .and_then(|r| r.get_f64(1))
                    .unwrap_or(0.0);
                // Axis-extreme points of the circle. Under KiCAD's 90°-step
                // rotations these map to axis-extreme points again, so the
                // transformed bbox still contains the circle.
                out.push((cx - r, cy - r));
                out.push((cx + r, cy + r));
            }
        }
    }
    for arc in node.find_all("arc") {
        for tag in ["start", "mid", "end"] {
            if let Some(p) = arc.find(tag) {
                if let (Some(x), Some(y)) = (p.get_f64(1), p.get_f64(2)) {
                    out.push((x, y));
                }
            }
        }
    }
    for pin in node.find_all("pin") {
        if let Some((x, y, rot)) = parse_at(pin) {
            out.push((x, y)); // electrical tip
            let len = pin.find("length").and_then(|l| l.get_f64(1)).unwrap_or(0.0);
            // The pin angle points from the tip toward the body; the drawn pin
            // line extends `length` mm that way.
            let rad = rot.to_radians();
            out.push((x + rad.cos() * len, y + rad.sin() * len));
        }
    }
    for sub in node.find_all("symbol") {
        collect_graphic_points(sub, out);
    }
}

// ─── Field text bbox estimate ─────────────────────────────────────────────────

/// Estimated rectangles of the visible Reference and Value field texts of a
/// *placed* `(symbol …)` node. Property `(at …)` positions are absolute
/// schematic coordinates, so no instance transform is applied; the box is
/// centred on the stored anchor (conservative for the common centred
/// justification), sized chars × font_size × 0.7 wide and font_size × 1.3
/// tall, swapping width/height when the field is rotated to vertical.
pub fn field_text_bbox_estimate(instance_node: &SexpNode) -> Vec<Rect> {
    let mut out = Vec::new();
    for prop in instance_node.find_all("property") {
        let Some(name) = prop.get(1).and_then(|n| n.as_str()) else {
            continue;
        };
        if name != "Reference" && name != "Value" {
            continue;
        }
        if is_hidden(prop) {
            continue;
        }
        let Some(value) = prop.get(2).and_then(|n| n.as_str()) else {
            continue;
        };
        let Some((x, y, rot)) = parse_at(prop) else {
            continue;
        };
        let size = prop
            .find("effects")
            .and_then(|e| e.find("font"))
            .and_then(|f| f.find("size"))
            .and_then(|s| s.get_f64(1))
            .unwrap_or(1.27);
        let mut w = (value.chars().count().max(1) as f64) * size * 0.7;
        let mut h = size * 1.3;
        let deg = ((rot % 360.0) + 360.0) % 360.0;
        if (deg - 90.0).abs() < 1.0 || (deg - 270.0).abs() < 1.0 {
            std::mem::swap(&mut w, &mut h);
        }
        out.push(Rect {
            min_x: x - w / 2.0,
            min_y: y - h / 2.0,
            max_x: x + w / 2.0,
            max_y: y + h / 2.0,
        });
    }
    out
}

fn is_hidden(prop: &SexpNode) -> bool {
    prop.find("effects")
        .and_then(|e| e.find("hide"))
        .map(|h| h.get(1).and_then(|v| v.as_str()) != Some("no"))
        .unwrap_or(false)
}

// ─── Combined instance extent ─────────────────────────────────────────────────

/// Full occupied extent of a placed symbol: body bbox ∪ (optionally) its
/// visible field-text boxes. `instance_node` is the raw placed `(symbol …)`
/// node (see `extract_symbol_instances_with_nodes`).
pub fn instance_extent(
    lib_syms: &[&SexpNode],
    inst: &SymbolInstance,
    instance_node: &SexpNode,
    include_text: bool,
) -> Option<Rect> {
    let mut rect = symbol_bbox_for_instance(lib_syms, inst);
    if include_text {
        for t in field_text_bbox_estimate(instance_node) {
            rect = Some(match rect {
                Some(r) => r.union(&t),
                None => t,
            });
        }
    }
    rect
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parser::parse_sexp;
    use crate::schematic::extract_symbol_instances_with_nodes;

    /// A Device:R-shaped symbol: body rectangle x ∈ [-1.016, 1.016],
    /// y ∈ [-2.54, 2.54]; pin tips at (0, ±3.81), lengths 1.27.
    const SCH: &str = r#"(kicad_sch
  (lib_symbols
    (symbol "Dev:R"
      (symbol "R_1_1"
        (rectangle (start -1.016 2.54) (end 1.016 -2.54))
        (pin passive line (at 0 3.81 270) (length 1.27) (name "~") (number "1"))
        (pin passive line (at 0 -3.81 90) (length 1.27) (name "~") (number "2"))
      )
    )
    (symbol "Dev:R_Derived"
      (extends "Dev:R")
    )
  )
  (symbol
    (lib_id "Dev:R")
    (at 101.6 101.6 0)
    (uuid "u-r1")
    (property "Reference" "R1" (at 104.14 100.33 0))
    (property "Value" "10k" (at 104.14 102.87 0))
    (property "Footprint" ""
      (at 101.6 101.6 0)
      (effects (font (size 1.27 1.27)) (hide yes))
    )
  )
)"#;

    fn setup(sch: &str) -> (SexpNode, ()) {
        (parse_sexp(sch).unwrap(), ())
    }

    #[test]
    fn body_bbox_at_rotation_0_matches_hand_computation() {
        let (tree, _) = setup(SCH);
        let lib_syms = tree
            .find("lib_symbols")
            .map(|n| n.find_all("symbol"))
            .unwrap_or_default();
        let pairs = extract_symbol_instances_with_nodes(&tree);
        let (inst, _) = &pairs[0];
        let r = symbol_bbox_for_instance(&lib_syms, inst).expect("bbox");
        // Local Y-up y ∈ [-3.81, 3.81] flips to screen y ∈ [-3.81, 3.81] around
        // the origin; x ∈ [-1.016, 1.016].
        assert!((r.min_x - (101.6 - 1.016)).abs() < 1e-9, "{r:?}");
        assert!((r.max_x - (101.6 + 1.016)).abs() < 1e-9, "{r:?}");
        assert!((r.min_y - (101.6 - 3.81)).abs() < 1e-9, "{r:?}");
        assert!((r.max_y - (101.6 + 3.81)).abs() < 1e-9, "{r:?}");
    }

    #[test]
    fn body_bbox_rotates_with_the_instance() {
        let rotated = SCH.replace("(at 101.6 101.6 0)", "(at 101.6 101.6 90)");
        let (tree, _) = setup(&rotated);
        let lib_syms = tree
            .find("lib_symbols")
            .map(|n| n.find_all("symbol"))
            .unwrap_or_default();
        let pairs = extract_symbol_instances_with_nodes(&tree);
        let r = symbol_bbox_for_instance(&lib_syms, &pairs[0].0).expect("bbox");
        // Long axis swings onto X.
        assert!((r.width() - 7.62).abs() < 1e-9, "{r:?}");
        assert!((r.height() - 2.032).abs() < 1e-9, "{r:?}");
    }

    #[test]
    fn derived_symbol_inherits_base_body() {
        let derived = SCH.replace("(lib_id \"Dev:R\")", "(lib_id \"Dev:R_Derived\")");
        let (tree, _) = setup(&derived);
        let lib_syms = tree
            .find("lib_symbols")
            .map(|n| n.find_all("symbol"))
            .unwrap_or_default();
        let pairs = extract_symbol_instances_with_nodes(&tree);
        let r = symbol_bbox_for_instance(&lib_syms, &pairs[0].0)
            .expect("derived symbol must inherit body via extends");
        assert!((r.height() - 7.62).abs() < 1e-9, "{r:?}");
    }

    #[test]
    fn field_text_boxes_cover_visible_fields_only() {
        let (tree, _) = setup(SCH);
        let pairs = extract_symbol_instances_with_nodes(&tree);
        let boxes = field_text_bbox_estimate(pairs[0].1);
        assert_eq!(
            boxes.len(),
            2,
            "Reference + Value; hidden Footprint skipped"
        );
        // "10k" at size 1.27: 3 chars × 1.27 × 0.7 wide.
        let value_box = &boxes[1];
        assert!((value_box.width() - 3.0 * 1.27 * 0.7).abs() < 1e-9);
        assert!((value_box.height() - 1.27 * 1.3).abs() < 1e-9);
    }

    #[test]
    fn instance_extent_unions_body_and_text() {
        let (tree, _) = setup(SCH);
        let lib_syms = tree
            .find("lib_symbols")
            .map(|n| n.find_all("symbol"))
            .unwrap_or_default();
        let pairs = extract_symbol_instances_with_nodes(&tree);
        let (inst, node) = &pairs[0];
        let body = symbol_bbox_for_instance(&lib_syms, inst).unwrap();
        let full = instance_extent(&lib_syms, inst, node, true).unwrap();
        assert!(
            full.max_x > body.max_x,
            "text at x=104.14 widens the extent"
        );
        let no_text = instance_extent(&lib_syms, inst, node, false).unwrap();
        assert_eq!(no_text, body);
    }

    #[test]
    fn rect_intersection_semantics() {
        let a = Rect {
            min_x: 0.0,
            min_y: 0.0,
            max_x: 10.0,
            max_y: 10.0,
        };
        let b = Rect {
            min_x: 5.0,
            min_y: 5.0,
            max_x: 15.0,
            max_y: 15.0,
        };
        let touching = Rect {
            min_x: 10.0,
            min_y: 0.0,
            max_x: 20.0,
            max_y: 10.0,
        };
        assert!(a.intersects(&b));
        let i = a.intersection(&b).unwrap();
        assert_eq!((i.min_x, i.min_y, i.max_x, i.max_y), (5.0, 5.0, 10.0, 10.0));
        assert!(
            !a.intersects(&touching),
            "edge-sharing boxes are not an overlap"
        );
        assert!((a.union(&b).max_x - 15.0).abs() < 1e-12);
    }
}
