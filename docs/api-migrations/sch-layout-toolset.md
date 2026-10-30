### New `sch_layout` toolset (minor release)

A new toolset of seven tools places schematic components and routes between
placed pins. `place_row` lines up a chain so each in-flow pin is collinear with
the previous out-flow pin, `repeat_placement` copies a placement across an array
and `check_placement` reports overlaps, off-grid pins and misaligned pairs.
`route_connection` draws a wire between two pins and refuses, with the list of
blockers, a route that would touch a foreign pin or another net's wire.
`apply_net` connects several pins with wires or labels, `mark_unused_pins` adds
no-connect flags to unattached pins, and `cleanup_junctions` removes duplicate
and orphaned junction dots. No existing tool's arguments or responses change.
