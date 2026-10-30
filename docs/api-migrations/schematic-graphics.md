### `add_schematic_graphics`, `list_schematic_graphics`, `edit_schematic_text` (minor release)

Three new `sch_batch` tools work with drawing-only schematic items.
`add_schematic_graphics(schematic, items)` adds polylines, rectangles, circles
and text in one write and returns their `uuids`; one bad item refuses the whole
call. `list_schematic_graphics(schematic)` lists top-level text, polylines,
rectangles, circles and arcs with their UUIDs, which `batch_delete` accepts.
`edit_schematic_text(schematic, uuid, text?, x?, y?, size?)` changes a top-level
text item in place. No existing tool's arguments or responses change. `sch_batch`
rises from 13 to 16 tools.
