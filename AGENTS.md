# AGENTS.md

Guidance for coding agents working on Copperpour. Read the human docs first; this file
adds only the traps they don't spell out.

- [CONTRIBUTING.md](CONTRIBUTING.md): setup, `protoc`, adding tools, PR checklist
- [DEV.md](DEV.md): crate layout, IPC vs. S-expression editing, structured errors, router
- [docs/RELIABILITY_CONTRACT.md](docs/RELIABILITY_CONTRACT.md): argument, mutation,
  result and recovery rules for every tool
- [docs/NAMING_CONVENTIONS.md](docs/NAMING_CONVENTIONS.md): identifiers, `KiCad` vs.
  `KiCAD`, `snake_case` JSON keys

## Before you push

Run what `.github/workflows/ci.yml` runs:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --locked --all-targets -- -D warnings
cargo test -p copperpour-core --locked reliability_contract
cargo test --workspace --locked --lib --tests
cargo test --workspace --locked --doc
```

`crates/schematic-viewer` is outside the workspace, so the commands above never build it.
If you touched it, also run
`cargo test --locked --manifest-path crates/schematic-viewer/Cargo.toml`. If you added or
removed a tool, run `cargo xtask fix-doc-counts` and commit the result.

## Public API

MCP tool names, toolset names, arguments, CLI flags, env vars, config keys, JSON keys and
documented paths are public API. Never rename or repurpose one silently: add an alias or
a deprecation period, and a migration note in `docs/api-migrations/` (see its README).

## Nets: identity vs. name

A net's identity is its `NetGraph` root (`tools/sch_connectivity.rs`), never its name.
One net can carry several names, such as a `+3V3` power symbol beside a `VCC` label, so
comparing by name fails against the net itself. Compare with `root_at`; use
`name_of_root` and `net_at` only to *report* a net.

KiCad joins a sheet's nets by name as well as by wire. `seed_net_graph` joins points by
geometry, then `merge_named_nets` joins points that share a name: two segments that each
carry a `SIG` label are one net, and every `GND` symbol on the sheet is one rail.

A net's reported name follows KiCad's precedence, encoded in `driver_priority`: global
label > power symbol > local label > hierarchical label, ties broken by name ascending
(measured against KiCad 10.0.5). A local label cannot rename a power rail.

## File writes

DEV.md describes the revision-checked atomic writes and the create-only `save()`. Never
delete a destination to get around a `conflict`, and never weaken `save()` into an
unconditional overwrite: a conflict from KiCad's autosave means the mechanism works, so
reload and reapply.

`write_atomic_if_unchanged` returns the same `SexpError::Conflict` when the file changed
before the write and when the readback fails after it. The first is a `conflict`
refusal; the second is `mutation_outcome_uncertain`, because the file was already
replaced. To tell them apart, call `transact_atomic` and compare inside its closure, as
`pcb_board.rs::write_board_edits` does.

For a post-write readback, parse with `parse_sexp` and read positions with `parse_at`,
as `sch_wiring.rs::observed_no_connect_moves` does. `Schematic::from_source` in
`copperpour-schematic-editor` prints every item it skips to stderr, so a readback through it
adds noise on each call.

## Parallel sessions

When several agents share one clone, give each task its own git worktree and its own
`CARGO_TARGET_DIR`. Switching branches in a shared checkout changes files under another
session's running tests, and a shared target directory can hand one worktree another's
stale build artifacts.

## This fork (dsandall/Copperpour)

This is a public fork of [Copperpour/Copperpour](https://github.com/Copperpour/Copperpour).
Keep this section when merging from upstream.

- **Public by default.** Everything on this fork is public, so push finished work here
  rather than holding it in a private remote. Before pushing, leave out secrets, private
  board designs and customer data.
- **Offer changes upstream, regularly.** Every time work lands here, and whenever you start
  a session in this repo, check `gh pr list -R Copperpour/Copperpour --author dsandall` and
  `git log copperpour/main..main`. Open a PR for each change that upstream doesn't have,
  and follow up on the ones already open. Keep carrying a change here only if upstream
  declines it, and record why in the table below.
- **Follow upstream's process for those PRs:** one reviewable outcome per PR, branched from
  current `copperpour/main`, with their commit style (`fix(scope): summary`) and the checks
  in "Before you push". Open an issue first for anything non-trivial. Contributions are
  AGPL-3.0 only; if an employer owns the work, get its permission first.
- **Branches.** `main` is `copperpour/main` plus this fork's merged topic branches. Merge
  upstream into `main` often. Never rebase or force-push it.
- **Remotes.** `github` is this fork and `copperpour` is upstream. Never base work on
  Konnect-EDA, which is closed source, and don't copy code from it.

Fork-only commits use `KIND: scope: summary`, subject at most 72 characters. The domain kind
is `TOOL`, for changes to the MCP tool surface. Scopes are toolset and crate names such as
`sch_layout`, `sch_wiring`, `sch_batch`, `sch_hierarchy`, `ipc`, `sexp`, `install` and `fork`.

| Carried here | Upstream status |
|--------------|-----------------|
| `sch_layout` toolset | To be offered: uses its own union-find, not `NetGraph` |
| Schematic graphics tools | PR pending |
| Sheet resize and pin side fix | PR pending |
| `add_junction` duplicate fix | PR pending |
