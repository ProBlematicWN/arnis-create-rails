//! Deterministic mini-world with test tracks for in-game verification.
//!
//! Generates a 128×128 world (one Anvil region, `region/r.0.0.mca`) with a
//! hardcoded set of tracks: straight north–south, straight east–west, a
//! corner turn, and an ascending slope. No OSM data involved — the layout
//! is fully deterministic, so the world regenerates identically on every
//! run.
//!
//! Stage 2 mapping applies: the straight segments are `create:track` blocks
//! (`shape=zo`/`xo`, `turn=false`); the single ascending cell on track D
//! stays a vanilla rail until stage 4.
//!
//! The track C corner (48,24) has no stage-2 equivalent: exactly like the
//! pipeline, the cell keeps the first segment's straight shape (`zo`), so
//! the E-W arm starts at (49,24). Both arms are complete straight Create
//! tracks; the corner joint is expected to be broken until stage 3 (curves).
//!
//! Used to verify the generated world in-game (MC 1.21.1 + Create) before
//! and during the Create track format work (see ROADMAP stage 0). The rail
//! selection reuses the pipeline's own `determine_rail_with_slope` and
//! `at_grade_rail_props`, so the mini-world's rails are exactly what the
//! pipeline would place for the same ground profile.
//!
//! Trains: Create trains are `create:carriage_contraption` entities (spawned
//! from `create:carriage` blocks), not vanilla `minecraft:minecart`. This
//! world places no train — to ride the tracks, put a Create carriage or
//! locomotive on a rail in-game. Pre-placing a Create carriage would mean
//! hand-crafting its `Contraption` NBT, which we avoid (see NOTES.md §7).
//!
//! ```bash
//! cargo run --release --no-default-features -- \
//!   --track-test-world=/path/to/saves/create-track-test
//! ```

use std::path::{Path, PathBuf};

use crate::args::{GameMode, WorldType};
use crate::block_definitions::{
    create_track_shape, BlockWithProperties, CREATE_TRACK, GRASS_BLOCK, GRAVEL, STONE,
};
use crate::bresenham::bresenham_line;
use crate::coordinate_system::cartesian::XZBBox;
use crate::coordinate_system::geographic::LLBBox;
use crate::element_processing::railways::{
    at_grade_rail_props, determine_rail_with_slope, smooth_diagonal_rails,
};
use crate::world_editor::WorldEditor;
use crate::world_utils;

/// World size: 128×128 blocks = 8×8 chunks = one Anvil region.
const WORLD_SIZE: i32 = 128;

/// Ground level. Matches `world_editor::common::DEFAULT_GROUND_LEVEL`, so
/// out-of-bbox filler chunks line up with the test terrain.
const GROUND_LEVEL: i32 = -62;

/// Raised mesa the slope track climbs onto: x ∈ [88, 120], z ∈ [56, 112],
/// one block above the surrounding plain.
const PLATEAU_MIN_X: i32 = 88;
const PLATEAU_MAX_X: i32 = 120;
const PLATEAU_MIN_Z: i32 = 56;
const PLATEAU_MAX_Z: i32 = 112;

/// Test tracks as polylines in absolute world coordinates.
///
/// A: straight north–south.
/// B: straight east–west.
/// C: corner turn (south → east).
/// D: ascending slope across the plateau edge.
const TRACKS: &[&[(i32, i32)]] = &[
    &[(16, 8), (16, 40)],
    &[(48, 16), (80, 16)],
    &[(48, 40), (48, 24), (80, 24)],
    &[(96, 48), (96, 64)],
];

/// Player spawn: standing on the ground next to track A.
const SPAWN: (i32, i32, i32) = (20, -61, 16);

/// Standalone `create:track` blocks (stage 1: prove a mod-namespace block with
/// a `shape` state property round-trips through the palette). Placed on the
/// flat ground, away from the vanilla test rails, in chunk (0,0).
/// `(x, z, shape)` — `xo` is the east–west straight, `zo` the north–south one.
const CREATE_TRACKS: &[(i32, i32, &str)] = &[(10, 10, "xo"), (12, 12, "zo")];

/// A straight north–south line of `create:track` blocks (`shape=zo`), for the
/// in-game compatibility check: is a generated track a *functional* Create
/// track (a carriage rides it end-to-end) or just a decorative block? Placed
/// on flat ground, clear of the vanilla test rails.
const CREATE_TRACK_LINE_X: i32 = 30;
const CREATE_TRACK_LINE_Z0: i32 = 20;
const CREATE_TRACK_LINE_Z1: i32 = 44;

/// Generate the track test world into `world_dir`.
///
/// `world_dir` must not exist or must be empty; a non-empty directory is
/// rejected so an existing world is never clobbered. Returns the world path.
pub fn generate(world_dir: &Path) -> Result<PathBuf, String> {
    if world_dir.exists() {
        let non_empty = std::fs::read_dir(world_dir)
            .map(|mut d| d.next().is_some())
            .unwrap_or(true);
        if non_empty {
            return Err(format!(
                "{} is not empty; delete it first",
                world_dir.display()
            ));
        }
    }

    let level_name = world_dir
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("Create Track Test World")
        .to_string();

    // 1. World skeleton: level.dat, icon, region template.
    world_utils::write_world_skeleton(world_dir, &level_name, true)?;
    // 2. The template region is replaced by the save below.
    world_utils::remove_untouched_template_region(world_dir);

    // 3. Editor over the 128×128 area.
    let xzbbox = XZBBox::rect_from_min_max(0, 0, WORLD_SIZE - 1, WORLD_SIZE - 1)
        .map_err(|e| format!("invalid test world bbox: {e}"))?;
    let llbbox = LLBBox::new(54.6, 9.9, 54.61, 9.91)
        .map_err(|e| format!("invalid test world llbbox: {e}"))?;
    let mut editor = WorldEditor::new(world_dir.to_path_buf(), &xzbbox, llbbox);

    // 4. Tracks first, exactly like the pipeline: the gravel bed and the rails
    //    land in still-air cells, and the ground placed afterwards only fills
    //    air (`set_with_props_if_absent`), so the bed survives.
    for track in TRACKS {
        place_track(&mut editor, track);
    }

    // 4b. Standalone Create tracks (stage 1): one block id, the shape carried
    //     as a `shape` state property. `turn`/`waterlogged` stay at their
    //     default `false` and are omitted.
    for (x, z, shape) in CREATE_TRACKS {
        editor.set_block_with_properties_absolute(
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape(shape))),
            *x,
            GROUND_LEVEL + 1,
            *z,
            None,
            None,
        );
    }

    // 4c. A straight Create track line (compatibility check): a run of
    //     `create:track` blocks with `shape=zo`, `turn=false`. If a carriage
    //     rides it end-to-end in-game, the generated track is functional,
    //     not decorative.
    for z in CREATE_TRACK_LINE_Z0..=CREATE_TRACK_LINE_Z1 {
        editor.set_block_with_properties_absolute(
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("zo"))),
            CREATE_TRACK_LINE_X,
            GROUND_LEVEL + 1,
            z,
            None,
            None,
        );
    }

    // 5. Flat ground, raised mesa.
    for z in 0..WORLD_SIZE {
        for x in 0..WORLD_SIZE {
            if on_plateau((x, z)) {
                editor.set_block_absolute(STONE, x, GROUND_LEVEL, z, None, None);
                editor.set_block_absolute(GRASS_BLOCK, x, GROUND_LEVEL + 1, z, None, None);
            } else {
                editor.set_block_absolute(GRASS_BLOCK, x, GROUND_LEVEL, z, None, None);
            }
        }
    }

    // 6. Save (replaces the template region).
    editor
        .save()
        .map_err(|e| format!("failed to save track test world: {e}"))?;

    // 7. World settings: creative, noon, void outside the island.
    world_utils::apply_java_world_settings(world_dir, GameMode::Creative, 6000, WorldType::Void)?;
    // 8. Spawn point.
    world_utils::set_spawn_in_level_dat(world_dir, SPAWN.0, SPAWN.1, SPAWN.2)?;

    Ok(world_dir.to_path_buf())
}

/// Whether (x, z) is on the raised mesa.
fn on_plateau(xz: (i32, i32)) -> bool {
    let (x, z) = xz;
    (PLATEAU_MIN_X..=PLATEAU_MAX_X).contains(&x) && (PLATEAU_MIN_Z..=PLATEAU_MAX_Z).contains(&z)
}

/// Ground level at (x, z): the base level, one block up on the mesa.
fn ground_level_at(xz: (i32, i32)) -> i32 {
    if on_plateau(xz) {
        GROUND_LEVEL + 1
    } else {
        GROUND_LEVEL
    }
}

/// Place one polyline the way `generate_at_grade_rail` does: bresenham +
/// diagonal smoothing, joint points skipped on later segments, gravel bed at
/// ground level, rail one block above. Rail selection is the pipeline's own
/// `determine_rail_with_slope`, fed the same (prev, current, next) context
/// and ground levels (from `ground_level_at` instead of the editor's
/// elevation grid — this world has no OSM data).
fn place_track(editor: &mut WorldEditor, track: &[(i32, i32)]) {
    for i in 1..track.len() {
        let (x0, z0) = track[i - 1];
        let (x1, z1) = track[i];

        let points = bresenham_line(x0, 0, z0, x1, 0, z1);
        let smoothed_points = smooth_diagonal_rails(&points);
        let skip_first = if i > 1 { 1 } else { 0 };

        for j in skip_first..smoothed_points.len() {
            let (bx, _, bz) = smoothed_points[j];

            let prev_ground = if j > 0 {
                let (px, _, pz) = smoothed_points[j - 1];
                ground_level_at((px, pz))
            } else {
                ground_level_at((bx, bz))
            };
            let next_ground = if j + 1 < smoothed_points.len() {
                let (nx, _, nz) = smoothed_points[j + 1];
                ground_level_at((nx, nz))
            } else {
                ground_level_at((bx, bz))
            };
            let current_ground = ground_level_at((bx, bz));

            // Fill any vertical gap under the rail when the ground rises step-wise.
            if prev_ground < current_ground {
                for fill_y in prev_ground..current_ground {
                    editor.set_block_absolute(GRAVEL, bx, fill_y, bz, None, None);
                }
            }

            editor.set_block_absolute(GRAVEL, bx, current_ground, bz, None, None);

            let prev_xz = if j > 0 {
                let (px, _, pz) = smoothed_points[j - 1];
                Some((px, pz))
            } else {
                None
            };
            let next_xz = if j + 1 < smoothed_points.len() {
                let (nx, _, nz) = smoothed_points[j + 1];
                Some((nx, nz))
            } else {
                None
            };

            let rail_block = determine_rail_with_slope(
                (bx, bz),
                prev_xz,
                next_xz,
                prev_ground,
                current_ground,
                next_ground,
            );

            editor.set_block_with_properties_absolute(
                at_grade_rail_props(rail_block),
                bx,
                current_ground + 1,
                bz,
                None,
                None,
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fastnbt::Value;

    /// Collect the block names referenced by a chunk's section palettes.
    fn chunk_block_names(chunk_nbt: &Value) -> Vec<String> {
        let mut names = Vec::new();
        let Value::Compound(root) = chunk_nbt else {
            return names;
        };
        let Some(Value::List(sections)) = root.get("sections") else {
            return names;
        };
        for section in sections {
            let Value::Compound(section) = section else {
                continue;
            };
            let Some(block_states) = section.get("block_states") else {
                continue;
            };
            let Value::Compound(block_states) = block_states else {
                continue;
            };
            let Some(palette) = block_states.get("palette") else {
                continue;
            };
            let Value::List(palette) = palette else {
                continue;
            };
            for item in palette {
                let Value::Compound(item) = item else {
                    continue;
                };
                if let Some(Value::String(name)) = item.get("Name") {
                    names.push(name.clone());
                }
            }
        }
        names
    }

    fn read_chunk(world: &Path, cx: usize, cz: usize) -> Vec<String> {
        let path = world.join("region").join("r.0.0.mca");
        let file = std::fs::File::open(&path).unwrap();
        let mut region = fastanvil::Region::from_stream(file).unwrap();
        let bytes = region.read_chunk(cx, cz).unwrap().unwrap();
        let nbt: Value = fastnbt::from_bytes(&bytes).unwrap();
        chunk_block_names(&nbt)
    }

    /// Read a sectioned chunk file and return its full NBT.
    fn read_chunk_nbt(world: &Path, cx: usize, cz: usize) -> Value {
        let path = world.join("region").join("r.0.0.mca");
        let file = std::fs::File::open(&path).unwrap();
        let mut region = fastanvil::Region::from_stream(file).unwrap();
        let bytes = region.read_chunk(cx, cz).unwrap().unwrap();
        fastnbt::from_bytes(&bytes).unwrap()
    }

    /// Collect the `shape` state property of every `create:track` palette entry
    /// in a chunk, in palette order.
    fn chunk_create_track_shapes(chunk_nbt: &Value) -> Vec<String> {
        let mut shapes = Vec::new();
        let Value::Compound(root) = chunk_nbt else {
            return shapes;
        };
        let Some(Value::List(sections)) = root.get("sections") else {
            return shapes;
        };
        for section in sections {
            let Value::Compound(section) = section else {
                continue;
            };
            let Some(block_states) = section.get("block_states") else {
                continue;
            };
            let Value::Compound(block_states) = block_states else {
                continue;
            };
            let Some(palette) = block_states.get("palette") else {
                continue;
            };
            let Value::List(palette) = palette else {
                continue;
            };
            for item in palette {
                let Value::Compound(item) = item else {
                    continue;
                };
                let Some(Value::String(name)) = item.get("Name") else {
                    continue;
                };
                if name != "create:track" {
                    continue;
                }
                let shape = match item.get("Properties") {
                    Some(Value::Compound(props)) => match props.get("shape") {
                        Some(Value::String(s)) => Some(s.clone()),
                        _ => None,
                    },
                    _ => None,
                };
                if let Some(s) = shape {
                    shapes.push(s);
                }
            }
        }
        shapes
    }

    #[test]
    fn generates_a_loadable_world() {
        let dir = tempfile::tempdir().unwrap();
        let world_dir = dir.path().join("create-track-test");
        let world = generate(&world_dir).unwrap();

        assert!(world.join("level.dat").exists());
        assert!(world.join("icon.png").exists());
        assert!(world.join("region").join("r.0.0.mca").exists());

        // Flat-world mode writes the whole region: 32×32 chunks, the area
        // itself being the 8×8 corner at (0..8, 0..8).
        let path = world.join("region").join("r.0.0.mca");
        let file = std::fs::File::open(&path).unwrap();
        let mut region = fastanvil::Region::from_stream(file).unwrap();
        let count = (0..32)
            .flat_map(|x| (0..32).map(move |z| (x, z)))
            .filter(|&(x, z)| region.read_chunk(x, z).unwrap().is_some())
            .count();
        assert_eq!(count, 1024, "expected the whole region to be written");

        // Ground everywhere; no rails in a track-free chunk.
        let names = read_chunk(&world, 0, 0);
        assert!(names.contains(&"minecraft:grass_block".to_string()));
        assert!(!names.iter().any(|n| n == "minecraft:rail"));

        // Track A (x=16, z=8..40) is a straight N-S at-grade rail: stage 2
        // makes it a Create track (`shape=zo`), gravel bed kept.
        let names = read_chunk(&world, 1, 0);
        assert!(names.contains(&"create:track".to_string()));
        assert!(!names.iter().any(|n| n == "minecraft:rail"));
        assert!(names.contains(&"minecraft:gravel".to_string()));

        // Track C corner (48,24) is in chunk (3,1). Stage 2 has no corner:
        // the cell keeps the first segment's straight shape (`zo`) and the
        // E-W arm starts at (49,24) — both arms are complete straight Create
        // tracks, the corner joint stays broken until stage 3.
        let names = read_chunk(&world, 3, 1);
        assert!(names.contains(&"create:track".to_string()));
        assert!(!names.iter().any(|n| n == "minecraft:rail"));

        // Track D (96,48..64) crosses the plateau edge: the single ascending
        // cell stays a vanilla rail (stage 4), the flat cells are Create
        // tracks; chunk (6,3) has stone, gravel and both.
        let names = read_chunk(&world, 6, 3);
        assert!(names.contains(&"minecraft:rail".to_string()));
        assert!(names.contains(&"create:track".to_string()));
        assert!(names.contains(&"minecraft:stone".to_string()));
        assert!(names.contains(&"minecraft:gravel".to_string()));
    }

    #[test]
    fn create_track_block_round_trips_with_shape() {
        let dir = tempfile::tempdir().unwrap();
        let world_dir = dir.path().join("create-track-test");
        let world = generate(&world_dir).unwrap();

        // The standalone `create:track` blocks live in chunk (0,0) at
        // (10,10) and (12,12). They must appear in the palette under the
        // `create` namespace, one entry per distinct `shape`, with the shape
        // carried as a state property (turn/waterlogged omitted = default
        // false).
        let names = read_chunk(&world, 0, 0);
        assert!(names.contains(&"create:track".to_string()));

        let nbt = read_chunk_nbt(&world, 0, 0);
        let shapes = chunk_create_track_shapes(&nbt);
        assert!(shapes.contains(&"xo".to_string()));
        assert!(shapes.contains(&"zo".to_string()));
    }

    #[test]
    fn create_track_line_is_present() {
        let dir = tempfile::tempdir().unwrap();
        let world_dir = dir.path().join("create-track-test");
        let world = generate(&world_dir).unwrap();

        // The straight Create track line at x=30, z=20..44 (`shape=zo`) spans
        // chunks (1,1) and (1,2). It must appear in the palette as
        // `create:track` so a carriage can ride it end-to-end in-game.
        let names = read_chunk(&world, 1, 1);
        assert!(names.contains(&"create:track".to_string()));
        let names = read_chunk(&world, 1, 2);
        assert!(names.contains(&"create:track".to_string()));
    }
}
