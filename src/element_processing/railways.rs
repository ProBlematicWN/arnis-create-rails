use crate::block_definitions::*;
use crate::bresenham::bresenham_line;
use crate::coordinate_system::cartesian::XZBBox;
use crate::element_processing::bridge_styles::{
    decorate_bridge_above_deck, place_arch_below_deck, place_pier_bent,
    resolve_bridge_style_with_outline, BridgeOutlineIndex, BridgePathSample, BridgeStyle,
};
use crate::element_processing::bridges::{BridgeStructureMap, BridgeSurfaceMap, RailDeck};
use crate::floodfill_cache::CoordinateBitmap;
use crate::osm_parser::{ProcessedElement, ProcessedWay};
use crate::world_editor::{create_curve_cells, curve_start, WorldEditor};
use std::collections::{HashMap, HashSet};
use std::sync::Mutex;

const CATENARY_MAST_INTERVAL: usize = 16;
const CATENARY_MAST_OFFSET: i32 = 3;
const CATENARY_WIRE_HEIGHT: i32 = 6;

/// Vertical offset in blocks from the terrain surface to the tunnel ceiling.
const RAIL_TUNNEL_DEPTH: i32 = 3;

pub(crate) const RAIL_BRIDGE_FLAT_CLEARANCE: i32 = 4;
pub(crate) const RAIL_BRIDGE_DIP_THRESHOLD: i32 = 4;
const RAIL_BRIDGE_RAMP_MIN: usize = 8;
const RAIL_BRIDGE_RAMP_MAX: usize = 30;
const RAIL_BRIDGE_RAMP_FRACTION: f32 = 0.25;

pub type RailBridgeInternalEndpoints = HashSet<(i32, i32)>;

const RAIL_TRACK_TYPES: &[&str] = &[
    "rail",
    "light_rail",
    "subway",
    "tram",
    "narrow_gauge",
    "monorail",
    "funicular",
    "miniature",
    "preserved",
    "disused",
];

/// Half-width of the outer stone shell (total footprint = 2 * WALL_RADIUS + 1 = 5).
const WALL_RADIUS: i32 = 2;

/// Half-width of the interior air space (total air width = 2 * AIR_RADIUS + 1 = 3).
const AIR_RADIUS: i32 = 1;

/// Number of interior Y-levels (rail + 3 air = 4 blocks for minecart clearance).
const INTERIOR_HEIGHT: i32 = 4;

/// Interval in centerline points between ceiling lights.
const LIGHT_INTERVAL: usize = 8;

/// Deterministic spatial hash for tunnel wall/ceiling block variety.
/// Returns CRACKED_STONE_BRICKS (~15%), MOSSY_STONE_BRICKS (~3%),
/// or STONE_BRICKS (~82%).
fn rail_tunnel_shell_block(x: i32, y: i32, z: i32) -> Block {
    let h = (x as u32)
        .wrapping_mul(73856093)
        .wrapping_add((y as u32).wrapping_mul(19349663))
        .wrapping_add((z as u32).wrapping_mul(83492791));
    let v = h % 100;
    if v < 15 {
        CRACKED_STONE_BRICKS
    } else if v < 18 {
        MOSSY_STONE_BRICKS
    } else {
        STONE_BRICKS
    }
}

#[allow(clippy::too_many_arguments)]
pub fn generate_railways(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    rail_tunnel_points: &mut Vec<(i32, i32)>,
    rail_bridge_internal_endpoints: &RailBridgeInternalEndpoints,
    bridge_outlines: &BridgeOutlineIndex,
    bridge_structures: &BridgeStructureMap,
    bridge_surface: &BridgeSurfaceMap,
    road_mask: &CoordinateBitmap,
    building_footprints: &CoordinateBitmap,
    rail_mask: &CoordinateBitmap,
) {
    let Some(railway_type) = element.tags.get("railway") else {
        return;
    };

    if wants_rail_tunnel(element) {
        if renders_as_rail_tunnel(element) {
            generate_rail_tunnel_shell(editor, element, rail_tunnel_points);
        }
        return;
    }

    if [
        "proposed",
        "abandoned",
        "construction",
        "razed",
        "turntable",
    ]
    .contains(&railway_type.as_str())
    {
        return;
    }

    if is_rail_bridge(element) {
        generate_rail_bridge(
            editor,
            element,
            rail_bridge_internal_endpoints,
            bridge_outlines,
            bridge_structures,
            bridge_surface,
        );
    } else {
        generate_at_grade_rail(editor, element);
        if catenary_wanted(element) {
            let points = build_smoothed_centerline(element);
            if points.len() >= 2 {
                generate_catenary(editor, &points, road_mask, building_footprints, rail_mask);
            }
        }
    }
}

fn renders_as_rail_tunnel(way: &ProcessedWay) -> bool {
    let Some(railway_type) = way.tags.get("railway").map(String::as_str) else {
        return false;
    };
    if way.nodes.len() < 2
        || !RAIL_TRACK_TYPES.contains(&railway_type)
        || way.tags.get("area").map(String::as_str) == Some("yes")
    {
        return false;
    }
    wants_rail_tunnel(way)
}

/// Subways run underground unless mapped on a bridge or with tunnel=no.
fn wants_rail_tunnel(way: &ProcessedWay) -> bool {
    let tunnel = way.tags.get("tunnel").map(String::as_str);
    if tunnel == Some("yes") {
        return true;
    }
    let is_subway = way.tags.get("railway").map(String::as_str) == Some("subway")
        || way.tags.get("subway").map(String::as_str) == Some("yes");
    is_subway && tunnel != Some("no") && !is_rail_bridge(way)
}

fn is_rail_bridge(way: &ProcessedWay) -> bool {
    if way.tags.get("indoor").map(|v| v.as_str()) == Some("yes") {
        return false;
    }
    way.tags
        .get("bridge")
        .map(|v| v.as_str())
        .is_some_and(|v| v != "no")
}

/// Track laid on the ground.
pub(crate) fn is_at_grade_track(way: &ProcessedWay) -> bool {
    let Some(railway_type) = way.tags.get("railway").map(String::as_str) else {
        return false;
    };
    way.nodes.len() >= 2
        && RAIL_TRACK_TYPES.contains(&railway_type)
        && way.tags.get("area").map(String::as_str) != Some("yes")
        && !is_rail_bridge(way)
        && !wants_rail_tunnel(way)
}

// Mirrors generate_railways' dispatch so the internal-endpoint set only counts rendered bridges.
pub(crate) fn renders_as_rail_bridge(way: &ProcessedWay) -> bool {
    let Some(railway_type) = way.tags.get("railway") else {
        return false;
    };
    if way.nodes.len() < 2 || !is_rail_bridge(way) || wants_rail_tunnel(way) {
        return false;
    }
    if [
        "proposed",
        "abandoned",
        "construction",
        "razed",
        "turntable",
    ]
    .contains(&railway_type.as_str())
    {
        return false;
    }
    true
}

/// Endpoints shared by 2+ rendered rail-bridge ways — used to suppress per-way ramps mid-bridge.
pub fn collect_rail_bridge_internal_endpoints(
    elements: &[ProcessedElement],
) -> RailBridgeInternalEndpoints {
    let mut counts: HashMap<(i32, i32), u32> = HashMap::new();
    for elem in elements {
        let ProcessedElement::Way(w) = elem else {
            continue;
        };
        if !renders_as_rail_bridge(w) {
            continue;
        }
        let s = &w.nodes[0];
        let e = &w.nodes[w.nodes.len() - 1];
        *counts.entry((s.x, s.z)).or_default() += 1;
        if (e.x, e.z) != (s.x, s.z) {
            *counts.entry((e.x, e.z)).or_default() += 1;
        }
    }
    counts
        .into_iter()
        .filter_map(|(k, c)| (c > 1).then_some(k))
        .collect()
}

fn generate_at_grade_rail(editor: &mut WorldEditor, element: &ProcessedWay) {
    // Cumulative cell index across segments so sleeper spacing stays consistent at OSM-node joins.
    // Cumulative cell index along the whole way so sleeper spacing stays
    // consistent across what used to be separate OSM segments.
    let mut tds: usize = 0;

    // One polyline across all segments: a corner where two OSM segments meet
    // (or a slope that starts at a node) has to stay visible to the curve
    // planner, and per-segment context would hide exactly that.
    let mut all_points: Vec<(i32, i32, i32)> = vec![];
    for i in 1..element.nodes.len() {
        let prev_node = element.nodes[i - 1].xz();
        let cur_node = element.nodes[i].xz();

        let points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
        let smoothed_points = smooth_diagonal_rails(&points);
        let skip_first = if all_points.is_empty() { 0 } else { 1 };
        all_points.extend(&smoothed_points[skip_first..smoothed_points.len()]);
    }

    // Rails are laid from the OSM node geometry (long legs, anchors only at
    // leg ends), never from this rasterization: a rasterized diagonal would
    // become a staircase of xo/zo cells that Create cannot connect.
    plan_create_legs(editor, &way_nodes_cells(element), element.id);

    // The rasterized polyline below is only the track bed: gravel fill, sleepers.
    // The bed goes through the corridor guard like every other writer: the
    // presurvey band claims the rail column from one below rail level, so the
    // guard refuses the bed there and the corridor floor pass rests the rails
    // on ground instead. That is deliberate - the goal invariant bans ballast
    // from every corridor cell (CREATE_CORRIDOR_WIDTH x Y..CREATE_CORRIDOR_UP
    // around a track cell), and on a staircase the bed of a higher column
    // lands exactly in the corridor of the lower span beside it (measured on
    // osm-rail37: 253 gravel cells inside corridors, 23 directly above a
    // rail, after the bed was exempted from the guard; a plain at-grade bed is
    // outside its own rail's corridor, so the refusal only ever costs ballast,
    // never support - the floor pass guarantees a floor under every written
    // rail cell).
    for j in 0..all_points.len() {
        let (bx, _, bz) = all_points[j];

        let prev_ground = if j > 0 {
            let (px, _, pz) = all_points[j - 1];
            editor.get_ground_level(px, pz)
        } else {
            editor.get_ground_level(bx, bz)
        };
        let current_ground = editor.get_ground_level(bx, bz);

        // Fill any vertical gap under the rail when terrain rises step-wise.
        if prev_ground < current_ground {
            for fill_y in prev_ground..current_ground {
                editor.set_block_absolute(GRAVEL, bx, fill_y, bz, None, None);
            }
        }

        editor.set_block(GRAVEL, bx, 0, bz, None, None);

        if tds.is_multiple_of(4) {
            editor.set_block(OAK_LOG, bx, 0, bz, None, None);
        }
        tds += 1;
    }
}

// Overhead line only for real catenary: contact_line, or a best-guess yes on a
// main/branch line. Never for third rail.
fn catenary_wanted(way: &ProcessedWay) -> bool {
    if way.tags.get("railway").map(String::as_str) != Some("rail") {
        return false;
    }
    match way.tags.get("electrified").map(String::as_str) {
        Some("contact_line") => true,
        Some("yes") => matches!(
            way.tags.get("usage").map(String::as_str),
            Some("main") | Some("branch")
        ),
        _ => false,
    }
}

pub(crate) fn build_smoothed_centerline(way: &ProcessedWay) -> Vec<(i32, i32)> {
    let mut points: Vec<(i32, i32)> = Vec::new();
    for window in way.nodes.windows(2) {
        let bp = bresenham_line(window[0].x, 0, window[0].z, window[1].x, 0, window[1].z);
        let smoothed = smooth_diagonal_rails(&bp);
        for (bx, _, bz) in smoothed.iter() {
            if points.last() != Some(&(*bx, *bz)) {
                points.push((*bx, *bz));
            }
        }
    }
    points
}

fn travel_dir(points: &[(i32, i32)], i: usize) -> (i32, i32) {
    let (cx, cz) = points[i];
    if i + 1 < points.len() {
        let (nx, nz) = points[i + 1];
        ((nx - cx).signum(), (nz - cz).signum())
    } else if i > 0 {
        let (px, pz) = points[i - 1];
        ((cx - px).signum(), (cz - pz).signum())
    } else {
        (1, 0)
    }
}

fn rail_within(mask: &CoordinateBitmap, cx: i32, cz: i32, dx: i32, dz: i32, max_dist: i32) -> bool {
    (1..=max_dist).any(|d| mask.contains(cx + dx * d, cz + dz * d))
}

// Rejects a side blocked by a road, building, or a parallel track within the mast footprint.
#[allow(clippy::too_many_arguments)]
fn mast_base_clear(
    editor: &WorldEditor,
    cx: i32,
    cz: i32,
    dx: i32,
    dz: i32,
    road_mask: &CoordinateBitmap,
    building_footprints: &CoordinateBitmap,
    rail_mask: &CoordinateBitmap,
) -> bool {
    let (x, z) = (
        cx + dx * CATENARY_MAST_OFFSET,
        cz + dz * CATENARY_MAST_OFFSET,
    );
    if road_mask.contains(x, z) || building_footprints.contains(x, z) {
        return false;
    }
    if rail_within(rail_mask, cx, cz, dx, dz, CATENARY_MAST_OFFSET + 2) {
        return false;
    }
    // The mast stands on the ground and reaches up to the wire: any corridor
    // cell in that column means the mast would grow inside the clearance a
    // train needs beside and above the rails.
    let ground = editor.get_ground_level(x, z);
    let wire = editor.get_ground_level(cx, cz) + CATENARY_WIRE_HEIGHT;
    (ground..=wire).all(|y| !editor.create_corridor_occupied(x, y, z))
}

#[allow(clippy::too_many_arguments)]
fn place_catenary_mast(
    editor: &mut WorldEditor,
    cx: i32,
    cz: i32,
    px: i32,
    pz: i32,
    wire_abs: i32,
    road_mask: &CoordinateBitmap,
    building_footprints: &CoordinateBitmap,
    rail_mask: &CoordinateBitmap,
) {
    // one consistent side, flipped if that side is blocked
    let side = if mast_base_clear(
        editor,
        cx,
        cz,
        px,
        pz,
        road_mask,
        building_footprints,
        rail_mask,
    ) {
        (px, pz)
    } else if mast_base_clear(
        editor,
        cx,
        cz,
        -px,
        -pz,
        road_mask,
        building_footprints,
        rail_mask,
    ) {
        (-px, -pz)
    } else {
        return;
    };
    let (sx, sz) = side;
    let (mx, mz) = (
        cx + sx * CATENARY_MAST_OFFSET,
        cz + sz * CATENARY_MAST_OFFSET,
    );

    // The catenary rides ABOVE the corridor the presurvey band claims (the
    // wire is five cells over the rails), but the band is laid out from the
    // presurvey ground level while the wire follows the road-overridden level
    // of its own moment, so the two can cross on a cut. The catenary is the
    // railway's own structure over the train envelope: its cells are exempt,
    // never refused by the guard and never dug out by the clearing.
    let report = editor.create_rails_report();
    let mast_ground = editor.get_ground_level(mx, mz);
    report.note_corridor_exempt((mx, mast_ground, mz));
    editor.set_block_absolute(GRAY_CONCRETE, mx, mast_ground, mz, None, None);
    for y in (mast_ground + 1)..=wire_abs {
        report.note_corridor_exempt((mx, y, mz));
        editor.set_block_absolute(COBBLESTONE_WALL, mx, y, mz, None, None);
    }

    let arm = if sx != 0 { CHAIN_X } else { CHAIN_Z };
    for d in 1..CATENARY_MAST_OFFSET {
        if report.corridor_contains((cx + sx * d, wire_abs, cz + sz * d)) {
            continue;
        }
        report.note_corridor_exempt((cx + sx * d, wire_abs, cz + sz * d));
        editor.set_block_absolute(arm, cx + sx * d, wire_abs, cz + sz * d, None, None);
    }
}

// Contact wire over the centreline plus cantilever masts at intervals.
fn generate_catenary(
    editor: &mut WorldEditor,
    points: &[(i32, i32)],
    road_mask: &CoordinateBitmap,
    building_footprints: &CoordinateBitmap,
    rail_mask: &CoordinateBitmap,
) {
    let n = points.len();
    if n < 2 {
        return;
    }

    for i in 0..n {
        let (cx, cz) = points[i];
        let (dx, dz) = travel_dir(points, i);
        let (px, pz) = (-dz, dx);
        let wire_abs = editor.get_ground_level(cx, cz) + CATENARY_WIRE_HEIGHT;

        let wire = if dx != 0 { CHAIN_X } else { CHAIN_Z };
        // Same exemption rule as the masts: the wire is the railway's own
        // structure above the train envelope (see place_catenary_mast) - but a
        // wire cell that is already inside a declared corridor is SKIPPED, not
        // exempted into it (measured on osm-rail38: 2 chain cells ended up
        // inside a corridor where the wire height and the flattened rail level
        // crossed; a gap in the catenary line costs less than a block the
        // train drives through).
        if !editor.create_corridor_occupied(cx, wire_abs, cz) {
            editor
                .create_rails_report()
                .note_corridor_exempt((cx, wire_abs, cz));
            editor.set_block_absolute(wire, cx, wire_abs, cz, None, None);
        }

        if i.is_multiple_of(CATENARY_MAST_INTERVAL) {
            place_catenary_mast(
                editor,
                cx,
                cz,
                px,
                pz,
                wire_abs,
                road_mask,
                building_footprints,
                rail_mask,
            );
        }
    }
}

// At-grade rail centerlines, used only to keep catenary masts off other tracks.
pub fn collect_at_grade_rail_mask(
    elements: &[ProcessedElement],
    xzbbox: &XZBBox,
) -> CoordinateBitmap {
    // No catenary anywhere means nothing reads the mask, so skip the allocation.
    if !elements
        .iter()
        .any(|e| matches!(e, ProcessedElement::Way(w) if catenary_wanted(w)))
    {
        return CoordinateBitmap::new_empty();
    }
    let mut bitmap = CoordinateBitmap::new(xzbbox);
    for element in elements {
        let ProcessedElement::Way(way) = element else {
            continue;
        };
        if way.nodes.len() < 2 {
            continue;
        }
        if way.tags.get("railway").map(String::as_str) != Some("rail") {
            continue;
        }
        if way.tags.get("subway").map(|v| v == "yes").unwrap_or(false) {
            continue;
        }
        if way.tags.get("tunnel").map(|v| v == "yes").unwrap_or(false) {
            continue;
        }
        if is_rail_bridge(way) {
            continue;
        }
        for (bx, bz) in build_smoothed_centerline(way) {
            bitmap.set(bx, bz);
        }
    }
    bitmap
}

/// Adds every rendered rail-tunnel shell coordinate to the shared tunnel footprint.
pub fn add_tunnel_footprint(
    elements: &[ProcessedElement],
    xzbbox: &XZBBox,
    footprint: &mut CoordinateBitmap,
) {
    if !elements
        .iter()
        .any(|e| matches!(e, ProcessedElement::Way(w) if renders_as_rail_tunnel(w)))
    {
        return;
    }
    if footprint.is_empty() {
        *footprint = CoordinateBitmap::new(xzbbox);
    }
    for element in elements {
        let ProcessedElement::Way(way) = element else {
            continue;
        };
        if !renders_as_rail_tunnel(way) {
            continue;
        }
        for (bx, bz) in build_smoothed_centerline(way) {
            for dx in -WALL_RADIUS..=WALL_RADIUS {
                for dz in -WALL_RADIUS..=WALL_RADIUS {
                    footprint.set(bx + dx, bz + dz);
                }
            }
        }
    }
}

fn generate_rail_bridge(
    editor: &mut WorldEditor,
    way: &ProcessedWay,
    internal_endpoints: &RailBridgeInternalEndpoints,
    bridge_outlines: &BridgeOutlineIndex,
    bridge_structures: &BridgeStructureMap,
    bridge_surface: &BridgeSurfaceMap,
) {
    if way.nodes.len() < 2 {
        return;
    }

    let style = resolve_bridge_style_with_outline(way, bridge_outlines);

    let mut all_points: Vec<(i32, i32)> = Vec::new();
    for window in way.nodes.windows(2) {
        let bp = bresenham_line(window[0].x, 0, window[0].z, window[1].x, 0, window[1].z);
        let smoothed = smooth_diagonal_rails(&bp);
        for (bx, _, bz) in smoothed.iter() {
            if all_points.last() != Some(&(*bx, *bz)) {
                all_points.push((*bx, *bz));
            }
        }
    }
    if all_points.is_empty() {
        return;
    }

    // Track on a road deck; the road bridge brings the supports.
    if let Some(RailDeck::Carried(ys)) = bridge_structures.rail_deck(way.id) {
        let y_at = |i: usize| ys.get(i).or(ys.last()).copied().unwrap_or_default();
        for (i, &(bx, bz)) in all_points.iter().enumerate() {
            let y = y_at(i);
            let prev = i.checked_sub(1).map(|p| (all_points[p], y_at(p)));
            let next = all_points.get(i + 1).map(|&n| (n, y_at(i + 1)));
            let rail_block = determine_rail_with_slope(
                (bx, bz),
                prev.map(|p| p.0),
                next.map(|n| n.0),
                prev.map_or(y, |p| p.1),
                y,
                next.map_or(y, |n| n.1),
            );
            editor.set_block_absolute(GRAVEL, bx, y, bz, None, None);
            editor.set_block_with_properties_absolute(
                at_grade_rail_props(rail_block),
                bx,
                y + 1,
                bz,
                None,
                None,
            );
        }
        return;
    }

    // Sample terrain at every centerline cell, not just OSM nodes, so a hill mid-span still clears the deck.
    let mut terrain_ys: Vec<i32> = Vec::with_capacity(all_points.len());
    let mut max_y = i32::MIN;
    let mut min_y = i32::MAX;
    for &(bx, bz) in &all_points {
        let y = editor.get_ground_level(bx, bz);
        terrain_ys.push(y);
        max_y = max_y.max(y);
        min_y = min_y.min(y);
    }
    let dip = max_y - min_y;
    // Shared viaduct level; the fallback covers ways the prescan did not see.
    let own_level = match bridge_structures.rail_deck(way.id) {
        Some(RailDeck::Level(y)) => Some(*y),
        _ => None,
    };
    let deck_y = own_level.unwrap_or_else(|| {
        // Arch needs vertical room for its curve on flat terrain.
        let flat_clearance = if style == BridgeStyle::Arch {
            RAIL_BRIDGE_FLAT_CLEARANCE.max(8)
        } else {
            RAIL_BRIDGE_FLAT_CLEARANCE
        };
        // Flat span: lift by clearance so the structure is visible. Canyon span: deck at terrain_max.
        if dip < RAIL_BRIDGE_DIP_THRESHOLD {
            max_y + flat_clearance
        } else {
            max_y
        }
    });

    let total = all_points.len();
    let last_idx = total - 1;

    let start_xz = all_points[0];
    let end_xz = all_points[last_idx];
    let start_ground = terrain_ys[0];
    let end_ground = terrain_ys[last_idx];
    // Shared endpoints stay at deck Y to avoid a mid-bridge dip between adjacent segments.
    let start_internal = internal_endpoints.contains(&start_xz);
    let end_internal = internal_endpoints.contains(&end_xz);

    // Ramp length sized so per-cell linear delta stays <= 1 (rail step limit). No horizontal cap:
    // when needed > total/2 the start/end ramps overlap and min(start, end) below produces a pyramid.
    let mut needed = 0usize;
    if !start_internal {
        needed = needed.max((deck_y - start_ground).max(0) as usize);
    }
    if !end_internal {
        needed = needed.max((deck_y - end_ground).max(0) as usize);
    }
    let needed = needed + 1;

    let raw_ramp = (total as f32 * RAIL_BRIDGE_RAMP_FRACTION) as usize;
    let ramp_length = raw_ramp
        .clamp(RAIL_BRIDGE_RAMP_MIN, RAIL_BRIDGE_RAMP_MAX)
        .max(needed);
    let denom = ramp_length.saturating_sub(1).max(1) as f32;

    let bridge_ys: Vec<i32> = (0..total)
        .map(|tds| {
            // Two rising ramps from each endpoint, capped at deck_y. min combines them: trapezoid
            // for long bridges (both ramps reach deck_y mid-span), pyramid for short ones.
            let start_ramp_y = if start_internal {
                deck_y
            } else {
                let t = (tds as f32 / denom).min(1.0);
                let span = (deck_y - start_ground) as f32;
                (start_ground as f32 + span * t).round() as i32
            };
            let end_ramp_y = if end_internal {
                deck_y
            } else {
                let dist_from_end = last_idx.saturating_sub(tds);
                let t = (dist_from_end as f32 / denom).min(1.0);
                let span = (deck_y - end_ground) as f32;
                (end_ground as f32 + span * t).round() as i32
            };
            let linear_y = start_ramp_y.min(end_ramp_y);
            // Clamp to local terrain so a mid-ramp hillside doesn't bury the deck/foundation.
            linear_y.max(terrain_ys[tds])
        })
        .collect();

    let foundation_block = style.foundation_block();
    let mut bridge_path: Vec<BridgePathSample> = Vec::with_capacity(total);
    let start_is_boundary = !start_internal;
    let end_is_boundary = !end_internal;
    // Suspension and cable-stayed spans hang from their pylons, not from piers.
    let cables_carry_deck = style.cables_carry_deck(total, start_is_boundary, end_is_boundary);

    for (i, &(bx, bz)) in all_points.iter().enumerate() {
        let y = bridge_ys[i];
        let prev_xz = if i > 0 { Some(all_points[i - 1]) } else { None };
        let next_xz = if i + 1 < total {
            Some(all_points[i + 1])
        } else {
            None
        };
        let prev_y = if i > 0 { bridge_ys[i - 1] } else { y };
        let next_y = if i + 1 < total { bridge_ys[i + 1] } else { y };
        let rail_block = determine_rail_with_slope((bx, bz), prev_xz, next_xz, prev_y, y, next_y);

        editor.set_block_absolute(foundation_block, bx, y - 1, bz, None, None);
        let bed_block = if i % 4 == 0 { OAK_LOG } else { GRAVEL };
        editor.set_block_absolute(bed_block, bx, y, bz, None, None);
        editor.set_block_with_properties_absolute(
            at_grade_rail_props(rail_block),
            bx,
            y + 1,
            bz,
            None,
            None,
        );

        // Smooth perpendicular from neighbouring centerline points.
        let p_prev = prev_xz.unwrap_or((bx, bz));
        let p_next = next_xz.unwrap_or((bx, bz));
        let dxp = (p_next.0 - p_prev.0) as f32;
        let dzp = (p_next.1 - p_prev.1) as f32;
        let mag = (dxp * dxp + dzp * dzp).sqrt().max(1e-6);
        let perp = (-dzp / mag, dxp / mag);
        bridge_path.push((bx, y, bz, perp));

        let blocked = bridge_surface.support_blocked(bx, bz, y);
        if style == BridgeStyle::Arch && !blocked {
            place_arch_below_deck(editor, bx, y, bz, terrain_ys[i], i, total, true, true);
        }
        let interval = style.pier_interval(0);
        if !cables_carry_deck && interval > 0 && i % interval == interval / 2 {
            place_pier_bent(editor, bridge_surface, style, bx, bz, y, perp, 0);
        }
    }

    decorate_bridge_above_deck(
        editor,
        bridge_surface,
        style,
        &bridge_path,
        0,
        start_is_boundary,
        end_is_boundary,
        None,
    );
}

/// Stage 2: straight at-grade rails become Create tracks — one block id
/// (`CREATE_TRACK`), the axis carried by the `shape` state property
/// (`turn=false`, no block entity). Slopes (stage 4) and curves (stage 3)
/// stay vanilla rails until converted.
/// `pub(crate)` so the track test world (`src/track_test_world.rs`) places
/// rails exactly as the pipeline does.
/// Rail cell semantics for the Create pipeline, replacing the vanilla rail
/// block constants: a straight along one axis, a quarter turn joining two
/// axes, or an ascending straight climbing toward one neighbour. Everything
/// the placement code needs is here, so no `minecraft:rail` block ever gets
/// written again.
#[derive(Clone, Copy)]
pub(crate) enum RailKind {
    StraightX,
    StraightZ,
    TurnNorthWest,
    TurnNorthEast,
    TurnSouthWest,
    TurnSouthEast,
    AscendEast,
    AscendWest,
    AscendNorth,
    AscendSouth,
}

/// Tolerance in cells: OSM nodes closer than this to the chord between real
/// corners are dropped so a leg stays one long span.
const CREATE_LEG_TOLERANCE: f64 = 1.5;
// Longest Create span we allow, in cells. Create's own `determineHandles` puts the
// Bezier handles at r*(4/3)*tan(theta/4), i.e. the radius comes from the CORNER,
// not from the whole span: a 500-cell smoothed leg would get a 200-cell handle and
// overshoot everything. So long legs get intermediate anchors every CREATE_MAX_SPAN
// cells, and a run of non-snapable legs is merged only up to CREATE_MAX_RUN_LEGS.
const CREATE_MAX_SPAN: usize = 14;
const CREATE_MAX_RUN_LEGS: usize = 3;

/// World cells of a way's OSM nodes, in travel order.
pub(crate) fn way_nodes_cells(way: &ProcessedWay) -> Vec<(i32, i32)> {
    let mut cells: Vec<(i32, i32)> = vec![];
    for n in &way.nodes {
        let c = (n.x, n.z);
        if cells.is_empty() || cells[cells.len() - 1] != c {
            cells.push(c);
        }
    }
    cells
}

/// Node of `nodes[from..to]` farthest from the chord (`ax`,`az`)→(+dx,+dz),
/// as (distance, index). Falls back to distance from the chord start when the
/// chord has no length.
fn far_node(
    nodes: &[(i32, i32)],
    from: usize,
    to: usize,
    ax: i32,
    az: i32,
    dx: f64,
    dz: f64,
) -> (f64, usize) {
    let len = dx.hypot(dz);
    let mut best: (f64, usize) = (0.0, from.min(to));
    for (k, node) in nodes.iter().enumerate().take(to).skip(from) {
        let d = if len == 0.0 {
            ((node.0 - ax) as f64).hypot((node.1 - az) as f64)
        } else {
            ((node.0 - ax) as f64 * dz - (node.1 - az) as f64 * dx).abs() / len
        };
        if best.0 < d {
            best = (d, k);
        }
    }
    best
}

/// Drop OSM nodes that only subdivide a straight leg or wiggle within
/// `CREATE_LEG_TOLERANCE` cells of the chord, so a leg spans many cells and
/// anchors only sit at real corners. Without this, every OSM node of a dense
/// way becomes a micro-turn.
pub(crate) fn simplify_leg_nodes(nodes: &[(i32, i32)]) -> Vec<(i32, i32)> {
    if nodes.len() < 3 {
        return nodes.to_vec();
    }
    let raw = douglas_peucker(nodes, 0, nodes.len() - 1, CREATE_LEG_TOLERANCE);
    let mut keep: Vec<(i32, i32)> = vec![];
    for p in raw {
        if keep.is_empty() || keep[keep.len() - 1] != p {
            keep.push(p);
        }
    }
    keep
}

fn douglas_peucker(nodes: &[(i32, i32)], lo: usize, hi: usize, tol: f64) -> Vec<(i32, i32)> {
    if hi <= lo + 1 {
        return vec![nodes[lo], nodes[hi]];
    }
    let (ax, az) = nodes[lo];
    let (bx, bz) = nodes[hi];
    let dx = (bx - ax) as f64;
    let dz = (bz - az) as f64;
    let (worst, worst_at) = far_node(nodes, lo + 1, hi, ax, az, dx, dz);
    if worst_at < hi && worst > tol {
        let mut left = douglas_peucker(nodes, lo, worst_at, tol);
        let right = douglas_peucker(nodes, worst_at, hi, tol);
        for p in right {
            left.push(p);
        }
        return left;
    }
    vec![nodes[lo], nodes[hi]]
}

/// `shape` for a straight Create track running along a canonical direction.
fn straight_shape(unit: (i32, i32)) -> &'static str {
    if unit.0 != 0 && unit.1 != 0 {
        if unit.0 * unit.1 > 0 {
            "pd"
        } else {
            "nd"
        }
    } else if unit.0 != 0 {
        "xo"
    } else {
        "zo"
    }
}

/// Outward axis of a curve end running along a canonical direction: Create
/// writes the axis pointing away from the spline, which is unoriented anyway.
fn curve_axes(unit: (i32, i32), outward: bool) -> (f64, f64, f64) {
    let s = if outward { -1.0 } else { 1.0 };
    (s * unit.0 as f64, 0.0, s * unit.1 as f64)
}

/// Y of a track cell that sits one block above the terrain surface.
fn anchor_y(editor: &WorldEditor, c: (i32, i32)) -> i32 {
    editor.get_ground_level(c.0, c.1) + 1
}

/// Cells of an exact axial or 45° leg, endpoints included.
fn leg_cells(from: (i32, i32), to: (i32, i32)) -> Vec<(i32, i32)> {
    let (dx, dz) = (to.0 - from.0, to.1 - from.1);
    let steps = dx.abs().max(dz.abs());
    if steps == 0 {
        return vec![from];
    }
    let fs = steps as f64;
    (0..steps + 1)
        .map(|k| {
            let t = k as f64 / fs;
            (
                from.0 + ((dx as f64) * t).round() as i32,
                from.1 + ((dz as f64) * t).round() as i32,
            )
        })
        .collect()
}

/// Cells that close the joint between one snapped leg and the next. `snap_dir`
/// rounds a leg onto a canonical axis, so a leg whose OSM chord sits a hair off
/// that axis ends BESIDE the node it was aimed at, not on it: a 200-cell chord
/// 3 degrees off `x` is straight enough for `xo` but its last cell is 10 rows
/// away from the next node, and the cells between belong to no leg at all. The
/// carriage then derails in the middle of a run - dumped `osm-rail20` left
/// `nd` at (207,-1569,-547) with the next diagonal starting at (208,-1570,-545)
/// and columns (208,-546),(209,-547) empty at every height. Bridge that offset
/// along the same staircase so the two legs share a cell; refuse above
/// `CREATE_LEG_JOIN_MAX`, where inventing cells would be a lie about the way.
pub(crate) fn bridge_leg_cells(prev: (i32, i32), head: (i32, i32)) -> Vec<(i32, i32)> {
    let (dx, dz) = (head.0 - prev.0, head.1 - prev.1);
    if dx.abs() <= 1 && dz.abs() <= 1 {
        return vec![];
    }
    if dx.abs() > CREATE_LEG_JOIN_MAX || dz.abs() > CREATE_LEG_JOIN_MAX {
        return vec![];
    }
    let all = leg_cells(prev, head);
    if all.len() <= 2 {
        return vec![];
    }
    all[1..all.len() - 1].to_vec()
}

#[derive(Clone)]
struct CreateLeg {
    cells: Vec<(i32, i32)>,
    unit: (i32, i32),
    straight: bool,
    vx: f64,
    vz: f64,
    osm_span: usize,
}

// ── Pre-write connection validator ──────────────────────────────────────────
// One candidate Create connection (an anchor pair plus the `Axes` each anchor
// gets) gets MEASURED before a single block or block entity of it gets written,
// so a connection Create cannot drive leaves a clean gap instead of an overshoot
// loop the carriage cannot leave. Every threshold below is UNVERIFIED against
// Create's own limits: they come from the geometry dumped out of `osm-rail19`
// (87 of its 93 anchor pairs carry two IDENTICAL `Axes` and a chord pointing
// exactly backwards along them, dot -1.00, i.e. the spline would have to turn
// 180 deg) and from what the hand-built reference worlds already drive. `tools/
// check_curves_be.py` prints the same gates so the numbers stay comparable.
const CREATE_TARGET_BEHIND_DOT: f64 = 0.0;
const CREATE_MAX_END_TURN_DEG: f64 = 90.0;
/// Rise over run of the steepest slope the reference worlds actually drive (1
/// in 5): this limit is PROVEN, the gate applying it is ours.
const CREATE_MAX_SLOPE: f64 = 0.4;
const CREATE_MAX_CHAIN_PER_CHORD: f64 = 1.4;
const CREATE_MAX_CHAIN_PER_LEG: f64 = 2.0;

/// One candidate Create connection, exactly the arguments `emit_create_curve`
/// would receive, plus how many cells of the OSM polyline it replaces.
#[derive(Clone)]
struct CreateConnection {
    a: (i32, i32, i32),
    b: (i32, i32, i32),
    shape_a: &'static str,
    shape_b: &'static str,
    axes_a: (f64, f64, f64),
    axes_b: (f64, f64, f64),
    osm_cells: usize,
}

/// A connection the validator refused, as the reason string the checker prints
/// beside the coordinates in `track_rejections.json`.
enum CreateReject {
    TargetBehindStart,
    EndTurnTooLarge,
    SlopeTooLarge,
    ChainOverChord,
    ChainOverOsmLeg,
    OverlapAnchorBusy,
    OverlapPrecedenceEarlier,
    OverlapUnresolved,
    CoverageGap,
    SpanJointOffAxis,
    LegJoinTooFar,
    DiagonalStepHeightChange,
    SpanAnchorRise,
    AnchorShapeOffChain,
}

fn create_reject_reason(why: CreateReject) -> &'static str {
    match why {
        CreateReject::TargetBehindStart => "target-behind-start",
        CreateReject::EndTurnTooLarge => "end-turn-over-90deg",
        CreateReject::SlopeTooLarge => "slope-over-0.4",
        CreateReject::ChainOverChord => "chain-over-1.4x-chord",
        CreateReject::ChainOverOsmLeg => "chain-over-2x-osm-leg",
        CreateReject::OverlapAnchorBusy => "overlap-anchor-cell-busy",
        CreateReject::OverlapPrecedenceEarlier => "overlap-precedence-earlier-span-wins",
        CreateReject::OverlapUnresolved => "overlap-unresolved-osm-situation",
        CreateReject::CoverageGap => "corridor-gap-unwritten",
        CreateReject::SpanJointOffAxis => "span-exit-not-on-shape-axis",
        CreateReject::LegJoinTooFar => "leg-join-gap-too-far",
        CreateReject::DiagonalStepHeightChange => "diagonal-step-height-change",
        CreateReject::SpanAnchorRise => "span-anchor-rise",
        CreateReject::AnchorShapeOffChain => "anchor-shape-retitled-to-chain-step",
    }
}

/// Log one refused connection in `track_rejections.json` (read by `tools/\
/// check_curves_be.py`) with its coordinates, what was measured and why. `start`\
/// and `end` are absolute cells; for an overlap the colliding cell is passed as\
/// both, which is the place the checker should look at.
fn note_create_rejection(
    editor: &mut WorldEditor,
    osm_id: u64,
    why: CreateReject,
    start: (i32, i32, i32),
    end: (i32, i32, i32),
    chain: usize,
    osm_cells: usize,
) {
    let chord = ((end.0 - start.0) as f64)
        .hypot((end.1 - start.1) as f64)
        .hypot((end.2 - start.2) as f64);
    editor
        .create_rails_report()
        .note_rejection(CreateRejectionRow {
            osm_id,
            reason: create_reject_reason(why),
            start,
            end,
            chain,
            chord,
            osm_cells,
        });
}

/// Precedence rule for two spans wanting the same absolute cell (see `claim_create_cells`):
/// the ALREADY WRITTEN span owns it, because its regions may have been flushed to
/// disk and its blocks cannot be retracted. A newcomer that spans strictly more
/// OSM cells than the owner deserves to win instead, and cannot be given that
/// priority after the fact: the overlap is logged as an UNRESOLVED OSM situation
/// (two railway ways physically on top of each other) so the checker reports the
/// gap instead of a silently chosen way. OSM ways are never merged.
fn overlap_rule(owner: &CreateClaim, newcomer_cells: usize) -> CreateReject {
    if newcomer_cells > owner.weight {
        CreateReject::OverlapUnresolved
    } else {
        CreateReject::OverlapPrecedenceEarlier
    }
}

/// Tangent `Axes` at the END anchor of a span: Create stores the axis of every
/// curve end pointing INTO the arc, which is the reverse of the direction the
/// travel arrives from. Dumped reference worlds agree (`tools/ref2_be.json`, all
/// 10 anchor pairs: the start `Axes` project +0.71..0.93 onto the chord and the
/// end `Axes` -0.71..-0.93, i.e. `Starts[1]` sits on the face the chain enters
/// through); the old `+travel` form put the handle beyond the anchor, which is
/// what made the chain overshoot and come back.
fn span_axes_into(vx: f64, vz: f64) -> (f64, f64, f64) {
    (-vx, 0.0, -vz)
}

/// -1/0/+1 of a cell delta: the unit step one track cell takes to its neighbour.
fn sign_step(d: i32) -> i32 {
    if d > 0 {
        1
    } else if d < 0 {
        -1
    } else {
        0
    }
}

/// The planned planned cell just before a run of legs along the same way (`None`
/// at a way end).
fn prev_planned_cell(legs: &[CreateLeg], from: usize) -> Option<(i32, i32)> {
    if from > 0 {
        let p = &legs[from - 1];
        return Some(p.cells[p.cells.len() - 1]);
    }
    None
}

/// The planned cell just after a run of legs along the same way (`None` at a way
/// end).
fn next_planned_cell(legs: &[CreateLeg], to: usize) -> Option<(i32, i32)> {
    if to + 1 < legs.len() {
        return Some(legs[to + 1].cells[0]);
    }
    None
}

/// Unit (x,z) the carriage TRAVELS at the ends of a run of legs, taken from the
/// planned cell physically next to each anchor rather than from the ideal tangent
/// of the leg: the anchor's base `shape` is named from that same neighbour cell
/// (see `run_end_shape`), and Create walks a joint only when the curve tangent in
/// `Connections.Axes` agrees with the shape — a freshly generated `osm-rail23`
/// The `shape` an anchor of a smoothed run must carry so a Create carriage can
/// walk the physical step to the cell actually next to it on the way. `TrackBlock
///.getConnected` (tools/create_cls/TrackBlock.txt) walks a `turn=false` track
/// cell only along `getTrackAxes` = `TrackShape.getAxes()`, and the axes dumped
/// from the enum (tools/create_cls/TrackShape.txt) name ONE axis per straight
/// shape: `zo` (0,0,1), `xo` (1,0,0), `pd` (1,0,1), `nd` (-1,0,1). So the shape
/// has to name the direction of the neighbour cell — taking it from the ideal
/// direction of the OSM leg instead parks an `nd`/`pd` shape whose axes point
/// away from anything (dumped in `osm-rail19`/`osm-rail20`: consecutive run
/// anchors 1 cell apart at offset (1,0,0), both shaped `nd`, so the chain died
/// at every span joint and the longest run stayed at one span, ~46 blocks).
fn run_end_shape(neighbour: Option<(i32, i32)>, anchor: (i32, i32), v: (f64, f64)) -> &'static str {
    if let Some(n) = neighbour {
        return straight_shape((sign_step(n.0 - anchor.0), sign_step(n.1 - anchor.1)));
    }
    straight_shape(nearest_unit_v(v.0, v.1))
}

/// The `TrackShape` that NAMES a step out of an anchor: `getConnected` walks a
/// `turn=false` cell only along `TrackShape.getAxes()`, so the shape at an anchor
/// has to name the cell its chain actually steps into. Axes dumped from
/// tools/create_cls/TrackShape.txt: `xo` (1,0,0), `zo` (0,0,1), `pd` (1,0,1), `nd
/// (-1,0,1), `AN` (0,1,-1), `AS` (0,1,1), `AE` (1,1,0), `AW` (-1,1,0) — and every
/// axis is walked in BOTH directions, so the sign of a flat step is irrelevant
/// while the sign of a stepped one fixes the name. A step Create has no shape for
/// (two horizontal components of different size) gives `None` = keep the planned
/// shape rather than invent one.
fn shape_for_step(anchor: (i32, i32, i32), step: (i32, i32, i32)) -> Option<&'static str> {
    let (dx, dy, dz) = (
        sign_step(step.0 - anchor.0),
        sign_step(step.1 - anchor.1),
        sign_step(step.2 - anchor.2),
    );
    if dx.abs() > 1 || dy.abs() > 1 || dz.abs() > 1 {
        return None;
    }
    if dx == 0 && dy == 0 && dz == 0 {
        return None;
    }
    if dy == 0 {
        if dz == 0 {
            return Some("xo");
        }
        if dx == 0 {
            return Some("zo");
        }
        if dx == dz {
            return Some("pd");
        }
        if dx == -dz {
            return Some("nd");
        }
        return None;
    }
    if dx == 0 && dz != 0 {
        if (dz > 0) == (dy > 0) {
            return Some("as");
        }
        return Some("an");
    }
    if dz == 0 && dx != 0 {
        if (dx > 0) == (dy > 0) {
            return Some("ae");
        }
        return Some("aw");
    }
    None
}

/// The first chain cell that is not the anchor itself: `create_curve_cells` star
/// ts at the anchor, so `chains[i][0] == anchor` for every span and the real ex
/// it step is the next cell that differs (measured: 96 of 123 `span-exit-not-on-
/// shape-axis` rows in a fresh `osm-rail28` passed the anchor as its own `end`, i
/// .e. the audit was measuring a step of (0,0,0)).
fn exit_cell(cells: &[(i32, i32, i32)], anchor: (i32, i32, i32)) -> (i32, i32, i32) {
    for c in cells.iter() {
        if *c != anchor {
            return *c;
        }
    }
    anchor
}

/// Measure a candidate connection before it is written: the number of `create:
/// fake_track` cells its chain would pave, plus the first threshold it violates
/// (`None` = write it). The chain is the one `emit_create_curve` would produce,
/// so the validator and the writer can never disagree about what gets laid.
fn check_create_connection(c: &CreateConnection) -> (usize, Option<CreateReject>) {
    let (dx, dy, dz) = (c.b.0 - c.a.0, c.b.1 - c.a.1, c.b.2 - c.a.2);
    let horiz = (dx as f64).hypot(dz as f64);
    let chord = horiz.hypot(dy as f64);
    if chord < 1.0 {
        return (0, Some(CreateReject::TargetBehindStart));
    }
    let (ux, uy, uz) = (
        (dx as f64) / chord,
        (dy as f64) / chord,
        (dz as f64) / chord,
    );
    let along = |axis: (f64, f64, f64)| -> f64 { axis.0 * ux + axis.1 * uy + axis.2 * uz };
    // `Axes` point into the arc, so the start tangent must project forwards onto
    // the chord and the end tangent backwards: the overshoot pair dumped from
    // `osm-rail19` measured -0.998 at its start anchor.
    if along(c.axes_a) < CREATE_TARGET_BEHIND_DOT || along(c.axes_b) > -CREATE_TARGET_BEHIND_DOT {
        return (0, Some(CreateReject::TargetBehindStart));
    }
    // How far the spline has to turn overall, from the start tangent to the
    // reverse of the end tangent: `atan2` keeps the 180 deg case (a doubled-back
    // OSM polyline) that a dot product alone would call straight.
    let (sx, sz) = (c.axes_a.0, c.axes_a.2);
    let (ex, ez) = (-c.axes_b.0, -c.axes_b.2);
    if sx != 0.0 || sz != 0.0 || ex != 0.0 || ez != 0.0 {
        let deg = (sx * ez - sz * ex).atan2(sx * ex + sz * ez).abs() * (180.0 / CREATE_PI);
        if deg > CREATE_MAX_END_TURN_DEG {
            return (0, Some(CreateReject::EndTurnTooLarge));
        }
    }
    let slope = if horiz > 0.0 {
        (dy as f64).abs() / horiz
    } else {
        99.0
    };
    if slope > CREATE_MAX_SLOPE {
        return (0, Some(CreateReject::SlopeTooLarge));
    }
    let start_a = curve_start(&c.a, &c.axes_a);
    let start_b = curve_start(&c.b, &c.axes_b);
    // Same chain measure the checker uses (`tools/check_curves_be.py` counts the
    // cells of this Bezier between the anchors), so a span the validator lets
    // through can never be flagged over the ratio afterwards.
    let chain = create_curve_cells(start_a, start_b, &c.axes_a, &c.axes_b).len();
    if (chain as f64) > CREATE_MAX_CHAIN_PER_CHORD * chord {
        return (chain, Some(CreateReject::ChainOverChord));
    }
    if c.osm_cells > 0 && (chain as f64) > CREATE_MAX_CHAIN_PER_LEG * (c.osm_cells as f64) {
        return (chain, Some(CreateReject::ChainOverOsmLeg));
    }
    (chain, None)
}

/// The connection one run of legs (`from..to` inclusive) would lay, with the
/// end tangents taken from `va`/`vb`: normally the vectors of the first and
/// last leg, or one shared direction when replanning a refused span with the
/// matching axis. `None` when an anchor cell is busy, which no amount of
/// replanning can fix for this pair of ends. The tangent the chain and the
/// validator use comes from `shape_a`/`shape_b`, because in Create the anchor's
/// shape names the axis the carriage leaves on.
#[allow(clippy::too_many_arguments)]
fn run_connection(
    editor: &mut WorldEditor,
    osm_id: u64,
    legs: &[CreateLeg],
    from: usize,
    to: usize,
    va: (f64, f64),
    vb: (f64, f64),
    shape_a: &'static str,
    shape_b: &'static str,
) -> Option<CreateConnection> {
    let (fax, faz) = legs[from].cells[0];
    let (tbx, tbz) = legs[to].cells[legs[to].cells.len() - 1];
    let osm_cells = legs[from..to + 1]
        .iter()
        .fold(0usize, |acc, leg| acc + leg.osm_span);
    let (yf, yt) = span_anchor_ys(
        editor,
        osm_id,
        (fax, faz),
        anchor_y(editor, (fax, faz)),
        (tbx, tbz),
        anchor_y(editor, (tbx, tbz)),
        0,
    );
    let ends = vec![(fax, yf, faz), (tbx, yt, tbz)];
    let group = editor.create_rails_report().group_for(osm_id, &ends);
    if !create_anchor_free(editor, osm_id, group, (fax, yf, faz))
        || !create_anchor_free(editor, osm_id, group, (tbx, yt, tbz))
    {
        return None;
    }
    Some(CreateConnection {
        a: (fax, yf, faz),
        b: (tbx, yt, tbz),
        shape_a,
        shape_b,
        axes_a: span_axes(va.0, va.1),
        axes_b: span_axes_into(vb.0, vb.1),
        osm_cells,
    })
}

/// Validate, claim and write a SET of connections AT ONCE: either every one of
/// them gets its anchor pair and its whole `create:fake_track` chain, or none of
/// them does and the first refusal lands in `track_rejections.json` with
/// coordinates and reason. Two half-spans can never be written (a chain that
/// stops at an intermediate anchor dead-ends the carriage), and a span can never
/// be laid on top of another span: every absolute cell — anchors and chain cells
/// — is claimed through `CreateRailReport::claim` BEFORE a single block or block
/// entity is written, and a cell owned by another group rejects the whole set.
/// That is the all-or-nothing write rule: an anchor cell is never taken over by
/// another span's `create:fake_track` body, and two `Connections` block entities
/// never share a cell (`insert_block_entity` appends, it does not dedupe).
fn emit_create_connections(
    editor: &mut WorldEditor,
    osm_id: u64,
    conns: &Vec<CreateConnection>,
    // Out-param: the cells this call ACTUALLY wrote (anchors plus chain body),
    // so `track_ways.json` can hand the checker the footprint of the way as it
    // exists in the world, not only what the planner meant to cover.
    written: &mut Vec<(i32, i32, i32)>,
) -> bool {
    let mut want: Vec<((i32, i32, i32), bool)> = vec![];
    let mut ends: Vec<(i32, i32, i32)> = vec![];
    let mut weight: usize = 0;
    // Chain cells are computed once here and reused for the write, so the anchor
    // shape can be named after the cell the chain REALLY steps into: `TrackBlock
    // .getConnected` leaves an anchor along `TrackShape.getAxes()`, so a shape
    // that does not name the chain's first cell leaves the carriage with nothing
    // to ride (a dumped osm-rail20 joint had an `xo` anchor while its chain left
    // at (1,0,-1), and the structural walk called that joint broken).
    let mut chains: Vec<Vec<(i32, i32, i32)>> = vec![];
    for c in conns {
        let (chain, why) = check_create_connection(c);
        if let Some(why) = why {
            note_create_rejection(editor, osm_id, why, c.a, c.b, chain, c.osm_cells);
            return false;
        }
        want.push((c.a, true));
        want.push((c.b, true));
        ends.push(c.a);
        ends.push(c.b);
        let cells = create_curve_cells(
            curve_start(&c.a, &c.axes_a),
            curve_start(&c.b, &c.axes_b),
            &c.axes_a,
            &c.axes_b,
        );
        for cell in cells.iter() {
            want.push((*cell, false))
        }
        chains.push(cells);
        weight += c.osm_cells
    }
    // NAME the anchors' shapes from the cells their own chain steps into. The
    // planner picked those shapes from the ideal direction of the OSM leg, and a
    // Bezier chain leaving an anchor does NOT follow the OSM staircase: measured
    // on a fresh `osm-rail23`, 123 of 178 anchor ends step OFF the axis of the
    // shape sitting there (`span-exit-not-on-shape-axis` rows; the dumped
    // reference worlds `create-track-test`/`track-stage2` have 0). `getConnected`
    // walks a `turn=false` cell only along `TrackShape.getAxes()` (tools/create_cls
    // /TrackBlock.txt), so such a joint cannot be passed and cut the longest run
    // down to 276 blocks. Retitling the shape to the chain's own first/last step
    // makes the shape and the chain agree at EVERY joint by construction — a
    // straight whose chain runs diagonally becomes `pd`/`nd` (both dumped as
    // straight, axes (1,0,1)/(-1,0,1)), and a joint whose chain steps diagonally
    // AND up/down becomes `AN`/`AS`/`AE`/`AW` (dumped axes (0,1,-1)/(0,1,1)/(1,1,0
    // )/(-1,1,0), i.e. Create's own name for a diagonal step that changes height
    // — the other half of the 25 `diagonal-step-height-change` rows measured on
    // `osm-rail27`). Naming from the OTHER span's chain instead (measured before:
    // longest run 542 -> 235) and driving `Axes` from the neighbour cell (8 spans
    // left) both failed; this one changes only the shape TITLE, never the Bezier
    // `Axes`, so the chain geometry and every gate stay bit-identical.
    let mut named: Vec<(&'static str, &'static str)> = vec![];
    for i in 0..conns.len() {
        let c = &conns[i];
        let cells = &chains[i];
        let first = exit_cell(cells, c.a);
        let last = exit_cell(cells, c.b);
        let na = match shape_for_step(c.a, first) {
            Some(s) => s,
            None => c.shape_a,
        };
        let nb = match shape_for_step(c.b, last) {
            Some(s) => s,
            None => c.shape_b,
        };
        if na != c.shape_a {
            note_create_rejection(
                editor,
                osm_id,
                CreateReject::AnchorShapeOffChain,
                c.a,
                first,
                cells.len(),
                c.osm_cells,
            )
        }
        if nb != c.shape_b {
            note_create_rejection(
                editor,
                osm_id,
                CreateReject::AnchorShapeOffChain,
                c.b,
                last,
                cells.len(),
                c.osm_cells,
            )
        }
        named.push((na, nb));
    }
    let group = editor.create_rails_report().group_for(osm_id, &ends);
    if let Some((cell, owner)) = editor
        .create_rails_report()
        .claim(&want, group, osm_id, weight)
    {
        note_create_rejection(
            editor,
            osm_id,
            overlap_rule(&owner, weight),
            cell,
            cell,
            0,
            weight,
        );
        return false;
    }
    for i in 0..conns.len() {
        let c = &conns[i];
        // The shape written stays the LEG TANGENT's (`shape_a`/`shape_b`): one
        // anchor shape names ONE axis walked in BOTH directions, so it has to
        // serve the neighbouring leg's straight run as well as this span's chain -
        // renaming it to the chain's exit step breaks that joint (measured: DRIVABLE
        // WALK 28 components / 661 blocks became 39 / 365 when the retitled shape
        // was written). The retitled name is kept only as a diagnostic row above.
        let (shape_a, shape_b) = (c.shape_a, c.shape_b);
        editor.emit_create_curve(c.a, c.b, shape_a, shape_b, c.axes_a, c.axes_b);
        // Self-audit of the joint itself: `getConnected` walks `getTrackAxes`,
        // which is `TrackShape.getAxes()`, and every shape axis is walked in
        // BOTH directions — dumped `track-stage2` and `create-track-test` have
        // 0 of their 14 anchor ends whose chain leaves off its shape axis
        // (`xo` at (-1,0,0), `zo` at (0,-1,0) and so on all count as ON axis),
        // while a freshly generated `osm-rail20` had 59 of 180. A chain leaving
        // off the axis is a joint the carriage cannot pass, so it gets a row.
        let cells = &chains[i];
        let first = exit_cell(cells, c.a);
        let last = exit_cell(cells, c.b);
        if first != c.a && joint_off_axis(c.a, first, shape_a) {
            note_create_rejection(
                editor,
                osm_id,
                CreateReject::SpanJointOffAxis,
                c.a,
                first,
                cells.len(),
                c.osm_cells,
            )
        }
        if last != c.b && joint_off_axis(c.b, last, shape_b) {
            note_create_rejection(
                editor,
                osm_id,
                CreateReject::SpanJointOffAxis,
                c.b,
                last,
                cells.len(),
                c.osm_cells,
            )
        }
        written.push(c.a);
        written.push(c.b);
        for cell in chains[i].iter() {
            written.push(*cell)
        }
    }
    true
}

/// One connection under the same all-or-nothing rule (see
/// `emit_create_connections`): measured, claimed and written as a set of one.
fn emit_create_connection(
    editor: &mut WorldEditor,
    osm_id: u64,
    c: CreateConnection,
    written: &mut Vec<(i32, i32, i32)>,
) -> bool {
    emit_create_connections(editor, osm_id, &vec![c.to_owned()], written)
}

/// Validate and write a run of legs: first as one connection, and if the
/// validator refuses it, twice more with a replan — once with the matching axis
/// (both ends take the direction of the longest leg in the run, which is what
/// the chain actually travels along), once split at the middle leg boundary so
/// an intermediate anchor breaks a doubled-back span in half. Every attempt is
/// measured before it is written, so either the whole run gets rails laid or
/// nothing does and the rejection stays in the sidecar.
fn write_run_of_legs(
    editor: &mut WorldEditor,
    osm_id: u64,
    legs: &[CreateLeg],
    from: usize,
    to: usize,
    written: &mut Vec<(i32, i32, i32)>,
) -> bool {
    let a = &legs[from];
    let b = &legs[to];
    let sa = run_end_shape(
        prev_planned_cell(legs, from),
        legs[from].cells[0],
        (a.vx, a.vz),
    );
    let sb = run_end_shape(
        next_planned_cell(legs, to),
        legs[to].cells[legs[to].cells.len() - 1],
        (b.vx, b.vz),
    );
    if let Some(conn) = run_connection(
        editor,
        osm_id,
        legs,
        from,
        to,
        (a.vx, a.vz),
        (b.vx, b.vz),
        sa,
        sb,
    ) {
        if emit_create_connection(editor, osm_id, conn, written) {
            return true;
        }
    }

    let mut li = from;
    for k in (from + 1)..(to + 1) {
        if legs[k].osm_span > legs[li].osm_span {
            li = k
        }
    }
    let m = &legs[li];
    if let Some(conn) = run_connection(
        editor,
        osm_id,
        legs,
        from,
        to,
        (m.vx, m.vz),
        (m.vx, m.vz),
        sa,
        sb,
    ) {
        if emit_create_connection(editor, osm_id, conn, written) {
            return true;
        }
    }

    // Split attempt: both halves must measure valid, otherwise the run stays a
    // clean gap (half a span would dead-end the carriage at the intermediate
    // anchor with nothing to ride on).
    let mid = (from + to) / 2;
    if mid > from && mid < to {
        let lsa = run_end_shape(
            prev_planned_cell(legs, from),
            legs[from].cells[0],
            (legs[from].vx, legs[from].vz),
        );
        let lsb = run_end_shape(
            next_planned_cell(legs, mid),
            legs[mid].cells[legs[mid].cells.len() - 1],
            (legs[mid].vx, legs[mid].vz),
        );
        let rsa = run_end_shape(
            prev_planned_cell(legs, mid + 1),
            legs[mid + 1].cells[0],
            (legs[mid + 1].vx, legs[mid + 1].vz),
        );
        let rsb = run_end_shape(
            next_planned_cell(legs, to),
            legs[to].cells[legs[to].cells.len() - 1],
            (legs[to].vx, legs[to].vz),
        );
        if let Some(lc) = run_connection(
            editor,
            osm_id,
            legs,
            from,
            mid,
            (legs[from].vx, legs[from].vz),
            (legs[mid].vx, legs[mid].vz),
            lsa,
            lsb,
        ) {
            if let Some(rc) = run_connection(
                editor,
                osm_id,
                legs,
                mid + 1,
                to,
                (legs[mid + 1].vx, legs[mid + 1].vz),
                (legs[to].vx, legs[to].vz),
                rsa,
                rsb,
            ) {
                // Both halves are written as ONE set: claiming every cell of both
                // before writing either means a half that collides with another
                // span cannot leave the other half hanging as a dead end.
                if emit_create_connections(
                    editor,
                    osm_id,
                    &vec![lc.to_owned(), rc.to_owned()],
                    written,
                ) {
                    return true;
                }
            }
        }
    }
    false
}

/// One row of `track_ways.json`: the cells one OSM railway way was meant to
/// cover, so the checker scores every way separately (OSM ways are never merged
/// into one row).
#[derive(Clone, serde::Serialize)]
pub(crate) struct CreateWayRow {
    osm_id: u64,
    cells: Vec<(i32, i32, i32)>,
    written: Vec<(i32, i32, i32)>,
}

#[derive(Clone, serde::Serialize)]
struct CreateWaysSidecar {
    ways: Vec<CreateWayRow>,
}

/// One row of `track_rejections.json`: a connection the validator refused, with
/// its coordinates, what was measured and why.
#[derive(Clone, serde::Serialize)]
pub(crate) struct CreateRejectionRow {
    osm_id: u64,
    reason: &'static str,
    start: (i32, i32, i32),
    end: (i32, i32, i32),
    chain: usize,
    chord: f64,
    osm_cells: usize,
}

#[derive(Clone, serde::Serialize)]
struct CreateRejectionsSidecar {
    rejections: Vec<CreateRejectionRow>,
}

/// Vegetation the generator scatters at ground level +1 (short grass, ferns,
/// flowers, leaf bushes): a Create track may replace it, so it never decides
/// where the rails sit and never survives inside the corridor the train needs.
/// Deliberately NOT here: anything a built structure or station part is made of
/// (bricks, slabs, doors, glass, rails), and `GRASS_BLOCK`/`MOSS_BLOCK` are
/// solid ground, not a plant.
pub(crate) const CREATE_REPLACEABLE_VEGETATION: &[Block] = &[
    GRASS,
    MOSS_CARPET,
    TALL_GRASS_BOTTOM,
    TALL_GRASS_TOP,
    FERN,
    LARGE_FERN_LOWER,
    LARGE_FERN_UPPER,
    DEAD_BUSH,
    RED_FLOWER,
    YELLOW_FLOWER,
    BLUE_FLOWER,
    WHITE_FLOWER,
    CORNFLOWER,
    OXEYE_DAISY,
    ALLIUM,
    LILY_OF_THE_VALLEY,
    RED_TULIP,
    ORANGE_TULIP,
    WHITE_TULIP,
    PINK_TULIP,
    SUNFLOWER_LOWER,
    SUNFLOWER_UPPER,
    LILAC_LOWER,
    LILAC_UPPER,
    ROSE_BUSH_LOWER,
    ROSE_BUSH_UPPER,
    PEONY_LOWER,
    PEONY_UPPER,
    SWEET_BERRY_BUSH,
    BROWN_MUSHROOM,
    RED_MUSHROOM,
    PUMPKIN,
    CACTUS,
    SUGAR_CANE,
    AZALEA,
    FLOWERING_AZALEA,
    SNOW_LAYER,
    LILY_PAD,
    CAVE_VINES,
    CAVE_VINES_PLANT,
    BIG_DRIPLEAF,
    SMALL_DRIPLEAF_LOWER,
    SMALL_DRIPLEAF_UPPER,
    OAK_LEAVES,
    BIRCH_LEAVES,
    SPRUCE_LEAVES,
    DARK_OAK_LEAVES,
    JUNGLE_LEAVES,
    ACACIA_LEAVES,
    CHERRY_LEAVES,
    MANGROVE_LEAVES,
    AZALEA_LEAVES,
    FLOWERING_AZALEA_LEAVES,
];

/// What natural ground and Arnis's own railway ballast are made of: loose
/// material a corridor may dig through without being part of anything built.
/// Deliberately NOT here: bricks, slabs, doors, glass, concrete, metals and
/// stone, so a station building or platform never loses a block because a
/// track runs through it — those cells stay blocked and get accounted instead.
pub(crate) const CREATE_CORRIDOR_MATERIALS: &[Block] = &[
    GRASS_BLOCK,
    DIRT,
    DIRT_PATH,
    COARSE_DIRT,
    PODZOL,
    SAND,
    SANDSTONE,
    SMOOTH_SANDSTONE,
    GRAVEL,
    MOSS_BLOCK,
    MUD,
    CLAY,
    SNOW_BLOCK,
    SNOW_LAYER,
    STONE,
    COBBLESTONE,
    MOSSY_COBBLESTONE,
    FARMLAND,
    CARROTS,
    POTATOES,
    WHEAT,
    OAK_LOG,
    BIRCH_LOG,
    SPRUCE_LOG,
    DARK_OAK_LOG,
    JUNGLE_LOG,
    ACACIA_LOG,
    CHERRY_LOG,
    MANGROVE_LOG,
];

/// Corridor width in cells, measured perpendicular to the travel direction and
/// centred on the track cell: 3 means the track cell plus one cell on each
/// side. The declared set guards all four sides (a superset), because at
/// declare time the shape of the neighbour that will claim the cell is unknown.
pub(crate) const CREATE_CORRIDOR_WIDTH: usize = 3;

/// How many levels above the track cell a corridor stays clear for: a Create
/// train cab is up to three blocks tall over the rails, so the cell the rails
/// sit in and the three cells above it must all stay empty (Y..Y+3 inclusive).
pub(crate) const CREATE_CORRIDOR_UP: usize = 3;

/// Can a Create corridor replace this block: one of the two sets above.
pub(crate) fn create_corridor_clears(block: Block) -> bool {
    CREATE_CORRIDOR_MATERIALS.contains(&block) || CREATE_REPLACEABLE_VEGETATION.contains(&block)
}

/// Declares the Create rail corridors of every at-grade railway way BEFORE any
/// tile runs. Tiles process ways in parallel, so a tile's ground pass can write
/// terrain into a corridor whose rails a parallel tile has not laid yet, and
/// the only later cleanup (the merged pass) cannot write regions already
/// streamed to disk. The presurvey arms the write guard for the whole run: it
/// walks the same rasterized centerline the track bed uses (the rails
/// themselves are laid from the simplified leg geometry, which stays within a
/// couple of cells of it) and claims a slightly wider band - two cells to each
/// side, rail level up to `CREATE_CORRIDOR_UP + 1` - so the exact corridor the
/// write-time claim produces is always inside it. The band deliberately
/// excludes the cell below the rails: that is where the gravel bed and the
/// sleepers go, and the corridor floor pass owns it instead.
pub fn presurvey_create_corridors(editor: &mut WorldEditor, elements: &[ProcessedElement]) {
    let report = editor.create_rails_report();
    for element in elements {
        let ProcessedElement::Way(way) = element else {
            continue;
        };
        let Some(railway_type) = way.tags.get("railway") else {
            continue;
        };
        if [
            "proposed",
            "abandoned",
            "construction",
            "razed",
            "turntable",
        ]
        .contains(&railway_type.as_str())
        {
            continue;
        }
        if wants_rail_tunnel(way) || is_rail_bridge(way) {
            // Tunnels bring their own shell ahead of the ground fill, bridges
            // ride above it; their cells are protected by what builds them.
            continue;
        }
        if way.nodes.len() < 2 {
            continue;
        }
        let mut all_points: Vec<(i32, i32, i32)> = Vec::new();
        for window in way.nodes.windows(2) {
            let prev_node = window[0].xz();
            let cur_node = window[1].xz();
            let points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
            let smoothed_points = smooth_diagonal_rails(&points);
            let skip_first = if all_points.is_empty() { 0 } else { 1 };
            all_points.extend(&smoothed_points[skip_first..smoothed_points.len()]);
        }
        // The bed raster below is only an approximation of where the rails
        // go: the planner lays them on the planned LEG cells, which deviate
        // laterally from the raster on smoothed diagonals (measured on
        // osm-rail41: every violating cell sat one Chebyshev step from an
        // actual track cell, `claimed_raw=false` at write time under a
        // raster-based presurvey). Claim the band around the real planned
        // cells as well - `note_corridor` expands each claim into its own
        // Chebyshev-1 x Y..Y+3 ring, so the two sources overlap freely.
        let leg_cells = planned_leg_cells(editor, way.id, &way_nodes_cells(way));
        for (lx, lz) in &leg_cells {
            let rail_y = editor.get_ground_level(*lx, *lz) + 1;
            for dy in -1..=(CREATE_CORRIDOR_UP as i32) {
                report.note_corridor((*lx, rail_y + dy, *lz));
            }
        }
        for (bx, _, bz) in &all_points {
            let rail_y = editor.get_ground_level(*bx, *bz) + 1;
            // The rail column AND the two cells beside it are claimed from one
            // BELOW the rail level: a span flattened below the per-column
            // estimate (span_anchor_ys, chain dips) puts the neighbour column's
            // SURFACE at the dipped rail's level, and that surface is exactly
            // the corridor of the dipped cell (measured on osm-rail35..37:
            // (-281,-1562,-37), the surface of a flat neighbour pair, survived
            // every run while only the centre column was claimed from below).
            // The cells two away keep their surface: their band starts at rail
            // level, one above the ground.
            for dy in -1..=(CREATE_CORRIDOR_UP as i32) {
                // The full Chebyshev-1 ring, diagonals included: a pd/nd span
                // cell's perpendicular corridor runs DIAGONALLY ((x±1, z±1)),
                // and an axial-only presurvey left exactly those cells
                // unguarded (measured on osm-rail39 with the diagonal-aware
                // checker window: 124 natural violations, all on diagonal
                // corridor columns of diagonal spans and tunnel rails).
                for dx in -1..=1 {
                    for dz in -1..=1 {
                        report.note_corridor((bx + dx, rail_y + dy, bz + dz));
                    }
                }
                // Cells two away stay axial-only and only from rail level up,
                // so the surface beside the protected band keeps its grass.
                if dy >= 0 {
                    for d in [(-2, 0), (2, 0), (0, -2), (0, 2)] {
                        report.note_corridor((bx + d.0, rail_y + dy, bz + d.1));
                    }
                }
            }
        }
    }
}

/// Re-opens every corridor the Create planner declared, once the world is
/// built. The rails are written before the ground generator, the cave carver
/// and the ground decorator, and the write guard keeps most later passes out
/// of a declared corridor - but blocks that were written before the corridor
/// was declared (earlier tiles' terrain, parallel tiles racing the rail write)
/// and writes through unguarded bulk paths still land inside it. This pass
/// restores the invariant a Create train needs - every corridor cell empty,
/// and every rail cell resting on real ground that is not a plant - and counts
/// what it removed and what it could not remove into `track_corridors.json`,
/// so the accounting says which block stopped which corridor. It runs per
/// tile right after that tile's ground pass and once more after every
/// terrain, decoration and structure pass has finished.
pub fn clear_create_corridors(editor: &mut WorldEditor) {
    let report = editor.create_rails_report();
    let cells = report.corridor_snapshot();
    for (x, y, z) in cells {
        // A tile only clears its own columns; the merged pass covers the rest.
        if !editor.owns(x, z) {
            continue;
        }
        if !report.corridor_next((x, y, z), false) {
            continue;
        }
        // Cells the corridor pass itself put back (the floor under the rails).
        if !report.corridor_contains((x, y, z)) {
            continue;
        }
        let block = editor.get_block_absolute(x, y, z);
        match block {
            None => {}
            Some(b) => {
                let name = b.name();
                if b.namespace() == "create" {
                    // Another track cell: never touched by the clearing.
                    continue;
                }
                if create_corridor_clears(b) {
                    editor.set_block_absolute(AIR, x, y, z, Some(&[b]), None);
                    report.note_corridor_outcome((x, y, z), name, true);
                } else {
                    // A built structure (bricks, glass, a mast, a tunnel
                    // shell): the corridor never dismantles it, it is only
                    // logged so the sidecar names what stands in the way.
                    report.note_corridor_outcome((x, y, z), name, false);
                }
            }
        }
    }
    // Rest the rails on real ground again: the cell under every written track
    // cell gets a floor when the carver took the ground away or a plant sat
    // there instead of soil. A below-cell that is itself part of a declared
    // corridor is left alone: on a flattened span it is the side cell of the
    // lower track beside it, and a grass_block floor written there is exactly
    // the regrowth this pass exists to remove (measured on osm-rail38: 46 of
    // the 47 remaining natural violations were these fills, written through
    // their own exemption). Such a cell belongs to the main clearing loop,
    // which airs whatever sits in it.
    for (x, y, z) in report.written_snapshot() {
        let below = (x, y - 1, z);
        if !editor.owns(x, z) {
            continue;
        }
        if !report.corridor_next(below, true) {
            continue;
        }
        if report.corridor_claimed(below) {
            continue;
        }
        match editor.get_block_absolute(below.0, below.1, below.2) {
            Some(b) => {
                if !CREATE_REPLACEABLE_VEGETATION.contains(&b) {
                    // Real ground (soil, ballast, a tunnel floor, a bridge
                    // deck): stays as it is.
                    continue;
                }
                // A plant under the rails: swap it for the soil it grew in.
                report.note_corridor_exempt(below);
                editor.set_block_absolute(GRASS_BLOCK, below.0, below.1, below.2, Some(&[b]), None);
                report.note_corridor_outcome(below, b.name(), true);
            }
            None => {
                // Nothing under the rails at all (a cave ate the ground): the
                // corridor keeps a floor so the train has something to roll on.
                report.note_corridor_exempt(below);
                editor.set_block_absolute(GRASS_BLOCK, below.0, below.1, below.2, None, None);
                report.note_corridor_outcome(below, "filled", true);
            }
        }
    }
}

/// Post-merge corridor sweep: a tile merge can reintroduce blocks into
/// corridor cells two ways - a prior tile's HALO data fills main-editor AIR
/// (write-if-air path), and the owning tile's merge then PRESERVES that halo
/// data wherever the owner's own write was refused by the corridor guard
/// (auth-overwrite-nonair keeps halo at auth-AIR). The tile's own clearing
/// pass cannot see either: it ran before the merge. This sweep re-opens every
/// corridor cell right after each merge, while the regions are still writable
/// (under eviction a region streams to disk as soon as its last contributing
/// tile merges, and the final pass can no longer write it). The
/// `corridor_next` dedupe deliberately does not apply: every merge can
/// re-introduce a block at a cell an earlier sweep already cleaned (halo
/// fills the AIR the sweep left), so the sweep is simply idempotent. Only
/// actual removals are accounted; standing protected blocks stay counted by
/// the tile and final passes.
pub fn sweep_create_corridors_after_merge(editor: &mut WorldEditor) {
    let report = editor.create_rails_report();
    for (x, y, z) in report.corridor_snapshot() {
        if !report.corridor_contains((x, y, z)) {
            continue;
        }
        if let Some(b) = editor.get_block_absolute(x, y, z) {
            if b.namespace() == "create" {
                continue;
            }
            if create_corridor_clears(b) {
                editor.set_block_absolute(AIR, x, y, z, Some(&[b]), None);
                report.note_corridor_outcome((x, y, z), b.name(), true);
            }
        }
    }
}

/// One blocked corridor cell of `track_corridors.json`: where a corridor had
/// to stop, and which block made it stop there.
#[derive(Clone, serde::Serialize)]
struct CreateCorridorBlockCell {
    cell: (i32, i32, i32),
    name: String,
}

#[derive(Clone, serde::Serialize)]
struct CreateCorridorsSidecar {
    corridor_cells: usize,
    cleared: HashMap<String, usize>,
    /// Blocks still standing in a corridor when the final pass ran (skipped,
    /// protected by policy) with their cells.
    blocked: HashMap<String, usize>,
    blocked_cells: Vec<CreateCorridorBlockCell>,
    /// Writes the guard refused because they landed in a declared corridor:
    /// the block never entered the world, this is prevention, not residual.
    refused: HashMap<String, usize>,
}

/// One absolute cell owned by one write group (a span, or one straight leg).
/// `weight` is how many OSM cells the owner spans — the precedence weight used
/// by `overlap_rule`; `track` distinguishes a real `create:track` cell (anchor
/// or straight body, carries a blockstate and possibly a block entity) from a
/// `create:fake_track` chain body, which is disposable filler.
#[derive(Clone)]
pub(crate) struct CreateClaim {
    group: u64,
    osm_id: u64,
    weight: usize,
    track: bool,
}

/// One absolute world cell.
type Cell = (i32, i32, i32);
/// An ordered list of absolute world cells.
type CellVec = Vec<Cell>;

/// What the Create rail planner decided, shared via `Arc` with the tile editors
/// so the sidecars written at `save()` see every tile. `claims` is the cell
/// occupancy ledger behind the all-or-nothing write rule: no `create:track`
/// block (anchor or straight body) and no `create:fake_track` chain body is ever
/// written into a cell another span already owns, so an anchor cell is never
/// replaced by another span's fake_track and two `Connections` block entities
/// never share a cell (`insert_block_entity` only appends, it does not dedupe).
/// `group_ids` keeps one claim group per LOGICAL write group (same OSM way, same
/// span endpoints), because a long railway way is dispatched to every tile it
/// crosses and each tile re-plans it identically: without a shared id the second
/// tile would read its own earlier claim as a conflict and reject the way. Each
/// tile then writes only the cells it `owns`, so the pieces stitch back into one
/// continuous run at the tile seams.
pub(crate) struct CreateRailReport {
    ways: Mutex<Vec<CreateWayRow>>,
    rejections: Mutex<Vec<CreateRejectionRow>>,
    claims: Mutex<HashMap<(i32, i32, i32), CreateClaim>>,
    groups: Mutex<u64>,
    group_ids: Mutex<HashMap<(u64, CellVec), u64>>,
    noted: Mutex<HashSet<u64>>,
    logged: Mutex<HashSet<(u64, &'static str, Cell, Cell)>>,
    corridor: Mutex<HashSet<(i32, i32, i32)>>,
    corridor_cleared: Mutex<HashMap<String, usize>>,
    corridor_blocked: Mutex<HashMap<String, usize>>,
    corridor_refused: Mutex<HashMap<String, usize>>,
    corridor_blocked_cells: Mutex<Vec<CreateCorridorBlockCell>>,
    corridor_exempt: Mutex<HashSet<(i32, i32, i32)>>,
    corridor_handled: Mutex<HashSet<(Cell, bool)>>,
}

impl CreateRailReport {
    pub(crate) fn new() -> CreateRailReport {
        CreateRailReport {
            ways: Mutex::new(vec![]),
            rejections: Mutex::new(vec![]),
            claims: Mutex::new(HashMap::new()),
            groups: Mutex::new(0),
            group_ids: Mutex::new(HashMap::new()),
            noted: Mutex::new(HashSet::new()),
            logged: Mutex::new(HashSet::new()),
            corridor: Mutex::new(HashSet::new()),
            corridor_cleared: Mutex::new(HashMap::new()),
            corridor_blocked: Mutex::new(HashMap::new()),
            corridor_refused: Mutex::new(HashMap::new()),
            corridor_blocked_cells: Mutex::new(vec![]),
            corridor_exempt: Mutex::new(HashSet::new()),
            corridor_handled: Mutex::new(HashSet::new()),
        }
    }

    /// One `track_ways.json` row per OSM way: a way whose centroid falls in one
    /// tile but whose geometry crosses several is processed by every overlapping
    /// tile, and the cells it plans are identical every time, so the row is
    /// recorded once. Keeping the duplicates would inflate the checker's
    /// intended-cell denominator (a way counted N times against blocks written
    /// once) and turn an intact way into a fake 1/N share.
    pub(crate) fn note_way(&self, row: CreateWayRow) {
        let fresh = if let Ok(mut n) = self.noted.lock() {
            n.insert(row.osm_id)
        } else {
            true
        };
        if !fresh {
            // A later tile re-plans the same OSM way: its PLANNED cells are the
            // same staircase (duplicating them would inflate the checker's
            // intended-cell denominator), but its WRITTEN cells are new -- each
            // tile writes and claims only the cells it `owns`, so the cells past
            // the tile seam exist in the world and must reach the sidecar too,
            // or the checker scores a way as half written while the carriage
            // rides every piece. Union, keeping the first tile's plan.
            if let Ok(mut w) = self.ways.lock() {
                for r in w.iter_mut() {
                    if r.osm_id == row.osm_id {
                        for c in row.written.iter() {
                            if !r.written.contains(c) {
                                r.written.push(*c)
                            }
                        }
                    }
                }
            }
            return;
        }
        if let Ok(mut w) = self.ways.lock() {
            w.push(row)
        }
    }

    pub(crate) fn note_rejection(&self, row: CreateRejectionRow) {
        let key = (row.osm_id, row.reason, row.start, row.end);
        let fresh = if let Ok(mut l) = self.logged.lock() {
            l.insert(key)
        } else {
            true
        };
        if !fresh {
            return;
        }
        if let Ok(mut j) = self.rejections.lock() {
            j.push(row)
        }
    }

    /// Fresh claim group id, one per write group (span or straight leg).
    pub(crate) fn next_group(&self) -> u64 {
        let mut id = 1u64;
        if let Ok(mut g) = self.groups.lock() {
            *g += 1;
            id = *g;
        }
        id
    }

    /// Group id of one LOGICAL write group, keyed by OSM way plus its span
    /// endpoints, so the same span planned again by another tile editor reuses
    /// one id: `claim` treats cells already owned by the SAME group as free, and
    /// the tile that owns a cell writes it while the others clip it out. A
    /// per-call id would make the second tile collide with the first one's claim
    /// and reject the whole way.
    pub(crate) fn group_for(&self, osm_id: u64, ends: &[Cell]) -> u64 {
        let key = (osm_id, ends.to_vec());
        if let Ok(m) = self.group_ids.lock() {
            if let Some(id) = m.get(&key) {
                return *id;
            }
        }
        let id = self.next_group();
        if let Ok(mut m) = self.group_ids.lock() {
            let _ = m.insert(key, id);
        }
        id
    }

    /// True when `c` is claimed by another group as a `track` cell belonging to
    /// ANOTHER OSM way. Cells the same way already claims are free: a long way is
    /// written as consecutive spans and straight legs, and a span's anchor cell is
    /// exactly the endpoint cell of the leg next to it, so treating a same-way
    /// claim as busy rejects every connecting curve and turns the rest of the way
    /// into a chain of gaps (the 1-tile test measured 0.31 of 213 intended cells
    /// before this rule, with the missing cells all logged as overlap rejections).
    /// Two ANCHORS of one way can still never share a cell — the validator only
    /// proposes spans that follow the polyline, so coinciding same-way anchors
    /// would already be a doubled-back span it refuses on the end-turn gate.
    pub(crate) fn anchor_busy(&self, osm_id: u64, group: u64, c: (i32, i32, i32)) -> bool {
        if let Ok(cl) = self.claims.lock() {
            match cl.get(&c) {
                Some(o) if o.group != group && o.osm_id != osm_id && o.track => return true,
                _ => {}
            }
        }
        false
    }

    /// Claim every cell of one write group or none of it: returns the first cell
    /// already owned by ANOTHER OSM way together with that owner. Cells claimed by
    /// the same group are re-claimed (an anchor cell is also on the chain list of
    /// its own span) and keep their `track` kind, so the ledger never downgrades
    /// a real `create:track` cell into a fake body. Cells claimed by another group
    /// of the SAME way are free too — spans and legs of one way meet at shared
    /// leg-endpoint cells by construction, and refusing those would leave the way
    /// as a string of short runs instead of one continuous one.
    pub(crate) fn claim(
        &self,
        cells: &Vec<((i32, i32, i32), bool)>,
        group: u64,
        osm_id: u64,
        weight: usize,
    ) -> Option<((i32, i32, i32), CreateClaim)> {
        if let Ok(mut c) = self.claims.lock() {
            for (cell, _) in cells {
                match c.get(cell) {
                    Some(owner) if owner.group != group && owner.osm_id != osm_id => {
                        return Some((*cell, owner.clone()))
                    }
                    _ => {}
                }
            }
            for (cell, track) in cells {
                let downgrade = match c.get(cell) {
                    Some(prev) => !track && prev.track,
                    _ => false,
                };
                if !downgrade {
                    c.insert(
                        *cell,
                        CreateClaim {
                            group,
                            osm_id,
                            weight,
                            track: *track,
                        },
                    );
                }
                self.note_corridor(*cell);
            }
        }
        None
    }

    /// True when the global claim ledger already holds ANY write within one
    /// block in x/z and four in y of `c`: the corridor cell is physically paved,
    /// by this way or by another one. The audit of a planned corridor runs once
    /// per tile a way is dispatched to and each tile writes only the cells it
    /// `owns`, so a per-tile `written` list would blame every cell past the tile
    /// seam as a hole; the ledger is the tile-independent answer.
    pub(crate) fn covered_near(&self, c: (i32, i32, i32)) -> bool {
        if let Ok(cl) = self.claims.lock() {
            for dx in -1..2 {
                for dz in -1..2 {
                    for dy in -4..5 {
                        let k = (c.0 + dx, c.1 + dy, c.2 + dz);
                        if cl.get(&k).is_some() {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// `track_ways.json` text for the whole run; empty when no railway way ran.
    pub(crate) fn sidecar_ways(&self) -> String {
        if let Ok(w) = self.ways.lock() {
            if w.is_empty() {
                return "".to_string();
            }
            return serde_json::to_string(&CreateWaysSidecar { ways: w.clone() })
                .unwrap_or("".to_string());
        }
        "".to_string()
    }

    /// Declares the corridor of one track cell: the cell itself, the levels
    /// above it up to `CREATE_CORRIDOR_UP` (a Create train cab is up to three
    /// blocks tall over the rails) and the cells beside it at every one of
    /// those levels, so neither terrain nor a station platform edge grows over
    /// the rails. The sides are declared on all four axes - a superset of the
    /// perpendicular corridor - because at declare time the shape of the
    /// neighbour that will claim the cell is unknown. Repeat declarations of
    /// the same cell are ignored.
    pub(crate) fn note_corridor(&self, cell: (i32, i32, i32)) {
        let half = (CREATE_CORRIDOR_WIDTH as i32 - 1) / 2;
        if let Ok(mut cells) = self.corridor.lock() {
            for up in 0..=(CREATE_CORRIDOR_UP as i32) {
                // The full Chebyshev-1 ring, diagonals included: the corridor
                // of a pd/nd track cell runs diagonally ((x±1, z±1)), and an
                // axial-only ring left exactly those cells outside the guard
                // set (measured on osm-rail41: 114 of the 115 violating cells
                // sit one diagonal step from their track cell).
                for dx in -half..=half {
                    for dz in -half..=half {
                        cells.insert((cell.0 + dx, cell.1 + up, cell.2 + dz));
                    }
                }
            }
        }
    }

    /// True when `c` belongs to a declared Create corridor. Cells the corridor
    /// pass itself put back (the floor under the rails) are exempt: the guard
    /// must not refuse the write that restores them.
    pub(crate) fn corridor_contains(&self, c: (i32, i32, i32)) -> bool {
        if let Ok(exempt) = self.corridor_exempt.lock() {
            if exempt.contains(&c) {
                return false;
            }
        }
        if let Ok(cells) = self.corridor.lock() {
            return cells.contains(&c);
        }
        false
    }

    /// True once any corridor cell is declared - lets the bulk column fills
    /// skip their per-cell corridor check entirely on worlds without Create
    /// rails, keeping the fast path exactly as fast as it was.
    pub(crate) fn corridor_any(&self) -> bool {
        if let Ok(cells) = self.corridor.lock() {
            return !cells.is_empty();
        }
        false
    }

    /// Raw membership in the declared corridor set, ignoring exemptions: true
    /// for a cell any track cell's corridor covers, even one the corridor pass
    /// itself put a floor into. The floor pass uses it to keep its fills out of
    /// cells a neighbouring span's corridor owns.
    pub(crate) fn corridor_claimed(&self, c: (i32, i32, i32)) -> bool {
        if let Ok(cells) = self.corridor.lock() {
            return cells.contains(&c);
        }
        false
    }

    /// True the first time this corridor cell is handled by any corridor pass:
    /// the pass runs once per tile and once again after the merge, so without
    /// this the same cell would be counted (and dug) several times.
    pub(crate) fn corridor_next(&self, cell: (i32, i32, i32), fill: bool) -> bool {
        if let Ok(mut done) = self.corridor_handled.lock() {
            return done.insert((cell, fill));
        }
        true
    }

    /// Marks a cell the corridor pass is allowed to write into its own corridor.
    pub(crate) fn note_corridor_exempt(&self, cell: (i32, i32, i32)) {
        if let Ok(mut exempt) = self.corridor_exempt.lock() {
            exempt.insert(cell);
        }
    }

    /// Every corridor cell that must stay EMPTY for a Create train to pass: the
    /// track cell itself, the levels above it and the cells beside it, for
    /// every track cell written. This is the list the post-build pass re-opens
    /// and the guard protects; `corridor_contains` filters out cells the
    /// corridor pass itself put back (the floor under the rails).
    pub(crate) fn corridor_snapshot(&self) -> Vec<(i32, i32, i32)> {
        if let Ok(cells) = self.corridor.lock() {
            let mut out: Vec<(i32, i32, i32)> =
                cells.iter().copied().collect::<Vec<(i32, i32, i32)>>();
            out.sort_unstable();
            return out;
        }
        vec![]
    }

    /// Every written track cell of every way, for the same post-build pass: the
    /// cell under each of these is where the train needs a floor.
    pub(crate) fn written_snapshot(&self) -> Vec<(i32, i32, i32)> {
        if let Ok(rows) = self.ways.lock() {
            let mut out: Vec<(i32, i32, i32)> = vec![];
            for row in rows.iter() {
                for c in row.written.iter() {
                    out.push((c.0, c.1, c.2));
                }
            }
            out.sort_unstable();
            out.dedup();
            return out;
        }
        vec![]
    }

    /// Records what the corridor pass found in one ledger cell, counted per
    /// block name so the accounting says which block stopped which corridor.
    pub(crate) fn note_corridor_outcome(&self, cell: (i32, i32, i32), name: &str, cleared: bool) {
        let key = name.to_string();
        if cleared {
            if let Ok(mut counts) = self.corridor_cleared.lock() {
                let next: usize = match counts.get(&key) {
                    Some(n) => n + 1usize,
                    None => 1usize,
                };
                counts.insert(key, next);
            }
            return;
        }
        if let Ok(mut counts) = self.corridor_blocked.lock() {
            let next: usize = match counts.get(&key) {
                Some(n) => n + 1usize,
                None => 1usize,
            };
            counts.insert(key.clone(), next);
        }
        if let Ok(mut cells) = self.corridor_blocked_cells.lock() {
            if cells.len() < 4000 {
                cells.push(CreateCorridorBlockCell { cell, name: key });
            }
        }
    }

    /// A generator, decorator or carver write was refused because it lands in a
    /// declared Create corridor: the corridor stays clear and the accounting
    /// names the block that would have paved it. Refused means prevented - the
    /// block never entered the world - so these counts are separate from the
    /// `blocked` cells that still stand in a corridor.
    pub(crate) fn note_guard_outcome(&self, cell: (i32, i32, i32), name: &str) {
        let _ = cell;
        let key = name.to_string();
        if let Ok(mut counts) = self.corridor_refused.lock() {
            let next: usize = match counts.get(&key) {
                Some(n) => n + 1usize,
                None => 1usize,
            };
            counts.insert(key, next);
        }
    }

    /// `track_corridors.json` text for the whole run: how many corridor cells
    /// exist, which blocks their clearing removed and refused, and where.
    pub(crate) fn sidecar_corridors(&self) -> String {
        if let Ok(cleared) = self.corridor_cleared.lock() {
            if let Ok(blocked) = self.corridor_blocked.lock() {
                if let Ok(refused) = self.corridor_refused.lock() {
                    if let Ok(cells) = self.corridor.lock() {
                        if let Ok(blocked_cells) = self.corridor_blocked_cells.lock() {
                            if cleared.is_empty() && blocked.is_empty() && refused.is_empty() {
                                return "".to_string();
                            }
                            return serde_json::to_string(&CreateCorridorsSidecar {
                                corridor_cells: cells.len(),
                                cleared: cleared.clone(),
                                blocked: blocked.clone(),
                                blocked_cells: blocked_cells.clone(),
                                refused: refused.clone(),
                            })
                            .unwrap_or("".to_string());
                        }
                    }
                }
            }
        }
        "".to_string()
    }

    /// `track_rejections.json` text for the whole run; empty when nothing was
    /// refused.
    pub(crate) fn sidecar_rejections(&self) -> String {
        if let Ok(j) = self.rejections.lock() {
            if j.is_empty() {
                return "".to_string();
            }
            return serde_json::to_string(&CreateRejectionsSidecar {
                rejections: j.clone(),
            })
            .unwrap_or("".to_string());
        }
        "".to_string()
    }
}

/// How close (radians) a leg may be to an axis or a 45° diagonal and still be
/// laid with straight blocks instead of a smoothed span (~5 degrees).
const CREATE_SNAP_TOL: f64 = 0.09;

/// Largest xz offset a snapped leg may be off its end node and still be joined
/// by real cells. `snap_dir` accepts up to `CREATE_SNAP_TOL` radians, so a long
/// chord can miss its node by several cells: 5 degrees over 200 cells is 17.
const CREATE_LEG_JOIN_MAX: i32 = 4;
const CREATE_PI: f64 = std::f64::consts::PI;
const CREATE_TAU: f64 = std::f64::consts::TAU;
const CREATE_SQRT2: f64 = std::f64::consts::SQRT_2;

fn angle_near(a: f64, b: f64) -> f64 {
    let mut d = (a - b).abs() % CREATE_TAU;
    if d > CREATE_PI {
        d = CREATE_TAU - d;
    }
    d
}

/// Canonical direction (axis / 45°) of a vector, snapped only when the true
/// direction is within `tol` radians of it; (0,0) means "needs a span".
fn snap_dir(dx: i32, dz: i32, tol: f64) -> (i32, i32) {
    if dx == 0 && dz == 0 {
        return (0, 0);
    }
    let ang = (dz as f64).atan2(dx as f64);
    let dirs: [(i32, i32); 8] = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ];
    let mut best: (i32, i32) = (0, 0);
    let mut best_d = tol;
    for u in dirs {
        let d = angle_near(ang, (u.1 as f64).atan2(u.0 as f64));
        if d < best_d {
            best_d = d;
            best = u;
        }
    }
    best
}

/// Nearest canonical direction, regardless of distance (for blockstate shapes).
fn nearest_unit_v(vx: f64, vz: f64) -> (i32, i32) {
    if vx == 0.0 && vz == 0.0 {
        return (1, 0);
    }
    snap_dir(
        (vx * 1000.0).round() as i32,
        (vz * 1000.0).round() as i32,
        CREATE_TAU,
    )
}

/// Bezier tangent axes of a span: the travel direction of the leg, matching the
/// hand-built reference worlds (`Start = anchor + (0.5,0,0.5) + 0.5 * travel`).
fn span_axes(vx: f64, vz: f64) -> (f64, f64, f64) {
    (vx, 0.0, vz)
}

/// Canonical (x,z) axis a Create straight shape travels along: `TrackBlock
/// .getConnected` walks `getTrackAxes`, which is `TrackShape.getAxes()`, so the
/// shape names the one cell direction an anchor can leave through.
fn shape_unit(shape: &'static str) -> (i32, i32) {
    match shape {
        "xo" => (1, 0),
        "zo" => (0, 1),
        "pd" => (1, 1),
        "nd" => (-1, 1),
        "ae" => (1, 0),
        "aw" => (-1, 0),
        "an" => (0, -1),
        _ => (0, 1),
    }
}

/// Whether the chain cell `cell` leaves the anchor `anchor` OFF the axis its
/// shape names (both directions along that axis count as ON: references dump
/// `xo` chains going to (-1,0,0) and `zo` chains to (0,-1,0)).
fn joint_off_axis(anchor: (i32, i32, i32), cell: (i32, i32, i32), shape: &'static str) -> bool {
    let u = shape_unit(shape);
    let step = (sign_step(cell.0 - anchor.0), sign_step(cell.2 - anchor.2));
    step != u && step != (-u.0, -u.1)
}

/// Lay the rails of one way from its node geometry: long legs of straight
/// blocks, a smoothed Create span over every leg that is neither axial nor
/// 45°, and a turn span at each leg joint where the direction changes. This
/// is what keeps a real OSM railway drivable: a rasterized diagonal would be
/// a staircase of `xo`/`zo` cells that Create cannot connect end to end.
/// The planned leg cells of a way: `simplify_leg_nodes` -> legs -> cells,
/// exactly the geometry `plan_create_legs` lays rails on (bridging cells at
/// leg joins included). Pure function of the node list, so the presurvey can
/// claim the corridor band around the REAL track cells instead of the bed
/// raster (measured on osm-rail41: the raster deviates laterally on smoothed
/// diagonals, and ground/decoration wrote plants into cells the raster-based
/// presurvey never claimed - `claimed_raw=false` on all 115 violations while
/// every one sits one Chebyshev step from an actual track cell).
pub(crate) fn planned_leg_cells(
    editor: &mut WorldEditor,
    osm_id: u64,
    nodes: &[(i32, i32)],
) -> Vec<(i32, i32)> {
    let pts = simplify_leg_nodes(nodes);
    if pts.len() < 2 {
        return vec![];
    }
    legs_from_points(editor, osm_id, &pts)
        .iter()
        .flat_map(|leg| leg.cells.iter().copied())
        .collect()
}

fn legs_from_points(editor: &mut WorldEditor, osm_id: u64, pts: &[(i32, i32)]) -> Vec<CreateLeg> {
    let mut legs: Vec<CreateLeg> = vec![];
    let mut prev_last: Option<(i32, i32)> = None;
    let nlegs = pts.len() - 1;
    for i in 0..nlegs {
        let from = pts[i];
        let to = pts[i + 1];
        let (dx, dz) = (to.0 - from.0, to.1 - from.1);
        let snap = snap_dir(dx, dz, CREATE_SNAP_TOL);
        let straight = snap.0 != 0 || snap.1 != 0;
        let len = (dx as f64).hypot(dz as f64);
        let inv = if len > 0.0 { 1.0 / len } else { 0.0 };
        let vx = (dx as f64) * inv;
        let vz = (dz as f64) * inv;
        let diag = snap.0 != 0 && snap.1 != 0;
        let steps_diag = (len / CREATE_SQRT2).round() as i32;
        let steps_ax = dx.abs().max(dz.abs());
        let steps = if diag { steps_diag } else { steps_ax };
        let to_eff = if straight {
            (from.0 + snap.0 * steps, from.1 + snap.1 * steps)
        } else {
            to
        };
        let mut cells = leg_cells(from, to_eff);
        if let Some(p) = prev_last {
            if !cells.is_empty() && cells[0] == p {
                cells.remove(0);
            } else if !cells.is_empty() {
                let head = cells[0];
                let bridge = bridge_leg_cells(p, head);
                let adjacent = (head.0 - p.0).abs() <= 1 && (head.1 - p.1).abs() <= 1;
                if bridge.is_empty() && !adjacent {
                    note_create_rejection(
                        editor,
                        osm_id,
                        CreateReject::LegJoinTooFar,
                        (p.0, anchor_y(editor, p), p.1),
                        (head.0, anchor_y(editor, head), head.1),
                        0,
                        (len + 0.5) as usize,
                    );
                }
                for (i, b) in bridge.iter().enumerate() {
                    cells.insert(i, *b);
                }
            }
        }
        if cells.is_empty() {
            continue;
        }
        prev_last = Some(cells[cells.len() - 1]);
        let osm_span = (len + 0.5) as usize;
        if straight {
            legs.push(CreateLeg {
                cells,
                unit: snap,
                straight,
                vx,
                vz,
                osm_span,
            });
            continue;
        }
        let mut s: usize = 0;
        while s < cells.len() {
            let e = (s + CREATE_MAX_SPAN).min(cells.len());
            legs.push(CreateLeg {
                cells: cells[s..e].to_vec(),
                unit: (0, 0),
                straight,
                vx,
                vz,
                osm_span: e - s,
            });
            s = e;
        }
    }

    legs
}

pub(crate) fn plan_create_legs(editor: &mut WorldEditor, nodes: &[(i32, i32)], osm_id: u64) {
    let pts = simplify_leg_nodes(nodes);
    if pts.len() < 2 {
        return;
    }
    let legs = legs_from_points(editor, osm_id, &pts);

    let mut owned: HashSet<(i32, i32)> = HashSet::new();
    // Anchor cells claimed by this way: two spans must never share an anchor cell,
    // the second `Connections` would overwrite the first and the pair stops mirroring
    // (a Create train then derails at that joint).
    let mut used: HashSet<(i32, i32)> = HashSet::new();
    // Every cell this way ACTUALLY wrote, in writing order: anchors, chain
    // bodies and straight bodies. `track_ways.json` reports it next to the
    // planned cells, because the Create raster (Bézier chains, `nd`/`pd` shapes
    // that step diagonally) does not sit on the planned OSM staircase, and a
    // carriage rides the written cells, not the planned ones.
    let mut written: Vec<(i32, i32, i32)> = vec![];

    // Consecutive legs that no straight block can follow become ONE PolyBezier
    // span through the joints: two spans must never share an anchor cell, or the
    // second `Connections` overwrites the first and the pair stops mirroring.
    let mut i: usize = 0;
    while i < legs.len() {
        if legs[i].straight {
            i += 1;
            continue;
        }
        let mut j = i;
        while j + 1 < legs.len() && !legs[j + 1].straight && (j + 1 - i) < CREATE_MAX_RUN_LEGS {
            j += 1;
        }
        let a = &legs[i];
        let b = &legs[j];
        let (fax, faz) = a.cells[0];
        let (tbx, tbz) = b.cells[b.cells.len() - 1];
        let fa = (fax, faz);
        let tb = (tbx, tbz);
        let too_short = (tbx - fax).abs() + (tbz - faz).abs() < 2;
        let clash = used.contains(&fa) || used.contains(&tb);
        if too_short {
            i = j + 1;
            continue;
        }
        if clash {
            // Unresolved overlap between spans: the earlier span owns this anchor
            // cell and a second `Connections` in the same cell would silently
            // overwrite the first. Nothing gets written here, and the checker
            // sees the coordinates in the rejection sidecar.
            let (fa3, tb3) = (
                (fa.0, anchor_y(editor, fa), fa.1),
                (tb.0, anchor_y(editor, tb), tb.1),
            );
            note_create_rejection(
                editor,
                osm_id,
                CreateReject::OverlapAnchorBusy,
                fa3,
                tb3,
                0,
                0,
            );
            i = j + 1;
            continue;
        }
        if !write_run_of_legs(editor, osm_id, &legs, i, j, &mut written) {
            // Clean gap: the validator refused every attempt before writing
            // anything, so the carriage stops at the previous anchor instead of
            // running into an overshoot loop. The rejection already owns its row
            // in `track_rejections.json` with coordinates and reason.
            i = j + 1;
            continue;
        }
        used.insert(fa);
        used.insert(tb);
        let last = j + 1;
        for leg in &legs[i..last] {
            for c in &leg.cells {
                owned.insert(*c);
            }
        }
        i = j + 1;
    }

    // Turn spans at joints where the canonical direction changes.
    for i in 1..legs.len() {
        let (a, b) = (&legs[i - 1], &legs[i]);
        if a.unit == b.unit || a.unit == (0, 0) || b.unit == (0, 0) {
            continue;
        }
        let back = free_tail(&a.cells, &owned);
        let fwd = free_head(&b.cells, &owned);
        let r = back.min(fwd).min(CREATE_TURN_BACKOFF);
        if r < 1 {
            continue;
        }
        let ca = a.cells[a.cells.len() - 1 - r];
        let cb = b.cells[r];
        let (ya, yb) = span_anchor_ys(
            editor,
            osm_id,
            ca,
            anchor_y(editor, ca),
            cb,
            anchor_y(editor, cb),
            r,
        );
        if used.contains(&ca) || used.contains(&cb) {
            // Same overlap rule as run spans: keep the earlier span, log the cell.
            note_create_rejection(
                editor,
                osm_id,
                CreateReject::OverlapAnchorBusy,
                (ca.0, ya, ca.1),
                (cb.0, yb, cb.1),
                0,
                2 * r,
            );
            continue;
        }
        let conn = CreateConnection {
            a: (ca.0, ya, ca.1),
            b: (cb.0, yb, cb.1),
            shape_a: straight_shape(a.unit),
            shape_b: straight_shape(b.unit),
            axes_a: curve_axes(a.unit, false),
            axes_b: curve_axes(b.unit, true),
            osm_cells: 2 * r,
        };
        let ends = vec![conn.a, conn.b];
        let group = editor.create_rails_report().group_for(osm_id, &ends);
        if !create_anchor_free(editor, osm_id, group, (ca.0, ya, ca.1))
            || !create_anchor_free(editor, osm_id, group, (cb.0, yb, cb.1))
        {
            continue;
        }
        used.insert(ca);
        used.insert(cb);
        if !emit_create_connection(editor, osm_id, conn, &mut written) {
            used.remove(&ca);
            used.remove(&cb);
            continue;
        }
        // The anchor cell itself belongs to the turn span too: one less than on
        // the B side, where `head_to` already reaches `cb`. Without it the straight
        // pass would try to lay a body over the anchor and the whole leg claim
        // would be refused.
        let tail_from = a.cells.len() - 1 - r;
        for k in tail_from..a.cells.len() {
            owned.insert(a.cells[k]);
        }
        let head_to = r + 1;
        for k in 0..head_to {
            owned.insert(b.cells[k]);
        }
    }

    // Straight blocks over the remaining cells of every straight leg, ONE LEG AT
    // A TIME AND ALL-OR-NOTHING: the whole leg is claimed in `CreateRailReport`
    // before its first block is written, so a leg that would have to replace
    // another span's `create:track` or `create:fake_track` cells is skipped as a
    // whole and logged, never laid cell by cell over them (that would leave an
    // anchor without its `Connections`, or a chain cell explained by two spans).
    for leg in &legs {
        if !leg.straight {
            continue;
        }
        let mut want: Vec<((i32, i32, i32), bool)> = vec![];
        for k in 0..leg.cells.len() {
            let c = leg.cells[k];
            if owned.contains(&c) {
                continue;
            }
            want.push(((c.0, anchor_y(editor, c), c.1), true))
        }
        if want.is_empty() {
            continue;
        }
        let lend0 = leg.cells[0];
        let lend1 = leg.cells[leg.cells.len() - 1];
        let ends = vec![
            (lend0.0, anchor_y(editor, lend0), lend0.1),
            (lend1.0, anchor_y(editor, lend1), lend1.1),
        ];
        let group = editor.create_rails_report().group_for(osm_id, &ends);
        if let Some((cell, owner)) =
            editor
                .create_rails_report()
                .claim(&want, group, osm_id, leg.cells.len())
        {
            note_create_rejection(
                editor,
                osm_id,
                overlap_rule(&owner, leg.cells.len()),
                cell,
                cell,
                0,
                leg.cells.len(),
            );
            continue;
        }
        // Continuous y along the leg instead of the raw terrain level: Create
        // has exactly 20 `TrackShape`s (dumped `TrackShape.txt`: NONE ZO XO PD
        // ND AN AS AE AW TN TS TE TW CR_* — and the only ascending ones are the
        // axial AN/AS/AE/AW), so a DIAGONAL leg that follows terrain columns
        // straight breaks where the terrace steps by one block: dumped
        // `osm-rail20` left a flat `nd` at (211, -1570, -548) beside a straight
        // at (212, -1571, -549), and no shape pair joins a diagonal step with a
        // height change. Carry the level through the leg instead: ±1 per axial
        // step, never a height change on a diagonal step, seeded from the span
        // cell already written next to the leg head so the joint is continuous.
        let mut ys: Vec<i32> = vec![];
        let head = leg.cells[0];
        let head_y = anchor_y(editor, head);
        let mut seed: Option<i32> = None;
        for w in written.iter() {
            let dx = (w.0 - head.0).abs();
            let dz = (w.2 - head.1).abs();
            if dx + dz == 1 && (w.1 - head_y).abs() <= 1 {
                seed = Some(w.1)
            }
        }
        // A DIAGONAL written neighbour pins the head to its exact level: the
        // only ascending `TrackShape`s are axial — AN (0,1,-1), AS (0,1,1),
        // AE (1,1,0), AW (-1,1,0) — so a step that changes x AND z can never
        // change y, and a head taken from raw terrain one block off makes such
        // a step. Dumped `osm-rail20` left `ANCHORpd` at (-186,-1562,-306)
        // beside `nd` at (-187,-1563,-307) (step (-1,-1,-1)) and 19 of its 25
        // walk components were 1 block apart this way, so the joint that seeds
        // a leg is exactly where the height change has to be forbidden.
        for w in written.iter() {
            if (w.0 - head.0).abs() == 1 && (w.2 - head.1).abs() == 1 {
                seed = Some(w.1)
            }
        }
        for k in 0..leg.cells.len() {
            let c = leg.cells[k];
            let want = editor.get_ground_level(c.0, c.1) + 1;
            let y = if k == 0 {
                match seed {
                    Some(s) => s,
                    None => want,
                }
            } else {
                let p = leg.cells[k - 1];
                let diag = (c.0 - p.0).abs() == 1 && (c.1 - p.1).abs() == 1;
                if diag {
                    ys[k - 1]
                } else {
                    let d = (want - ys[k - 1]).clamp(-1, 1);
                    ys[k - 1] + d
                }
            };
            ys.push(y)
        }
        for k in 0..leg.cells.len() {
            let c = leg.cells[k];
            // Cells outside this tile's strict bounds belong to a neighbouring
            // tile editor, which draws the same leg there with the same plan
            // (the claims ledger is shared and the group id is keyed by the leg
            // itself, so neither tile sees the other as a conflict). Writing
            // them here would be a second, redundant `create:track` blockstate.
            if !editor.owns(c.0, c.1) || owned.contains(&c) {
                continue;
            }
            let prev = if k > 0 { Some(leg.cells[k - 1]) } else { None };
            let next = if k + 1 < leg.cells.len() {
                Some(leg.cells[k + 1])
            } else {
                None
            };
            let ry = ys[k];
            let _ground = ry - 1;
            let occupied =
                editor.check_for_block_absolute(c.0, ry, c.1, Some(&[CREATE_TRACK]), None)
                    || editor.check_for_block_absolute(
                        c.0,
                        ry + 1,
                        c.1,
                        Some(&[CREATE_TRACK]),
                        None,
                    )
                    || editor.check_for_block_absolute(
                        c.0,
                        ry - 1,
                        c.1,
                        Some(&[CREATE_TRACK]),
                        None,
                    );
            if occupied {
                continue;
            }
            // The SHAPE has to agree with the y actually written, because
            // `TrackBlock.getConnected` walks `Axes`: the dumped `TrackShape`
            // constructors give ONE axis each for the ascending shapes — AN
            // (0,1,-1), AS (0,1,1), AE (1,1,0), AW (-1,1,0) — so an ascending
            // block reaches exactly one cell one BELOW and one cell one ABOVE
            // its own position, with the horizontal component pointing at the
            // higher side. Picking the kind from the raw terrain (`determine_rail_with_slope`,
            // written for vanilla `minecraft:rail`, which puts the slope block
            // BELOW the step) disagrees with that rule by one cell: dumped
            // `osm-rail20` had `zo` at (-128,-1562,526) beside `as` at
            // (-128,-1562,527) — `as` reaches (526,-1563), not the `zo`, and the
            // carriage stops. Choose the shape from `ys` instead: ascend
            // whenever a written neighbour is one above this cell.
            let y_prev = if k > 0 { ys[k - 1] } else { ry };
            let y_next = if k + 1 < leg.cells.len() {
                ys[k + 1]
            } else {
                ry
            };
            let up: (i32, i32) = if y_next > ry && y_prev <= ry {
                (
                    sign_step(next.unwrap().0 - c.0),
                    sign_step(next.unwrap().1 - c.1),
                )
            } else if y_prev > ry && y_next <= ry {
                (
                    sign_step(prev.unwrap().0 - c.0),
                    sign_step(prev.unwrap().1 - c.1),
                )
            } else {
                (0, 0)
            };
            let shape: &'static str = match up {
                (0, 1) => "as",
                (0, -1) => "an",
                (1, 0) => "ae",
                (-1, 0) => "aw",
                _ => straight_shape(leg.unit),
            };
            let props: BlockWithProperties =
                BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape(shape)));
            // Written ABSOLUTE at the same y the anchors and chain cells use
            // (`get_ground_level+1`), not terrain-relative y=1: `get_absolute_y`
            // takes the top of the column, which is one block above the ground
            // wherever vegetation or a building sits on the way. Dumped proof in
            // `osm-rail20`: 702 of 3748 planned cells resolved with a y offset of
            // 1, and a straight at (212, -1571, -549) left its chain neighbour
            // (211, -1570, -548) one block above, a joint the carriage cannot
            // pass because a flat `aw` shape never steps vertically.
            editor.set_block_with_properties_absolute(props, c.0, ry, c.1, None, None);
            written.push((c.0, ry, c.1))
        }
    }

    // What this way was MEANT to cover, as its own row of `track_ways.json`, so
    // the checker scores every OSM way separately and a gap in one way is a
    // visible share instead of an average over the whole network. OSM ways are
    // never merged: one row per way, keyed by the OSM id.
    let mut planned: Vec<(i32, i32, i32)> = vec![];
    for leg in &legs {
        for c in &leg.cells {
            planned.push((c.0, anchor_y(editor, (c.0, c.1)), c.1));
        }
    }
    // Self-audit of the written footprint against the planned corridor: a run of
    // planned cells with NO written cell near them is a hole the carriage rides
    // into. Logging the hole with the written cells on either side (instead of
    // blaming one validator threshold) keeps every gap checkable: the checker
    // pairs each gap in `PER-OSM-WAY` with a row here.
    let paved = |c: (i32, i32, i32)| -> bool {
        written
            .iter()
            .any(|w| (w.0 - c.0).abs() <= 1 && (w.2 - c.2).abs() <= 1 && (w.1 - c.1).abs() <= 4)
    };
    let mut covered: HashSet<(i32, i32, i32)> = HashSet::new();
    for c in planned.iter() {
        if paved(*c) || editor.create_rails_report().covered_near(*c) {
            covered.insert(*c);
        }
    }
    let near_written = |c: (i32, i32, i32)| -> bool { covered.contains(&c) };
    let mut k = 0;
    while k < planned.len() {
        if near_written(planned[k]) {
            k += 1;
            continue;
        }
        let mut e = k;
        while e < planned.len() && !near_written(planned[e]) {
            e += 1
        }
        let before = if k > 0 { planned[k - 1] } else { planned[k] };
        let after = if e < planned.len() {
            planned[e]
        } else {
            planned[e - 1]
        };
        note_create_rejection(
            editor,
            osm_id,
            CreateReject::CoverageGap,
            before,
            after,
            e - k,
            e - k,
        );
        k = e + 1
    }
    // Self-audit of the shape axes against the written geometry: a pair of
    // written cells that step DIAGONALLY in x and z at different heights is a
    // joint no `TrackShape` can join (the ascending shapes are axial only), i.e.
    // a hole the carriage rides into. Logged, not guessed, so the checker can
    // pair it with the walk component it splits.
    {
        let mut paved_cells: HashSet<(i32, i32, i32)> = HashSet::new();
        for w in written.iter() {
            paved_cells.insert(*w);
        }
        for w in written.iter() {
            let c = *w;
            for dx in [-1, 1] {
                for dz in [-1, 1] {
                    let n = (c.0 + dx, c.1 + 1, c.2 + dz);
                    if paved_cells.contains(&n) {
                        note_create_rejection(
                            editor,
                            osm_id,
                            CreateReject::DiagonalStepHeightChange,
                            c,
                            n,
                            0,
                            written.len(),
                        )
                    }
                }
            }
        }
    }
    if !planned.is_empty() {
        editor.create_rails_report().note_way(CreateWayRow {
            osm_id,
            cells: planned.to_vec(),
            written: written.to_vec(),
        });
    }
}

/// True when a Create curve anchor CAN own this absolute cell. The cell is
/// True when a Create curve anchor CAN own this absolute cell: rejected only
/// when another write group already claims it as a real `create:track` cell
/// (anchor or straight body) — one cell cannot host two `Connections`, the
/// later one would win the per-cell entity dedup and the earlier pair would
/// stop mirroring (a train derails there). A `create:fake_track` body of an
/// earlier chain is NOT a conflict: `emit_create_curve` whitelists that block
/// as replaceable, and the anchor entity inserted later wins the dedup over the
/// fake entity it shared the cell with. The answer comes from the shared claims
/// ledger rather than from the editor's block view, because a tile editor only
/// ever sees the cells it owns: a block-view check makes the plan depend on
/// which tile runs first, so the same span end turns into an anchor in one tile
/// and a plain leg body in the next, and the carriage derails at the seam.
fn create_anchor_free(editor: &WorldEditor, osm_id: u64, group: u64, c: (i32, i32, i32)) -> bool {
    !editor.create_rails_report().anchor_busy(osm_id, group, c)
}

/// Cells free at the far end of a run, i.e. not owned by a curve already.
fn free_tail(cells: &[(i32, i32)], owned: &HashSet<(i32, i32)>) -> usize {
    let n = cells.len();
    let mut k = 0;
    while k + 1 < n && !owned.contains(&cells[n - 1 - k]) {
        k += 1;
    }
    k
}

fn free_head(cells: &[(i32, i32)], owned: &HashSet<(i32, i32)>) -> usize {
    let n = cells.len();
    let mut k = 0;
    while k + 1 < n && !owned.contains(&cells[k]) {
        k += 1;
    }
    k
}

/// State properties for a straight Create track cell (`turn = false`, so no
/// block entity is written).
pub(crate) fn at_grade_rail_props(kind: RailKind) -> BlockWithProperties {
    match kind {
        RailKind::StraightX => {
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("xo")))
        }
        RailKind::StraightZ => {
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("zo")))
        }
        RailKind::AscendEast => {
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("ae")))
        }
        RailKind::AscendWest => {
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("aw")))
        }
        RailKind::AscendNorth => {
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("an")))
        }
        RailKind::AscendSouth => {
            BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("as")))
        }
        // Turns have no straight representation: they are always spanned by a
        // smoothed curve that writes its own anchors.
        _ => BlockWithProperties::new(CREATE_TRACK, Some(create_track_shape("xo"))),
    }
}

/// Choose between a flat or ascending rail block based on the ground-level
/// difference between the previous, current, and next track points.
/// Selects the rail block for one cell from its (prev, current, next)
/// context and the ground levels. `pub(crate)` so the track test world
/// (`src/track_test_world.rs`) places rails exactly as the pipeline does.
pub(crate) fn determine_rail_with_slope(
    current: (i32, i32),
    prev: Option<(i32, i32)>,
    next: Option<(i32, i32)>,
    prev_ground: i32,
    current_ground: i32,
    next_ground: i32,
) -> RailKind {
    // Ascending toward the *higher* neighbour.
    if next_ground > current_ground {
        if let Some((nx, nz)) = next {
            return ascending_toward(current, (nx, nz));
        }
    }
    if prev_ground > current_ground {
        if let Some((px, pz)) = prev {
            return ascending_toward(current, (px, pz));
        }
    }
    // Flat section – fall back to standard direction logic.
    determine_rail_direction(current, prev, next)
}

/// Return the ascending kind that climbs from `from` toward `to`.
fn ascending_toward(from: (i32, i32), to: (i32, i32)) -> RailKind {
    let (fx, fz) = from;
    let (tx, tz) = to;
    let dx = tx - fx;
    let dz = tz - fz;
    if dx.abs() >= dz.abs() {
        if dx > 0 {
            RailKind::AscendEast
        } else {
            RailKind::AscendWest
        }
    } else if dz < 0 {
        RailKind::AscendNorth
    } else {
        RailKind::AscendSouth
    }
}

pub(crate) fn smooth_diagonal_rails(points: &[(i32, i32, i32)]) -> Vec<(i32, i32, i32)> {
    let mut smoothed = Vec::new();

    for i in 0..points.len() {
        let current = points[i];
        smoothed.push(current);

        if i + 1 >= points.len() {
            continue;
        }

        let next = points[i + 1];
        let (x1, y1, z1) = current;
        let (x2, _, z2) = next;

        // If points are diagonally adjacent
        if (x2 - x1).abs() == 1 && (z2 - z1).abs() == 1 {
            // Look ahead to determine best intermediate point
            let look_ahead = if i + 2 < points.len() {
                Some(points[i + 2])
            } else {
                None
            };

            // Look behind
            let look_behind = if i > 0 { Some(points[i - 1]) } else { None };

            // Choose intermediate point based on the overall curve direction
            let intermediate = if let Some((prev_x, _, _prev_z)) = look_behind {
                if prev_x == x1 {
                    // Coming from vertical, keep x constant
                    (x1, y1, z2)
                } else {
                    // Coming from horizontal, keep z constant
                    (x2, y1, z1)
                }
            } else if let Some((next_x, _, _next_z)) = look_ahead {
                if next_x == x2 {
                    // Going to vertical, keep x constant
                    (x2, y1, z1)
                } else {
                    // Going to horizontal, keep z constant
                    (x1, y1, z2)
                }
            } else {
                // Default to horizontal first if no context
                (x2, y1, z1)
            };

            smoothed.push(intermediate);
        }
    }

    smoothed
}

fn determine_rail_direction(
    current: (i32, i32),
    prev: Option<(i32, i32)>,
    next: Option<(i32, i32)>,
) -> RailKind {
    let (x, z) = current;

    match (prev, next) {
        (Some((px, pz)), Some((nx, nz))) => {
            if px == nx {
                RailKind::StraightZ
            } else if pz == nz {
                RailKind::StraightX
            } else {
                // Calculate relative movements
                let from_prev = (px - x, pz - z);
                let to_next = (nx - x, nz - z);

                match (from_prev, to_next) {
                    // East to North or North to East
                    ((-1, 0), (0, -1)) | ((0, -1), (-1, 0)) => RailKind::TurnNorthWest,
                    // West to North or North to West
                    ((1, 0), (0, -1)) | ((0, -1), (1, 0)) => RailKind::TurnNorthEast,
                    // East to South or South to East
                    ((-1, 0), (0, 1)) | ((0, 1), (-1, 0)) => RailKind::TurnSouthWest,
                    // West to South or South to West
                    ((1, 0), (0, 1)) | ((0, 1), (1, 0)) => RailKind::TurnSouthEast,
                    _ => {
                        if (px - x).abs() > (pz - z).abs() {
                            RailKind::StraightX
                        } else {
                            RailKind::StraightZ
                        }
                    }
                }
            }
        }
        (Some((px, pz)), None) | (None, Some((px, pz))) => {
            if px == x {
                RailKind::StraightZ
            } else if pz == z {
                RailKind::StraightX
            } else {
                RailKind::StraightZ
            }
        }
        (None, None) => RailKind::StraightZ,
    }
}

/// How far a smoothed Create quarter turn may back off along its legs to reach
/// its anchors, in cells.
const CREATE_TURN_BACKOFF: usize = 8;

pub fn generate_roller_coaster(editor: &mut WorldEditor, element: &ProcessedWay) {
    if let Some(roller_coaster) = element.tags.get("roller_coaster") {
        if roller_coaster == "track" {
            // Check if it's indoor (skip if yes)
            if let Some(indoor) = element.tags.get("indoor") {
                if indoor == "yes" {
                    return;
                }
            }

            // Check if layer is negative (skip if yes)
            if let Some(layer) = element.tags.get("layer") {
                if let Ok(layer_value) = layer.parse::<i32>() {
                    if layer_value < 0 {
                        return;
                    }
                }
            }

            let elevation_height = 4; // 4 blocks in the air
            let pillar_interval = 6; // Support pillars every 6 blocks

            for i in 1..element.nodes.len() {
                let prev_node = element.nodes[i - 1].xz();
                let cur_node = element.nodes[i].xz();

                let points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
                let smoothed_points = smooth_diagonal_rails(&points);

                for j in 0..smoothed_points.len() {
                    let (bx, _, bz) = smoothed_points[j];

                    // Place track foundation at elevation height
                    editor.set_block(IRON_BLOCK, bx, elevation_height, bz, None, None);

                    let prev = if j > 0 {
                        Some(smoothed_points[j - 1])
                    } else {
                        None
                    };
                    let next = if j < smoothed_points.len() - 1 {
                        Some(smoothed_points[j + 1])
                    } else {
                        None
                    };

                    let rail_block = determine_rail_direction(
                        (bx, bz),
                        prev.map(|(x, _, z)| (x, z)),
                        next.map(|(x, _, z)| (x, z)),
                    );

                    // Place rail on top of the foundation
                    editor.set_block_with_properties(
                        at_grade_rail_props(rail_block),
                        bx,
                        elevation_height + 1,
                        bz,
                        None,
                        None,
                    );

                    // Place support pillars every pillar_interval blocks
                    if bx % pillar_interval == 0 && bz % pillar_interval == 0 {
                        // Create a pillar from ground level up to the track
                        for y in 1..elevation_height {
                            editor.set_block(IRON_BLOCK, bx, y, bz, None, None);
                        }
                    }
                }
            }
        }
    }
}

/// Phase 1 of underground railway generation: place the structural tunnel shell and rail
/// track.  Called during element processing (step 4) so that all non-AIR
/// blocks survive the underground stone fill in step 6.
///
/// Centerline points are collected into `rail_tunnel_points` for phase 2.
fn generate_rail_tunnel_shell(
    editor: &mut WorldEditor,
    element: &ProcessedWay,
    rail_tunnel_points: &mut Vec<(i32, i32)>,
) {
    for i in 1..element.nodes.len() {
        let prev_node = element.nodes[i - 1].xz();
        let cur_node = element.nodes[i].xz();

        let points = bresenham_line(prev_node.x, 0, prev_node.z, cur_node.x, 0, cur_node.z);
        let smoothed = smooth_diagonal_rails(&points);

        for j in 0..smoothed.len() {
            let (bx, _, bz) = smoothed[j];

            // Record centerline point for phase 2 air-carving, skipping
            // duplicate shared endpoints between adjacent segments.
            if rail_tunnel_points.last().copied() != Some((bx, bz)) {
                rail_tunnel_points.push((bx, bz));
            }

            let ground_y = editor.get_ground_level(bx, bz);
            let ceil_y = ground_y - RAIL_TUNNEL_DEPTH;
            let floor_y = ceil_y - INTERIOR_HEIGHT - 1;

            // Safety: skip if the tunnel would go below world minimum.
            if floor_y <= crate::world_editor::min_y() {
                continue;
            }

            // Ground levels at adjacent points, used for slope-aware rail
            // placement. Because the tunnel depth is fixed, surface-level
            // differences map 1:1 to rail-level differences.
            let prev_ground = if j > 0 {
                let (px, _, pz) = smoothed[j - 1];
                editor.get_ground_level(px, pz)
            } else {
                ground_y
            };
            let next_ground = if j + 1 < smoothed.len() {
                let (nx, _, nz) = smoothed[j + 1];
                editor.get_ground_level(nx, nz)
            } else {
                ground_y
            };

            // Place tunnel shell (5x5 footprint, full height).
            // Interior positions deliberately get non-AIR blocks too so that
            // the ground fill (skip_existing: true) leaves them alone.
            // Wall/ceiling blocks get random cracked/mossy variants for variety.
            for dx in -WALL_RADIUS..=WALL_RADIUS {
                for dz in -WALL_RADIUS..=WALL_RADIUS {
                    for y in floor_y..=ceil_y {
                        let is_wall_or_ceiling =
                            dx.abs() == WALL_RADIUS || dz.abs() == WALL_RADIUS || y == ceil_y;

                        let block = if y == floor_y {
                            // Entire floor row: polished deepslate
                            POLISHED_DEEPSLATE
                        } else if is_wall_or_ceiling {
                            // Visible wall/ceiling: mix in cracked and mossy
                            rail_tunnel_shell_block(bx + dx, y, bz + dz)
                        } else {
                            // Interior placeholder (carved in phase 2)
                            STONE_BRICKS
                        };
                        editor.set_block_absolute(block, bx + dx, y, bz + dz, None, None);
                    }
                }
            }

            // Place rail on the structural floor (one above floor_y).
            let prev_xz = if j > 0 {
                let (px, _, pz) = smoothed[j - 1];
                Some((px, pz))
            } else {
                None
            };
            let next_xz = if j + 1 < smoothed.len() {
                let (nx, _, nz) = smoothed[j + 1];
                Some((nx, nz))
            } else {
                None
            };

            let rail_block = determine_rail_with_slope(
                (bx, bz),
                prev_xz,
                next_xz,
                prev_ground,
                ground_y,
                next_ground,
            );
            // Whitelist: allow overwriting the STONE_BRICKS placeholder.
            editor.set_block_with_properties_absolute(
                at_grade_rail_props(rail_block),
                bx,
                floor_y + 1,
                bz,
                Some(&[STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS]),
                None,
            );
        }
    }
}

/// Phase 2 of underground railway generation: carve the 3x3 air interior and place
/// ceiling lights.  Called AFTER ground generation so that the carved
/// air blocks are not overwritten by the underground stone fill.
///
/// Every tile a tunnel crosses records the whole centerline, so the merged list holds each
/// point once per tile. A point is carved only the first time: carving just turns
/// whitelisted blocks to air, and a lantern is never on the whitelist, so a second carve
/// changes nothing. The lanterns still follow their positions in the full list.
pub fn carve_rail_tunnel_interior(editor: &mut WorldEditor, rail_tunnel_points: &[(i32, i32)]) {
    // Whitelist: allow overwriting shell blocks and ground-fill STONE
    // so the tunnel is actually hollow.
    let carve_whitelist: &[Block] = &[
        STONE_BRICKS,
        CRACKED_STONE_BRICKS,
        MOSSY_STONE_BRICKS,
        STONE,
    ];
    let mut carved: HashSet<(i32, i32)> = HashSet::with_capacity(rail_tunnel_points.len());
    for (idx, &(bx, bz)) in rail_tunnel_points.iter().enumerate() {
        let first = carved.insert((bx, bz));
        let lantern = idx % LIGHT_INTERVAL == 0;
        if !first && !lantern {
            continue;
        }
        let ground_y = editor.get_ground_level(bx, bz);
        let ceil_y = ground_y - RAIL_TUNNEL_DEPTH;
        let floor_y = ceil_y - INTERIOR_HEIGHT - 1;

        if floor_y <= crate::world_editor::min_y() {
            continue;
        }

        if first {
            for dx in -AIR_RADIUS..=AIR_RADIUS {
                for dz in -AIR_RADIUS..=AIR_RADIUS {
                    for y in (floor_y + 1)..ceil_y {
                        // Skip the center rail block.
                        if dx == 0 && dz == 0 && y == floor_y + 1 {
                            continue;
                        }
                        editor.set_block_absolute(
                            AIR,
                            bx + dx,
                            y,
                            bz + dz,
                            Some(carve_whitelist),
                            None,
                        );
                    }
                }
            }
        }

        // Periodic ceiling lighting.
        if lantern {
            editor.set_block_absolute(SEA_LANTERN, bx, ceil_y - 1, bz, None, None);
        }
    }
}

/// The two anchor heights a span may use: a span may climb at most ONE block,
/// because the straight run next to its anchors names each step with a slope
/// shape (`an`/`as`/`ae`/`aw`, axes dumped from tools/create_cls/TrackShape.txt
/// as (0,1,-1), (0,1,1), (1,1,0), (-1,1,0)), and Create has no shape for a step
/// that is diagonal in x,z AND changes height (117 such pairs in `osm-rail21`
/// against 0 in the mod-made reference worlds, e.g. (-53,-1566,-410) `ANCHORxo`
/// -> (-52,-1567,-411) for way 77908996). A terrain step wider than one block
/// is therefore flattened to the lower anchor and reported rather than climbed
/// inside the curve, which is what left 178 planned cells off the raster in
/// `osm-rail23`.
/// The height pair a span over `ca`..`cb` must be laid at. Terrain puts the two
/// anchor cells at their own `get_ground_level() + 1`, and Create has NO shape
/// for a step that is diagonal in x,z AND changes height: the ascending `Track-
/// Shape`s dumped from tools/create_cls/TrackShape.txt are `AN` (0,1,-1), `AS`
/// (0,1,1), `AE` (1,1,0), `AW` (-1,1,0) — every one of them axial in x,z — and
/// `TrackBlock.getConnected` walks a `turn=false` cell only along `TrackShape.
/// getAxes()` (tools/create_cls/TrackBlock.txt), so such a step has nothing to
/// walk. Measured on a freshly generated `osm-rail24` (log tmp/logs/chk_osm-rail24
/// .log, probe tmp/shape_probe.py): 117 pairs of written cells one apart in bot
/// h x,z at a height difference of 1, while the reference worlds `create-track-
/// test`/`track-stage2` (user-laid Create) have 0, so a rising span is the only
/// thing that invents them. A span whose terrain anchors differ is therefore la
/// id FLAT at the lower of the two heights and the climb moves into the straigh
/// t legs, whose shape choice does name an axial step up/down (`as`/`an`/`ae`/
/// `aw`); the flattened rise gets a row in `track_rejections.json` so the effec
/// t of the rule stays visible.
///
/// NOT USED, and it must not be wired back in without a per-span condition: MEA
/// SURED, flattening EVERY rise (also the 1-block rises `span_anchor_ys` keeps)
/// regressed the walk — osm-rail30 (log tmp/logs/chk_osm-rail30.log) went to 58
/// DRIVABLE WALK components / longest run 276 blocks against the 28 components
/// / 661 blocks the baseline `span_anchor_ys` state measures, because a 1-block
/// rise is legal in Create when the chain climbs along an AXIAL x,z direction na
/// med by `AN`/`AS`/`AE`/`AW`, and flattening destroys the geometry of spans suc
/// h as start [3,-1568,-430] end [44,-1566,-448] (dx 41, dz -18, dy 2: oblique,
/// not axial). Keep it only as the fallback for spans whose chain provably steps
/// diagonally in x,z while changing height.
#[allow(dead_code)]
fn flat_span_ys(
    editor: &mut WorldEditor,
    osm_id: u64,
    ca: (i32, i32),
    ya: i32,
    cb: (i32, i32),
    yb: i32,
) -> (i32, i32) {
    if ya == yb {
        return (ya, yb);
    }
    let low = if ya < yb { ya } else { yb };
    note_create_rejection(
        editor,
        osm_id,
        CreateReject::SpanAnchorRise,
        (ca.0, ya, ca.1),
        (cb.0, yb, cb.1),
        (yb - ya).unsigned_abs() as usize,
        0,
    );
    (low, low)
}

fn span_anchor_ys(
    editor: &mut WorldEditor,
    osm_id: u64,
    ca: (i32, i32),
    ya: i32,
    cb: (i32, i32),
    yb: i32,
    backoff: usize,
) -> (i32, i32) {
    let rise = (yb - ya).abs();
    if rise <= 1 {
        return (ya, yb);
    }
    let common = if ya < yb { ya } else { yb };
    note_create_rejection(
        editor,
        osm_id,
        CreateReject::SpanAnchorRise,
        (ca.0, ya, ca.1),
        (cb.0, yb, cb.1),
        rise as usize,
        2 * backoff,
    );
    (common, common)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::coordinate_system::geographic::LLBBox;
    use crate::osm_parser::ProcessedNode;
    use fastnbt::Value;
    use std::path::PathBuf;

    fn test_editor(xzbbox: &XZBBox) -> WorldEditor<'_> {
        let llbbox = LLBBox::new(54.6, 9.9, 54.61, 9.91).unwrap();
        WorldEditor::new(PathBuf::from("/dev/null/unused"), xzbbox, llbbox)
    }

    /// The `shape` state property of the block at (x, y-offset, z), if stored.
    fn track_shape(editor: &WorldEditor, x: i32, y: i32, z: i32) -> Option<String> {
        let absolute_y = editor.get_absolute_y(x, y, z);
        let Some(Value::Compound(props)) = editor.block_properties_absolute(x, absolute_y, z)
        else {
            return None;
        };
        match props.get("shape") {
            Some(Value::String(s)) => Some(s.clone()),
            _ => None,
        }
    }

    // The same carve as before deduplication: every listed point, in order.
    fn carve_every_point(editor: &mut WorldEditor, points: &[(i32, i32)]) {
        let wl: &[Block] = &[
            STONE_BRICKS,
            CRACKED_STONE_BRICKS,
            MOSSY_STONE_BRICKS,
            STONE,
        ];
        for (idx, &(bx, bz)) in points.iter().enumerate() {
            let ceil_y = editor.get_ground_level(bx, bz) - RAIL_TUNNEL_DEPTH;
            let floor_y = ceil_y - INTERIOR_HEIGHT - 1;
            for dx in -AIR_RADIUS..=AIR_RADIUS {
                for dz in -AIR_RADIUS..=AIR_RADIUS {
                    for y in (floor_y + 1)..ceil_y {
                        if !(dx == 0 && dz == 0 && y == floor_y + 1) {
                            editor.set_block_absolute(AIR, bx + dx, y, bz + dz, Some(wl), None);
                        }
                    }
                }
            }
            if idx % LIGHT_INTERVAL == 0 {
                editor.set_block_absolute(SEA_LANTERN, bx, ceil_y - 1, bz, None, None);
            }
        }
    }

    // Tiles list a tunnel once each; carving each point once must build the same bore,
    // lanterns included.
    #[test]
    fn repeated_tunnel_points_carve_the_same_bore() {
        let xzbbox = XZBBox::rect_from_min_max(0, 0, 80, 80).unwrap();
        let line: Vec<(i32, i32)> = (20..=50)
            .map(|x| (x, 40))
            .chain((41..=60).map(|z| (50, z)))
            .collect();
        // Two tiles' copies, the second starting mid-way, plus a stray repeat.
        let mut points = line.clone();
        points.extend_from_slice(&line[7..]);
        points.extend_from_slice(&line);
        points.push((33, 40));

        let build = |carve: &dyn Fn(&mut WorldEditor)| {
            let mut editor = test_editor(&xzbbox);
            for x in 10..=70 {
                for z in 30..=70 {
                    let ground = editor.get_ground_level(x, z);
                    for y in ground - 12..ground {
                        let block = if (x + z) % 5 == 0 { RAIL } else { STONE };
                        editor.set_block_absolute(block, x, y, z, None, None);
                    }
                }
            }
            carve(&mut editor);
            let mut cells = Vec::new();
            for x in 10..=70 {
                for z in 30..=70 {
                    let ground = editor.get_ground_level(x, z);
                    for y in ground - 12..ground {
                        cells.push(editor.get_block_absolute(x, y, z).map(|b| b.id()));
                    }
                }
            }
            cells
        };
        let want = build(&|e| carve_every_point(e, &points));
        let got = build(&|e| carve_rail_tunnel_interior(e, &points));
        assert_eq!(got, want);
        assert!(got.contains(&Some(SEA_LANTERN.id())));
    }

    /// A straight north-south rail from z=20 to z=60 at x=50.
    fn straight_rail_ns(tags: &[(&str, &str)]) -> ProcessedWay {
        let mut t = HashMap::new();
        for (k, v) in tags {
            t.insert(k.to_string(), v.to_string());
        }
        ProcessedWay {
            id: 2,
            nodes: vec![
                ProcessedNode {
                    id: 3,
                    tags: HashMap::new(),
                    x: 50,
                    z: 20,
                },
                ProcessedNode {
                    id: 4,
                    tags: HashMap::new(),
                    x: 50,
                    z: 60,
                },
            ],
            tags: t,
        }
    }

    // A straight east-west rail from x=20 to x=60 at z=50 (so cell index == x - 20).
    fn straight_rail(tags: &[(&str, &str)]) -> ProcessedWay {
        let mut t = HashMap::new();
        for (k, v) in tags {
            t.insert(k.to_string(), v.to_string());
        }
        ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 20,
                    z: 50,
                },
                ProcessedNode {
                    id: 2,
                    tags: HashMap::new(),
                    x: 60,
                    z: 50,
                },
            ],
            tags: t,
        }
    }

    fn run_collecting_tunnel_points(
        editor: &mut WorldEditor,
        way: &ProcessedWay,
        rail_mask: &CoordinateBitmap,
    ) -> Vec<(i32, i32)> {
        let clear = CoordinateBitmap::new(rail_mask_bbox());
        let mut rail_tunnel_points = Vec::new();
        let internal = HashSet::new();
        let outlines = BridgeOutlineIndex::build(&[]);
        let structures = BridgeStructureMap::build(&[], editor, &outlines, 1.0);
        generate_railways(
            editor,
            way,
            &mut rail_tunnel_points,
            &internal,
            &outlines,
            &structures,
            &BridgeSurfaceMap::empty(),
            &clear,
            &clear,
            rail_mask,
        );
        rail_tunnel_points
    }

    fn run(editor: &mut WorldEditor, way: &ProcessedWay, rail_mask: &CoordinateBitmap) {
        let _ = run_collecting_tunnel_points(editor, way, rail_mask);
    }

    fn rail_mask_bbox() -> &'static XZBBox {
        use std::sync::OnceLock;
        static BBOX: OnceLock<XZBBox> = OnceLock::new();
        BBOX.get_or_init(|| XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap())
    }

    #[test]
    fn electrified_rail_gets_catenary_over_a_plain_rail() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = straight_rail(&[
            ("railway", "rail"),
            ("usage", "main"),
            ("electrified", "contact_line"),
        ]);
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));

        assert!(
            editor.check_for_block(20, 1, 50, Some(&[CREATE_TRACK])),
            "straight at-grade rail is a Create track"
        );
        assert_eq!(
            track_shape(&editor, 20, 1, 50).as_deref(),
            Some("xo"),
            "shape follows the E-W axis"
        );
        assert!(
            !editor.check_for_block(20, 0, 49, Some(&[ANVIL])),
            "no decorative rails"
        );
        assert!(
            editor.check_for_block(20, CATENARY_WIRE_HEIGHT, 50, Some(&[CHAIN_X])),
            "contact wire"
        );
        assert!(
            editor.check_for_block(20, 3, 53, Some(&[COBBLESTONE_WALL])),
            "mast column"
        );
        assert!(
            editor.check_for_block(20, 0, 53, Some(&[GRAY_CONCRETE])),
            "mast foundation"
        );
    }

    #[test]
    fn tram_gets_no_catenary() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = straight_rail(&[("railway", "tram"), ("electrified", "contact_line")]);
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));

        assert!(
            editor.check_for_block(20, 1, 50, Some(&[CREATE_TRACK])),
            "tram keeps its straight rail as a Create track"
        );
        assert!(
            !editor.check_for_block(20, CATENARY_WIRE_HEIGHT, 50, Some(&[CHAIN_X, CHAIN_Z])),
            "catenary is heavy-rail only"
        );
    }

    #[test]
    fn heavy_rail_tunnel_builds_and_carves_an_underground_track() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = straight_rail(&[("railway", "rail"), ("tunnel", "yes")]);

        let rail_tunnel_points =
            run_collecting_tunnel_points(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));
        assert_eq!(rail_tunnel_points.len(), 41, "tunnel centerline collected");

        carve_rail_tunnel_interior(&mut editor, &rail_tunnel_points);

        let floor_y = -RAIL_TUNNEL_DEPTH - INTERIOR_HEIGHT - 1;
        assert!(
            editor.check_for_block_absolute(20, floor_y + 1, 50, Some(&[CREATE_TRACK]), None,),
            "rail is preserved on the tunnel floor"
        );
        assert!(
            !editor.check_for_block_absolute(
                24,
                floor_y + 2,
                50,
                Some(&[
                    STONE,
                    STONE_BRICKS,
                    CRACKED_STONE_BRICKS,
                    MOSSY_STONE_BRICKS,
                ]),
                None,
            ),
            "tunnel interior is carved after shell placement"
        );
        assert!(
            editor.check_for_block_absolute(
                24,
                floor_y + 2,
                52,
                Some(&[STONE_BRICKS, CRACKED_STONE_BRICKS, MOSSY_STONE_BRICKS]),
                None,
            ),
            "tunnel wall remains around the carved interior"
        );
        assert!(
            !editor.check_for_block(24, 1, 50, Some(&[CREATE_TRACK])),
            "tunnel rail is not rendered at grade"
        );
    }

    #[test]
    fn rail_tunnel_routing_accepts_only_track_ways() {
        assert!(renders_as_rail_tunnel(&straight_rail(&[
            ("railway", "rail"),
            ("tunnel", "yes"),
        ])));
        assert!(renders_as_rail_tunnel(&straight_rail(&[(
            "railway", "subway"
        )])));
        assert!(!renders_as_rail_tunnel(&straight_rail(&[
            ("railway", "rail"),
            ("tunnel", "yes"),
            ("area", "yes"),
        ])));
        assert!(!renders_as_rail_tunnel(&straight_rail(&[
            ("railway", "platform"),
            ("tunnel", "yes"),
        ])));
        assert!(!renders_as_rail_tunnel(&straight_rail(&[
            ("railway", "station"),
            ("subway", "yes"),
        ])));
    }

    #[test]
    fn elevated_subway_renders_as_bridge_not_tunnel() {
        let viaduct =
            straight_rail(&[("railway", "subway"), ("bridge", "viaduct"), ("layer", "2")]);
        assert!(!renders_as_rail_tunnel(&viaduct));
        assert!(renders_as_rail_bridge(&viaduct));

        let surface = straight_rail(&[("railway", "subway"), ("tunnel", "no")]);
        assert!(!renders_as_rail_tunnel(&surface));

        let bored = straight_rail(&[("railway", "subway"), ("bridge", "yes"), ("tunnel", "yes")]);
        assert!(renders_as_rail_tunnel(&bored));
        assert!(!renders_as_rail_bridge(&bored));
    }

    #[test]
    fn non_track_tunnel_ways_do_not_generate_rails() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        for way in [
            straight_rail(&[("railway", "rail"), ("tunnel", "yes"), ("area", "yes")]),
            straight_rail(&[("railway", "platform"), ("tunnel", "yes")]),
            straight_rail(&[("railway", "station"), ("subway", "yes")]),
        ] {
            let mut editor = test_editor(&xzbbox);
            let points =
                run_collecting_tunnel_points(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));
            assert!(points.is_empty(), "non-track way must not become a tunnel");
            assert!(
                !editor.check_for_block(20, 1, 50, Some(&[CREATE_TRACK])),
                "non-track way must not fall through to at-grade rail rendering"
            );
        }
    }

    #[test]
    fn rail_tunnel_footprint_covers_the_full_shell_only_for_tracks() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let non_tracks = vec![
            ProcessedElement::Way(straight_rail(&[("railway", "platform"), ("tunnel", "yes")])),
            ProcessedElement::Way(straight_rail(&[("railway", "station"), ("subway", "yes")])),
        ];
        let mut footprint = CoordinateBitmap::new_empty();
        add_tunnel_footprint(&non_tracks, &xzbbox, &mut footprint);
        assert!(
            footprint.is_empty(),
            "platform and station ways do not allocate a tunnel footprint"
        );

        footprint = CoordinateBitmap::new(&xzbbox);
        footprint.set(10, 10);
        let track = vec![ProcessedElement::Way(straight_rail(&[
            ("railway", "rail"),
            ("tunnel", "yes"),
        ]))];
        add_tunnel_footprint(&track, &xzbbox, &mut footprint);

        assert!(
            footprint.contains(10, 10),
            "existing footprint is preserved"
        );
        assert!(footprint.contains(40, 50), "track centerline is protected");
        assert!(footprint.contains(40, 52), "outer tunnel wall is protected");
        assert!(
            !footprint.contains(40, 53),
            "footprint stops outside the 5x5 shell"
        );
    }

    #[test]
    fn at_grade_rail_props_maps_every_cell_kind_to_create() {
        for (kind, shape) in [(RailKind::StraightZ, "zo"), (RailKind::StraightX, "xo")] {
            let props = at_grade_rail_props(kind);
            assert_eq!(props.block, CREATE_TRACK);
            let expected = Value::String(shape.to_string());
            let stored = match props.properties.as_deref() {
                Some(Value::Compound(m)) => m.get("shape"),
                _ => None,
            };
            assert_eq!(
                stored,
                Some(&expected),
                "straight rail carries its axis as shape"
            );
        }
        // Quarter turns and ascents have no single-cell representation: the
        // pipeline replaces their whole window with curve anchors plus a
        // fake-track chain, so a lone cell of those kinds still maps to Create
        // (never a vanilla rail block).
        for kind in [
            RailKind::TurnNorthEast,
            RailKind::TurnNorthWest,
            RailKind::TurnSouthEast,
            RailKind::TurnSouthWest,
            RailKind::AscendNorth,
            RailKind::AscendSouth,
            RailKind::AscendEast,
            RailKind::AscendWest,
        ] {
            let props = at_grade_rail_props(kind);
            assert_eq!(props.block, CREATE_TRACK, "curve cell never stays vanilla");
        }
    }

    #[test]
    fn at_grade_straight_rails_become_create_tracks() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();

        // East-west way: `create:track` with `shape=xo` on the gravel bed.
        let mut editor = test_editor(&xzbbox);
        let way = straight_rail(&[("railway", "rail")]);
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));
        assert!(
            editor.check_for_block(20, 1, 50, Some(&[CREATE_TRACK])),
            "E-W straight rail is a Create track"
        );
        assert_eq!(track_shape(&editor, 20, 1, 50).as_deref(), Some("xo"));
        assert!(
            editor.check_for_block(20, 0, 50, Some(&[GRAVEL, OAK_LOG])),
            "gravel bed / sleeper stays under the track"
        );

        // North-south way: `shape=zo`.
        let mut editor = test_editor(&xzbbox);
        let way = straight_rail_ns(&[("railway", "rail")]);
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));
        assert!(
            editor.check_for_block(50, 1, 20, Some(&[CREATE_TRACK])),
            "N-S straight rail is a Create track"
        );
        assert_eq!(track_shape(&editor, 50, 1, 20).as_deref(), Some("zo"));
    }

    /// True if the block at (x, y-offset, z) is a `create:track` curve anchor.
    fn curve_anchor_at(editor: &WorldEditor, x: i32, y: i32, z: i32) -> bool {
        let absolute_y = editor.get_absolute_y(x, y, z);
        let Some(Value::Compound(props)) = editor.block_properties_absolute(x, absolute_y, z)
        else {
            return false;
        };
        let Some(Value::String(s)) = props.get("turn") else {
            return false;
        };
        s == "true"
    }

    #[test]
    fn create_curve_anchors_exist_around_an_osm_corner() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 200.0).unwrap();
        let mut editor = test_editor(&xzbbox);

        // E-W leg (20,50)->(40,50) then north to (40,30).
        let way = ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 20,
                    z: 50,
                },
                ProcessedNode {
                    id: 2,
                    tags: HashMap::new(),
                    x: 40,
                    z: 50,
                },
                ProcessedNode {
                    id: 3,
                    tags: HashMap::new(),
                    x: 40,
                    z: 30,
                },
            ],
            tags: HashMap::from([("railway".to_string(), "rail".to_string())]),
        };
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));

        let anchors = (30..41).any(|x| (41..51).any(|z| curve_anchor_at(&editor, x, 1, z)));
        assert!(
            anchors,
            "the smoothed corner writes turn=true anchors, not just straights"
        );
    }

    #[test]
    fn every_corner_of_a_zigzag_osm_way_gets_its_own_arc() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 200.0).unwrap();
        let mut editor = test_editor(&xzbbox);

        // Three 90 degree turns at (50,150), (50,120) and (80,120).
        let way = ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 20,
                    z: 150,
                },
                ProcessedNode {
                    id: 2,
                    tags: HashMap::new(),
                    x: 50,
                    z: 150,
                },
                ProcessedNode {
                    id: 3,
                    tags: HashMap::new(),
                    x: 50,
                    z: 120,
                },
                ProcessedNode {
                    id: 4,
                    tags: HashMap::new(),
                    x: 80,
                    z: 120,
                },
                ProcessedNode {
                    id: 5,
                    tags: HashMap::new(),
                    x: 80,
                    z: 90,
                },
            ],
            tags: HashMap::from([("railway".to_string(), "rail".to_string())]),
        };
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));

        for (cx, cz) in [(50, 150), (50, 120), (80, 120)] {
            let arc = (cx - 10..cx + 11).into_iter().any(|x| {
                (cz - 10..cz + 11).into_iter().any(|z| {
                    curve_anchor_at(&editor, x, 1, z)
                        && editor.check_for_block(x, 1, z, Some(&[CREATE_TRACK]))
                })
            });
            assert!(arc, "corner ({cx},{cz}) has a Create curve anchor");
        }
    }

    #[test]
    fn at_grade_corner_becomes_a_smoothed_create_curve() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);

        // East–west leg to (40,50) then north to (40,30): the 90 degree change
        // happens exactly where the two OSM segments meet.
        let way = ProcessedWay {
            id: 1,
            nodes: vec![
                ProcessedNode {
                    id: 1,
                    tags: HashMap::new(),
                    x: 20,
                    z: 50,
                },
                ProcessedNode {
                    id: 2,
                    tags: HashMap::new(),
                    x: 40,
                    z: 50,
                },
                ProcessedNode {
                    id: 3,
                    tags: HashMap::new(),
                    x: 40,
                    z: 30,
                },
            ],
            tags: HashMap::from([("railway".to_string(), "rail".to_string())]),
        };
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));

        assert!(
            editor.check_for_block(20, 1, 50, Some(&[CREATE_TRACK])),
            "E-W arm keeps Create straights",
        );
        assert!(
            editor.check_for_block(40, 1, 30, Some(&[CREATE_TRACK])),
            "N-S arm keeps Create straights",
        );
        assert!(
            !editor.check_for_block(40, 1, 50, Some(&[CREATE_TRACK])),
            "the corner cell gets no straight of its own",
        );

        // The smoothed arc paints fake track inside the corner window.
        let arc_found = (32..41).into_iter().any(|x| {
            (42..51)
                .into_iter()
                .any(|z| editor.check_for_block(x, 1, z, Some(&[CREATE_FAKE_TRACK])))
        });
        assert!(
            arc_found,
            "smoothed arc painted fake track in the corner window"
        );
    }

    #[test]
    fn catenary_mast_flips_away_from_a_parallel_track() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = straight_rail(&[
            ("railway", "rail"),
            ("usage", "main"),
            ("electrified", "contact_line"),
        ]);

        // Parallel track 4 cells to the +z side (its rail head lands at the +3 mast base).
        let mut rail_mask = CoordinateBitmap::new(&xzbbox);
        for x in 18..62 {
            rail_mask.set(x, 54);
        }
        run(&mut editor, &way, &rail_mask);

        assert!(
            !editor.check_for_block(20, 0, 53, Some(&[GRAY_CONCRETE])),
            "mast must not stand on the neighbouring track's side"
        );
        assert!(
            editor.check_for_block(20, 0, 47, Some(&[GRAY_CONCRETE])),
            "mast flips to the clear outer side"
        );
    }

    #[test]
    fn the_validator_refuses_a_span_whose_target_is_behind_its_axes() {
        // The dumped overshoot pair from osm-rail19: the anchor at (-230,-1561,
        // -224) carries Axes[0] = (0.464007,0,-0.885832) and a Position offset
        // of (-15,0,34), so its own Axes and its chord point exactly opposite
        // (dot -0.998): the spline would have to turn 180 degrees, which is the
        // U-turn loop seen in the world. Measured, not guessed.
        let conn = CreateConnection {
            a: (-230, -1561, -224),
            b: (-245, -1561, -190),
            shape_a: "xo",
            shape_b: "nd",
            axes_a: (0.464007, 0.0, -0.885832),
            axes_b: (0.377897, 0.0, -0.925848),
            osm_cells: 37,
        };
        let (_, why) = check_create_connection(&conn);
        assert!(
            why.is_some(),
            "the dumped overshoot span must be refused before writing"
        );
    }

    #[test]
    fn the_validator_accepts_a_flat_span_travelling_straight() {
        let conn = CreateConnection {
            a: (0, 0, 0),
            b: (10, 0, 0),
            shape_a: "xo",
            shape_b: "xo",
            axes_a: (1.0, 0.0, 0.0),
            axes_b: (-1.0, 0.0, 0.0),
            osm_cells: 10,
        };
        let (chain, why) = check_create_connection(&conn);
        assert!(why.is_none(), "a straight span must get written");
        assert!(chain > 0, "the chain must be measured before writing");
    }

    #[test]
    fn a_span_can_never_take_over_another_span_anchor_cell() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);

        // First span: east–west, its END anchor at (50,12,50).
        let first = CreateConnection {
            a: (30, 12, 50),
            b: (50, 12, 50),
            shape_a: "xo",
            shape_b: "xo",
            axes_a: (1.0, 0.0, 0.0),
            axes_b: (-1.0, 0.0, 0.0),
            osm_cells: 20,
        };
        // Second span starts exactly on that anchor cell: one cell cannot host
        // two `Connections` and its chain must never bury the first anchor.
        let second = CreateConnection {
            a: (50, 12, 50),
            b: (50, 12, 30),
            shape_a: "zo",
            shape_b: "zo",
            axes_a: (0.0, 0.0, -1.0),
            axes_b: (0.0, 0.0, 1.0),
            osm_cells: 20,
        };
        let mut written: Vec<(i32, i32, i32)> = vec![];
        assert!(
            emit_create_connection(&mut editor, 1, first, &mut written),
            "the first span is measurable and gets written"
        );
        assert!(
            !emit_create_connection(&mut editor, 2, second, &mut written),
            "the span wanting an already owned cell writes nothing"
        );
        let body = editor.create_rails_report().sidecar_rejections();
        assert!(
            body.contains("overlap-precedence-earlier-span-wins"),
            "the overlap is logged with the precedence rule and its coordinates"
        );
        assert!(
            curve_anchor_at(&editor, 50, 12, 50),
            "the earlier anchor keeps its turn=true blockstate"
        );
    }

    #[test]
    fn a_duplicated_osm_way_lays_nothing_and_logs_the_overlap() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);

        // Two OSM railway ways physically on top of each other (a duplicated
        // relation): the ways are never merged, so the second one is refused
        // whole and shows up in the rejection sidecar instead of re-laying the
        // first one's cells.
        let nodes: Vec<(i32, i32)> = vec![(20, 50), (60, 50)];
        plan_create_legs(&mut editor, &nodes, 1);
        plan_create_legs(&mut editor, &nodes, 2);
        let body = editor.create_rails_report().sidecar_rejections();
        assert!(
            body.contains("overlap-precedence-earlier-span-wins"),
            "the duplicated way is refused by the precedence rule"
        );
        assert!(
            !body.contains("target-behind-start"),
            "the refusal is an overlap, not a geometry gate"
        );
    }

    /// Is there a Create track block in this column, within a few blocks of the
    /// surface the rails are laid on?
    fn create_track_near(editor: &WorldEditor, x: i32, z: i32) -> bool {
        let base = editor.get_ground_level(x, z) + 1;
        let ys: &[Block] = &[CREATE_TRACK];
        for y in (base - 6)..=(base + 6) {
            if editor.check_for_block_absolute(x, y, z, Some(ys), None) {
                return true;
            }
        }
        false
    }

    // `snap_dir` rounds a leg that is 4 degrees off the x axis onto `xo`, so the
    // leg ends at (60,50) while the OSM corner it was aimed at sits at (60,53):
    // the two legs used to leave (60,51),(60,52) belonging to nothing at all,
    // and a carriage derailed in the middle of the run.
    #[test]
    fn a_snapped_leg_joint_bridges_the_hole_it_used_to_leave() {
        let xzbbox = XZBBox::rect_from_min_max(0, 0, 140, 120).unwrap();
        let mut editor = test_editor(&xzbbox);
        let nodes: Vec<(i32, i32)> = vec![(20, 50), (60, 53), (100, 50)];
        plan_create_legs(&mut editor, &nodes, 4243);
        assert!(
            create_track_near(&editor, 60, 51),
            "the joint cell above the snapped end is laid"
        );
        assert!(
            create_track_near(&editor, 60, 52),
            "and so is the one beside it, so the two legs share a cell"
        );
    }

    // The offset is honest only while it is small: 200 cells at 3 degrees miss
    // the corner by 10 rows, and 10 invented cells would be a lie about the way,
    // so the joint is refused and shows up in the rejection sidecar instead.
    #[test]
    fn a_joint_too_far_to_bridge_is_logged_not_guessed() {
        let xzbbox = XZBBox::rect_from_min_max(0, 0, 460, 140).unwrap();
        let mut editor = test_editor(&xzbbox);
        let nodes: Vec<(i32, i32)> = vec![(20, 50), (220, 60), (420, 60)];
        plan_create_legs(&mut editor, &nodes, 902011);
        let body = editor.create_rails_report().sidecar_rejections();
        assert!(
            body.contains("leg-join-gap-too-far"),
            "the open joint is logged with its coordinates"
        );
    }

    // A ONE block step stays (the straight run names it with a slope shape);
    // only a wider step is flattened to the lower plane, because Create has no
    // shape for a step that is diagonal in x,z AND changes height.
    // A way that steps diagonally in x,z at one height: `TrackBlock.getConnected`
    // walks a `turn=false` cell only along `TrackShape.getAxes()` (tools/create_cls
    // dump: `PD` (1,0,1), `ND` (-1,0,1)), so a flat `xo`/`zo` base shape on a
    // diagonal step leaves the cell with an axis that points nowhere the carriage
    // came from - the joint is open. Measured: 93 of 93 diagonal pairs in a fresh
    // osm-rail output changed height, and the reference worlds have 0 such pairs.
    #[test]
    fn a_diagonal_step_names_a_diagonal_base_shape() {
        let xzbbox = XZBBox::rect_from_xz_lengths(200.0, 120.0).unwrap();
        let mut editor = test_editor(&xzbbox);
        let way = ProcessedWay {
            id: 7,
            nodes: vec![
                ProcessedNode {
                    id: 71,
                    tags: HashMap::new(),
                    x: 20,
                    z: 20,
                },
                ProcessedNode {
                    id: 72,
                    tags: HashMap::new(),
                    x: 40,
                    z: 40,
                },
            ],
            tags: {
                let mut t = HashMap::new();
                t.insert("railway".to_string(), "rail".to_string());
                t
            },
        };
        run(&mut editor, &way, &CoordinateBitmap::new(&xzbbox));
        let got_a = track_shape(&editor, 25, 1, 25);
        let got_b = track_shape(&editor, 26, 1, 26);
        let shapes = [got_a.as_deref(), got_b.as_deref()];
        assert!(
            shapes.iter().any(|s| *s == Some("pd") || *s == Some("nd")),
            "a diagonal step is named PD/ND, not a flat XO/ZO"
        );
    }

    #[test]
    fn shape_for_step_names_every_joinable_step() {
        assert_eq!(
            shape_for_step((0, 0, 0), (1, 0, 0)),
            Some("xo"),
            "flat x step"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (0, 0, 1)),
            Some("zo"),
            "flat z step"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (1, 0, 1)),
            Some("pd"),
            "flat diagonal step"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (-1, 0, 1)),
            Some("nd"),
            "flat other diagonal"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (0, 1, 1)),
            Some("as"),
            "axial rise +z"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (0, 1, -1)),
            Some("an"),
            "axial rise -z"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (0, -1, -1)),
            Some("as"),
            "a descent shares the AS axis (axes are walked both ways)"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (1, 1, 0)),
            Some("ae"),
            "axial rise +x"
        );
        assert_eq!(
            shape_for_step((0, 0, 0), (1, 1, 1)),
            None,
            "no shape joins a diagonal that also rises"
        );
    }

    #[test]
    fn a_span_over_a_step_in_terrain_keeps_both_anchors_in_one_plane() {
        let xzbbox = XZBBox::rect_from_min_max(0, 0, 460, 140).unwrap();
        let mut editor = test_editor(&xzbbox);
        let (ya, yb) = span_anchor_ys(&mut editor, 902012, (20, 50), 61, (24, 54), 62, 2);
        assert_eq!(
            (ya, yb),
            (61, 62),
            "a one block rise stays - a slope shape covers it"
        );
        let (ca, cb) = span_anchor_ys(&mut editor, 902012, (20, 50), 61, (24, 54), 63, 2);
        assert_eq!(
            (ca, cb),
            (61, 61),
            "a two block rise is flattened to the lower plane"
        );
        let body = editor.create_rails_report().sidecar_rejections();
        assert!(
            body.contains("span-anchor-rise"),
            "the step the span would have climbed is logged"
        );
    }
}
