"""Lens housing with stacked optical elements and a matching clip-on hood.

Shows the recommended multi-part layout (see
docs/organizing_a_project.md). Generic Components live in the REUSABLE
zone with no project-specific defaults; the specific design for an
M57-throat camera lives in the CONCRETE zone as thin subclasses that
fill in measurements as class attributes.

Run:
    python examples/lens-housing.py                                  # print variant (default)
    scadwright build examples/lens-housing.py --variant=display        # assembled view
    scadwright build examples/lens-housing.py --variant=print          # split + spread for inspection
"""

import re

from scadwright import BBox, Component, Param, arg, from_json, print_stamp
from scadwright.api.tolerances import default_eps
from scadwright.boolops import difference, intersection, union
from scadwright.composition_helpers import arrange_on_bed
from scadwright.design import Design, run, variant
from scadwright.primitives import circle, cylinder, polygon, scad_import, text
from scadwright.shapes import FilletRing, Funnel, Sector, Tube

from printer import BAMBU_X2D_PLATE
from spec import BronicaS2Bayonet
import lap_joint  # noqa: F401 — registers .lap_split_x() / .lap_splay() on Node
from lens_internals import (
    DEFAULT_ELEMENT_CLEARANCES,
    AlmondStop,
    Element,
    ElementClearances,
    ElementHolder,
    FilletSection,
    HexStop,
    HousingSection,
    MatingDisc,
    SquareStop,
    Stop,
    StopHolder,
    SweptSlot,
    TruncFilletRing,
    WaterhouseStop,
    element,
)


# =============================================================================
# REUSABLE: LensHousing and LensHood
# =============================================================================


# Lock spec for the mating disc ↔ locking lens-mount ring lives in a
# shared module so both designs reference one source of truth without
# pulling each other's Design class into scope (which would collide on
# variant names like `print`).
from mating_disc_lock import (
    BRONICA_S2_MATING_DISC_BEAD,
    BRONICA_S2_MATING_DISC_TABS,
    HOUSING_GROOVE_ANGLE_DEG,
    MATING_DISC_CLEARANCE,
    MatingDiscBead,
    MatingDiscTabs,
)


class LensHousing(Component):
    """Generic lens barrel: lower recess + flange + upper housing (narrow
    or funnel-widened) + element holders + front fillet. A matching hood
    Component can read the outer dimensions and hood cone angle off this
    housing to mate without shared scope."""

    FRONT_FILLET_OFFSET = 5.0   # aperture rim inset from the front element's grip ID
    FRONT_RIM_MIN = 2.0         # minimum radial width of the truncated front-bezel flat
    FRONT_AXIAL_MAX = 15.0      # cap on the bezel's axial extent; the rim widens past
                                # the minimum to hold it on small-front-element designs

    # Camera-side axial bound of the lens-mount ring, in the housing frame
    # (the flange-side bound is `mount_offset`). Together they span the z
    # range the mount ring physically occupies around the barrel. A stop
    # slot overlapping that range is trapped behind the ring, so `build()`
    # warns when one does. Left None on the generic housing (no ring
    # declared); the concrete camera-specific subclass sets it.
    mount_ring_z_bot = None

    stop = Param(Stop, default=None)
    mating_disc = Param(MatingDisc, default=None)
    mating_disc_tabs = Param(MatingDiscTabs, default=None)
    mating_disc_bead = Param(MatingDiscBead, default=None)
    helicoid_focus_slot = Param(SweptSlot, default=None)
    aperture_feeler_slot = Param(SweptSlot, default=None)
    equations = """
        # When omitted (default False), the lower-housing mid and inner
        # segments are included only if an element holder or the stop slot
        # reaches into their z range; outer is always present. True forces
        # all three — used by the lower_housing_testfit variant to exercise
        # the full lower housing.
        ?force_all_lower_segments:bool = False if ?force_all_lower_segments is None else ?force_all_lower_segments
        # Wall-thickness split: the lower (M57-fit) section needs to fit
        # inside the camera's stepped M57 throat — its OD is constrained
        # from above by camera-body geometry at each axial step. The lower
        # housing is therefore three stacked segments (outer/mid/inner,
        # flange-side to deepest), each with its own OD and wall. The
        # flange-flange/funnel/upper section sits outside the camera body
        # and is wide enough to host a stepped lap joint internally —
        # typically a thick (~5 mm radial) wall, independent of the lower
        # segments because it's constrained by different physical realities.
        hood_base_angle = 90 - fov_angle / 2
        lower_outer_od, lower_outer_len, lower_outer_thk > 0
        lower_mid_od,   lower_mid_len,   lower_mid_thk   > 0
        lower_inner_od, lower_inner_len, lower_inner_thk > 0
        upper_barrel_thk > 0
        # Bore-side fillet ring angle (from horizontal). 45° is the
        # self-supporting threshold for FDM overhangs; steeper (more
        # vertical, larger angle here) is easier to print but consumes
        # more axial extent inside the wider segment.
        bore_fillet_base_angle > 0
        bore_fillet_base_angle <= 90
        flange_flange_od, flange_flange_len, fov_angle, hood_base_angle > 0
        grip_depth, grip_width > 0
        mount_offset >= 0
        # Alignment-groove cross-section: V-prism cut into the bottom
        # face of the upper housing at the stop-slot's angular position
        # (+Y in housing frame). Width = chord on the surface; depth =
        # how far the apex pokes into the upper-housing material.
        alignment_groove_depth, alignment_groove_width > 0
        # Aperture-cover snap interface. Optional — present only when a
        # clip-on aperture cover is paired with this housing (the concrete
        # S2 subclass sets them). The housing cuts two triangular axial
        # grooves into the upper-housing wall that receive the cover's
        # snap ridges; the cover reads these same values so its ridges
        # match. `offset_deg` is how far past each stop-slot edge (into
        # the lower hemisphere) the grooves sit; `ridge_height` is the
        # radial engagement depth and `ridge_base` the circumferential
        # base of the triangular profile; `axial` is the cover's axial
        # extent (which sets the groove length); `clearance` is the
        # per-face gap the groove leaves around the seated ridge.
        ?aperture_cover_groove_offset_deg > 0
        ?aperture_cover_ridge_height > 0
        ?aperture_cover_ridge_base > 0
        ?aperture_cover_axial > 0
        ?aperture_cover_clearance >= 0
        # Aperture-cover light baffles. A pair of partial fillet (45°
        # chamfer) rings flank the cover axially — flat face to the cover
        # edge, slant pointing away — blocking light at the cover-to-wall
        # gap. `wall_thk` is the cover's radial thickness, which is also the
        # rings' radial extent (single-sourced so they match); `fillet_angle`
        # the slant; `fillet_clearance` the gap from each ring's flat face
        # to the cover edge.
        ?aperture_cover_wall_thk > 0
        ?aperture_cover_fillet_angle > 0
        ?aperture_cover_fillet_angle < 90
        ?aperture_cover_fillet_clearance >= 0
        # Hood snap groove. Optional — present only when a clip-on hood is
        # paired with this housing. A continuous 360° triangular groove in
        # the upper-barrel outer wall (the front-fillet rim's outer face)
        # receives the hood coupler's snap bead, centered at the seated
        # coupler midpoint (housing top − half the coupler overlap). The
        # bead lives on the hood; this groove is the housing's only change
        # for it. `engage` = bead-apex depth past the barrel OD; `axial` =
        # bead axial width; `clearance` = per-face groove play; the coupler
        # overlap positions the groove.
        ?hood_snap_engage > 0
        ?hood_snap_axial > 0
        ?hood_snap_clearance >= 0
        ?hood_coupler_overlap > 0
        # Segment IDs derived from each segment's OD and wall thickness.
        lower_outer_id = lower_outer_od - 2 * lower_outer_thk
        lower_mid_id   = lower_mid_od   - 2 * lower_mid_thk
        lower_inner_id = lower_inner_od - 2 * lower_inner_thk
        # Stepped-bore monotonicity: the throat narrows as we go deeper, so
        # each segment's ID must be no greater than the segment above it.
        lower_outer_id >= lower_mid_id
        lower_mid_id   >= lower_inner_id
        # Axial layout. Outer spans the flange-side step and extends above
        # the flange into the mount-ring zone (so it ends at mount_offset);
        # mid and inner stack below, into the throat. All three lengths
        # measure axial extent.
        lower_outer_z_top = mount_offset
        lower_outer_z_bot = -lower_outer_len
        lower_mid_z_top   = lower_outer_z_bot
        lower_mid_z_bot   = lower_mid_z_top - lower_mid_len
        lower_inner_z_top = lower_mid_z_bot
        lower_inner_z_bot = lower_inner_z_top - lower_inner_len
        # `housing_insertion_depth` is the total depth past the flange the
        # lower segments occupy; `lower_housing_len` is the full lower-tube
        # length including the above-flange mount-ring portion that outer
        # carries.
        housing_insertion_depth = lower_outer_len + lower_mid_len + lower_inner_len
        lower_housing_len = housing_insertion_depth + mount_offset
        unconstricted = tuple(e for e in elements:tuple if not e.constricted)
        max_upper_ele_dia = max((e.dia for e in unconstricted), default=0.0)
        # Bore at the funnel base (top of the lower section) is outer's ID.
        is_wide = max_upper_ele_dia > lower_outer_id
        expansion_funnel_len = min((e.face_z_bot for e in unconstricted), default=inf)
        upper_housing_len = max(e.face_z_top for e in elements)
        upper_housing_od = (max_upper_ele_dia + 2 * upper_barrel_thk) if is_wide else flange_flange_od
        # Per-element fit: the bore only narrows with depth, so the deepest
        # point an element reaches (its face_z_bot) determines the binding
        # segment.
        all(
            not e.constricted
            or e.throat_dia_required <= (
                lower_outer_id if e.face_z_bot >= lower_outer_z_bot
                else lower_mid_id if e.face_z_bot >= lower_mid_z_bot
                else lower_inner_id
            )
            for e in elements
        )
        # Wide-mode invariant: in funnel variants the funnel runs from
        # mount_offset (the top of the mount-ring zone, where the lower
        # tube ends) to expansion_funnel_len (the bottom of the lowest
        # unconstricted element). The funnel needs positive height, so the
        # lowest unconstricted element must sit strictly above the mount-
        # ring zone. (`element()` now defines constricted as face_z_bot <
        # mount_offset, so unconstricted elements have face_z_bot >=
        # mount_offset; this rule guards the equality edge case.)
        not is_wide or expansion_funnel_len > mount_offset
        # Stop slot must lie entirely in a section we can handle:
        #   (a) below the flange-flange (z <= mount_offset) — uniform
        #       lower-housing OD;
        #   (b) above the flange/funnel transition — uniform upper-
        #       housing OD;
        #   (c) (wide variant only) entirely within the funnel range
        #       [mount_offset, expansion_funnel_len] — handled by the
        #       per-gripper-OD logic in `wall_od_at_z` and the StopHolder
        #       od_top/od_bot pair.
        # The flange-overlap and mount-ring/flange transition zones stay
        # forbidden because they have a wall-OD discontinuity no cylindrical
        # gripper can span. The `?stop` references make these lines auto-
        # skip when no stop is supplied.
        slot_lo = ?stop.spacing - (stop.thk + stop.clr) / 2
        slot_hi = ?stop.spacing + (stop.thk + stop.clr) / 2
        upper_floor = expansion_funnel_len if is_wide else mount_offset + flange_flange_len
        ?slot_hi <= mount_offset or slot_lo >= upper_floor or (is_wide and slot_lo >= mount_offset and slot_hi <= expansion_funnel_len)
    """

    def _lower_segments_present(self):
        """Set of lower-housing segment names ({'outer'}, {'outer','mid'},
        or {'outer','mid','inner'}) actually included in the build.
        Outer is always present (needed for mechanical fit with the
        camera's outer throat step). Mid is included only if some
        element holder or the stop slot reaches into mid's z range
        (face_z_bot < lower_outer_z_bot); inner only if something
        reaches into inner's z range (face_z_bot < lower_mid_z_bot).
        `force_all_lower_segments=True` (set on the testfit housing)
        forces all three regardless — exercises the full structural
        geometry for fit-check prints.
        """
        present = {"outer"}
        if self.force_all_lower_segments:
            return {"outer", "mid", "inner"}
        # Deepest z reached by any element holder or the stop slot.
        z_candidates = [e.face_z_bot for e in self.elements]
        if self.stop is not None:
            z_candidates.append(self.stop.spacing - (self.stop.thk + self.stop.clr) / 2)
        if not z_candidates:
            return present  # no elements, no stop — only the structural outer ring
        deepest = min(z_candidates)
        if deepest < self.lower_outer_z_bot:
            present.add("mid")
        if deepest < self.lower_mid_z_bot:
            present.add("inner")
        return present

    def _effective_housing_insertion_depth(self):
        """Actual axial depth below the flange the lower housing extends,
        accounting for any segments trimmed by `_lower_segments_present`.
        Equal to `housing_insertion_depth` (the FULL depth) when all
        three segments are present; less when mid/inner are excluded."""
        present = self._lower_segments_present()
        depth = self.lower_outer_len
        if "mid" in present:
            depth += self.lower_mid_len
        if "inner" in present:
            depth += self.lower_inner_len
        return depth

    def _sections(self):
        """List of `HousingSection` Components describing the housing's
        wall geometry. Order doesn't matter — sections may overlap in z
        (e.g. flange + funnel in wide mode where the flange's wider OD
        wins). The single source of truth for both `build()` and
        `wall_od_at_z`.

        The lower housing is up to three stacked segments — outer
        (flange-side, widest, continues into the mount-ring zone above
        the flange), mid (intermediate, carries the helicoid focus
        slot), inner (deepest, narrowest). Mid and inner are only
        present if `_lower_segments_present()` says so. The flange-
        flange wall sits on top of outer, and the mating disc (if
        present) wraps outer."""
        outer_id = self.lower_outer_id
        outer_od = self.lower_outer_od
        mo = self.mount_offset
        flange_top = mo + self.flange_flange_len
        present = self._lower_segments_present()
        sections = [
            # Lower outer: from the outer/mid step up through the flange
            # plane into the mount-ring zone (z_top = mount_offset). The
            # flange-flange and (in narrow mode) the upper tube continue
            # this OD upward.
            HousingSection(
                z_bot=self.lower_outer_z_bot, z_top=self.lower_outer_z_top,
                bot_id=outer_id, top_id=outer_id,
                thk=self.lower_outer_thk,
                # Overlap the flange-flange (and, in wide mode, the funnel)
                # stacked above at z = mount_offset so the union fuses
                # instead of meeting on a coplanar face.
                top_overshoot=default_eps(),
            ),
        ]
        if "mid" in present:
            sections.append(
                # Lower mid: middle step. Carries the helicoid focus slot.
                HousingSection(
                    z_bot=self.lower_mid_z_bot, z_top=self.lower_mid_z_top,
                    bot_id=self.lower_mid_id, top_id=self.lower_mid_id,
                    thk=self.lower_mid_thk,
                    # Overlap outer, stacked above at lower_outer_z_bot.
                    top_overshoot=default_eps(),
                )
            )
        if "inner" in present:
            sections.append(
                # Lower inner: deepest step. The helicoid slot does NOT enter
                # this segment (the cutter is bounded above by the mid/inner
                # boundary).
                HousingSection(
                    z_bot=self.lower_inner_z_bot, z_top=self.lower_inner_z_top,
                    bot_id=self.lower_inner_id, top_id=self.lower_inner_id,
                    thk=self.lower_inner_thk,
                    # Overlap mid, stacked above at lower_mid_z_bot.
                    top_overshoot=default_eps(),
                )
            )
        if "mid" in present:
            sections.append(
                # Bore fillet at the outer/mid step. Sits inside outer just
                # above the step plane; closes the (small) wall-overlap and
                # gives the bore a self-supporting slope for print.
                FilletSection(
                    z_bot=self.lower_outer_z_bot,
                    bot_id=self.lower_mid_id,
                    top_id=self.lower_outer_id,
                    base_angle=self.bore_fillet_base_angle,
                )
            )
        if "inner" in present:
            sections.append(
                # Bore fillet at the mid/inner step. Without this the inner
                # wall doesn't touch mid's wall — there's a 1 mm radial gap
                # at the step plane.
                FilletSection(
                    z_bot=self.lower_mid_z_bot,
                    bot_id=self.lower_inner_id,
                    top_id=self.lower_mid_id,
                    base_angle=self.bore_fillet_base_angle,
                )
            )
        # Flange-flange: thicker wall sitting on top of the mount ring.
        # Bore matches outer (its lower neighbor). Always present.
        sections.append(HousingSection(
            z_bot=mo, z_top=flange_top,
            bot_id=outer_id, top_id=outer_id,
            thk=(self.flange_flange_od - outer_id) / 2,
            # Overlap the upper tube stacked above at flange_top (non-wide
            # mode); in wide mode the funnel/upper own that seam instead.
            top_overshoot=default_eps(),
        ))
        if self.is_wide:
            # Funnel widens the bore from outer's ID to the largest
            # unconstricted element; its wall uses upper_barrel_thk.
            # Overlaps the flange in z=[mo, flange_top]; in wall_od_at_z
            # the flange's wider OD wins via max() across containing
            # sections.
            sections.append(HousingSection(
                z_bot=mo, z_top=self.expansion_funnel_len,
                bot_id=outer_id, top_id=self.max_upper_ele_dia,
                thk=self.upper_barrel_thk,
            ))
            # Upper (above the funnel): uniform wide bore.
            sections.append(HousingSection(
                z_bot=self.expansion_funnel_len, z_top=self.upper_housing_len,
                bot_id=self.max_upper_ele_dia, top_id=self.max_upper_ele_dia,
                thk=self.upper_barrel_thk,
                # Dip into the conical funnel below at expansion_funnel_len.
                # The funnel can't overshoot (it's conical), so the
                # cylindrical upper tube carries the overlap downward.
                bot_overshoot=default_eps(),
            ))
        else:
            # Non-wide upper: continues outer's bore at the flange-flange OD.
            sections.append(HousingSection(
                z_bot=flange_top, z_top=self.upper_housing_len,
                bot_id=outer_id, top_id=outer_id,
                thk=(self.upper_housing_od - outer_id) / 2,
            ))
        if self.mating_disc is not None:
            # Mating disc: extra ring on the housing's outer surface
            # bridging the gap to the lens-mount ring's bore. ID =
            # lower_outer_od (wraps outer concentrically); OD and z range
            # come from the MatingDisc spec.
            md = self.mating_disc
            sections.append(HousingSection(
                z_bot=md.z_bot, z_top=md.z_top,
                bot_id=outer_od, top_id=outer_od,
                thk=(md.od - outer_od) / 2,
            ))
        return sections

    def wall_od_at_z(self, z):
        """Outer-wall diameter at axial position z. Used to size each
        stop-holder gripper to its local wall thickness so a gripper
        that lands in a narrowing funnel doesn't poke out past the wall.
        Returns the max OD across all sections containing z (overlapping
        sections — flange + funnel in wide mode — resolve via max)."""
        matching = [s for s in self._sections() if s.z_bot <= z <= s.z_top]
        if not matching:
            return 0.0
        return max(s.od_at(z) for s in matching)

    def wall_id_at_z(self, z):
        """Inner bore diameter at axial position z. Used to size the
        stop disc and the gripper's inner cylinder to the local bore,
        so the disc fills the bore and the gripper intrudes only
        `grip_depth/2` into the optical path regardless of which
        section the stop lands in. Overlapping sections resolve via
        MIN — wall material lives wherever any section has material,
        so the bore is bounded by the narrowest inner radius at z."""
        matching = [s for s in self._sections() if s.z_bot <= z <= s.z_top]
        if not matching:
            return 0.0
        return min(s.id_at(z) for s in matching)

    def _fuse_holder_to_section(self, holder, e, holder_od, sections):
        """If `holder`'s OD matches a cylindrical section's inner bore at
        `e.spacing`, fuse them with `self_only=True` so the coincident
        cylindrical surface doesn't survive the yield-union as an
        internal seam (which would re-emerge as a non-manifold edge
        after `halve`). `self_only=True` returns just the extended
        holder; the host section is already in the barrel union, so it
        doesn't need to come along too. Funnel sections
        (`bot_id != top_id`) don't qualify — their inner radius varies
        with z, so they have no flat cylindrical match to the holder.
        When no match exists (e.g. a wide-mode unconstricted element
        inside a funnel), the holder and barrel don't share a
        coincident face and a plain yield is fine."""
        host = next(
            (s for s in sections
             if s.z_bot <= e.spacing <= s.z_top
             and s.bot_id == s.top_id
             and abs(s.bot_id - holder_od) < 1e-6),
            None,
        )
        return holder.fuse(host, self_only=True) if host is not None else holder

    def _front_fillet(self):
        """Build the same TruncFilletRing instance used in `build()`,
        so its tight_bbox / equations are usable from here without
        duplicating the construction args."""
        import math
        front_ele = max(self.elements, key=lambda e: e.spacing)
        inner = front_ele.dia - self.grip_depth + self.FRONT_FILLET_OFFSET
        outer = self.upper_housing_od
        # Rim width floors at FRONT_RIM_MIN. A small front element makes the
        # cone cross a wider radial span, which at the fixed FOV angle would
        # drive the bezel's axial extent up; widen the rim past the floor to
        # cap that extent at FRONT_AXIAL_MAX. Widening truncates the same
        # FOV-derived cone lower without changing its slope, so optical
        # clearance is unchanged — only the flat front face grows.
        span = (outer - inner) / 2
        rim_width = max(
            self.FRONT_RIM_MIN,
            span - self.FRONT_AXIAL_MAX / math.tan(math.radians(self.hood_base_angle)),
        )
        return TruncFilletRing(
            id=inner,
            od=outer,
            base_angle=self.hood_base_angle,
            slant="inwards",
            rim_width=rim_width,
        )

    def _swept_slot_cutter(self, slot, wall_od):
        """Generic curved-bar cutter for a swept slot (helicoid focus
        slot or aperture feeler slot). One shell `slot.depth` mm thick
        (radially, from `wall_od/2` inward), intersected with a wedge
        that covers both the locked-side arc (centered at `slot.angle`,
        `slot.width` wide at `wall_od`) and the sweep arc (extending CW
        from `slot.angle` by `sweep_arc_deg`). Single uniform depth —
        the pin/feeler rides the groove throughout the bayonet rotation;
        nothing needs a full through-cut.

        The shell's outer cylinder pokes past the wall OD by a small
        delta for a clean exterior cut. `slot.bottom_overshoot` extends
        the cutter past `z_bot` axially — set this when the slot's deep
        end sits exactly on a segment boundary (e.g. the helicoid slot's
        z_bot at the mid/inner step) to drop the coplanar bottom face
        and avoid a rendering sliver. Leave it 0 when the slot's deep
        end is in the segment interior."""
        import math
        s = slot
        outer_delta = default_eps()           # past OD; just for clean exterior cut
        h = (s.z_top - s.z_bot) + s.bottom_overshoot
        outer_r = wall_od / 2 + outer_delta
        inner_r = wall_od / 2 - s.depth
        # Angular half-width = (arc / 2) / radius, in degrees. Use the
        # segment OD as the reference radius so `width` reads as the
        # arc length along the visible outer face.
        half_arc_deg = math.degrees((s.width / 2) / (wall_od / 2))
        shell = Tube(h=h, od=2 * outer_r, id=2 * inner_r)
        # Wedge angular extent: swept convex hull of the pin/feeler at
        # LOCK (centered at `slot.angle`, ±half_arc_deg wide) and at
        # INSERTION (centered at `slot.angle − sweep_arc_deg`, also
        # ±half_arc_deg wide). The pin/feeler width contributes half_arc
        # on BOTH sides — locked-side (CCW of `slot.angle`) and
        # insertion-side (CW of `slot.angle − sweep_arc_deg`). Sector
        # `angles=(a, b)` sweeps CCW from a to b.
        wedge = Sector(
            r=outer_r,
            angles=(
                s.angle - s.sweep_arc_deg - half_arc_deg,
                s.angle + half_arc_deg,
            ),
        ).linear_extrude(height=h)
        return intersection(shell, wedge).up(s.z_bot - s.bottom_overshoot)

    def _alignment_groove_cutter(self):
        """V-prism cutter for the alignment groove on the bottom face
        of the upper housing. Sits at theta=90° (housing-frame +Y, the
        stop-slot's angular position) so that when the lens locks to the
        camera both end up pointing straight up in camera-frame. Cross-
        section is a triangle of base `alignment_groove_width` and depth
        `alignment_groove_depth`, base on the bottom face (z=mount_offset)
        and apex up into the flange-flange. Extends radially from just
        inside the mating disc's OD out past the flange-flange's OD."""
        gw = self.alignment_groove_width
        gd = self.alignment_groove_depth
        r_inner = self.mating_disc.od / 2
        r_outer = self.flange_flange_od / 2
        overshoot = default_eps()
        # Triangle built in the 2D X-Y plane with apex at +Y. Base sits
        # below Y=0 by `overshoot` so the cutter opens past the bottom
        # face for a clean exterior cut.
        tri = polygon(points=[
            (-gw / 2, -overshoot),
            (gw / 2, -overshoot),
            (0, gd),
        ])
        # Extrude in Z, then rotate so Z (extrusion) becomes the radial
        # +Y direction and Y (apex) becomes axial +Z. After rotate
        # [90,0,0]: original Y→Z, original Z→-Y, so the extrusion runs
        # from Y=0 toward -Y. Translating by +(r_outer + overshoot) in
        # Y lands the prism spanning Y ∈ [r_inner − overshoot, r_outer + overshoot].
        # Translating up by mount_offset puts the base on the bottom
        # face (with the overshoot poking past it).
        length = (r_outer - r_inner) + 2 * overshoot
        return (
            tri.linear_extrude(height=length)
            .rotate([90, 0, 0])
            .translate([0, r_outer + overshoot, self.mount_offset])
        )

    def stop_externally_accessible(self):
        """True when the stop slot opens on the exposed barrel — the upper
        housing or (wide mode) the funnel — rather than recessed into the
        mount-ring zone below the flange. The aperture cover and its snap
        interface only apply to an accessible slot; a slot pulled into the
        mount-ring zone (e.g. by a deep inward inf_offset) sits behind the
        lens-mount ring, so no external cover is built for it. Mirrors the
        internal-slot case (a) of the slot-placement guard: internal iff
        slot_hi <= mount_offset."""
        if self.stop is None:
            return False
        slot_hi = self.stop.spacing + (self.stop.thk + self.stop.clr) / 2
        return slot_hi > self.mount_offset

    def _warn_if_stop_behind_mount_ring(self):
        """Warn when the stop slot's axial extent overlaps the lens-mount
        ring zone [mount_ring_z_bot, mount_offset]. A slot there is
        radially surrounded by the ring, so the ring must come off to
        change the aperture — a swap the Waterhouse-stop design is meant
        to avoid. `stop_externally_accessible` only compares against the
        ring's flange-side top (`mount_offset`); this check uses the full
        axial span, so it also catches a slot pulled deep enough to sit
        against the ring's camera-side end. Skipped when no ring bound is
        declared (generic housing) or no stop is present."""
        if self.stop is None or self.mount_ring_z_bot is None:
            return
        half = (self.stop.thk + self.stop.clr) / 2
        slot_lo = self.stop.spacing - half
        slot_hi = self.stop.spacing + half
        ring_bot, ring_top = self.mount_ring_z_bot, self.mount_offset
        if slot_hi > ring_bot and slot_lo < ring_top:
            import warnings
            warnings.warn(
                f"stop slot z=[{slot_lo:.2f}, {slot_hi:.2f}] overlaps the "
                f"lens-mount ring zone z=[{ring_bot:.2f}, {ring_top:.2f}]: "
                f"the mount ring must be removed to change the aperture. "
                f"Move the stop clear of the ring — in front of it "
                f"(slot_lo >= {ring_top:.2f}) or behind it "
                f"(slot_hi <= {ring_bot:.2f}).",
                stacklevel=2,
            )

    def _aperture_cover_groove_cutters(self):
        """Two triangular axial grooves in the upper-housing wall that
        receive the aperture cover's snap ridges.

        The stop slot opens across the +Y semicircle (its edges at the
        ±X points). Each groove sits `aperture_cover_groove_offset_deg`
        past one of those edges, into the lower hemisphere, so the cover
        — wrapping more than 180° — seats its ridges past the barrel's
        widest line for an over-center snap. Standard angles are
        `−offset` and `180 + offset` (symmetric about +Y).

        Cross-section (radial × circumferential) is a triangle: apex
        pointing inward to `ridge_height + clearance` below the wall OD,
        base `ridge_base + 2·clearance` wide on the surface. Axial length
        is `aperture_cover_axial + 2·clearance`, centered on the stop's z
        so the captured ridge has a hair of axial play at each end. The
        base overshoots the OD by an eps so the difference cuts cleanly."""
        s = self.stop
        z_center = s.spacing
        r = self.wall_od_at_z(z_center) / 2.0
        depth = self.aperture_cover_ridge_height + self.aperture_cover_clearance
        base = self.aperture_cover_ridge_base + 2 * self.aperture_cover_clearance
        axial = self.aperture_cover_axial + 2 * self.aperture_cover_clearance
        offset = self.aperture_cover_groove_offset_deg
        eps = default_eps()
        # Triangle in the X-Y plane at standard angle 0 (radial = +X):
        # base on the OD surface (overshooting +eps outward), apex inward.
        tri = polygon(points=[
            (r + eps, -base / 2),
            (r + eps, +base / 2),
            (r - depth, 0.0),
        ])
        prism = tri.linear_extrude(height=axial).up(z_center - axial / 2)
        return union(
            prism.rotate([0, 0, -offset]),
            prism.rotate([0, 0, 180 + offset]),
        )

    def _aperture_cover_baffle_top_z(self):
        """Subject-side z of the upper light-baffle ring's far edge — the
        first clean z above the aperture cover. The cover's top edge, plus
        the flat-face clearance, plus the ring's axial rise. `_front_band`
        uses it to keep the front engravings clear of the ring."""
        import math
        cover_top = self.stop.spacing + self.aperture_cover_axial / 2.0
        rise = self.aperture_cover_wall_thk * math.tan(
            math.radians(self.aperture_cover_fillet_angle)
        )
        return cover_top + self.aperture_cover_fillet_clearance + rise

    def _aperture_cover_baffle_rings(self):
        """Pair of partial fillet (45° chamfer) rings flanking the aperture
        cover as light baffles — one flush against each axial edge of the
        cover, the flat face to the cover edge (a tight clearance away) and
        the slant pointing away from the cover, so light can't pass through
        the cover-to-wall gap at the cover's ends.

        Each ring's radial extent equals the cover's wall thickness and its
        axial rise follows from the slant angle. Both span the same +Y arc
        as the cover (±(90 + groove_offset)), built as a triangular profile
        in the (radial, axial) plane and revolved over that arc, then
        rotated to center on +Y."""
        import math
        s = self.stop
        r = self.wall_od_at_z(s.spacing) / 2.0
        t = self.aperture_cover_wall_thk
        rise = t * math.tan(math.radians(self.aperture_cover_fillet_angle))
        clr = self.aperture_cover_fillet_clearance
        half = self.aperture_cover_axial / 2.0
        z_ct = s.spacing + half            # cover top (subject-side) edge
        z_cb = s.spacing - half            # cover bottom (camera-side) edge
        arc_half = 90.0 + self.aperture_cover_groove_offset_deg
        eps = default_eps()
        # Profiles in the (radial, axial) plane. Inner edge overshoots eps
        # into the wall so the union with the barrel closes cleanly.
        # Top ring: flat face down at the cover top edge (+clr), slant up-in.
        top = polygon(points=[
            (r - eps, z_ct + clr),
            (r + t, z_ct + clr),
            (r - eps, z_ct + clr + rise),
        ])
        # Bottom ring: flat face up at the cover bottom edge (−clr), slant down-in.
        bot = polygon(points=[
            (r - eps, z_cb - clr),
            (r + t, z_cb - clr),
            (r - eps, z_cb - clr - rise),
        ])

        def ring(profile):
            return profile.rotate_extrude(angle=2 * arc_half).rotate(
                [0, 0, 90 - arc_half]
            )

        return union(ring(top), ring(bot))

    def _hood_snap_groove_cutter(self):
        """Continuous 360° triangular groove in the upper barrel's outer
        wall (here the front-fillet rim's outer cylinder) that receives
        the lens hood's snap bead. Centered at the seated coupler midpoint
        — the housing's top (front fillet included) less half the hood
        coupler overlap. Profile matches the hood bead with
        `hood_snap_clearance` of play on every face: apex inward to
        `engage + clearance` past the barrel OD, base overshooting the OD
        by eps for a clean cut. Built in the (radial, axial) plane and
        rotate-extruded."""
        r = self.upper_housing_od / 2.0
        top_z = self.upper_housing_len + self._front_fillet().z_t
        z_center = top_z - self.hood_coupler_overlap / 2.0
        depth = self.hood_snap_engage + self.hood_snap_clearance
        half_w = self.hood_snap_axial / 2.0 + self.hood_snap_clearance
        eps = default_eps()
        cross = polygon(points=[
            (r + eps, z_center - half_w),
            (r + eps, z_center + half_w),
            (r - depth, z_center),
        ])
        return cross.rotate_extrude()

    def _mating_disc_tabs_geometry(self):
        """Tabs protruding radially outward from the mating disc's OD.

        Each tab is a parallel-sided box (not an angular wedge): in the
        tab's local frame +X is radial outward at the tab's center
        angle, +Y is tangential. The box spans
          x ∈ [disc_r − fuse_eps, disc_r + tabs.radial]
          y ∈ [−tabs.circumferential/2, +tabs.circumferential/2]
          z ∈ [tabs.z_top − tabs.axial, tabs.z_top]
        rotated to each angle in `tabs.angles_deg + housing_tab_offset_deg`.
        The notches in the lens-mount ring are placed at the UNOFFSET
        angles (via the conversion in lens_mount_locking_ring.py); the
        offset is applied here only to compensate for the testfit
        print's angular mirroring (see mating_disc_lock.py for the
        why). Parallel sides ensure a precise fit against the matching
        pocket in the ring's bore wall.
        """
        tabs = self.mating_disc_tabs
        disc_r = self.mating_disc.od / 2
        fuse_eps = default_eps()
        rect = polygon(points=[
            (disc_r - fuse_eps, -tabs.circumferential / 2),
            (disc_r + tabs.radial, -tabs.circumferential / 2),
            (disc_r + tabs.radial, +tabs.circumferential / 2),
            (disc_r - fuse_eps, +tabs.circumferential / 2),
        ])
        z_bot = tabs.z_top - tabs.axial
        one_tab = rect.linear_extrude(height=tabs.axial).up(z_bot)
        return union(*(
            one_tab.rotate([0, 0, theta + tabs.housing_tab_offset_deg])
            for theta in tabs.angles_deg
        ))

    def _mating_disc_bead_geometry(self):
        """Snap-fit bead around the mating disc's OD. Continuous 360°
        triangular ridge: apex at radius disc_r + bead.height, base
        flush with the disc OD over an axial span of bead.axial,
        centered at bead.z_center. Built as a 2D triangle in the
        (radial, axial) plane (= xy in the polygon, x → radial, y →
        axial), rotate-extruded around the z axis.
        """
        bead = self.mating_disc_bead
        disc_r = self.mating_disc.od / 2
        half_w = bead.axial / 2
        # The base sits on the disc OD; sample the inner edge slightly
        # inside the disc (- fuse_eps) so the union with the disc
        # closes cleanly without a coplanar face.
        fuse_eps = default_eps()
        cross = polygon(points=[
            (disc_r - fuse_eps, bead.z_center - half_w),
            (disc_r + bead.height, bead.z_center),
            (disc_r - fuse_eps, bead.z_center + half_w),
        ])
        return cross.rotate_extrude()

    def tight_bbox(self):                                  # framework hook: declare true post-Difference extents
        # Default tight_bbox would walk into the slot's `difference()`
        # and raise. The slot is fully contained within the barrel, so
        # it doesn't change outer extents. Compute the true envelope
        # from the wall sections (max OD across all of them) plus the
        # front fillet's truncated rim height (read from the fillet's
        # own equation-derived z_t — no math duplication).
        sections = self._sections()
        max_r = max(
            max(s.od_at(s.z_bot) for s in sections),
            max(s.od_at(s.z_top) for s in sections),
        ) / 2
        min_z = -self._effective_housing_insertion_depth()
        max_z = self.upper_housing_len + self._front_fillet().z_t
        return BBox(min=(-max_r, -max_r, min_z), max=(max_r, max_r, max_z))

    def build(self):
        self._warn_if_stop_behind_mount_ring()
        # Above the flange the bore is uniform (outer continues through
        # the mount-ring zone, then the flange-flange/funnel/upper take
        # over), so `upper_id` is one value. Below the flange the bore
        # steps through outer/mid/inner; constricted-element holders pick
        # their OD from the local wall bore via `wall_id_at_z(spacing)`
        # so each holder sits cleanly inside the segment containing it.
        upper_id = self.max_upper_ele_dia if self.is_wide else self.lower_outer_id

        # Wall sections: outer / mid / inner / flange-flange / (funnel /
        # upper) / optional mating disc. Single source of truth
        # (`_sections()`) used by both this build and `wall_od_at_z`.
        sections = self._sections()
        # Bare barrel — engravings are no longer part of build() so the
        # force_render scope in the variant chain caches just the
        # structural mesh. See `engravings()` for the cutter geometry
        # and the variant methods for where it's applied.
        barrel = union(*sections)

        # Slot OD: the maximum housing OD at the slot's z range. In the
        # uniform-OD allowed bands this is just the local OD; in the
        # funnel band the slot's TOP has the largest local OD (funnel
        # widens upward), so we use that. +1 mm radial overshoot avoids
        # cylinder-on-cylinder coplanar faces in the difference.
        if self.stop is not None:
            s = self.stop
            slot_h = s.thk + s.clr
            half_gw = 0.5 * self.grip_width
            slot_od = self.wall_od_at_z(s.spacing + slot_h / 2)
            slot_id = self.wall_id_at_z(s.spacing) - 0.1
            slot = Tube(
                h=slot_h, od=slot_od + 1, id=slot_id,
            ).halve([0, 1, 0]).up(s.spacing - slot_h / 2)
            barrel = difference(barrel, slot)

        # Skip the slot cutters when the corresponding lower segment is
        # absent — no segment, no material to cut, and no camera-body
        # pin to clear at this depth either.
        present = self._lower_segments_present()
        if self.helicoid_focus_slot is not None and "mid" in present:
            barrel = difference(
                barrel,
                self._swept_slot_cutter(self.helicoid_focus_slot, self.lower_mid_od),
            )
        if self.aperture_feeler_slot is not None and "inner" in present:
            barrel = difference(
                barrel,
                self._swept_slot_cutter(self.aperture_feeler_slot, self.lower_inner_od),
            )

        barrel = difference(barrel, self._alignment_groove_cutter())

        # Aperture-cover snap grooves. Cut in build() (not a variant) so
        # they appear in every housing variant. Present only when the slot is
        # externally accessible (a recessed mount-ring-zone slot has no cover)
        # and the cover interface is configured. Held for reuse below: the
        # stop grippers fill the wall out to its OD at the same z, so they
        # must be cut by these same grooves or they obstruct the cover's
        # snap ridges.
        cover_grooves = None
        if self.stop_externally_accessible() and self.aperture_cover_ridge_height is not None:
            cover_grooves = self._aperture_cover_groove_cutters()
            barrel = difference(barrel, cover_grooves)

        # Aperture-cover light baffles: a partial fillet ring flush against
        # each axial edge of the cover, blocking light at the cover-to-wall
        # gap. Unioned (protrusions); present whenever the slot is externally
        # accessible and the cover's wall thickness (= the rings' radial
        # extent) is configured.
        if self.stop_externally_accessible() and self.aperture_cover_wall_thk is not None:
            barrel = union(barrel, self._aperture_cover_baffle_rings())

        if self.mating_disc_tabs is not None and self.mating_disc is not None:
            barrel = union(barrel, self._mating_disc_tabs_geometry())

        if self.mating_disc_bead is not None and self.mating_disc is not None:
            barrel = union(barrel, self._mating_disc_bead_geometry())

        # Hood snap groove: a 360° groove receiving the lens hood's snap
        # bead. It lands on the front-fillet rim's outer wall (yielded
        # separately below), so cut it from both the barrel and the fillet
        # — the barrel cut is a no-op when the groove sits above the
        # cylindrical top, and vice versa, which keeps it correct wherever
        # the seated midpoint falls.
        hood_groove = (
            self._hood_snap_groove_cutter()
            if self.hood_snap_engage is not None else None
        )
        if hood_groove is not None:
            barrel = difference(barrel, hood_groove)

        yield barrel

        for e in self.elements:
            # Constricted elements sit inside the stepped lower bore, so
            # pick the holder OD from the local wall ID at the element's
            # center — the holder mates with whatever segment contains it.
            # Unconstricted elements live above the flange where the bore
            # is uniform.
            holder_od = self.wall_id_at_z(e.spacing) if e.constricted else upper_id
            # Known limitation (wide variants only): when an unconstricted
            # element's dia equals max_upper_ele_dia, holder_od == its
            # own dia and the holder's surround tube has thk =
            # -e.clearances.radial/2 (negative). The fix is to size the
            # widest element's holder off upper_housing_od (wall outer)
            # rather than upper_id (wall inner == element dia). Leaving
            # as a follow-up.
            holder = ElementHolder(
                element=e, od=holder_od,
                grip_depth=self.grip_depth, grip_width=self.grip_width,
            )
            yield self._fuse_holder_to_section(holder, e, holder_od, sections)

        if self.stop is not None:
            # Per-gripper OD: each gripper's outer cylinder is sized to
            # the local wall OD at the gripper's BOTTOM (the narrowest
            # point in its z extent for an upward-widening funnel), so
            # the gripper is fully embedded in wall material with no
            # radial overshoot anywhere along its height.
            upper_z_bot = s.spacing + slot_h / 2
            lower_z_bot = s.spacing - slot_h / 2 - half_gw
            holder = StopHolder(
                stop=self.stop,
                od_top=self.wall_od_at_z(upper_z_bot),
                od_bot=self.wall_od_at_z(lower_z_bot),
                barrel_id=self.wall_id_at_z(s.spacing),
                grip_depth=self.grip_depth, grip_width=self.grip_width,
            )
            # The grippers reach the wall OD at the slot z, filling the band
            # the snap grooves cut from the barrel. Cut the same grooves from
            # the grippers so the cover's ridge channel stays clear; the
            # notch sits past the slot edges and leaves the disc grip intact.
            yield difference(holder, cover_grooves) if cover_grooves is not None else holder

        # Front fillet, fused onto the barrel's top. The barrel is a
        # generic union, so its bbox-derived "top" anchor drifts off the
        # axis when outward features are asymmetric (the +Y baffle rings,
        # the mating-disc tabs). Declare the true coaxial top so the bezel
        # lands on the axis; attach(fuse=True) still resolves to the
        # fillet's shift bond (manifold-safe — no coincident-cylinder weld).
        barrel = barrel.with_anchor(
            "top", at=(0, 0, self.upper_housing_len), normal=(0, 0, 1),
        )
        fillet = self._front_fillet().attach(barrel, fuse=True)
        if hood_groove is not None:
            fillet = difference(fillet, hood_groove).with_bbox_from(fillet)
        yield fillet

    def engravings(self):
        """Cutter geometry for the housing's engravings, in housing-local
        frame.

        Returned as a single union so a variant chain can apply
        ``difference(housing.force_render(), housing.engravings())``
        and keep the per-glyph CGAL work out of the cached subtree.

        - Upper-housing label: text on the topmost uniform section's
          ``outer_wall`` (angle=0). Picks the same section ``build()``
          would have decorated (``z_top == upper_housing_len`` and
          uniform OD).
        - Front-fillet labels: the BRIOTAR lens mark (angle=0) and the
          ZenzaNOT label on the opposite side (angle=180), both on
          the truncated fillet's ``slanted_face``. Positioned with
          ``.up(upper_housing_len)`` to match where ``attach(barrel,
          fuse=True)`` lands the fillet in ``build()`` (sub-mm fuse offset
          is below engraving tolerance).
        - Optional optical-plan diagram (angle=180) when ``--optical_plan``
          is supplied."""
        upper_section = next(
            s for s in self._sections()
            if s.z_top == self.upper_housing_len and s.bot_id == s.top_id
        )
        upper_label = "\n".join(
            [f"{design_name}:"]
            + [e.code for e in self.elements]
            + [print_stamp(allow_dirty=ALLOW_DIRTY_STAMP)]
        )
        # Center the label block in the same clean band the optical diagram
        # uses: between the stop slot's subject-side edge and the subject-
        # facing rim. Centering on the section instead would drop the block's
        # bottom lines into the slot. Place at the section's default position,
        # measure the real ink extent, then shift to the band's mid-height.
        band_bot, band_top = self._front_band(upper_section)
        upper_cutter = upper_section.text_geometry(
            label=upper_label, on="outer_wall", angle=0,
            font=LABEL_FONT, font_size=4, line_spacing=1.4, relief=-0.6,
        )
        uz = upper_cutter.bbox
        upper_cutter = upper_cutter.up(
            (band_bot + band_top) / 2.0 - (uz.min[2] + uz.max[2]) / 2.0
        )
        fillet = self._front_fillet().up(self.upper_housing_len)
        fillet_cutter = self._fillet_label(fillet, FRONT_FILLET_LABEL, angle=0)
        bezel_back_cutter = self._fillet_label(fillet, BEZEL_BACK_LABEL, angle=180)
        cutters = [upper_cutter, fillet_cutter, bezel_back_cutter]
        if OPTICAL_PLAN_SVG:
            cutters.append(self._optical_plan_cutter(upper_section))
        return union(*cutters)

    def _fillet_label(self, fillet, label, *, angle, relief=-0.6):
        """Inset `label` on the front fillet's slanted face in `LABEL_FONT`,
        sized to the axial height the default font gives at font_size 4. The
        heavier face then holds that height and only runs wider on the slant.
        bbox height is linear in font size, so one probe of each font solves
        it."""
        def h(font, fs):
            b = fillet.text_geometry(
                label=label, on="slanted_face", angle=angle,
                font=font, font_size=fs, relief=relief,
            ).bbox
            return b.max[2] - b.min[2]
        font_size = 4.0 * h(None, 4) / h(LABEL_FONT, 4)
        return fillet.text_geometry(
            label=label, on="slanted_face", angle=angle,
            font=LABEL_FONT, font_size=font_size, relief=relief,
        )

    def _front_band(self, upper_section):
        """Axial extent of the clean band on the upper barrel where the
        front-side engravings (the optical diagram at angle 180 and the
        upper text label at angle 0) may be drawn — bounded so nothing that
        clips onto the housing covers them:

        - bottom: the subject-side edge of the aperture-slot cover and its
          upper light-baffle ring (the cover is centered on the stop slot
          and reaches `aperture_cover_axial/2` past it; the ring sits a
          clearance plus its axial rise beyond that); falls back to the
          cover top, or the slot's own subject edge, when those aren't
          configured;
        - top: the subject-facing rim pulled down by the hood / lens-cap
          coupler overlap, since either ring slides over the top
          `hood_coupler_overlap` mm of the barrel.

        The front fillet's inward slant is on the bore, so the outer wall
        stays at the section OD up to the rim and engravings ride onto it.

        Returns `(band_bot, band_top)`."""
        if self.stop is not None:
            slot_top = self.stop.spacing + (self.stop.thk + self.stop.clr) / 2
            if self.aperture_cover_wall_thk is not None:
                cover_top = self._aperture_cover_baffle_top_z()
            elif self.aperture_cover_axial is not None:
                cover_top = self.stop.spacing + self.aperture_cover_axial / 2
            else:
                cover_top = slot_top
            band_bot = max(slot_top, cover_top)
        else:
            band_bot = upper_section.z_bot
        rim = self.upper_housing_len + self._front_fillet().z_t
        band_top = (
            rim - self.hood_coupler_overlap
            if self.hood_coupler_overlap is not None else rim
        )
        return band_bot, band_top

    @staticmethod
    def _svg_import_extent(path):
        """`(width, height)` in mm of `path` as OpenSCAD imports it: the
        SVG's px `width`/`height` taken at `SVG_IMPORT_DPI`. The diagram is
        authored centered in its viewBox, so this canvas extent is centered
        on the imported geometry too — which is what `wrap_2d`'s `size=`
        scales against and what centers the placement."""
        with open(path, encoding="utf-8") as f:
            head = f.read(2000)

        def px(attr):
            m = re.search(rf'{attr}\s*=\s*"([0-9.]+)', head)
            if not m:
                raise ValueError(f"optical_plan: {path} has no numeric {attr!r} attribute")
            return float(m.group(1))

        scale = 25.4 / SVG_IMPORT_DPI
        return px("width") * scale, px("height") * scale

    def _optical_plan_cutter(self, upper_section):
        """Inset cutter for the optional optical-diagram SVG, wrapped on the
        upper barrel opposite the text label.

        The SVG carries the optical axis (an arrow) along its X direction.
        `wrap_2d` maps a profile's +Y to the barrel axis and +X to the
        circumference, so the profile is rotated -90deg about Z: the arrow
        then runs axially with its tip toward the camera (-Z). The diagram
        fills the clean band above the stop slot, inset to the same depth
        as the text label so the vertical print needs no support."""
        w_mm, h_mm = self._svg_import_extent(OPTICAL_PLAN_SVG)
        logo = scad_import(
            OPTICAL_PLAN_SVG, bbox=((0.0, 0.0, 0.0), (w_mm, h_mm, 0.0)),
        ).rotate([0, 0, -90])

        # Diagram band: the clean zone from `_front_band`, already pulled in
        # so the aperture cover (bottom) and the hood / lens-cap rings (top)
        # can't cover the drawing.
        band_bot, band_top = self._front_band(upper_section)
        # The diagram fills `fill_frac` of the band, centered in it. The
        # imported SVG canvas carries a margin of empty space around the
        # inked drawing, so `canvas_margin` oversizes the canvas a touch to
        # bring the inked drawing itself to `fill_frac` of the span.
        fill_frac = 0.85
        canvas_margin = 1.06
        axial = (band_top - band_bot) * fill_frac * canvas_margin
        mid_wall = (upper_section.z_bot + upper_section.z_top) / 2
        at_z = (band_bot + band_top) / 2 - mid_wall

        # After the -90deg rotation the canvas' axial extent is the SVG's X
        # (w_mm) and its circumferential extent is the SVG's Y (h_mm). Size
        # axially; scale width by the same factor to keep aspect.
        circ = axial * h_mm / w_mm
        return upper_section.wrap_2d_geometry(
            profile=logo, relief=-0.6, on="outer_wall",
            angle=180, at_z=at_z, size=(circ, axial),
            projection="wrap", segments=64,
        )


def _snap_bead_geometry(*, id_coupler, housing_upper_od, snap_engage,
                        snap_axial, coupler_overlap):
    """Continuous 360° triangular bead on a coupler's inner wall at its
    axial midpoint, protruding inward so its apex sits `snap_engage` past
    the barrel OD (the snap interference) — seating into the groove the
    housing cuts at the matching point. Base overshoots eps into the
    coupler wall so the union closes without a coplanar face. Cross-section
    is built in the (radial, axial) plane and rotate-extruded, like the
    housing's mating-disc bead. Shared by the lens hood and lens cap so
    both beads match the one housing groove."""
    r_in = id_coupler / 2.0
    r_apex = housing_upper_od / 2.0 - snap_engage
    z_center = coupler_overlap / 2.0
    half_w = snap_axial / 2.0
    eps = default_eps()
    cross = polygon(points=[
        (r_in + eps, z_center - half_w),
        (r_in + eps, z_center + half_w),
        (r_apex, z_center),
    ])
    return cross.rotate_extrude()


class LensHood(Component):
    """Generic clip-on hood sized by a paired housing. `housing_upper_od`
    and `hood_base_angle` come from the housing; the rest are hood-specific.
    Built in its mounted orientation: coupler at z=0, funnel flaring upward.
    The print variant flips it."""

    equations = """
        housing_upper_od, hood_base_angle, wall_thk, hood_length, coupler_overlap > 0
        hood_clr >= 0
        id_coupler = housing_upper_od + hood_clr
        od_coupler = id_coupler + 2 * wall_thk
        flare_od = housing_upper_od + 2 * hood_length * tan(90 - hood_base_angle)
        # Optional raised-text label wrapped around the coupler's outer
        # wall. Empty string = no engraving. Applied directly to the
        # coupler Tube in build() (via `add_text` on Tube's built-in
        # `outer_wall` anchor) so the text wraps with the framework's
        # default cylindrical orientation: upright glyphs, line reading
        # L-to-R when viewed from outside, glyph bottoms toward -Z.
        ?label:str = ?label or ""
        # Optional label wrapped opposite `label` (see build()). Empty string
        # = none. `label_font` is a fontconfig family name (e.g.
        # "Verdana:style=Bold") shared by both labels; empty falls back to the
        # default font.
        ?back_label:str = ?back_label or ""
        ?label_font:str = ?label_font or ""
        # Optional snap ring. When configured, a continuous 360° triangular
        # bead on the coupler's inner wall (at its axial midpoint) seats
        # into a matching groove the housing cuts, holding the hood on.
        # `snap_engage` is how far the bead apex reaches past the barrel OD
        # (the snap interference); `snap_axial` the bead's axial width.
        # Both are read from the paired housing so the bead matches the
        # groove. Omitted (None) = no bead.
        ?snap_engage > 0
        ?snap_axial > 0
    """

    # Bore-side fillet angle at the coupler/funnel junction. Its outer face
    # is cylindrical at od_coupler, so the hood's straight outer wall is the
    # coupler plus this fillet (see _cylinder_band) — the band the labels
    # center and size on.
    COUPLER_FILLET_ANGLE = 63
    # Axial ink height of the front model-designation label. Held constant
    # so the label keeps its size whatever font sets it; the font may run
    # wider circumferentially without growing taller.
    FRONT_LABEL_HEIGHT = 4.8

    def build(self):                                       # framework hook: required; returns the shape
        coupler = Tube(
            h=self.coupler_overlap, id=self.id_coupler, thk=self.wall_thk,
        )
        parts = [coupler]
        # Model designation on the front (angle=0), a second label opposite it
        # on the back (angle=180), both in `label_font`, both centered axially
        # on the outer cylinder (coupler + junction fillet). The back label
        # fills most of the cylinder height; the front label holds a fixed
        # axial height.
        font = self.label_font or None
        if self.label:
            parts.append(self._centered_band_text(
                coupler, self.label, angle=0, font=font,
                font_size=self._font_size_for_height(
                    coupler, self.label, angle=0,
                    target_h=self.FRONT_LABEL_HEIGHT, font=font,
                ),
            ))
        if self.back_label:
            parts.append(self._centered_band_text(
                coupler, self.back_label, angle=180, font=font,
                font_size=self._font_size_for_height(
                    coupler, self.back_label, angle=180,
                    target_h=0.75 * self._cylinder_band(), font=font,
                ),
            ))
        if self.snap_engage is not None:
            parts.append(self._coupler_snap_bead())
        yield union(*parts) if len(parts) > 1 else coupler
        yield Funnel(
            h=self.hood_length, thk=self.wall_thk,
            bot_od=self.housing_upper_od, top_od=self.flare_od,
        ).attach(coupler, fuse=True)
        yield FilletRing(
            id=self.id_coupler, od=self.od_coupler,
            base_angle=self.COUPLER_FILLET_ANGLE, slant="inwards",
        ).attach(coupler, fuse=True)

    def _coupler_snap_bead(self):
        """Snap bead on the coupler inner wall (see `_snap_bead_geometry`)."""
        return _snap_bead_geometry(
            id_coupler=self.id_coupler, housing_upper_od=self.housing_upper_od,
            snap_engage=self.snap_engage, snap_axial=self.snap_axial,
            coupler_overlap=self.coupler_overlap,
        )

    def _cylinder_band(self):
        """Axial height (from z=0) of the hood's straight outer wall: the
        coupler plus the junction fillet, whose outer face is cylindrical at
        od_coupler. The funnel only flares past od_coupler above this, so this
        is the visible cylinder the labels center and size on — not the
        coupler alone."""
        import math
        fillet_h = math.tan(math.radians(self.COUPLER_FILLET_ANGLE)) * (
            self.od_coupler - self.id_coupler
        ) / 2.0
        return self.coupler_overlap + fillet_h

    def _centered_band_text(self, coupler, label, *, angle, font_size, font=None):
        """Raised `label` on the outer wall, centered axially on the cylinder
        (_cylinder_band). Measures the real glyph-ink bbox and shifts so the
        visible ink midpoint sits at the cylinder's mid-height — correct
        whatever ascenders or descenders the string carries."""
        glyphs = coupler.text_geometry(
            label=label, on="outer_wall", angle=angle, font=font,
            font_size=font_size, relief=0.5, at_z=0.0,
        )
        gz = glyphs.bbox
        return glyphs.up(self._cylinder_band() / 2.0 - (gz.min[2] + gz.max[2]) / 2.0)

    def _font_size_for_height(self, coupler, label, *, angle, target_h, font=None):
        """Font size at which `label`'s ink is `target_h` mm tall axially. The
        bbox height is linear in font size, so one probe at a reference size
        solves it — independent of font, so a wider/heavier face holds the
        same height and only runs wider circumferentially."""
        ref = target_h
        gz = coupler.text_geometry(
            label=label, on="outer_wall", angle=angle, font=font,
            font_size=ref, relief=0.5, at_z=0.0,
        ).bbox
        return target_h * ref / (gz.max[2] - gz.min[2])


class LensCap(Component):
    """Clip-on lens cap: the same snapping coupler as `LensHood`, but the
    flaring funnel is replaced by a flat closing disc.

    The disc sits at the funnel-shoulder plane (the coupler top), so the
    snap-to-shoulder distance — and thus the seated position on the
    housing — matches the hood exactly; cap and hood fit the same barrel
    and the same snap groove (their beads come from one shared helper,
    `_snap_bead_geometry`). The outward face carries an inset label. Built
    in mounted orientation (coupler at z=0, disc on top); the print
    variant flips it disc-down."""

    equations = """
        housing_upper_od, wall_thk, coupler_overlap, cap_thk > 0
        hood_clr >= 0
        id_coupler = housing_upper_od + hood_clr
        od_coupler = id_coupler + 2 * wall_thk
        ?label:str = ?label or ""
        ?label_font:str = ?label_font or ""
        ?snap_engage > 0
        ?snap_axial > 0
    """

    # Label inset depth (mm) and the fraction of the disc diameter the
    # label spans. Inset (not raised) so the cap prints disc-face-down
    # with the lettering crisp against the bed and no support.
    LABEL_DEPTH = 0.6
    LABEL_WIDTH_FRAC = 0.72

    def build(self):                                       # framework hook: required; returns the shape
        eps = default_eps()
        coupler = Tube(h=self.coupler_overlap, id=self.id_coupler, thk=self.wall_thk)
        parts = [coupler]
        if self.snap_engage is not None:
            parts.append(_snap_bead_geometry(
                id_coupler=self.id_coupler, housing_upper_od=self.housing_upper_od,
                snap_engage=self.snap_engage, snap_axial=self.snap_axial,
                coupler_overlap=self.coupler_overlap,
            ))
        yield union(*parts) if len(parts) > 1 else coupler
        # Flat lid at the funnel-shoulder plane (coupler top), overlapping
        # the coupler by eps so the union closes without a coplanar face.
        disc = cylinder(
            h=self.cap_thk + eps, d=self.od_coupler,
        ).up(self.coupler_overlap - eps)
        if self.label:
            # Inset, flat-centered label on the disc's flat top. The
            # cylinder's top is a planar rim anchor, so add_text would lay
            # the label around the rim as an arc by default;
            # `text_curvature="flat"` keeps it flat and centered.
            disc = disc.add_text(
                label=self.label, on="top", text_curvature="flat",
                relief=-self.LABEL_DEPTH,
                font=self.label_font or None,
                font_size=self._cap_label_font_size(),
                halign="center", valign="center",
            )
        yield disc

    def _cap_label_font_size(self):
        """Font size at which the label spans `LABEL_WIDTH_FRAC` of the
        disc diameter. bbox width is linear in font size, so one probe at
        a reference size solves it."""
        ref = 10.0
        b = text(
            self.label, size=ref, font=self.label_font or None,
            halign="center", valign="center",
        ).bbox
        return self.LABEL_WIDTH_FRAC * self.od_coupler * ref / (b.max[0] - b.min[0])

    def tight_bbox(self):                                  # framework hook: declare true extents (inset text uses Difference)
        r = self.od_coupler / 2.0
        return BBox(min=(-r, -r, 0.0), max=(r, r, self.coupler_overlap + self.cap_thk))


class ApertureCover(Component):
    """Clip-on cover for the housing's Waterhouse-stop slot.

    A C-clip arc wrapping the +Y (skyward, when locked) semicircle of
    the upper housing, covering the stop slot so stray light can't reach
    the bore. Built in the housing's world frame (centered on the slot's
    z), so a display view places it directly and the print variant flips
    it onto the bed.

    Mating is a radial over-center snap: the arc wraps past the barrel's
    widest line, so pressing it down from +Y flexes the ends out and
    they spring back — the wrap itself retains it. A pair of triangular
    ridges on the inner face seat into matching grooves the housing cut
    (`offset_deg` past each slot edge), adding a tactile detent; because
    each housing groove is a finite axial pocket just longer than its
    ridge, the seated ridge is captured axially and the cover can't slide
    off along the lens axis. The arc terminates right at the ridges (no
    free material past them — that would risk cracking rather than
    flexing on a brittle print), with a flat tab over each ridge to give
    the leverage to splay the ends and pop it free.

    The shared dimensions (barrel OD at the slot, slot z, the groove
    angle, the ridge profile, and the cover's axial extent) are read
    from the paired housing; the cover owns its wall thickness, the slip
    clearance over the barrel, and the removal tabs."""

    equations = """
        barrel_od, wall_thk, cover_axial > 0
        groove_offset_deg > 0
        ridge_height, ridge_base, tab_radial, tab_thk > 0
        id_clearance >= 0
        # How far (arc length, mm) the band extends past the ridges on each
        # side — extra over-center wrap for circumferential grip, with the
        # removal tabs sitting flush at these ends.
        end_extension >= 0
        # Slip fit over the barrel, then the wall outward.
        id = barrel_od + 2 * id_clearance
        od = id + 2 * wall_thk
        # `z_center` is the slot's z in housing-world frame; declared via
        # the bottom-of-cover derivation (it may be any sign in general).
        cover_z_bot = z_center - cover_axial / 2
    """

    def build(self):                                       # framework hook: required; returns the shape
        import math
        r_barrel = self.barrel_od / 2.0
        r_in = self.id / 2.0
        r_out = self.od / 2.0
        z_bot = self.z_center - self.cover_axial / 2.0
        eps = default_eps()

        # Ridges (and the tabs over them) sit `groove_offset_deg` past
        # each slot edge, i.e. 90 + offset from the +Y center — standard
        # angles 90 ± ridge_a, which work out to −offset and 180 + offset.
        ridge_a = 90.0 + self.groove_offset_deg
        ridge_angles = (90.0 - ridge_a, 90.0 + ridge_a)
        # The band extends `end_extension` (arc length) past each ridge for
        # extra over-center wrap (circumferential grip); the removal tabs
        # sit flush at these ends. The ridges themselves stay at
        # `ridge_angles` (the housing grooves).
        end_a = ridge_a + math.degrees(self.end_extension / r_out)
        a0, a1 = 90.0 - end_a, 90.0 + end_a

        # Arc band, centered on +Y (90°): an annular sector (outer pie
        # minus the inner bore disc) extruded over the cover's axial
        # extent.
        band = difference(
            Sector(r=r_out, angles=(a0, a1)),
            circle(r=r_in),
        ).linear_extrude(height=self.cover_axial).up(z_bot)

        # Ridges: triangular axial prisms on the inner wall, apex inward
        # to r_barrel − ridge_height (so the apex reaches ridge_height
        # past the barrel surface, into the groove). Base overshoots eps
        # outward into the wall so the union closes without a coplanar
        # face. Matched to the housing grooves.
        tri = polygon(points=[
            (r_in + eps, -self.ridge_base / 2),
            (r_in + eps, +self.ridge_base / 2),
            (r_barrel - self.ridge_height, 0.0),
        ])
        ridge = tri.linear_extrude(height=self.cover_axial).up(z_bot)
        ridges = [ridge.rotate([0, 0, a]) for a in ridge_angles]

        # Tabs: a flat radial ear flush against each arc end, its body
        # extending back along the band so it fuses (rather than
        # overhanging into space), reaching tab_radial past the OD over the
        # full cover axial extent. a0 is the lower-angle end (band lies CCW
        # = +local-Y from it); a1 the upper end (band lies CW = −local-Y).
        def _tab(into_arc):
            y0, y1 = (0.0, self.tab_thk) if into_arc > 0 else (-self.tab_thk, 0.0)
            return polygon(points=[
                (r_out - eps, y0),
                (r_out + self.tab_radial, y0),
                (r_out + self.tab_radial, y1),
                (r_out - eps, y1),
            ]).linear_extrude(height=self.cover_axial).up(z_bot)
        tabs = [_tab(+1).rotate([0, 0, a0]), _tab(-1).rotate([0, 0, a1])]

        return union(band, *ridges, *tabs)

    def tight_bbox(self):                                  # framework hook: declare true post-Difference extents
        # build() unions an annular-sector Difference with the ridges and
        # tabs; the default tight_bbox would walk into that Difference and
        # raise. The tabs are the widest radial feature (OD + tab_radial),
        # so a disc of that radius bounds every feature; axially the cover
        # spans cover_axial centered on z_center.
        r = self.od / 2.0 + self.tab_radial
        half = self.cover_axial / 2.0
        return BBox(
            min=(-r, -r, self.z_center - half),
            max=(r, r, self.z_center + half),
        )


# =============================================================================
# CONCRETE: the specific S2 design
# =============================================================================


GRIP_DEPTH = 1.5
GRIP_WIDTH = 1.0

# Aperture-cover radial wall thickness. Single-sourced here because two
# parts depend on it: the cover wall itself, and the housing's pair of
# light-baffle fillet rings, whose radial extent matches the cover
# thickness so they sit flush with the cover's outer profile.
APERTURE_COVER_WALL_THK = 2.5

# Lens hood coupler overlap (axial length of the coupler tube that slides
# over the housing's upper barrel). Shared by the hood (coupler length)
# and the housing (which centers the hood-snap groove at the seated
# coupler midpoint = housing top − half this).
HOOD_COUPLER_OVERLAP = 6.0
# Hood-to-housing snap ring: a continuous 360° triangular bead on the
# hood coupler's inner wall seats into a matching groove in the housing's
# upper barrel — the same retention idea as the mating-disc bead, with
# the protrusion on the hood and the groove on the housing. `engage` is
# how far the bead apex reaches past the barrel OD (the snap
# interference); `axial` the bead's axial width; `clearance` the groove's
# per-face play past the bead. Tune `engage` for snap firmness on the
# actual filament.
HOOD_SNAP_ENGAGE = 0.3
HOOD_SNAP_AXIAL = 1.5
HOOD_SNAP_CLEARANCE = 0.1

# Hood label font: wide, heavy Verdana Bold. Thick strokes survive FDM at
# the shallow relief used for the labels. A fontconfig family name (the form
# OpenSCAD renders from); scadwright resolves it to a file for the real
# glyph metrics that drive sizing and centering.
LABEL_FONT = "Verdana:style=Bold"

# Bayonet contract — single source of truth for dimensions shared with
# the lens mount. The housing reads three values from `BronicaS2Bayonet`:
#   - mount_protrusion: axial extent of the lens-mount ring above the
#     camera flange (z=[0, mount_protrusion]). The M57-restricted lower
#     tube extends up to this z; the flange-flange rests on top of the
#     ring; the funnel starts here with a sharper angle so the housing
#     stays the same total length and no elements move in absolute z.
#   - lens_mount_od: OD of the lens-mount ring; sets `flange_flange_od`
#     so the two parts rest flush at the assembly seam.
#   - lens_mount_bore: inner bore of the ring; the mating disc fits
#     inside it (with MATING_DISC_CLEARANCE slip).
# Local alias for the most-used value — variant code stays readable.
MOUNT_OFFSET = BronicaS2Bayonet.mount_protrusion

# MATING_DISC_CLEARANCE is imported above from mating_disc_lock so both
# the housing (sizing the mating disc OD) and the locking ring (sizing
# the tab-notch cutout relative to the disc edge) agree on one value.

# Waterhouse stop: 1.5mm disc, 0.5mm slot clearance for sliding fit.
# Depth and hole-diameter list come from the design (defaults below or
# loaded from --config).
STOP_THK = 1.5
STOP_CLR = .5

# Built-in defaults (used unless --from-json overrides). Front to rear.
# `mounted_od` is optional and only applies to the element that carries it
# (its mount/cell is wider than the bare lens); the housing uses
# max(diameter, mounted_od) for clearance on those elements.
# `extra_radial` (mm) is added on top of DEFAULT_ELEMENT_CLEARANCES.radial
# for that element's holder — looser fit for elements whose OD measures
# slightly larger than the catalog dia or whose edge profile chips into
# the surround. Default 0 (use the project-wide radial clearance as-is).
DEFAULT_ELEMENTS_DATA = (
    {"code": "L7325",  "type": "NMN", "diameter": 52.0, "edge_thickness": 6.2, "spacing":  34.6, "extra_radial": 0.3, "extra_axial": -0.1},
    {"code": "L14523", "type": "ACH", "diameter": 35.0, "mounted_od": 44.4, "edge_thickness": 13, "spacing":  12.6, "extra_radial": 0.8},
    {"code": "L13088", "type": "BCX", "diameter": 42.5, "edge_thickness":  2.2, "spacing":  -9.4, "extra_radial": 0.3, "extra_axial": 0.1},
)
DEFAULT_STOP_DEPTH = -2.7
DEFAULT_STOP_HOLE_D = (20.0, 14.3, 10.0, 7.3)
DEFAULT_FOV_ANGLE = 57.0
DEFAULT_FOCAL_LENGTH = 89
DEFAULT_MAX_APERTURE = 2.9
DEFAULT_DESIGN_NAME = "lens_housing"


# CLI flag: suppress the lock mechanism (tabs + bead) in the
# `lower_housing_testfit` variant. Default: lock mech IS present, so the
# testfit piece exercises the same disc geometry the production housing
# uses against the locking lens-mount ring. Pass `--no_locking_mech` to
# print a plain disc instead (e.g. to test pre-lock-mech fit, or to
# isolate which feature is causing a fit problem).
NO_LOCKING_MECH = arg("no_locking_mech", default=False, type=bool)


# CLI flag: optional optical-diagram SVG engraved on the upper barrel
# opposite the text label. No default — omit `--optical_plan` and the
# engraving is simply skipped. The SVG is authored as filled paths (an
# arrow along the optical axis plus the element cross-sections); strokes
# don't import. See `_optical_plan_cutter` for placement.
OPTICAL_PLAN_SVG = arg("optical_plan", default=None, type=str)

# CLI flag: stamp the barrel even when the working tree has uncommitted
# changes. The stamp then carries a `-dirty` suffix, recording that the
# commit alone won't reproduce the geometry. Off by default so a printed
# part's SHA is always enough to recover the source that made it; turn it
# on for throwaway test prints mid-iteration.
ALLOW_DIRTY_STAMP = arg("allow_dirty", default=False, type=bool)

# OpenSCAD imports an SVG's `width`/`height` (px) at this DPI, so the
# imported geometry's mm extent is `px * 25.4 / SVG_IMPORT_DPI`. Verified
# against this OpenSCAD build by extruding the SVG and measuring the STL
# bbox; matches the convention documented in docs/wrap_2d.md.
SVG_IMPORT_DPI = 72.0


# Optional design override: invoke with `--from-json <path>`. The JSON
# carries `elements` (list of element dicts) and `stop` (dict with `depth`
# and `hole_diameters`); optionally `fov` (hood half-angle source), `name`
# (design label, surfaced as `design_name` for downstream use), and
# `focal_length` / `max_aperture` (engraved on the front fillet ring).
# `from_json(default=...)` deep-merges the JSON over `default` — recursive
# for dicts, replace-in-place for everything else — so missing keys (or
# no `--from-json` at all) fall through to the defaults automatically.
_cfg = from_json(default={
    "elements": DEFAULT_ELEMENTS_DATA,
    "stop": {"depth": DEFAULT_STOP_DEPTH, "hole_diameters": DEFAULT_STOP_HOLE_D},
    "fov": DEFAULT_FOV_ANGLE,
    "name": DEFAULT_DESIGN_NAME,
    "focal_length": DEFAULT_FOCAL_LENGTH,
    "max_aperture": DEFAULT_MAX_APERTURE,
    "inf_offset": 0,
})
_elements_data = _cfg["elements"]
# Axial focus correction (mm). Shifts the entire optical core — elements,
# their grippers and surrounds, and the stop (slot + holder grippers + discs)
# — along the lens axis to reach true infinity focus, correcting the gap
# between the approximate element/stop positions and the empirically-found
# ones. +z = toward the subject (away from the camera); negative shifts back
# toward the camera. Folded into the element spacings and the stop depth
# before any geometry is derived, so everything positioned off them moves
# with it; the camera-referenced mount/flange/lower housing stays fixed.
INF_OFFSET = float(_cfg["inf_offset"])
STOP_DEPTH = float(_cfg["stop"]["depth"]) + INF_OFFSET
STOP_HOLE_D = tuple(float(d) for d in _cfg["stop"]["hole_diameters"])
FOV_ANGLE = float(_cfg["fov"])
design_name = _cfg["name"]
FOCAL_LENGTH = float(_cfg["focal_length"])
MAX_APERTURE = float(_cfg["max_aperture"])

FRONT_FILLET_LABEL = f"BRIOTAR {FOCAL_LENGTH:g}mm f/{MAX_APERTURE:g}"
# Label on the front bezel, opposite the BRIOTAR label.
BEZEL_BACK_LABEL = "ZenzaNOT"

def _effective_dia(e):
    """Use the mount OD when wider than the bare lens; bare diameter otherwise."""
    d = float(e["diameter"])
    if "mounted_od" in e:
        return max(d, float(e["mounted_od"]))
    return d


def _element_clearances(e):
    """Per-element clearances. Starts from the project default and adds
    the dict's `extra_radial` (a DIAMETER delta — `clearances.radial`
    is added directly to the element diameter to get the holder's bore,
    so the actual per-side radial gap is `radial / 2`). Axial uses the
    default; override per-element with an `extra_axial` key if a future
    element needs it."""
    return ElementClearances(
        axial=DEFAULT_ELEMENT_CLEARANCES.axial + float(e.get("extra_axial", 0)),
        radial=DEFAULT_ELEMENT_CLEARANCES.radial + float(e.get("extra_radial", 0)),
    )


ELEMENTS = tuple(
    element(
        code=e["code"], type=e["type"],
        dia=_effective_dia(e), edge_thk=float(e["edge_thickness"]),
        spacing=float(e["spacing"]) + INF_OFFSET,
        grip_width=GRIP_WIDTH,
        mount_offset=BronicaS2Bayonet.mount_protrusion,
        clearances=_element_clearances(e),
    )
    for e in _elements_data
)
THE_STOP = Stop(spacing=STOP_DEPTH, thk=STOP_THK, clr=STOP_CLR)


# Stepped lap joint between the two printed housing halves. Confined to
# z >= MOUNT_OFFSET (the lap_split z_min in `_splayed_halves` below), so
# it only runs through the flange-flange / funnel / upper sections — the
# thin-walled stepped lower segments below the flange get a clean halve
# only. The upper wall is thick enough (5 mm radial) to host the step
# internally with both layers >=2.5 mm thick everywhere; no external rib
# needed. `LAP_MID_RADIUS = 29.5` sits roughly midway between the upper-
# section bore (r ≈ 26.5, set by outer's ID continued upward) and the
# flange/funnel/upper OD (r ≈ 31–34, depending on element widths).
LAP_OVERLAP = 3.0
LAP_MID_RADIUS = 29.5
# Press-fit clearance between the lap-joint tongue and groove: shrinks
# the tongue (NOT the groove) at the tip and at the radial step face,
# so the parts seat with ~LAP_CLEARANCE mm of play in both directions
# instead of binding on a tight FDM print. Outer (visible) cylinder
# surface is unchanged so the joint flushes at the wall.
# Was 0.2 — that snap was too aggressive: the joint went together fine
# but cracked the wall on disassembly. Bumped to 0.4 since the lens-
# mount ring and hood handle alignment, the lap joint just needs to
# keep the halves indexed (light-tight + radial register, not load-
# bearing).
LAP_CLEARANCE = 0.4
# Facet count of the keyed lap step. The step is a regular octagon, not a
# circle: its flats and vertices against LAP_CLEARANCE vary the seam gap
# (interference at the vertices, clearance at the flats) so the two halves
# snap/index together instead of sliding freely. Must divide the print
# `$fn` (8 divides 128) so the octagon's vertices land on the wall's and
# the seam stays manifold on export. Set to None for a free-sliding step.
LAP_KEY_SIDES = 8


class S2LensHousing(LensHousing):
    # Lower housing — three stepped segments sourced from the bayonet
    # spec. The M57 throat narrows in steps as the housing reaches deeper,
    # and each segment's OD matches one step. Outer is the flange-side
    # (widest, continues above the flange through the mount-ring zone);
    # mid carries the helicoid focus slot; inner is the deepest and
    # narrowest. Physical fit, not a design choice.
    lower_outer_od = BronicaS2Bayonet.lower_housing_outer_od
    lower_outer_len = BronicaS2Bayonet.lower_housing_outer_len
    lower_outer_thk = BronicaS2Bayonet.lower_housing_outer_thk
    lower_mid_od = BronicaS2Bayonet.lower_housing_mid_od
    lower_mid_len = BronicaS2Bayonet.lower_housing_mid_len
    lower_mid_thk = BronicaS2Bayonet.lower_housing_mid_thk
    lower_inner_od = BronicaS2Bayonet.lower_housing_inner_od
    lower_inner_len = BronicaS2Bayonet.lower_housing_inner_len
    lower_inner_thk = BronicaS2Bayonet.lower_housing_inner_thk
    # Upper section (flange-flange / funnel / upper) wall thickness: thick
    # enough to host the stepped lap joint internally (5 mm radial gives
    # 2.5/2.5 mm layers around mid_radius). The upper section sits outside
    # the camera body so its OD is unconstrained by the throat.
    upper_barrel_thk = 5.0   # radial wall (thick — hosts the lap joint internally)
    # Bore-side fillet ring angle at each interior step in the stacked
    # lower housing. 45° is the self-supporting threshold for FDM
    # overhangs printed lens-side-down (the slanted inner cone IS the
    # overhang surface). Steeper would print easier but consume more
    # axial extent inside the wider segment.
    bore_fillet_base_angle = 45.0
    # Alignment groove cross-section, sourced from the bayonet spec so
    # the housing's bottom-face groove and the lens-mount ring's outer-
    # wall groove match in profile.
    alignment_groove_depth = BronicaS2Bayonet.alignment_groove_depth
    alignment_groove_width = BronicaS2Bayonet.alignment_groove_width
    # Aperture-cover snap interface (read by the paired ApertureCover so
    # its ridges match these grooves). Grooves sit 15° past each stop-
    # slot edge, into the lower hemisphere, so the cover's >180° wrap
    # seats them past the barrel's widest line for an over-center snap.
    # Triangular profile 1.5 mm deep × 1.8 mm wide; the cover is 10 mm
    # axial; 0.1 mm per-face clearance. Deepened from 0.8 mm (and the
    # clearance tightened from 0.15) after a test print where the snap
    # barely engaged — 1.5 mm gives a real detent and the steeper, less-
    # tapered wedge bears on the groove flanks instead of rattling.
    aperture_cover_groove_offset_deg = 15.0
    aperture_cover_ridge_height = 1.5
    aperture_cover_ridge_base = 1.8
    aperture_cover_axial = 10.0
    aperture_cover_clearance = 0.1
    # Light baffles flanking the aperture cover: a 45° partial fillet ring
    # flush against each axial edge (tight 0.3 mm clearance), radial extent
    # = the cover's wall thickness, blocking light at the cover-to-wall gap.
    aperture_cover_wall_thk = APERTURE_COVER_WALL_THK
    aperture_cover_fillet_angle = 45.0
    aperture_cover_fillet_clearance = 0.3
    # Hood snap groove (receives the lens hood's coupler bead, centered at
    # the seated coupler midpoint). The bead lives on the hood; this
    # groove is the housing's only change for the hood snap.
    hood_snap_engage = HOOD_SNAP_ENGAGE
    hood_snap_axial = HOOD_SNAP_AXIAL
    hood_snap_clearance = HOOD_SNAP_CLEARANCE
    hood_coupler_overlap = HOOD_COUPLER_OVERLAP
    # The flange-flange is the wider band of the housing's wall (above the
    # lower segments) whose bottom rim rests on top of the lens-mount
    # ring. Set equal to the ring's OD (from the bayonet spec) so the
    # joint flushes at the seam.
    flange_flange_od = BronicaS2Bayonet.lens_mount_od
    flange_flange_len = 5.0
    fov_angle = FOV_ANGLE
    grip_depth = GRIP_DEPTH
    grip_width = GRIP_WIDTH
    mount_offset = BronicaS2Bayonet.mount_protrusion
    # Camera-side end of the mount ring, in the housing frame: the ring's
    # entry rim sits `insertion_depth` below the flange, so it occupies
    # z=[-insertion_depth, mount_offset]. Derived from the same bayonet-
    # spec terms the lens-mount ring uses for `insertion_depth` (lug axial
    # thickness + axial play + skirt). Drives the stop/ring overlap warning.
    mount_ring_z_bot = -(
        BronicaS2Bayonet.cam_lug_axial
        + BronicaS2Bayonet.lug_axial_clearance
        + BronicaS2Bayonet.mount_skirt
    )
    # Mating disc: bridges the radial gap between the outer segment and
    # the lens-mount ring's bore (from the bayonet spec). Disc OD = bore
    # − MATING_DISC_CLEARANCE (slip-fit, ~0.15 mm radial). Spans
    # z=[1.2, mount_offset] in the housing's world frame: the bottom is
    # 1.2 mm above the rotation channel's roof (the channel cavity tops
    # out at world z=0 = mount_offset − rest_z; that 1.2 mm of axial
    # clearance keeps the disc out of the channel and away from the
    # camera lugs that sit there when locked); the top flushes with the
    # lens-mount tube's top face at world z=mount_offset.
    mating_disc = MatingDisc(
        od=BronicaS2Bayonet.lens_mount_bore - MATING_DISC_CLEARANCE,
        z_bot=1.2, z_top=BronicaS2Bayonet.mount_protrusion,
    )
    # Tabs on the mating disc — shared module-level constant so the
    # locking lens-mount ring can import the same spec and cut matching
    # pockets in its bore wall.
    mating_disc_tabs = BRONICA_S2_MATING_DISC_TABS
    # Snap-fit retention bead — shared module-level constant so the
    # locking ring can cut the matching groove in its bore wall.
    mating_disc_bead = BRONICA_S2_MATING_DISC_BEAD
    # Helicoid focus slot: lives entirely within the mid segment. z_top
    # stays at -offset_from_flange (top of the slot, fixed by the camera-
    # body pin position). z_bot sits at the mid/inner step so the slot
    # never opens into the inner segment. The sweep portion extends CW
    # from the locked-side center by `lock_angle_deg` so the pin clears
    # at every angle as the lens rotates from insertion to locked.
    helicoid_focus_slot = SweptSlot(
        angle=BronicaS2Bayonet.helicoid_focus_slot_angle,
        width=BronicaS2Bayonet.helicoid_focus_slot_width,
        depth=BronicaS2Bayonet.helicoid_focus_slot_depth,
        z_bot=-(BronicaS2Bayonet.lower_housing_outer_len
                + BronicaS2Bayonet.lower_housing_mid_len),
        z_top=-BronicaS2Bayonet.helicoid_focus_slot_offset_from_flange,
        sweep_arc_deg=BronicaS2Bayonet.lock_angle_deg,
        # Slot's deep end sits exactly on the mid/inner step plane, so
        # the cutter overshoots past z_bot by `default_eps()` to drop
        # the otherwise-coplanar bottom face.
        bottom_overshoot=default_eps(),
    )
    # Aperture feeler slot: lives in the inner segment, directly opposite
    # the helicoid focus slot (3 o'clock vs the helicoid's 9). Same sweep
    # arc so the feeler clears at every angle during lens rotation. Axial
    # extent spans the entire inner segment — z_top at the mid/inner step
    # so the feeler doesn't catch on the slot's flange-side rim as the
    # lens slides in, z_bot at inner's deep end (the housing's deepest
    # point) with `default_eps()` overshoot past it so the slot opens
    # cleanly off the end of the throat rather than terminating on a
    # coplanar bottom face.
    aperture_feeler_slot = SweptSlot(
        angle=BronicaS2Bayonet.aperture_feeler_slot_angle,
        width=BronicaS2Bayonet.aperture_feeler_slot_width,
        depth=BronicaS2Bayonet.aperture_feeler_slot_depth,
        z_top=-(BronicaS2Bayonet.lower_housing_outer_len
                + BronicaS2Bayonet.lower_housing_mid_len),
        z_bot=-(BronicaS2Bayonet.lower_housing_outer_len
                + BronicaS2Bayonet.lower_housing_mid_len
                + BronicaS2Bayonet.lower_housing_inner_len),
        sweep_arc_deg=BronicaS2Bayonet.lock_angle_deg,
        bottom_overshoot=default_eps(),
    )
    elements = ELEMENTS
    stop = THE_STOP

    def lap_split_halves(self):
        """The two housing halves, lap-jointed at X=0 — but NOT yet laid
        out for the bed. Returned as a (minus, plus) pair, each in the
        housing's native upright orientation (optical axis along +Z).
        Callers post-process for whichever bed layout they want
        (`splayed_halves` adds `lap_splay`, `print_vertical` rotates
        lens-side-down).

        The lap is confined to z >= MOUNT_OFFSET (the thick-walled
        section above the mount ring); the thin-walled lower section
        below z=MOUNT_OFFSET gets a clean halve only.

        Each lap_split_x result is wrapped in `force_render()` so
        OpenSCAD computes the lap-jointed half once via CGAL on first
        compile and caches the resulting mesh; subsequent F5 frames
        (camera rotation in preview) draw the cached mesh instead of
        re-evaluating the deep CSG tree every frame.

        Engravings live OUTSIDE force_render: the bare structural half
        is what gets cached, and the engraving cutter is differenced in
        via preview-CSG per frame. Without this split, every glyph
        difference ends up inside the CGAL evaluation and preview-open
        time balloons. Both halves take the difference: the upper label
        and the BRIOTAR bezel label sit on the +X half (angle=0), while
        the ZenzaNOT bezel label (and the optional optical-plan diagram)
        sit on the -X half (angle=180). The cross-side cutters are no-ops
        (each cutter only meets the half it sits on)."""
        eng = self.engravings()
        minus_bare = self.lap_split_x(
            side="minus", overlap=LAP_OVERLAP, mid_radius=LAP_MID_RADIUS,
            z_min=MOUNT_OFFSET, clearance=LAP_CLEARANCE, key_sides=LAP_KEY_SIDES,
        )
        minus = difference(minus_bare.force_render(), eng).with_bbox_from(minus_bare)
        plus_bare = self.lap_split_x(
            side="plus", overlap=LAP_OVERLAP, mid_radius=LAP_MID_RADIUS,
            z_min=MOUNT_OFFSET, clearance=LAP_CLEARANCE, key_sides=LAP_KEY_SIDES,
        )
        plus = difference(plus_bare.force_render(), eng).with_bbox_from(plus_bare)
        return minus, plus

    def splayed_halves(self):
        """The two housing halves, lap-jointed at X=0 and laid flat for
        the bed (`lap_splay`). Returned as a (minus, plus) pair."""
        minus, plus = self.lap_split_halves()
        return (
            minus.lap_splay(side="minus", lap_overlap=LAP_OVERLAP),
            plus.lap_splay(side="plus", lap_overlap=LAP_OVERLAP),
        )


class S2LensHood(LensHood):
    wall_thk = 2.0
    # Slip clearance over the barrel, diametral. 0.15 (0.075/side) is a
    # snug slip — tighter than the old 0.25 — relying on the snap ring for
    # retention. The housing barrel is unchanged; this shrinks the coupler
    # bore.
    hood_clr = 0.15
    hood_length = 34.0   # cone axial length; flare_od follows from the fov angle
    coupler_overlap = HOOD_COUPLER_OVERLAP


class S2LensCap(LensCap):
    # Coupler fit mirrors S2LensHood exactly so the cap and hood share
    # the housing's barrel and snap groove (single source = the hood).
    # The lid is a 2 mm flat disc.
    wall_thk = S2LensHood.wall_thk
    hood_clr = S2LensHood.hood_clr
    coupler_overlap = S2LensHood.coupler_overlap
    cap_thk = 2.0


class S2ApertureCover(ApertureCover):
    # Cover-owned dimensions. The 2.5 mm wall over a ~68 mm barrel keeps
    # the C-clip stiff. The band extends 4 mm past each ridge (wrap ~222°)
    # for over-center grip — bumped from a flush-to-ridge ~214° after a
    # test print gripped too loosely; the removal tabs sit flush at those
    # ends (5 mm past the OD for a finger grip). The housing-driven values
    # (barrel OD at the slot, slot z, groove angle, ridge profile, axial
    # extent) come from the Design wiring below.
    wall_thk = APERTURE_COVER_WALL_THK
    id_clearance = 0.2
    tab_radial = 5.0
    tab_thk = 2.5
    end_extension = 4.0


# =============================================================================
# DESIGN: shared parts + variant methods
# =============================================================================


def _make_stop_discs(housing, hole_diameters):
    """One `WaterhouseStop` per hole diameter, all sharing the same
    barrel dimensions, thickness, and depth. Disc diameters are sourced
    from the housing at the stop's z, so the inner semicircle fills the
    local bore and the outer semicircle flushes with the local wall OD
    regardless of which section (lower / flange / funnel / upper) the
    stop sits in. Lives at module scope (not as a Design method)
    because Python comprehensions inside a class body can't see the
    surrounding class attributes — `housing.wall_id_at_z(...)` only
    resolves when called from a function or module-level scope."""
    return tuple(
        WaterhouseStop(
            inner_diameter=housing.wall_id_at_z(STOP_DEPTH),
            outer_diameter=housing.wall_od_at_z(STOP_DEPTH),
            thk=STOP_THK,
            hole_d=h,
            spacing=STOP_DEPTH,
        )
        for h in hole_diameters
    )


def _make_shape_stops(housing, equiv_dia):
    """One each of the non-circular stops (square, hexagon, almond), all
    sized so their aperture matches the area of a circle of diameter
    `equiv_dia` — i.e. they pass the same light as the round stop of that
    f-number. Barrel dimensions come from the housing at the stop's z, the
    same way `_make_stop_discs` sizes the round discs."""
    common = dict(
        inner_diameter=housing.wall_id_at_z(STOP_DEPTH),
        outer_diameter=housing.wall_od_at_z(STOP_DEPTH),
        thk=STOP_THK,
        spacing=STOP_DEPTH,
        equiv_area_circle_dia=equiv_dia,
    )
    return (SquareStop(**common), HexStop(**common), AlmondStop(**common))


class LensHousingDesign(Design):
    name = design_name    # output filename stem; from JSON or DEFAULT_DESIGN_NAME
    housing = S2LensHousing()
    hood = S2LensHood(
        housing_upper_od=housing.upper_housing_od,
        hood_base_angle=housing.hood_base_angle,
        # Snap bead matches the groove the housing cuts — read its profile
        # off the housing so the two stay in step.
        snap_engage=housing.hood_snap_engage,
        snap_axial=housing.hood_snap_axial,
        label=FRONT_FILLET_LABEL,
        back_label=BEZEL_BACK_LABEL,
        label_font=LABEL_FONT,
    )
    # Lens cap: the hood's snapping coupler with a flat lid instead of the
    # funnel. Same coupler fit and snap bead (read off the housing) so it
    # clips onto the same barrel; "ZenzaNOT" inset on the outward face.
    lens_cap = S2LensCap(
        housing_upper_od=housing.upper_housing_od,
        snap_engage=housing.hood_snap_engage,
        snap_axial=housing.hood_snap_axial,
        label=BEZEL_BACK_LABEL,
        label_font=LABEL_FONT,
    )
    stop_discs = _make_stop_discs(housing, STOP_HOLE_D)
    # Clip-on aperture cover, reading the shared interface off the
    # housing: the wall OD and z at the stop slot, the snap-groove angle,
    # the ridge profile, and the cover's axial extent. Its own wall,
    # wrap, slip clearance, and tabs live on S2ApertureCover.
    aperture_cover = S2ApertureCover(
        barrel_od=housing.wall_od_at_z(housing.stop.spacing),
        z_center=housing.stop.spacing,
        cover_axial=housing.aperture_cover_axial,
        groove_offset_deg=housing.aperture_cover_groove_offset_deg,
        ridge_height=housing.aperture_cover_ridge_height,
        ridge_base=housing.aperture_cover_ridge_base,
    )

    @variant(fn=128, default=True)
    def print_vertical(self):
        # Both halves printed vertically (optical axis = print Z), lens-
        # side-down: the OD step at the flange is an inward step (no
        # overhang), the lap-joint features get first-layer accuracy at
        # the bed, and the wider lens-side face gives a generous bed-
        # contact area. Suited for the Bambu X2D's dual-head PETG-on-PLA
        # support workflow — element-gripper undersides print over PETG
        # and break off cleanly. Trades ~25% extra wall-clock vs the
        # horizontal `print` variant for smoother bore circumferential
        # finish and stronger resistance to bayonet-engagement torque.
        minus, plus = self.housing.lap_split_halves()

        # Lens-side-down: rotate 180° about X (flips Y and Z, keeps X
        # so minus stays at world X<=0 and plus at world X>=0). The
        # lens-side face ends up below z=0; arrange_on_bed below uses
        # tight_bbox to lift it back to the bed (LensHousing's
        # tight_bbox override gives the truncated fillet rim's true
        # z-extent, so the rim lands flush with z=0).
        minus_v = minus.rotate([180, 0, 0])
        plus_v = plus.rotate([180, 0, 0])

        gap = 12   # ~1.2cm clearance between objects

        # Place each half so its cut face sits at world X = ±gap/2. Lap
        # tongues extend at most LAP_OVERLAP - LAP_CLEARANCE past the cut
        # face, so the actual material gap between the two halves is
        # gap - 2*(LAP_OVERLAP - LAP_CLEARANCE) — comfortably non-zero
        # for any reasonable gap. arrange_on_bed can't preserve this overlap
        # if the halves are passed in individually (it would space them
        # apart by `gap`), so the pair is bonded into one unit first and
        # then laid out alongside the stop disc.
        pair = union(minus_v.left(gap / 2), plus_v.right(gap / 2))

        # flat_stop = self.stop_discs[0].down(STOP_DEPTH - STOP_THK / 2)
        # return arrange_on_bed(pair, flat_stop, gap=gap, plate=BAMBU_X2D_PLATE)
        return arrange_on_bed(pair, gap=gap, plate=BAMBU_X2D_PLATE)

    @variant(fn=60)
    def print_vertical_preview(self):
        # Same geometry as `print_vertical`, rendered at lower fn for a
        # fast preview. SCAD-render only; not for slicing.
        return self.print_vertical()

    @variant(fn=128)
    def print_hood(self):
        # Hood by itself, flare-down on the bed (the wide flare gives
        # the largest bed-contact footprint and the coupler / fillet
        # detail prints upward without overhangs). The coupler's label
        # is baked into `LensHood.build()` (via `add_text` on the inner
        # Tube before assembly), so it travels with the hood through
        # the flip and lands oriented correctly on the print surface.
        return arrange_on_bed(self.hood.rotate([180, 0, 0]), plate=BAMBU_X2D_PLATE)

    @variant(fn=128)
    def print_lens_cap(self):
        # Flat disc face-down on the bed: maximum adhesion, the inset
        # label prints crisp against the glass, and the inward snap bead
        # is a shallow self-supporting overhang. Built coupler-up with the
        # disc on top, so flip 180° about X to put the disc on the bed.
        return arrange_on_bed(self.lens_cap.rotate([180, 0, 0]), plate=BAMBU_X2D_PLATE)

    @variant(fn=128)
    def print_aperture_cover(self):
        if not self.housing.stop_externally_accessible():
            raise ValueError(
                "No aperture cover for this design: the stop slot is recessed "
                "into the mount-ring zone (below the flange), where the lens-"
                "mount ring covers it. Move the stop above the flange (a less "
                "negative inf_offset) to use an external cover."
            )
        # Aperture cover alone, axis vertical, standing on its bottom ring
        # face. The cover is built in housing-world frame with the lens
        # axis already along +Z, so no rotation is needed — the ridges and
        # tabs run vertically (no overhangs). arrange_on_bed drops it to
        # the bed via tight_bbox. The footprint is a thin arc ring, so a
        # brim at slice time helps first-layer adhesion.
        return arrange_on_bed(self.aperture_cover, plate=BAMBU_X2D_PLATE)

    @variant(fn=48)
    def print_stops(self):
        # Four waterhouse stop discs (f/4, f/5.6, f/8, f/11 — indices 0-3
        # of the JSON `hole_diameters` list), laid flat in a 2x2 square.
        # Adjust the slice if the JSON list ordering changes.
        #
        # arrange_on_bed packs parts left-to-right and only wraps to a new
        # row when the plate width runs out, so on a wide bed all four land
        # in one line. Place the 2x2 grid by hand instead, then hand the
        # bonded block to arrange_on_bed for the drop-to-bed and fit check.
        # The discs share one outer diameter, so a single pitch (OD + gap)
        # spaces every column and row without overlap.
        discs = [self.stop_discs[i] for i in (0, 1, 2, 3)]  # f4, f5.6, f8, f11
        pitch = self.stop_discs[0].outer_diameter + 10
        grid = union(*(
            d.right((i % 2) * pitch).forward((i // 2) * pitch)
            for i, d in enumerate(discs)
        ))
        return arrange_on_bed(grid, plate=BAMBU_X2D_PLATE)

    @variant(fn=48)
    def print_shape_stops(self):
        # One square, one hexagon, one almond stop, laid flat in a row. All
        # three match the area of the f/5.6 circular aperture (index 1 of
        # the JSON `hole_diameters`), so they pass the same light as the
        # round f/5.6 stop but shape the aperture differently.
        shapes = _make_shape_stops(self.housing, STOP_HOLE_D[1])  # f/5.6
        return arrange_on_bed(*shapes, gap=10, plate=BAMBU_X2D_PLATE)

    @variant(fn=48)
    def design_spread_out(self):
        # Inspection layout: hood + the two distinct splayed halves
        # (showing the lap joint) + one stop disc per entry in
        # `STOP_HOLE_D`, all in a single +X row. Too large for the print
        # bed; use the `print` variant for actual printing — `assert_fit`
        # is disabled so this layout is allowed to overflow.
        minus, plus = self.housing.splayed_halves()
        flipped_hood = self.hood.flip("z")   # wide flare on the print bed
        flat_stops = [
            disc.down(STOP_DEPTH - STOP_THK / 2) for disc in self.stop_discs
        ]
        gap = self.housing.upper_housing_od   # ~one barrel-diameter of breathing room

        return arrange_on_bed(
            flipped_hood, minus, plus, *flat_stops,
            gap=gap, assert_fit=False,
        )

    @variant(fn=48)
    def display(self):
        # Lift the hood one coupler_overlap above the true housing top (which
        # sits above upper_housing_len because of the front fillet ring) so
        # the housing's upper rim is visible between the two parts. The
        # widest-aperture stop disc shows fully inserted at its real depth.
        housing_top_z = self.housing.bbox.max[2]
        gap = self.hood.coupler_overlap
        parts = [
            difference(self.housing, self.housing.engravings()),
            self.hood.up(housing_top_z + gap),
            self.stop_discs[0],
        ]
        # The cover only exists for an externally accessible slot; omit it
        # when the stop is recessed behind the mount ring.
        if self.housing.stop_externally_accessible():
            parts.append(self.aperture_cover.color("orange"))
        return union(*parts)


if __name__ == "__main__":
    run()
