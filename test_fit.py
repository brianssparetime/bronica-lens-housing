"""Trial-fit and inspection variants for the lens housing.

Diagnostic pieces that exercise individual fits without committing to a
full housing print — kept apart from `lens_housing.py`'s production
print/display variants so each file's variant list stays focused. The
housing model itself (components, JSON config, constants) lives in
`lens_housing.py`; this file imports it and adds its own Design.

Run:
    python test_fit.py --variant=lower_housing_testfit --from-json <design>.json
    python test_fit.py --variant=element_grippers     --from-json <design>.json
    python test_fit.py --variant=half_housing         --from-json <design>.json
"""

from scadwright.boolops import difference, intersection, union
from scadwright.composition_helpers import arrange_on_bed
from scadwright.design import Design, run, variant
from scadwright.primitives import cube
from scadwright.shapes import Sector

from printer import BAMBU_X2D_PLATE
from lens_internals import ElementTestGripper
from lens_housing import (
    GRIP_DEPTH,
    GRIP_WIDTH,
    NO_LOCKING_MECH,
    S2LensHousing,
    design_name,
)
from rear_lens_cap import BronicaS2CameraMount
from lens_mount_ring import BronicaS2LensMount, BronicaS2LensMountSupports


class TestFitDesign(Design):
    name = design_name    # output filename stem; from JSON or DEFAULT_DESIGN_NAME
    housing = S2LensHousing()
    # Dedicated housing instance for the `lower_housing_testfit` variant
    # that always includes ALL three lower segments (outer + mid + inner)
    # regardless of element/stop placement. The production `housing` trims
    # unused mid/inner segments; this one doesn't, so the printed test
    # piece exercises the full structural lower-housing geometry against
    # the camera's stepped throat.
    testfit_housing = S2LensHousing(force_all_lower_segments=True)
    # Lens-mount ring + its modeled supports, for the `mount_ring_slice`
    # variant. Sized off the same camera mount as the production ring in
    # lens_mount_ring.py, so the slice matches the printed part.
    camera_mount = BronicaS2CameraMount()
    lens_mount = BronicaS2LensMount.from_camera_mount(camera_mount)
    lens_mount_supports = BronicaS2LensMountSupports.from_camera_mount(camera_mount)

    @variant(fn=60)
    def lower_housing_testfit(self):
        """One-piece truncated housing for trial-fitting against the
        camera throat. Strips the assembled housing down to the
        structural lower wall (outer + mid + inner segments + mating
        disc) plus the bottom 2 mm of the flange-flange rim, with the
        helicoid focus slot still cut. No element holders, no stop
        slot/holder/disc, no front fillet, no lap split — slip the
        printed result into the camera body to check stepped-OD
        clearance and helicoid-pin clearance without committing to a
        full housing print. Element data is irrelevant; defaults or
        any --from-json design produce the same geometry.

        Lens-side-down orientation (matches print_vertical): rotated
        180° about X so the truncated flange-flange rim (the widest
        cross-section) sits on the bed for solid adhesion. The throat
        extends upward; every OD step is inward in this orientation,
        so no support material is needed."""
        h = self.testfit_housing
        barrel = union(*h._sections())
        if h.helicoid_focus_slot is not None:
            barrel = difference(
                barrel,
                h._swept_slot_cutter(h.helicoid_focus_slot, h.lower_mid_od),
            )
        if h.aperture_feeler_slot is not None:
            barrel = difference(
                barrel,
                h._swept_slot_cutter(h.aperture_feeler_slot, h.lower_inner_od),
            )
        barrel = difference(barrel, h._alignment_groove_cutter())
        # Lock mechanism (tabs + bead) — present by default so the
        # testfit piece exercises the same disc geometry the production
        # housing presents to the locking lens-mount ring. Pass
        # `--no_locking_mech` to suppress both together (e.g. to test
        # pre-lock-mech fit, or isolate which feature is binding).
        if not NO_LOCKING_MECH:
            if h.mating_disc_tabs is not None and h.mating_disc is not None:
                barrel = union(barrel, h._mating_disc_tabs_geometry())
            if h.mating_disc_bead is not None and h.mating_disc is not None:
                barrel = union(barrel, h._mating_disc_bead_geometry())
        # Trim everything above the bottom 2 mm of the upper housing.
        # The slab spans z = [z_bot, z_top] generously past the housing's
        # vertical extent on the bottom side (overshoot is harmless;
        # intersection clips to the barrel).
        z_top = h.mount_offset + 2.0
        z_bot = -h.housing_insertion_depth - 1
        big = 200.0
        trimmer = cube([big, big, z_top - z_bot], center="xy").up(z_bot)
        trimmed = intersection(barrel, trimmer)
        # Lens-side-down: rotate so the rim faces the bed, then lift so
        # the (formerly top) trim plane lands at z=0.
        return trimmed.rotate([180, 0, 0]).up(z_top)

    @variant(fn=96)
    def element_grippers(self):
        # Per-element diagnostic half-grippers for test-fitting individual
        # elements without printing a full housing. Each gripper is the
        # same three-tube stack as ElementHolder (rim / middle / rim) with
        # rim and middle inner diameters and axial extents matched to the
        # housing exactly — a successful fit here predicts a fit in the
        # real part. Outer wall is a uniform 2 mm-walled cylinder per
        # gripper (sized off the middle tube; rim tubes come out thicker).
        # Halved at X=0 keeping +X so each gripper prints as a half-shell
        # with the element code engraved on its outer apex (the anchor
        # sits at +X, so engravings stay on the kept half). Standing up
        # along Z; arrange_on_bed lays them along +X in elements-tuple order.
        halves = []
        for e in self.housing.elements:
            g = ElementTestGripper(
                element=e,
                grip_depth=GRIP_DEPTH,
                grip_width=GRIP_WIDTH,
                wall_thk=2.0,
            )
            cutter = g.text_geometry(
                label=e.code, on="outer_wall", angle=0,
                font_size=2.5, relief=-0.4,
            )
            # with_bbox_from(g): difference()'s bbox can't be tightened
            # without CSG evaluation, but the engraving only removes
            # material from the gripper's outer face, so the true bbox
            # is the gripper's. Without this, halve()'s Intersection
            # bbox can't be clipped and arrange_on_bed below errors out.
            engraved = difference(g, cutter).with_bbox_from(g)
            halves.append(engraved.halve([1, 0, 0]))
        return arrange_on_bed(*halves, gap=10, plate=BAMBU_X2D_PLATE)

    @variant(fn=48)
    def half_housing(self):
        # One splayed half (the -X half, with lap-joint features visible)
        # for inspecting a single shell in isolation.
        minus, _ = self.housing.splayed_halves()
        minus_bb = minus.bbox
        gap = self.housing.upper_housing_od
        return minus.left(minus_bb.max[0] + gap / 2)

    @variant(fn=96)
    def mount_ring_slice(self):
        """A 1/12 (30°) wedge of the lens-mount ring, sliced through a
        rotation channel so the channel — and the modeled support filling
        it — both cross the cut fully. The exposed cross-section lets the
        axial stack (rest plane, rotation-channel floor/ceiling, bayonet
        lip, entry rim, bore) be checked by eye for anything that doesn't
        line up. Support shown in red.

        Centered in the rotation-only zone CCW of the widest lug's entry
        slot (`c - w/2 - lock/2`): there the rotation channel (~109° wide)
        and its in-channel support filler (~59° wide) both span well past
        the 30° cut on either side, so each cut face shows a clean channel
        section with its capturing lip — not the open entry slot (which
        would otherwise mask the channel where the two overlap)."""
        lm = self.lens_mount
        c, w, _ = lm.lugs[0]                       # widest lug
        center = c - w / 2 - lm.lock_angle_deg / 2
        half = 15.0                                # 30° = one twelfth
        wedge = (
            Sector(r=lm.outer_or + 4.0, angles=(center - half, center + half))
            .linear_extrude(height=lm.h + 2.0)
            .down(1.0)
        )
        return union(
            intersection(lm, wedge),
            intersection(self.lens_mount_supports, wedge).red(),
        )


if __name__ == "__main__":
    run()
