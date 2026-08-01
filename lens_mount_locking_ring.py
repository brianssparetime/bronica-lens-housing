"""Bronica S2 lens-mount LOCKING with a sewing-pin pivot latch.

Fresh attempt at the latch problem that doesn't rely on a printed
hinge/spring (the failure mode of every prior design). The lens-mount
ring grows two integrated pillow-block bosses on its OD wall; a separate
4 mm-tall curved lever sits between them and pivots on a single sewing
pin (~0.6 mm dia) that drops through the upper block, the lever, and
the lower block. Geometry is modelled on the stock Bronica latch:

  * see-saw kinematics — pivot in the middle of the lever, tooth at one
    tangential end, press extension at the other. Pressing the press end
    inward rotates the lever around the pin and swings the tooth outward
    (radially out of the rotation channel, releasing the lug).
  * both inner and outer profiles are arcs concentric with the lens
    (curvature matches the lens-ring OD and the focus-collar bore)
  * the lever sits well inside the focus-collar gap (outer face at
    ~outer_or + 2.5 mm, leaving ~6 mm of headroom) so the swing of
    either end never reaches the surround
  * chisel tooth dipping inward from the lever's inner face, swinging
    radially through a notch in the OD wall into the rotation channel

Bias toward locked (the role the printed flex used to serve) is
deferred — eventually a small torsion spring on the pin; for now the
lever is free to swing.

Run:
    python s2-evolving/lens_mount_locking_ring.py
"""

import math

from scadwright import Component
from scadwright.api.tolerances import default_eps
from scadwright.boolops import difference, union
from scadwright.composition_helpers import arrange_on_bed
from scadwright.design import Design, run, variant
from scadwright.errors import SCADwrightError
from scadwright.primitives import cylinder, polygon
from scadwright.shapes import Arc

from mating_disc_lock import (
    BRONICA_S2_MATING_DISC_BEAD,
    BRONICA_S2_MATING_DISC_TABS,
    HOUSING_GROOVE_ANGLE_DEG,
    MatingDiscBead,
    MatingDiscTabs,
)
from printer import BAMBU_X2D_PLATE
from rear_lens_cap import BronicaS2CameraMount
from lens_mount_ring import (
    BronicaS2LensMount,
    BronicaS2LensMountSupports,
    LensMount,
    LensMountSupports,
)
from spec import BronicaS2Bayonet


def _tooth_pivot_angles(c, w, lock_angle_deg, tooth_clearance_deg, tooth_w_deg,
                        notch_angular_pad_deg,
                        tooth_to_pivot_arc_deg, pivot_to_press_arc_deg):
    """Lens-frame angles for the tooth, the pivot pin, and the press end.

    Anchor: the LOCKED-lug CCW edge — where the lug sits when fully
    rotated from entry into the locked seat. The lug rotates CW
    (decreasing angle) from entry to locked by `lock_angle_deg`, so
    its CCW edge lands at (c − lock_angle_deg) + w/2. The NOTCH'S CW
    edge sits one `tooth_clearance_deg` CCW of that boundary; the
    tooth is centered in the notch. Result: the tooth's straight
    (CW-facing, blocking) face sits right against the locked lug's
    CCW edge, so the lug at locked is sandwiched between the
    rotation-channel CW endwall and the tooth — very little
    rotational play.

    Physical lug-to-tooth-face clearance (the actual play between
    locked and the tooth catching during unlock-rotation):
        physical_gap_deg = tooth_clearance_deg + notch_angular_pad_deg
    The notch-pad sits between the notch wall and the tooth's CW
    face, so it adds to the lug-to-face gap.

    The notch lives at lens-frame ~285°-290° for the Bronica third
    lug — well inside the rotation channel (244°-344.5°), so the
    bayonet lip exists at the notch's azimuth to host it.
    """
    notch_half_deg = tooth_w_deg / 2 + notch_angular_pad_deg
    locked_lug_ccw_edge = (c - lock_angle_deg) + w / 2
    notch_cw_edge = locked_lug_ccw_edge + tooth_clearance_deg
    tooth_center = notch_cw_edge + notch_half_deg
    pivot_center = tooth_center + tooth_to_pivot_arc_deg
    press_center = pivot_center + pivot_to_press_arc_deg
    return tooth_center, pivot_center, press_center


class LockingLensMount(LensMount):
    """`LensMount` with two integrated pillow-block bosses and a notch
    through the OD wall for the latch tooth.

    The pillow blocks share a vertical pin axis at `pivot_center_deg`,
    radially `pillow_pin_radial_inset` outside the OD. Lower block sits
    below the lever's axial range; upper block sits above it; both have
    a tangentially-extruded "A" plan profile (truncated-triangle base
    flush on the OD, semicircular head outside) with an axial hole at
    the head's center for the sewing pin. The notch is a small
    rectangular cutter through the OD wall at the tooth's angular
    position so the tooth (carried by the separate lever) can swing
    radially in and out of the rotation channel.

    The base `LensMount` cutters (bore, entry channels, rotation
    channels, alignment groove) are re-emitted here so the notch joins
    the same Difference.
    """

    # Pin and pillow-block geometry
    pin_dia = 0.6                  # sewing-pin nominal diameter (mm)
    pillow_hole_d = 1.5            # block hole — 50% wider than before so
                                   # the cold pin (0.6 mm) drops in without
                                   # any force. Earlier 1.0 design printed
                                   # too tight on FDM (effective ~0.8 mm
                                   # post-shrink); user tried to widen with
                                   # a heated needle and the whole A-block
                                   # melted before the wall could yield.
                                   # 1.5 → effective ~1.3 mm in-print → pin
                                   # has ~0.35 mm clearance per side.
                                   # Matches the lever pivot hole size.
    # Axial thickness of each A-block is DERIVED (see `_pillow_axial`):
    # the lever and the spring nub are fixed-height, so the blocks take
    # up the slack between them and the ring's overall axial extent (h),
    # splitting it evenly. Floored at 0.75 mm — below that they're too
    # thin to hold the pin / print, and a too-small `mount_skirt` is the
    # likely cause, so we fail loudly rather than overlap the lever.
    pillow_axial_min = 0.75
    pillow_lever_axial_gap = 0.2   # air gap between block and lever face
    pillow_base_width_tan = 6.0    # tangential width at OD (mm) — wider than
                                   # the head so the truncated-triangle base
                                   # gives a stable footprint on the OD
    pillow_pin_radial_inset = 2.45 # distance from OD to pin axis (mm). Pin
                                   # at outer_or + 2.45 = 36.50. Brought
                                   # in from 2.8 to make room for the
                                   # larger head_r (= more wall around the
                                   # 1.5 mm pin hole) while keeping the
                                   # A-block outer (= pin + head_r) inside
                                   # the focus collar's narrow lower bore.
    pillow_head_r = 1.55           # semicircle radius around the hole (mm).
                                   # 1.55 leaves 0.80 mm wall around the
                                   # 1.5 mm pin hole (was 0.70 around the
                                   # old 1.0 hole) and keeps the A-block
                                   # outer at outer_or + 2.45 + 1.55 = 38.05
                                   # (0.20 mm clear of the narrow lower
                                   # collar bore at radius 38.25).

    # Lever envelope (defines the pillow z-stacking and the notch height).
    # The lever ITSELF is built in `PivotLatchLever`; only `lever_axial`
    # is needed here (mount uses it in `_lever_z()` to position the
    # pillows). Other lever-shape params (radial thickness, tapers,
    # corner fillet, swing range) live on `BronicaS2PivotLatchLever`.
    lever_axial = 4.0
    # Axial height of the spring nub — the circular post on the lever's
    # top face that the torsion-spring coil wraps. Fixed (like the lever)
    # so the spring fit is stable when `h` changes; the A-blocks absorb
    # any axial slack instead. Set to 1.0 so the derived A-blocks land at
    # 1.40 mm at the current mount_skirt (h=8.2).
    pivot_nub_axial = 1.0
    tooth_to_pivot_arc_deg = 22.0  # tooth-center → pivot
    pivot_to_press_arc_deg = 22.0  # pivot → press-end (symmetric body around
                                   # the pivot)

    # Tooth geometry (shared contract with `PivotLatchLever`)
    tooth_angular_width_deg = 5.0
    tooth_clearance_from_locked_lug_deg = 0.5  # Angular gap from the
                                   # locked lug's CCW edge to the notch's
                                   # CW edge. Physical lug-to-tooth-face
                                   # gap = this + `notch_angular_pad_deg`
                                   # (the notch-pad sits between the notch
                                   # wall and the tooth's CW face). With
                                   # 0.5° + 1.0° = 1.5° → ~0.9 mm at the
                                   # OD: lug can rotate ~1.5° CCW from
                                   # locked before the tooth catches it.
    tooth_axial_clearance = 0.05   # axial gap between tooth and channel walls
    tooth_axial_thk = 3.0          # tooth axial thickness. ~2.6× the lug
                                   # (1.15 mm), ~75% of the lever (4 mm).
                                   # Aligned with the lever top (tooth_z_hi
                                   # = lever top); extends down by this
                                   # amount; bottom hangs in mid-print and
                                   # is held up by the tooth support
                                   # beneath. Provides full lug overlap
                                   # (z=[5.0, 6.15]) for blocking strength
                                   # plus structural mass for stiffness.
    notch_angular_pad_deg = 1.0    # angular pad each side of the tooth in
                                   # the notch. The lever's swing_deg=5 around
                                   # the pin moves the tooth ~1.2 mm at its
                                   # tip — almost all radial; tangential
                                   # component is only ~0.4° in lens-frame at
                                   # the tooth's radial. 1° pad covers that
                                   # plus print tolerance and filler-removal
                                   # margin without making the slot any wider
                                   # than necessary.
    notch_axial_pad = 0.5          # axial pad on each side of the tooth —
                                   # generous so the tooth doesn't bind on
                                   # the notch ceiling/floor during the
                                   # see-saw swing (0.1 mm was too tight
                                   # after print tolerances)

    # Lens-frame center of the lug the latch anchors to. The Bronica
    # original engages the third lug (around 324.25°), not the widest
    # one — that puts the latch at ~12 o'clock when the alignment
    # marker is at 9 o'clock (the reference frame the user described),
    # matching the Bronica position. The Design class uses the same
    # selector, so the pillow blocks, notch cut, and lever pivot all
    # land on the same lug.
    latch_lug_target_deg = 324.25

    # Tabs on the lens housing's mating disc mate with pockets cut into
    # this ring's bore wall. Spec lives in mating_disc_lock.py so both
    # parts share one source of truth.
    mating_disc_tabs = BRONICA_S2_MATING_DISC_TABS
    # Snap-fit retention bead on the disc seats into a matching groove
    # cut into the bore wall. Provides axial retention.
    mating_disc_bead = BRONICA_S2_MATING_DISC_BEAD

    def _mating_disc_tab_notches(self):
        """Cuboid cutters for the tabs on the lens housing's mating
        disc. One cutter per tab angle. Each cutter:
          - radial: from bore_inner_r (overshooting inward by 0.5 mm
            for a clean cut) outward to (tab outer + clearance)
          - tangential: ±(circ/2 + clearance) at the tab's center angle
          - axial: from (h − tab.axial − clearance) overshooting past
            the ring's top edge for a clean cut

        Tab angles are in HOUSING-FRAME degrees; converted to
        ring-frame here by subtracting the groove offset so the two
        alignment grooves (housing at housing-frame 90°, ring at
        ring-frame groove_angle_deg) line up at lock.
        """
        tabs = self.mating_disc_tabs
        if tabs is None:
            return None
        disc_r = tabs.disc_od / 2
        c = tabs.clearance
        radial_overshoot = default_eps()
        axial_overshoot = default_eps()
        x_inner = self.bore_inner_r - radial_overshoot
        x_outer = disc_r + tabs.radial + c
        y_half = tabs.circumferential / 2 + c
        # tabs.z_top is in HOUSING-frame; the housing's z=0 sits at the
        # camera-flange plane and z increases toward the lens. In the
        # ring frame z=0 is the LENS side (bottom flange) and z=h is
        # the CAMERA side (bayonet lip, where the entry channels are
        # open). So housing z X corresponds to ring z =
        # (h − insertion_depth) − X, putting the tab top (housing z =
        # tabs.z_top = mount_protrusion) at ring z = 0 — flush with
        # the ring's lens-side bottom face.
        tab_lens_end = (self.h - self.insertion_depth) - tabs.z_top
        tab_camera_end = tab_lens_end + tabs.axial
        z_bot = tab_lens_end - axial_overshoot
        z_top = tab_camera_end + c
        cutter_2d = polygon(points=[
            (x_inner, -y_half),
            (x_outer, -y_half),
            (x_outer, +y_half),
            (x_inner, +y_half),
        ])
        cutter_3d = cutter_2d.linear_extrude(height=z_top - z_bot).up(z_bot)
        ring_to_housing_offset = HOUSING_GROOVE_ANGLE_DEG - self.groove_angle_deg
        return union(*(
            cutter_3d.rotate([0, 0, housing_theta - ring_to_housing_offset])
            for housing_theta in tabs.angles_deg
        ))

    def _mating_disc_bead_groove_cutter(self):
        """Triangular groove cut into the ring's bore wall to receive
        the disc's snap-fit bead. Continuous 360°, triangular cross-
        section matching the bead — apex `groove_radial_clearance` past
        the bead's apex, axial width `bead.axial + 2 * axial_clearance`,
        centered at the same z as the bead.

        Built as a 2D triangle in (radial, axial) and rotate-extruded
        around the z axis. The triangle's BASE (radially inward face)
        overshoots the bore wall slightly so the cut union with the
        bore cavity is clean.
        """
        bead = self.mating_disc_bead
        if bead is None:
            return None
        # Bead apex sits at radius (disc_r + bead.height). Groove apex
        # past that by groove_radial_clearance so the bead seats with
        # play. Disc r derived from the tabs spec (same disc).
        if self.mating_disc_tabs is None:
            return None
        disc_r = self.mating_disc_tabs.disc_od / 2
        bead_apex_r = disc_r + bead.height
        groove_apex_r = bead_apex_r + bead.groove_radial_clearance
        half_w = bead.axial / 2 + bead.groove_axial_clearance
        # Bead z_center is in HOUSING frame; convert to ring frame the
        # same way as for the tab notches.
        bead_z_center_ring = (self.h - self.insertion_depth) - bead.z_center
        # Triangle in (radial, axial) = (x, y), point inward to the
        # bore so the cut overshoots the bore wall. Apex at groove_apex_r,
        # base spans bore_inner_r − overshoot at ±half_w.
        radial_overshoot = default_eps()
        x_base = self.bore_inner_r - radial_overshoot
        cross = polygon(points=[
            (x_base, bead_z_center_ring - half_w),
            (groove_apex_r, bead_z_center_ring),
            (x_base, bead_z_center_ring + half_w),
        ])
        return cross.rotate_extrude()

    def _latch_lug(self):
        """Return the (center_deg, width_deg, radial_proj_mm) tuple of
        the lug the latch engages."""
        return min(self.lugs, key=lambda t: abs(t[0] - self.latch_lug_target_deg))

    def _lever_z(self):
        """Bottom-face-flush layout: lever bottom sits
        `pillow_lever_axial_gap` above the lower pillow's top. The
        pivot cylinder sits on the lever's TOP face, on the camera
        side, filling the space between the lever top and the upper
        pillow (minus a matching air gap)."""
        (_, z_lower_top), _ = self._pillow_z()
        z_lever_bot = z_lower_top + self.pillow_lever_axial_gap
        z_lever_top = z_lever_bot + self.lever_axial
        return z_lever_bot, z_lever_top

    def _pillow_axial(self):
        """Derived axial thickness of each A-block. The lever, the spring
        nub, and the two block-to-lever gaps are all fixed-height; the two
        blocks split whatever axial space is left over in the ring's
        overall extent (h), so they absorb a change in `mount_skirt`/`h`
        instead of the nub. Floored at `pillow_axial_min` — below that the
        configuration can't fit the fixed lever + nub in h with printable
        blocks (likely a too-small `mount_skirt`), so we raise rather than
        silently overlap the lever."""
        slack = (
            self.h - self.lever_axial - self.pivot_nub_axial
            - 2 * self.pillow_lever_axial_gap
        )
        thickness = slack / 2.0
        if thickness < self.pillow_axial_min:
            raise SCADwrightError(
                f"LockingLensMount: derived A-block thickness {thickness:.3f} mm "
                f"< pillow_axial_min {self.pillow_axial_min} mm. The ring's axial "
                f"extent (h={self.h:.3f}) can't fit the fixed lever "
                f"({self.lever_axial}) + spring nub ({self.pivot_nub_axial}) + "
                f"gaps with printable blocks — mount_skirt is likely too small."
            )
        return thickness

    def _pillow_z(self):
        """Lower pillow flush with the ring's bottom edge (z=0); upper
        pillow flush with the ring's top (z=h). The lever sits above the
        lower pillow with `pillow_lever_axial_gap` clearance, and the
        spring nub on the lever's top face fills the remaining gap to the
        upper pillow (minus the matching air gap). The nub leaves open
        space around the pin axis between the lever top and the upper
        pillow top — the torsion spring's coil wraps around it there.
        """
        pillow_axial = self._pillow_axial()
        z_lower_bot = 0.0
        z_lower_top = z_lower_bot + pillow_axial
        z_upper_top = self.h
        z_upper_bot = z_upper_top - pillow_axial
        return (z_lower_bot, z_lower_top), (z_upper_bot, z_upper_top)

    def _pivot_cylinder_height(self):
        """Axial height of the spring nub on the lever's top face. Fixed
        (`pivot_nub_axial`) so the spring fit is stable across `h`
        changes; the A-blocks absorb the slack instead. By construction
        this equals the gap from the lever top to the upper pillow bottom
        minus `pillow_lever_axial_gap`, since `_pillow_axial` is derived
        to make the whole stack sum to h."""
        return self.pivot_nub_axial

    def _tooth_z(self):
        """Tooth axial range — top aligned with the camera-side end of
        the lug (z = rest_z + lug_axial), extending downward by
        `tooth_axial_thk`. Fully covers the lug's axial extent and dips
        below the rotation channel into the OD-wall material at the
        tooth azimuth for structural mass. The tooth top sits slightly
        above the lever's top face when the lever's top is below the
        lug top — that overshoot is a small free-standing ear at the
        tooth azimuth above the lever body. Notch and notch filler
        scale with this z range."""
        z_hi = self.rest_z + self.lug_axial
        z_lo = z_hi - self.tooth_axial_thk
        return z_lo, z_hi

    def _pillow_profile_2d(self):
        """2D plan-view profile of one pillow block at azimuth = 0
        (oriented along +X). Truncated triangular base flush on the OD,
        semicircular head with central hole. Caller rotates this to the
        pivot azimuth and extrudes axially.

        Geometry, in mm, local +X = radial outward, +Y = tangential:
          base at x = outer_or − fuse_eps (slightly into OD wall for clean
            union with the lens-mount body), y = ±base_width/2
          triangle shoulders at x = x_center, y = ±head_r (the semicircle
            meets the truncated triangle on a chord at y = ±head_r)
          semicircle of radius head_r centered at (x_center, 0)
        """
        fuse_eps = default_eps()
        x_base = self.outer_or - fuse_eps
        x_center = self.outer_or + self.pillow_pin_radial_inset
        head_r = self.pillow_head_r
        base_half = self.pillow_base_width_tan / 2
        # Sample the semicircle so the head is smooth (24 segments).
        n_arc = 24
        arc_pts = [
            (
                x_center + head_r * math.cos(math.radians(a)),
                head_r * math.sin(math.radians(a)),
            )
            for a in [-90.0 + i * 180.0 / n_arc for i in range(n_arc + 1)]
        ]
        # Walk the perimeter CCW: lower base corner, semicircle (−Y → +Y),
        # upper base corner, back to lower base corner via the base edge.
        pts = [
            (x_base, -base_half),  # base, lower-Y
            *arc_pts,              # arc from (x_center, -head_r) round to (x_center, +head_r)
            (x_base, +base_half),  # base, upper-Y
        ]
        return polygon(points=pts)

    def _tooth_pivot_angles_resolved(self):
        """Resolved (tooth_center, pivot_center, press_center) in
        lens-frame degrees. Helper so each pillow/notch/pin-hole method
        can fetch the angles without threading them through call args."""
        latch_c, latch_w, _ = self._latch_lug()
        return _tooth_pivot_angles(
            latch_c, latch_w,
            self.lock_angle_deg,
            self.tooth_clearance_from_locked_lug_deg,
            self.tooth_angular_width_deg,
            self.notch_angular_pad_deg,
            self.tooth_to_pivot_arc_deg,
            self.pivot_to_press_arc_deg,
        )

    def _notch_cutter(self):
        # Cuts the entire OD-wall thickness at the tooth-swept angular
        # range and tooth z-range (with small pad). The wall material
        # at the tooth's azimuth becomes a slot the tooth dips through.
        tooth_center_deg, _, _ = self._tooth_pivot_angles_resolved()
        notch_half_deg = self.tooth_angular_width_deg / 2 + self.notch_angular_pad_deg
        tooth_z_lo, tooth_z_hi = self._tooth_z()
        notch_z_lo = tooth_z_lo - self.notch_axial_pad
        notch_z_hi = tooth_z_hi + self.notch_axial_pad
        notch_arc = Arc(
            # Span from just inside the lug-arc outer wall to past the OD,
            # so the wall material in the notch's z-range is fully cut.
            r=(self.body_or + 0 + self.outer_or + 1.0) / 2,
            width=(self.outer_or + 1.0) - self.body_or,
            angles=(
                tooth_center_deg - notch_half_deg,
                tooth_center_deg + notch_half_deg,
            ),
        )
        return (
            notch_arc
            .linear_extrude(height=notch_z_hi - notch_z_lo)
            .up(notch_z_lo)
        )

    def _pin_hole_cutter(self):
        # Single vertical cylinder going through both A-blocks' xy at
        # the pivot azimuth. Passes through the open air between the
        # blocks (in the lever's z-range) too, but the lever isn't
        # part of this print so that's a no-op there.
        _, pivot_center_deg, _ = self._tooth_pivot_angles_resolved()
        (z_low_bot, _), (_, z_up_top) = self._pillow_z()
        pin_hole_height = z_up_top - z_low_bot + 1.0  # pad both ends
        pin_x = (self.outer_or + self.pillow_pin_radial_inset) * math.cos(math.radians(pivot_center_deg))
        pin_y = (self.outer_or + self.pillow_pin_radial_inset) * math.sin(math.radians(pivot_center_deg))
        return (
            cylinder(h=pin_hole_height, d=self.pillow_hole_d, fn=32)
            .up(z_low_bot - 0.5)
            .right(pin_x)
            .back(-pin_y)
        )

    def _pillow_block(self, z_bot):
        # 2D pillow profile extruded by the derived block thickness, raised
        # to `z_bot`, and rotated to the pivot azimuth. Same profile for
        # lower and upper blocks; only the z differs.
        _, pivot_center_deg, _ = self._tooth_pivot_angles_resolved()
        return (
            self._pillow_profile_2d()
            .linear_extrude(height=self._pillow_axial())
            .up(z_bot)
            .rotate([0, 0, pivot_center_deg])
        )

    def _solid(self):
        # Union the pillow blocks INTO the blank before the cutters
        # are applied, so the pin-hole cutter actually pierces them
        # (the pin xy is outside the blank's OD wall — without uniting
        # them in, the cutter would have no material to cut).
        (z_low_bot, _), (z_up_bot, _) = self._pillow_z()
        return union(
            super()._solid(),
            self._pillow_block(z_low_bot),
            self._pillow_block(z_up_bot),
        )

    def _cutters(self):
        yield from super()._cutters()
        yield self._notch_cutter()
        yield self._pin_hole_cutter()
        tab_notches = self._mating_disc_tab_notches()
        if tab_notches is not None:
            yield tab_notches
        bead_groove = self._mating_disc_bead_groove_cutter()
        if bead_groove is not None:
            yield bead_groove


class PivotLatchLever(Component):
    """See-saw lever — the moving half of the latch.

    Pivot pin in the MIDDLE of the lever (not at one end). Tangentially
    on one side of the pivot, the lever ends in a chisel tooth dipping
    radially inward through the OD-wall notch into the rotation channel.
    The other side is the press end — the user pushes it radially
    inward toward the lens axis; the lever pivots around the pin in
    plan view; the tooth end swings radially outward, retracting from
    the rotation channel.

    Inner and outer faces of the lever body are arcs concentric with the
    LENS axis at neutral. After rotation around the pin, both faces tilt
    rigidly with the lever — one end's inner face moves closer to the OD,
    the other end's moves farther; ditto outer faces toward the focus
    collar. The chunky 4 mm radial thickness gives the lever stiffness
    while both states still clear the focus-collar bore (~2 mm margin).

    Tooth: lower-theta face vertical (locking face — blocks the lug on
    unmount-rotation); higher-theta face ramped at `tooth_ramp_deg`.
    When a torsion-spring bias is added later, lug pressure on the ramp
    during lock-rotation auto-retracts the tooth.
    """

    equations = """
        outer_or > 0
        lever_axial > 0
        lever_inner_clearance >= 0
        lever_radial_thk_wide > 0
        lever_radial_thk_narrow > 0
        wide_axial_top > 0
        wide_axial_top < lever_axial
        pin_dia > 0
        pin_hole_lever > 0
        pillow_pin_radial_inset > 0
        pivot_cylinder_height >= 0
        pivot_cylinder_d > 0
        tooth_angular_width_deg > 0
        tooth_radial_depth > 0
        tooth_ramp_deg >= 0
        tooth_z_lo >= 0
        tooth_z_hi > tooth_z_lo
        press_taper_ratio_wide >= 0
        press_taper_ratio_wide < 1
        tooth_taper_ratio_wide >= 0
        tooth_taper_ratio_wide < 1
        press_taper_ratio_narrow >= 0
        press_taper_ratio_narrow < 1
        tooth_taper_ratio_narrow >= 0
        tooth_taper_ratio_narrow < 1
        outer_corner_fillet >= 0
        spring_hole_r > 0
        tooth_center_deg = tooth_center_deg
        pivot_center_deg = pivot_center_deg
        press_center_deg = press_center_deg
        spring_hole_center_deg = spring_hole_center_deg
        lever_inner_r = outer_or + lever_inner_clearance
        lever_outer_r_wide = lever_inner_r + lever_radial_thk_wide
        lever_outer_r_narrow = lever_inner_r + lever_radial_thk_narrow
        press_outer_r_wide = lever_outer_r_wide * (1 - press_taper_ratio_wide) + lever_inner_r * press_taper_ratio_wide
        tooth_outer_r_wide = lever_outer_r_wide * (1 - tooth_taper_ratio_wide) + lever_inner_r * tooth_taper_ratio_wide
        press_outer_r_narrow = lever_outer_r_narrow * (1 - press_taper_ratio_narrow) + lever_inner_r * press_taper_ratio_narrow
        tooth_outer_r_narrow = lever_outer_r_narrow * (1 - tooth_taper_ratio_narrow) + lever_inner_r * tooth_taper_ratio_narrow
    """

    fuse_eps = default_eps()  # volumetric overlap for clean STL unions
    n_arc = 32                # circle facets for the cylindrical pin hole

    def tight_bbox(self):                                  # framework hook: walk past Difference
        # Default tight_bbox can't tighten a Difference (the pin-hole
        # cut). Compute the lever's actual xy bbox: an annular sector
        # spanning [tooth_low_edge, press_center] at radii [inner, outer],
        # plus the tooth tip dipping inward at tooth_center. The WIDE
        # profile's outer radius is the larger of the two, so it bounds
        # the bbox.
        from scadwright import BBox
        tooth_low_edge = self.tooth_center_deg - self.tooth_angular_width_deg / 2
        sample_angles = [tooth_low_edge, self.press_center_deg]
        for cardinal in (0, 90, 180, 270):
            if tooth_low_edge <= cardinal <= self.press_center_deg:
                sample_angles.append(cardinal)
        body_pts = [
            (self.lever_outer_r_wide * math.cos(math.radians(a)),
             self.lever_outer_r_wide * math.sin(math.radians(a)))
            for a in sample_angles
        ]
        body_pts.extend([
            (self.lever_inner_r * math.cos(math.radians(a)),
             self.lever_inner_r * math.sin(math.radians(a)))
            for a in (tooth_low_edge, self.press_center_deg)
        ])
        # Tooth tip — at tooth_center azimuth, radial = lever_inner_r - tooth_radial_depth.
        tip_r = self.lever_inner_r - self.tooth_radial_depth
        body_pts.append((
            tip_r * math.cos(math.radians(self.tooth_center_deg)),
            tip_r * math.sin(math.radians(self.tooth_center_deg)),
        ))
        xs = [p[0] for p in body_pts]
        ys = [p[1] for p in body_pts]
        z_max = max(self.lever_axial + self.pivot_cylinder_height, self.tooth_z_hi)
        return BBox(min=(min(xs), min(ys), 0.0),
                    max=(max(xs), max(ys), z_max))

    def _body_2d(self, profile):
        """Plan-view profile of the lever body for one axial section.

        `profile` is 'wide' or 'narrow'. The lever's outer face uses
        different radii / taper ratios in the two axial sections so
        the narrow section fits the focus collar's narrow camera-side
        constriction, while the wide section uses a more generous
        profile.

        Inner face: arc concentric with the lens at lever_inner_r, from
        tooth's lower-theta edge to the press end (full angular extent,
        same for both profiles).

        Outer face: both halves taper inward from the pivot. Each side
        is a circular arc tangent-matched at the pivot to the OTHER
        side's tangent direction (purely tangential at the pivot), and
        passing through (r=ends_outer_r, end_angle).

        After construction, all convex corners are filleted by
        `outer_corner_fillet` via offset(+r).offset(-r). The fillet
        radius must be < half the lever's radial thickness at the
        thinnest point or it erodes the body; the narrow profile's
        tooth-end can reach ~1 mm thick, which bounds the fillet.
        """
        if profile == 'wide':
            lever_outer_r = self.lever_outer_r_wide
            press_outer_r = self.press_outer_r_wide
            tooth_outer_r = self.tooth_outer_r_wide
        elif profile == 'narrow':
            lever_outer_r = self.lever_outer_r_narrow
            press_outer_r = self.press_outer_r_narrow
            tooth_outer_r = self.tooth_outer_r_narrow
        else:
            raise ValueError(f"profile must be 'wide' or 'narrow', got {profile!r}")

        tooth_low = self.tooth_center_deg - self.tooth_angular_width_deg / 2

        # Each tapered outer arc: tangent at the pivot (so the arc's
        # center lies on the radial line through the pivot, on the
        # lens-axis side of the pivot, at distance R_arc), passing
        # through (r=end_outer_r, end_alpha) in the pivot-local frame.
        # Returns sample points along the arc in pivot-local frame.
        def tapered_arc_local(end_center_deg, end_outer_r, n_samples=24):
            alpha = math.radians(end_center_deg - self.pivot_center_deg)
            px = end_outer_r * math.cos(alpha)
            py = end_outer_r * math.sin(alpha)
            dx = lever_outer_r - px
            R_arc = (dx * dx + py * py) / (2 * dx)
            Cx_local = lever_outer_r - R_arc
            beta_at_end = math.atan2(py - 0, px - Cx_local)
            return [
                (
                    Cx_local + R_arc * math.cos(beta),
                    R_arc * math.sin(beta),
                )
                for beta in [t * beta_at_end / n_samples for t in range(n_samples + 1)]
            ]

        press_arc_local = tapered_arc_local(self.press_center_deg, press_outer_r)
        tooth_arc_local = tapered_arc_local(tooth_low, tooth_outer_r)

        cos_p = math.cos(math.radians(self.pivot_center_deg))
        sin_p = math.sin(math.radians(self.pivot_center_deg))
        def to_lens(pts_local):
            return [
                (lx * cos_p - ly * sin_p, lx * sin_p + ly * cos_p)
                for (lx, ly) in pts_local
            ]
        press_arc_lens = to_lens(press_arc_local)
        tooth_arc_lens = to_lens(tooth_arc_local)

        # Build the perimeter polygon CCW.
        pts = []
        # 1. Inner arc CCW from tooth_low to press_center at r=lever_inner_r.
        n_inner = 48
        for i in range(n_inner + 1):
            theta = tooth_low + i * (self.press_center_deg - tooth_low) / n_inner
            pts.append((
                self.lever_inner_r * math.cos(math.radians(theta)),
                self.lever_inner_r * math.sin(math.radians(theta)),
            ))
        # 2. Press-side outer (reversed: press end → just-before-pivot;
        #    we drop the pivot point because the tooth-side outer arc
        #    starts there).
        for pt in reversed(press_arc_lens[1:]):
            pts.append(pt)
        # 3. Tooth-side outer (forward: pivot → tooth_low).
        for pt in tooth_arc_lens:
            pts.append(pt)

        body = polygon(points=pts)
        if self.outer_corner_fillet > 0:
            # SHRINK then GROW = morphological opening on the polygon,
            # which is what actually rounds CONVEX corners. (The reverse,
            # grow-then-shrink, is closing and returns sharp corners on
            # a convex polygon.)
            f = self.outer_corner_fillet
            body = body.offset(r=-f).offset(r=f)
            # Restore the sharp corner at the body's tooth-end inner —
            # that's where the tooth's CW face meets the body, and a
            # rounded corner there leaves a visible gap between the
            # body and the tooth. Square placed in the body interior
            # at the original sharp-corner position; union fills in
            # the wedge that offset-opening shaved off.
            sharp_fill = (
                polygon(points=[
                    (self.lever_inner_r, 0),
                    (self.lever_inner_r + f, 0),
                    (self.lever_inner_r + f, f),
                    (self.lever_inner_r, f),
                ])
                .rotate([0, 0, tooth_low])
            )
            body = union(body, sharp_fill)
        return body

    def build(self):
        # Two stacked extrusions of the lever body. The WIDE section
        # (lens-side, lever-local z=[0, wide_axial_top]) sits in the
        # collar's wide region; the NARROW section
        # (camera-side, lever-local z=[wide_axial_top, lever_axial])
        # fits the collar's narrow camera-side constriction. Outer
        # face steps INWARD going up in lever-local z, OUTWARD going
        # down — so printing assembled-bottom-on-bed is self-
        # supporting and assembled-top-on-bed needs support for the
        # wide section on top.
        wide_body = self._body_2d('wide').linear_extrude(height=self.wide_axial_top)
        narrow_body = (
            self._body_2d('narrow')
            .linear_extrude(height=self.lever_axial - self.wide_axial_top)
            .up(self.wide_axial_top)
        )
        body_3d = union(wide_body, narrow_body)

        # ----- Tooth chisel. -----
        # Local frame: tooth_center azimuth, +X = radial outward. Outer
        # face overlaps the lever's inner face by fuse_eps for a clean
        # union. Tooth's lower-theta face vertical (blocking); higher-
        # theta face ramped.
        tooth_arc_mm = math.radians(self.tooth_angular_width_deg) * self.lever_inner_r
        x_outer = self.lever_inner_r + self.fuse_eps
        x_inner = self.lever_inner_r - self.tooth_radial_depth
        y_cw = -tooth_arc_mm / 2
        y_ccw_outer = +tooth_arc_mm / 2
        ramp_drop = self.tooth_radial_depth * math.tan(math.radians(self.tooth_ramp_deg))
        y_ccw_inner = max(y_ccw_outer - ramp_drop, y_cw + 0.1)
        tooth_2d = polygon(points=[
            (x_inner, y_cw),
            (x_outer, y_cw),
            (x_outer, y_ccw_outer),
            (x_inner, y_ccw_inner),
        ])
        tooth_3d = (
            tooth_2d
            .linear_extrude(height=self.tooth_z_hi - self.tooth_z_lo)
            .up(self.tooth_z_lo)
            .rotate([0, 0, self.tooth_center_deg])
        )

        # ----- Pivot cylinder sitting on the lever's top face. -----
        # Diameter matches the A-block semi-circle (= 2 * pillow_head_r
        # via the lens mount's wiring), so the torsion spring's coil
        # can wrap around it. Bridges the assembly gap to within
        # `pillow_lever_axial_gap` of the upper A-block bottom — the
        # rest of the space around the pin is open for the spring coil
        # and legs.
        pin_x = (self.outer_or + self.pillow_pin_radial_inset) * math.cos(math.radians(self.pivot_center_deg))
        pin_y = (self.outer_or + self.pillow_pin_radial_inset) * math.sin(math.radians(self.pivot_center_deg))
        if self.pivot_cylinder_height > 0:
            pivot_cyl = (
                cylinder(h=self.pivot_cylinder_height, d=self.pivot_cylinder_d, fn=self.n_arc)
                .up(self.lever_axial)
                .right(pin_x)
                .back(-pin_y)
            )
            body_3d = union(body_3d, pivot_cyl)

        # ----- Pin hole through the lever AND the cylinder at pivot xy.
        # The pin xy is at the same radial inset as the lens-mount
        # pillow heads (`pillow_pin_radial_inset` outside the OD), so
        # the lever body must enclose this xy at the pivot azimuth.
        # With lever_inner_r ≤ outer_or + 1.5 and lever_outer_r at
        # +3 mm, the pin at outer_or + 2 sits inside the body.
        pin_hole_h = self.lever_axial + self.pivot_cylinder_height + 1.0
        pin_hole = (
            cylinder(h=pin_hole_h, d=self.pin_hole_lever, fn=self.n_arc)
            .down(0.5)
            .right(pin_x)
            .back(-pin_y)
        )

        # ----- Torsion-spring leg hole on the press arm (opposite the
        # tooth), out toward the press end for leverage. Same bore and
        # vertical orientation as the pin hole; a plain through-hole of
        # the lever body (no nub to clear here). Position is set by the
        # mount/Design via `spring_hole_center_deg` / `spring_hole_r`.
        spring_x = self.spring_hole_r * math.cos(math.radians(self.spring_hole_center_deg))
        spring_y = self.spring_hole_r * math.sin(math.radians(self.spring_hole_center_deg))
        spring_hole = (
            cylinder(h=self.lever_axial + 1.0, d=self.pin_hole_lever, fn=self.n_arc)
            .down(0.5)
            .right(spring_x)
            .back(-spring_y)
        )

        yield difference(union(body_3d, tooth_3d), pin_hole, spring_hole)


class BronicaS2LockingLensMount(LockingLensMount, BronicaS2LensMount):
    """Bronica S2 lens mount with pillow blocks for the sewing-pin latch."""


class LockingLensMountSupports(LensMountSupports, LockingLensMount):
    """`LensMountSupports` extended with two additional pieces for the
    pillow-block + notch features:

      (1) BETWEEN-BLOCKS COLUMN — sits in the axial gap between the
          two A-blocks. Top face is `support_air_gap` below the upper
          block, bottom face is `support_air_gap` above the lower block;
          the slicer bridges those small gaps so the upper block prints
          on a foundation instead of overhanging the empty gap.
      (2) NOTCH FILLER — fills the OD-wall slot the tooth swings
          through. Separate manifold from the rotation-channel filler
          (the tall tooth makes inward extraction impossible), removed
          OUTWARD through the OD-wall opening once the bore supports
          are out.

    (No footprint pad needed under the lower A-block now that the block
    sits flush with the ring bottom (z=0). The cylinder hanging from
    the lever's bottom takes the pad's role: it puts a stable PLA
    pillar onto the lower A-block top in the assembled state, with
    open space around the pin for the torsion spring.)
    """

    def _pillow_profile_2d_resized(self, base_gap_mm, delta_mm):
        """Pillow-block plan-view profile resized for use as PETG support.

        Args:
          base_gap_mm: explicit air gap between the inner (OD-side) edge
            of the support and the lens-mount OD wall. Must be > 0 so the
            support is unambiguously disconnected from the lens mount
            in CGAL (no tangent face).
          delta_mm: change to the outer extents (head_r, base_half).
            Positive = OUTSET (support larger than the actual A-block);
            negative = INSET (support smaller). The footprint pad uses
            a positive outset so its silhouette extends past the lower
            A-block above — distinct geometry that Bambu's "split to
            objects" doesn't conflate with the A-block.
        """
        x_base = self.outer_or + base_gap_mm
        x_center = self.outer_or + self.pillow_pin_radial_inset
        head_r = self.pillow_head_r + delta_mm
        base_half = self.pillow_base_width_tan / 2 + delta_mm
        n_arc_pad = 24
        arc_pts = [
            (
                x_center + head_r * math.cos(math.radians(a)),
                head_r * math.sin(math.radians(a)),
            )
            for a in [-90.0 + i * 180.0 / n_arc_pad for i in range(n_arc_pad + 1)]
        ]
        pts = [
            (x_base, -base_half),
            *arc_pts,
            (x_base, +base_half),
        ]
        return polygon(points=pts)

    def build(self):
        # Bore pillars + rotation-channel fillers from the base supports.
        yield from super().build()

        # ----- Geometry shared with the lens mount's pillow placement.
        latch_c, latch_w, latch_p = self._latch_lug()
        tooth_center_deg, pivot_center_deg, _ = _tooth_pivot_angles(
            latch_c, latch_w,
            self.lock_angle_deg,
            self.tooth_clearance_from_locked_lug_deg,
            self.tooth_angular_width_deg,
            self.notch_angular_pad_deg,
            self.tooth_to_pivot_arc_deg,
            self.pivot_to_press_arc_deg,
        )
        (z_low_bot, z_low_top), (z_up_bot, z_up_top) = self._pillow_z()
        ag = self.support_air_gap
        # Pad and column have the same A-shape outline as the A-blocks
        # they sit under/between, inset by `ag` for air gap to the
        # blocks and to the OD wall. They print as separate floating
        # PETG manifolds — Bambu Studio's "Split to PARTS" (not "to
        # Objects") preserves their original position; the user
        # assigns the PETG filament to each part. The bore filler
        # uses the same pattern.
        pad_col_2d = self._pillow_profile_2d_resized(ag, -ag)

        # ----- (1) Column between lower and upper A-blocks.
        col_bot_z = z_low_top + ag
        col_top_z = z_up_bot - ag
        if col_top_z > col_bot_z:
            yield (
                pad_col_2d
                .linear_extrude(height=col_top_z - col_bot_z)
                .up(col_bot_z)
                .rotate([0, 0, pivot_center_deg])
            )

        # ----- (2) Notch filler — sits inside the OD-wall slot. With
        # the axially-thick tooth (and therefore axially-tall notch),
        # this filler can no longer be pushed INWARD with the bore
        # support — the filler dips into the bottom-flange z range
        # where the flange material at radial [bore_r, body_or] blocks
        # inward travel after ~1.6 mm. So the filler is detached from
        # the rotation-channel filler (`ag` air gap radially) and
        # extracted SEPARATELY by sliding it OUTWARD through the notch
        # opening in the OD wall after the bore support is removed.
        filler_outer_r = self.body_or + latch_p - ag    # rot filler outer edge
        notch_support_inner_r = filler_outer_r + ag     # air gap, not fuse overlap
        notch_support_outer_r = self.outer_or - ag
        notch_half_deg = self.tooth_angular_width_deg / 2 + self.notch_angular_pad_deg
        # Tangential air-gap: convert mm to deg at the notch's mid-radius.
        notch_mid_r = (notch_support_inner_r + notch_support_outer_r) / 2
        ang_pad_deg = math.degrees(ag / notch_mid_r)
        notch_support_arc = Arc(
            r=notch_mid_r,
            width=notch_support_outer_r - notch_support_inner_r,
            angles=(
                tooth_center_deg - notch_half_deg + ang_pad_deg,
                tooth_center_deg + notch_half_deg - ang_pad_deg,
            ),
        )
        # Axial range matches the tooth — slips into the notch with
        # `tooth_axial_clearance` + `ag` of gap to the notch walls
        # (notch z spans tooth z ± notch_axial_pad; the support sits
        # in the inner tooth-z region, leaving the pad as air gap).
        tooth_z_lo, tooth_z_hi = self._tooth_z()
        yield (
            notch_support_arc
            .linear_extrude(height=tooth_z_hi - tooth_z_lo)
            .up(tooth_z_lo)
        )

        # ----- (3) Tab-notch supports — one PETG plug per alignment-tab
        # notch in the bore wall. Each notch is a small pocket open at
        # the bed (ring's lens-side face) and CLOSED at its top (z =
        # tab.axial + clearance above the bed). Without support, that
        # top face is an overhang of (tab.circumferential + 2*clearance)
        # ≈ 3.3 mm tangentially that the slicer would have to bridge.
        # The plug fills the pocket interior, inset by `support_air_gap`
        # on every face for a clean removable fit (push outward through
        # the bed-side opening to extract).
        tabs = self.mating_disc_tabs
        if tabs is not None:
            disc_r = tabs.disc_od / 2
            # Notch interior (matching the cutter in
            # `_mating_disc_tab_notches`):
            notch_x_inner = self.bore_inner_r
            notch_x_outer = disc_r + tabs.radial + tabs.clearance
            notch_y_half = tabs.circumferential / 2 + tabs.clearance
            tab_lens_end_ring = (self.h - self.insertion_depth) - tabs.z_top
            tab_camera_end_ring = tab_lens_end_ring + tabs.axial
            # Support inset by `ag` on every face:
            sx_inner = notch_x_inner + ag
            sx_outer = notch_x_outer - ag
            sy_half = notch_y_half - ag
            sz_bot = tab_lens_end_ring          # on the bed (notch opens here)
            sz_top = tab_camera_end_ring + tabs.clearance - ag
            support_2d = polygon(points=[
                (sx_inner, -sy_half),
                (sx_outer, -sy_half),
                (sx_outer, +sy_half),
                (sx_inner, +sy_half),
            ])
            support_3d = (
                support_2d
                .linear_extrude(height=sz_top - sz_bot)
                .up(sz_bot)
            )
            ring_to_housing_offset = HOUSING_GROOVE_ANGLE_DEG - self.groove_angle_deg
            for housing_theta in tabs.angles_deg:
                yield support_3d.rotate([0, 0, housing_theta - ring_to_housing_offset])


class BronicaS2LockingLensMountSupports(
    LockingLensMountSupports, BronicaS2LockingLensMount
):
    """Bronica S2 supports for the locking lens mount print."""


class PivotLatchLeverSupports(PivotLatchLever):
    """PETG support pad under the chisel tooth for printing the lever.

    In upright print orientation the lever body sits flat on the bed
    (wide profile at z=0 with full perimeter contact), the narrow
    profile above is an inward radial step (self-supporting), and the
    pivot cylinder rides on top — none of these need support. The
    chisel tooth dips radially INSIDE the lever's inner face, so its
    bottom face overhangs air below z=tooth_z_lo at the tooth azimuth.

    The support is a single pad at the tooth's footprint, inset by
    `support_air_gap` on every side, extending from the bed up to
    `support_air_gap` below the tooth's bottom face. Inherits
    PivotLatchLever for the geometry parameters; build() yields the
    support only (not the lever body) and travels with the lever in
    `arrange_on_bed` so it lands at the same xy.
    """

    support_air_gap = 0.1

    def tight_bbox(self):                                  # framework hook
        from scadwright import BBox
        tooth_arc_mm = math.radians(self.tooth_angular_width_deg) * self.lever_inner_r
        ag = self.support_air_gap
        x_outer = self.lever_inner_r - ag
        x_inner = self.lever_inner_r - self.tooth_radial_depth + ag
        y_half = tooth_arc_mm / 2 - ag
        # Pad's local-frame corners, rotated to tooth_center_deg.
        c = math.cos(math.radians(self.tooth_center_deg))
        s = math.sin(math.radians(self.tooth_center_deg))
        corners = [(x, y) for x in (x_inner, x_outer) for y in (-y_half, y_half)]
        xs = [x * c - y * s for x, y in corners]
        ys = [x * s + y * c for x, y in corners]
        z_top = max(0.0, self.tooth_z_lo - ag)
        return BBox(min=(min(xs), min(ys), 0.0),
                    max=(max(xs), max(ys), z_top))

    def build(self):
        ag = self.support_air_gap
        # Pad footprint = tooth chisel inset by `ag` on every side, so
        # it doesn't fuse to the lever's inner face or the tooth's
        # walls. The pad and tooth are at the same azimuth.
        tooth_arc_mm = math.radians(self.tooth_angular_width_deg) * self.lever_inner_r
        x_outer = self.lever_inner_r - ag
        x_inner = self.lever_inner_r - self.tooth_radial_depth + ag
        y_cw = -tooth_arc_mm / 2 + ag
        y_ccw_outer = +tooth_arc_mm / 2 - ag
        ramp_drop = self.tooth_radial_depth * math.tan(math.radians(self.tooth_ramp_deg))
        y_ccw_inner = max(y_ccw_outer - ramp_drop, y_cw + 0.1)
        pad_2d = polygon(points=[
            (x_inner, y_cw),
            (x_outer, y_cw),
            (x_outer, y_ccw_outer),
            (x_inner, y_ccw_inner),
        ]).rotate([0, 0, self.tooth_center_deg])

        pad_h = self.tooth_z_lo - ag
        if pad_h > 0:
            yield pad_2d.linear_extrude(height=pad_h)


class BronicaS2PivotLatchLever(PivotLatchLever):
    """Bronica S2 sewing-pin latch lever, sized to the Bronica bayonet.

    Most lever-shape params live here as defaults; the Design class only
    passes through the runtime values that depend on the paired lens
    mount (outer_or, resolved pivot/tooth angles, the tooth z range in
    lever-local coords, the pivot-cylinder height) plus the tooth's
    angular width / radial depth (shared with the mount's notch and
    z-range calcs)."""

    lever_inner_clearance = 1.15   # radial gap between OD and lever inner face
                                   # at NEUTRAL. The lever inner sweeps inward
                                   # at −swing (LOCKED); with swing_deg=4 the
                                   # swept inner at the tooth-end corner lands
                                   # at ~34.15 mm — 0.10 mm clear of the lens-
                                   # mount ring's OD at 34.05. Going tighter on
                                   # this or back to swing_deg=5 crashes the
                                   # ring OD.
    # Lever radial thickness is SPLIT axially to fit the focus collar's
    # stepped bore. The lever spans assembled z=[1.7, 5.7] (4 mm tall);
    # in the lens-mount frame z=h=8.5 is the camera side, so the
    # collar's narrow constriction (camera-side, z=[3.7, 8.5]) covers
    # lever-local z=[2.0, 4.0] (top 2.0 mm) and the wide region covers
    # lever-local z=[0, 2.0] (bottom 2.0 mm). The top uses the
    # `_narrow` profile (smaller thickness, more taper); the bottom
    # uses the `_wide` profile.
    lever_radial_thk_wide = 3.20    # wide-zone pivot outer at 35.55 +
                                    # 3.20 = 38.75 (1.10 mm clear of wide
                                    # collar bore at 39.85).
    lever_radial_thk_narrow = 2.50  # narrow-zone pivot outer at 35.55 +
                                    # 2.50 = 38.05 (0.20 mm clear of
                                    # narrow collar bore at 38.25).
    swing_deg = 4.0                # lever rotation about pin from neutral
                                   # at LOCKED / UNLOCKED extremes. Bounded
                                   # from above by the lens-mount ring's OD:
                                   # at −swing the lever inner face sweeps
                                   # toward the ring; with
                                   # lever_inner_clearance=1.15 the swept
                                   # inner lands 0.10 mm clear of OD at 4°.
                                   # 5° would crash by 0.20 mm. 4° gives
                                   # 0.95 mm radial tooth swing — 0.7 mm
                                   # LOCKED penetration into the lug, 1.2 mm
                                   # UNLOCKED clearance.
    tooth_radial_depth = 3.5       # dips inward from lever inner face. With
                                   # lever_inner_clearance=1.15 (lever inner
                                   # at outer_or + 1.15 = 35.20) the tip
                                   # lands at 31.70 at NEUTRAL. The lever
                                   # rests at one of two states:
                                   #   −swing° (LOCKED): tip ~30.87, ~0.6 mm
                                   #     into the lug arc, blocking the lug
                                   #   +swing° (UNLOCKED): tip ~33.23,
                                   #     ~1.8 mm clear of the lug-arc outer
                                   #     wall, lug can rotate freely past it.
    tooth_ramp_deg = 35.0          # lever-rotation-relief ramp on the
                                   # tooth's higher-theta face. The CW
                                   # (lower-theta) face is vertical for
                                   # locking; the CCW face is ramped so
                                   # spring-biased lock-rotation auto-
                                   # retracts the tooth.
    # Lever outer-face tapers — both press AND tooth sides curve inward
    # from the pivot. Each profile (wide/narrow) has its own taper:
    # the narrow profile needs aggressive taper to keep the swung
    # corners clear of the narrow collar bore, the wide profile can
    # stay generous.
    press_taper_ratio_wide = 0.4    # wide press end: outer face moves
                                    # this fraction of lever_radial_thk_wide
                                    # inward at press_center.
    tooth_taper_ratio_wide = 0.2    # wide tooth end: light taper. Lever
                                    # thickness at tooth_low_edge in wide:
                                    # 3.20 * 0.8 = 2.56 mm.
    press_taper_ratio_narrow = 0.55 # narrow press end: aggressive taper.
                                    # press_outer_r_narrow = 38.05 − 0.55 *
                                    # 2.50 = 36.675. Swung corner at −4°:
                                    # 37.87 (0.38 mm clear of narrow bore
                                    # 38.25).
    tooth_taper_ratio_narrow = 0.55 # narrow tooth end: same. tooth_outer_r_
                                    # narrow = 36.675. Swung corner at +4°:
                                    # 38.00 (0.25 mm clear of narrow bore).
                                    # Lever thickness at tooth_low_edge in
                                    # narrow: 2.50 * 0.45 = 1.125 mm — thin,
                                    # bounds outer_corner_fillet at 0.3.
    # Cosmetic fillet on the lever's outer corners (the two tangential
    # endcap-to-outer-face corners). Applied via offset-out + offset-in,
    # so the two inner corners get rounded too — that's harmless. Must
    # be < half the lever's radial thickness at its thinnest point
    # (= lever_radial_thk_narrow * (1 − tooth_taper_ratio_narrow) /2
    # = 0.5625 mm); above that the morphological-opening fillet erodes
    # the thin region.
    outer_corner_fillet = 0.3


class BronicaS2PivotLatchLeverSupports(PivotLatchLeverSupports, BronicaS2PivotLatchLever):
    """Bronica S2 tooth support. Multi-inherits from
    `BronicaS2PivotLatchLever` so it picks up the Bronica concrete
    defaults (radial thicknesses, taper ratios, swing range, etc.) the
    same way `BronicaS2LensMountSupports` picks up its concrete from
    `BronicaS2LensMount`."""


class lens_mount_locking_ring(Design):
    """Sewing-pin pivot latch — pair of pillow blocks on the lens mount
    plus a separate curved lever pivoted on a real metal pin."""

    camera_mount = BronicaS2CameraMount()
    lens_mount = BronicaS2LockingLensMount.from_camera_mount(camera_mount)
    # PETG supports — bore pillars + rotation-channel fillers (same as
    # the non-locking print) PLUS three pillow-block supports (pad under
    # lower block, column between blocks, notch filler).
    lens_mount_supports = BronicaS2LockingLensMountSupports.from_camera_mount(camera_mount)

    # Lever derives its dimensions from the lens mount; resolve them up
    # front so the Component is fully specified.
    _latch_lug = lens_mount._latch_lug()
    _tooth_center, _pivot_center, _press_center = _tooth_pivot_angles(
        _latch_lug[0], _latch_lug[1],
        lens_mount.lock_angle_deg,
        lens_mount.tooth_clearance_from_locked_lug_deg,
        lens_mount.tooth_angular_width_deg,
        lens_mount.notch_angular_pad_deg,
        lens_mount.tooth_to_pivot_arc_deg,
        lens_mount.pivot_to_press_arc_deg,
    )
    _z_lever_bot, _z_lever_top = lens_mount._lever_z()
    # Tooth z handed to the lever in LEVER-LOCAL coords (z=0 = lever's
    # own bottom face); the assembled view shifts the whole lever up by
    # _z_lever_bot to put the tooth back at the lug's lens-frame z.
    _lens_tooth_z_lo, _lens_tooth_z_hi = lens_mount._tooth_z()
    _tooth_z_lo = _lens_tooth_z_lo - _z_lever_bot
    _tooth_z_hi = _lens_tooth_z_hi - _z_lever_bot

    # Lever-local z where the wide section ends and the narrow section
    # begins. The collar's narrow constriction sits at assembled
    # z=[h − focus_collar_narrow_axial, h] (camera side of the ring);
    # the lever's bottom face is at assembled z=_z_lever_bot; so the
    # boundary in the lever's own frame is the difference.
    _wide_axial_top = max(
        0.0,
        (lens_mount.h - BronicaS2Bayonet.focus_collar_narrow_axial) - _z_lever_bot,
    )

    # Torsion-spring leg hole on the press arm, placed as far from the
    # pivot as the press arm's radial taper allows while leaving a ~0.3 mm
    # wall on each side of the 1.5 mm hole. The camera-side (narrow)
    # section is the binding wall: it thins from 2.50 mm at the pivot to
    # 1.12 mm at the press tip, so the hole can't reach the tip. At
    # pivot + 11.8° the narrow section is 2.10 mm thick; the hole's radial
    # midline there is outer_or + 2.20 (≈ 7.45 mm from the pivot, about
    # half the 14 mm arm). Re-derive both offsets if the lever's taper
    # ratios / arc angles change.
    _spring_hole_center = _pivot_center + 11.8
    _spring_hole_r = lens_mount.outer_or + 2.20

    # Only the runtime-resolved values and shared-contract values that
    # the mount owns. The lever's own shape (radial thicknesses, taper
    # ratios, fillet, swing range, tooth_radial_depth, tooth_ramp_deg)
    # lives on `BronicaS2PivotLatchLever` as class defaults.
    _lever_kwargs = dict(
        outer_or=lens_mount.outer_or,
        lever_axial=lens_mount.lever_axial,
        wide_axial_top=_wide_axial_top,
        pin_dia=lens_mount.pin_dia,
        pin_hole_lever=lens_mount.pillow_hole_d,  # same as pillow blocks (visible)
        pillow_pin_radial_inset=lens_mount.pillow_pin_radial_inset,
        pivot_cylinder_height=lens_mount._pivot_cylinder_height(),
        pivot_cylinder_d=2 * lens_mount.pillow_head_r,
        tooth_angular_width_deg=lens_mount.tooth_angular_width_deg,
        tooth_z_lo=_tooth_z_lo,
        tooth_z_hi=_tooth_z_hi,
        pivot_center_deg=_pivot_center,
        press_center_deg=_press_center,
        tooth_center_deg=_tooth_center,
        spring_hole_center_deg=_spring_hole_center,
        spring_hole_r=_spring_hole_r,
    )
    lever = BronicaS2PivotLatchLever(**_lever_kwargs)
    lever_supports = BronicaS2PivotLatchLeverSupports(**_lever_kwargs)

    # Pin xy in lens frame (used for the see-saw rotation in the
    # assembled-locked / assembled-unlocked variants).
    _pin_x = (lens_mount.outer_or + lens_mount.pillow_pin_radial_inset) * math.cos(math.radians(_pivot_center))
    _pin_y = (lens_mount.outer_or + lens_mount.pillow_pin_radial_inset) * math.sin(math.radians(_pivot_center))

    @variant(fn=128, default=True)
    def print(self):
        # Single STL with two pack groups at matching xy positions:
        #   group 1 — lens mount (PLA) + bore/A-block supports (PETG)
        #   group 2 — lever (PLA), UPRIGHT (assembled-bottom face on
        #     the bed). Wide profile prints first (full bed contact);
        #     the narrow profile above is an INWARD radial step =
        #     self-supporting; the pivot cylinder rides on top of the
        #     narrow profile as a small column. The chisel tooth dips
        #     radially inward of the lever's inner face, leaving its
        #     underside overhanging air below z=tooth_z_lo — that one
        #     overhang is filled by the lever's PETG support pad.
        #
        # Bambu Studio workflow:
        #   1. Load the STL.
        #   2. Right-click → Split → To PARTS (NOT "To Objects").
        #      "Split to Parts" keeps each shell in its original
        #      position; "Split to Objects" auto-drops each shell to
        #      the bed, which collides PETG supports with the PLA
        #      parts they're meant to support.
        #   3. Assign Generic PLA to the lens-mount and lever parts,
        #      Generic PETG to the support parts.
        #   4. `support: off` on the whole object — the PETG parts
        #      ARE the supports.
        return arrange_on_bed(
            union(self.lens_mount, self.lens_mount_supports.red()),
            union(self.lever, self.lever_supports.red()),
            plate=BAMBU_X2D_PLATE,
            gap=10.0,
        )

    @variant(fn=60)
    def print_preview(self):
        # Same geometry as `print`, rendered at lower fn for a fast
        # preview. SCAD-render only; not for slicing.
        return self.print()

    def _seat_lever(self, swing_deg):
        # Translate lever up to its axial seat between the pillow
        # blocks, then rotate around the pin axis (vertical, at
        # (_pin_x, _pin_y)) by `swing_deg` in plan view to show the
        # locked or unlocked state.
        return (
            self.lever
            .up(self._z_lever_bot)
            .right(-self._pin_x).back(self._pin_y)   # pin → origin
            .rotate([0, 0, swing_deg])
            .right(self._pin_x).back(-self._pin_y)   # back to lens-frame
        )

    @variant(fn=96)
    def pivot_assembled_locked(self):
        # Locked: lever swung so tooth side is radially inward
        # (engaged with the lug), press side radially outward.
        seated = self._seat_lever(-self.lever.swing_deg)
        return union(self.lens_mount, seated.red())

    @variant(fn=96)
    def pivot_assembled_unlocked(self):
        # Unlocked: lever swung so tooth side is radially outward
        # (cleared from the rotation channel), press side depressed
        # toward the lens axis. Both ends must clear OD + focus collar.
        seated = self._seat_lever(+self.lever.swing_deg)
        return union(self.lens_mount, seated.red())


if __name__ == "__main__":
    run()
