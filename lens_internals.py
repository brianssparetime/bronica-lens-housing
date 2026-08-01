"""Internals of a stacked-element lens housing — the Components and
value types a `LensHousing` is built FROM.

Splits cleanly along the REUSABLE / CONCRETE axis: nothing in this file
is S2-specific or Bronica-specific, so a non-S2 lens design could
reuse all of it. Concrete sizing (defaults, ELEMENTS tuple, the
LensHousingDesign variants) lives in `lens_housing.py`.
"""

from collections import namedtuple

from scadwright import BBox, Component, Param
from scadwright.api.tolerances import default_eps
from scadwright.boolops import difference, union
from scadwright.component.anchors import anchor
from scadwright.primitives import circle, cylinder, polygon, square
from scadwright.shapes import FilletRing, Sector, Tube, almond_profile, regular_polygon


# Per-element fit clearances around the lens body, separately tunable
# per element (different glass shapes/edge profiles benefit from
# different fit looseness). Both fields in mm.
#
# `axial` is per-side: the gap between the element's flat face (top or
# bottom) and the holder's surround floor. Total axial slop = 2 * axial
# (one gap above, one below).
#
# `radial` is DIAMETER-based, NOT radius-based: it's added directly to
# the element diameter to compute the surround tube's bore
# (`id = e.dia + clearances.radial` in ElementHolder.build()). So the
# actual per-side radial gap between the element's outer edge and the
# surround's inner wall is `radial / 2`. A `radial` of 0.4 mm means
# 0.2 mm gap on each side, not 0.4 mm. (Naming is legacy; the math is
# what counts.)
ElementClearances = namedtuple("ElementClearances", "axial radial")
DEFAULT_ELEMENT_CLEARANCES = ElementClearances(axial=0.2, radial=0.3)


# Mating disc: an annular ring on the lens-housing's outer surface that
# bridges the radial gap between the housing's outer lower segment and
# the lens-mount ring's inner bore. Built as part of the housing (so it
# gets halved with the housing's lap-split halves and needs no separate
# support); functionally it's the physical interface to the lens-mount
# ring.
#   - od: lens-mount ring's bore (the disc fits snugly inside it).
#   - z_bot, z_top: axial range in housing world frame. Typically z_bot
#     sits a small clearance above the rotation channel's roof, and
#     z_top flushes with the top of the lens-mount tube (= MOUNT_OFFSET
#     in the housing's world frame).
# The disc's ID is taken from `lower_outer_od` so the disc wraps the
# outer segment concentrically; outer is the only lower segment that
# reaches above the flange into this z-range.
MatingDisc = namedtuple("MatingDisc", "od z_bot z_top")


# Swept slot: a shallow axial groove in a segment wall that clears a
# camera-body feature (e.g. the focus-helicoid pin in the mid segment,
# or the aperture-feeler in the inner segment). Uniform radial `depth`
# mm in from the OD across the slot's whole angular extent — the
# pin/feeler rides in this groove throughout the bayonet rotation. Two
# angular regions union into one cut:
#   - LOCKED side: `width` mm circumferential at the OD, centered at
#     `angle`. The pin/feeler sits here when the lens is locked.
#   - SWEEP side: extends CW from `angle` by `sweep_arc_deg` (toward
#     the alignment groove) to cover the pin/feeler's path as the lens
#     rotates from insertion to locked. Zero disables the sweep portion.
# Fields:
#   - angle: deg, 0 = +X axis (3 o'clock looking from camera toward subject).
#   - width: mm circumferential at OD on the locked side, centered on `angle`.
#   - depth: mm radial inward from the OD. Uniform across the slot.
#   - z_bot, z_top: axial range in housing world frame.
#   - sweep_arc_deg: deg, angular extent of the sweep portion CW from `angle`.
#   - bottom_overshoot: mm past z_bot to extend the cutter — set this
#     when the slot's deep end sits exactly on a segment boundary so the
#     cutter pokes into the next segment's z-range and drops an otherwise-
#     coplanar bottom face.
SweptSlot = namedtuple(
    "SweptSlot",
    "angle width depth z_bot z_top sweep_arc_deg bottom_overshoot",
)


class HousingSection(Component):
    """One axial section of the lens housing's wall. Cylindrical when
    bot_id == top_id, conical (a Funnel) otherwise. Carries both its
    axial extent and its radial profile in one Component, so the
    housing's geometry (built from sections) and its `wall_od_at_z`
    lookup (querying sections) share one source of truth — no parallel
    analytic predictor."""

    equations = """
        z_top > z_bot
        bot_id, top_id, thk > 0
        ?top_overshoot >= 0
        ?bot_overshoot >= 0
    """

    # Cylindrical anchor for the outer wall — lets `add_text(on="outer_wall")`
    # wrap a label around the section. Safe only on cylindrical sections
    # (`bot_id == top_id`); conical (funnel) sections would need a separate
    # conical anchor with r1/r2/length surface_params.
    outer_wall = anchor(
        at="(bot_id + 2*thk) / 2.0, 0, (z_bot + z_top) / 2",
        normal=(1.0, 0.0, 0.0),
        kind="cylindrical",
        surface_params={
            "axis": (0.0, 0.0, 1.0),
            "radius": "(bot_id + 2*thk) / 2.0",
            "length": "z_top - z_bot",
        },
    )

    # Cylindrical anchor for the inner bore — lets a content-piece (e.g.
    # an `ElementHolder` whose OD matches this section's ID) call
    # `holder.fuse(section, self_only=True)` to merge against the inner
    # wall with a framework-managed eps extension. Inward normal. Same
    # cylindrical-only caveat as `outer_wall`.
    inner_wall = anchor(
        at="bot_id / 2.0, 0, (z_bot + z_top) / 2",
        normal=(-1.0, 0.0, 0.0),
        kind="cylindrical",
        surface_params={
            "axis": (0.0, 0.0, 1.0),
            "radius": "bot_id / 2.0",
            "length": "z_top - z_bot",
            "inner": True,
        },
    )

    def build(self):                                       # framework hook: required; returns the shape
        from scadwright.shapes import Funnel
        # Optional build-only axial overshoot: extends a section past its
        # logical z-extent so a stacked neighbor overlaps it instead of
        # meeting on an exactly coplanar face (which survives CGAL STL
        # export as a non-manifold edge). The logical z_bot/z_top that
        # od_at / id_at / wall_od_at_z read stay untouched. A conical
        # section can't take an overshoot — extending the cone would drift
        # the bore — so the caller overshoots the cylindrical member of
        # each seam instead.
        top_o = self.top_overshoot or 0.0
        bot_o = self.bot_overshoot or 0.0
        if self.bot_id == self.top_id:
            h = (self.z_top - self.z_bot) + top_o + bot_o
            return Tube(h=h, od=self.bot_id + 2 * self.thk, id=self.bot_id).up(self.z_bot - bot_o)
        if top_o or bot_o:
            raise ValueError(
                "HousingSection: overshoot is only valid on a cylindrical "
                "section (bot_id == top_id); overshoot a cylindrical "
                "neighbor of this conical section instead."
            )
        return Funnel(
            h=self.z_top - self.z_bot, thk=self.thk, bot_id=self.bot_id, top_id=self.top_id,
        ).up(self.z_bot)

    def od_at(self, z):
        """Outer diameter at axial position z (must be in [z_bot, z_top])."""
        return self.id_at(z) + 2 * self.thk

    def id_at(self, z):
        """Inner diameter at axial position z (must be in [z_bot, z_top])."""
        if self.bot_id == self.top_id:
            return self.bot_id
        t = (z - self.z_bot) / (self.z_top - self.z_bot)
        return self.bot_id + t * (self.top_id - self.bot_id)


class FilletSection(Component):
    """Bore-side fillet ring at an interior step in the stacked lower
    housing. Lives inside the wider segment (the one above the step in
    housing-frame z), occupying axial extent
    `(top_id − bot_id) / 2 × tan(base_angle)` just above the step plane.
    Three things at once:
      1. Closes the radial gap between segment walls at the step. At the
         mid/inner step the inner wall ends ~1 mm radially inside mid's
         inner wall, so without this fillet the two segments are
         physically disjoint.
      2. Gives the bore step a self-supporting slope in the print
         (lens-side-down) orientation: `base_angle` is the slanted-cone
         angle from horizontal; ~45° is the FDM self-supporting
         threshold.
      3. Participates in `wall_id_at_z` so a stop or constricted-element
         holder landing inside the fillet's z range sizes off the
         locally-narrowed bore.
    `top_id` (the wider segment's bore, above the step) must exceed
    `bot_id` (the narrower segment's bore, below)."""

    equations = """
        bot_id, top_id > 0
        top_id > bot_id
        base_angle > 0
        base_angle <= 90
        h_full = tan(base_angle) * (top_id - bot_id) / 2
        z_top = z_bot + h_full
    """

    def build(self):                                       # framework hook: required; returns the shape
        return FilletRing(
            id=self.bot_id, od=self.top_id,
            base_angle=self.base_angle, slant="inwards",
        ).up(self.z_bot)

    def id_at(self, z):
        """Inner bore diameter at z. Linearly opens from `bot_id` at the
        step plane (z_bot) to `top_id` at the top of the fillet (z_top),
        matching the FilletRing's slanted inner cone."""
        t = (z - self.z_bot) / (self.z_top - self.z_bot)
        return self.bot_id + t * (self.top_id - self.bot_id)

    def od_at(self, z):
        """Outer extent at z. The fillet's outer surface is cylindrical
        at the wider segment's bore radius (i.e., the wider segment's
        inner wall) — it sits inside that bore. wall_od_at_z's MAX
        across containing sections picks the wider segment's actual OD
        instead, so this value is harmless."""
        return self.top_id


# spacing is from flange (z=0) to element center; +z = toward object.
# `face_z_top`, `face_z_bot`, `constricted`, `throat_dia_required` are
# precomputed by `element(...)` so the housing's `equations` list can
# reference them as attributes directly. `clearances` is an
# `ElementClearances(axial, radial)` namedtuple; the holder reads both
# values from this field. `code` (catalog code, e.g. "L7325") and
# `type` (e.g. "NMN", "ACH", "BCX") identify the lens.
Element = namedtuple(
    "Element",
    "code type dia edge_thk spacing face_z_top face_z_bot constricted throat_dia_required clearances",
)


class ElementHolder(Component):
    """Two-sided clamp around one Element, symmetric about the element center.

    The holder's outer cylinder (`od`) is sized to mate exactly with the
    containing housing section's inner bore. The caller fuses the holder
    against that section via `holder.fuse(section, self_only=True)`; the
    framework auto-detects the matching cylindrical surfaces (holder's
    `outer_wall` against the section's `inner_wall`) and applies a
    radial eps extension via `fuse_extend` below to prevent the
    coincident-surface seam from surviving the yield-union as a
    non-manifold edge after `halve`. `self_only=True` returns just the
    extended holder (no union with the host) so the host section isn't
    duplicated in the SCAD output — important inside a generator
    `build()` where the host is already yielded separately.
    """

    element = Param(Element)
    equations = """
        od, grip_depth, grip_width > 0
    """

    # Cylindrical outer surface of the holder, centered axially on the
    # element's spacing with length matching the element's declared
    # z-extent (face_z_top − face_z_bot). Lets the caller call
    # `holder.fuse(section, self_only=True)` to auto-match against
    # `HousingSection.inner_wall`.
    outer_wall = anchor(
        at="od / 2.0, 0, element.spacing",
        normal=(1.0, 0.0, 0.0),
        kind="cylindrical",
        surface_params={
            "axis": (0.0, 0.0, 1.0),
            "radius": "od / 2.0",
            "length": "element.face_z_top - element.face_z_bot",
        },
    )

    def build(self):                                       # framework hook: required; returns the shape
        e = self.element
        clr = e.clearances
        # Built as one solid so the housing's lap split cuts it cleanly: a
        # full-height loose surround (bore = element dia + radial
        # clearance) with a tighter-bore gripper embedded at each end. The
        # grippers overlap the surround rather than abutting it, and
        # nothing is reflected — abutting seams or a mirrored copy leave
        # coincident faces that CGAL exports as non-manifold slivers where
        # the lap groove crosses a holder. `face_z_top`/`face_z_bot` (from
        # the element() factory, also used by the housing equations) stay
        # the source of truth for the z-extent.
        gripper_h = 0.5 * self.grip_width
        surround = Tube(h=e.face_z_top - e.face_z_bot, od=self.od, id=e.dia + clr.radial).up(e.face_z_bot)
        grip_bot = Tube(h=gripper_h, od=self.od, id=e.dia - self.grip_depth).up(e.face_z_bot)
        grip_top = Tube(h=gripper_h, od=self.od, id=e.dia - self.grip_depth).up(e.face_z_top - gripper_h)
        return union(surround, grip_bot, grip_top)

    def fuse_extend(self, anchor, eps: float):
        """Radial-extension lever for `fuse(host, self_only=True)`. Only
        the `outer_wall` cylindrical anchor is exposed (inner=False), so
        the extension grows `od` by 2*eps; the holder pokes its outer
        cylinder one eps per side into the host wall, producing the
        non-coincident union CGAL needs."""
        if anchor.kind != "cylindrical":
            return None
        return ElementHolder(
            element=self.element,
            od=self.od + 2 * eps,
            grip_depth=self.grip_depth,
            grip_width=self.grip_width,
        )


def element(*, code, type, dia, edge_thk, spacing, grip_width, mount_offset=0,
            clearances=DEFAULT_ELEMENT_CLEARANCES):
    """Build an `Element` with its computed geometric fields filled in.

    The caller supplies `grip_width` (the holder's per-side gripper
    height) so the housing's equations can reason about each element's
    z-extent through attribute access. `mount_offset` is the axial
    extent of the lens-mount ring above the camera flange (z=0); the
    M57-restricted lower section of the housing extends up to this z,
    so an element is `constricted` when its lower face is below the
    top of the mount-ring zone (face_z_bot < mount_offset).
    `clearances` is an `ElementClearances(axial, radial)` pair; defaults
    to the project-wide `DEFAULT_ELEMENT_CLEARANCES`. Override per
    element when a particular piece of glass needs a tighter or looser
    fit (e.g. a chipped edge, an irregular OD, or a known-bad-tolerance
    cell).
    """
    # Half-extent of the element holder in z, computed from the EXACT
    # geometry built in ElementHolder.build(): one half = surround_h
    # (= 0.5 * edge_thk + axial clearance) + 0.5 * grip_width (the
    # mirror-copied gripper above the surround). Element spacings
    # (relative to the camera flange) are the source of truth; the
    # housing's `upper_housing_len = max(e.face_z_top)` is then EXACTLY
    # the topmost holder's top face, with no approximate-formula gap.
    half = 0.5 * edge_thk + clearances.axial + 0.5 * grip_width
    return Element(
        code=code, type=type,
        dia=dia, edge_thk=edge_thk, spacing=spacing,
        face_z_top=spacing + half,
        face_z_bot=spacing - half,
        constricted=(spacing - half) < mount_offset,
        throat_dia_required=dia + clearances.radial,
        clearances=clearances,
    )


class ElementTestGripper(Component):
    """Stand-alone diagnostic half-shell of one element's gripper.

    Three stacked tubes (bottom rim / middle / top rim) reproducing
    `ElementHolder`'s inward features — rim ID, middle ID, and axial
    extents — exactly, so a successful fit here predicts a fit in the
    housing. The outer wall is a smooth uniform-OD cylinder sized to
    give the middle (surround) tube `wall_thk` radial wall; rim tubes
    consequently have slightly thicker walls (the surplus being half
    of `grip_depth + element.clearances.radial`). Intended to be
    halved sagittally and printed individually for per-element fit
    checks without committing to a full housing.
    """

    element = Param(Element)
    equations = """
        grip_depth, grip_width, wall_thk > 0
        gripper_h = grip_width / 2
        surround_h = element.face_z_top - element.spacing - gripper_h
        rim_id = element.dia - grip_depth
        surround_id = element.dia + element.clearances.radial
        od = surround_id + 2 * wall_thk
        total_h = 2 * (gripper_h + surround_h)
    """

    outer_wall = anchor(
        at="od / 2, 0, total_h / 2",
        normal=(1.0, 0.0, 0.0),
        kind="cylindrical",
        surface_params={
            "axis": (0.0, 0.0, 1.0),
            "radius": "od / 2",
            "length": "total_h",
        },
    )

    def build(self):                                       # framework hook: required; returns the shape
        bottom_rim = Tube(h=self.gripper_h, od=self.od, id=self.rim_id)
        middle = Tube(
            h=2 * self.surround_h, od=self.od, id=self.surround_id,
        ).up(self.gripper_h)
        top_rim = Tube(
            h=self.gripper_h, od=self.od, id=self.rim_id,
        ).up(self.gripper_h + 2 * self.surround_h)
        return union(bottom_rim, middle, top_rim)


# A waterhouse stop is a thin disc slid laterally into the lens barrel
# through a side slot to set the aperture. `spacing` is the stop center's
# z-position, `thk` the disc thickness, `clr` the slot's z-clearance.
Stop = namedtuple("Stop", "spacing thk clr")


class WaterhouseStop(Component):
    """A waterhouse stop disc.

    Viewed face-on, it's a 2D paddle: a smaller semicircle of radius
    `inner_diameter`/2 in -Y joined to a larger semicircle of radius
    `outer_diameter`/2 in +Y along their flat edges, with a central
    aperture hole of diameter `hole_d`. Extruded by `thk` along the
    lens axis and centered on `spacing`. When inserted, the lower
    (inner-diameter) semicircle sits in the bore as the aperture plate;
    the upper (outer-diameter) semicircle fills the wall slot, leaving
    a flush exterior. The hole sits on the barrel's axis."""

    equations = """
        inner_diameter, outer_diameter, thk, hole_d > 0
        outer_diameter > inner_diameter
        hole_d < inner_diameter
        # `spacing` is the disc's z-center; `slot_bottom` is the
        # extruder's lower edge. The derivation establishes `spacing` as
        # a Param(float) without an artificial inequality (it can be
        # negative — the stop sits below the flange in the default).
        slot_bottom = spacing - thk / 2
    """

    def build(self):                                       # framework hook: required; returns the shape
        upper = Sector(r=self.outer_diameter / 2, angles=(0, 180))
        lower = Sector(r=self.inner_diameter / 2, angles=(180, 360))
        paddle_2d = difference(union(upper, lower), circle(d=self.hole_d))
        paddle = paddle_2d.linear_extrude(height=self.thk)
        return paddle.up(self.slot_bottom).red()


# Shared geometry for the non-circular stops below: the same two-semicircle
# paddle as WaterhouseStop plus one equal-area invariant. Each shaped stop
# adds one derivation line (its aperture dimension) and cuts a different
# center hole. `equiv_area_circle_dia` is the diameter of the circle whose
# area the shaped hole matches — so a shaped stop passes the same light as
# the round stop of that f-number. `target_area` is that shared area.
_SHARED_STOP_EQS = """
    inner_diameter, outer_diameter, thk, equiv_area_circle_dia > 0
    outer_diameter > inner_diameter
    equiv_area_circle_dia < inner_diameter
    slot_bottom = spacing - thk / 2
    target_area = pi * equiv_area_circle_dia**2 / 4
"""


def _stop_paddle(inner_diameter, outer_diameter, thk, slot_bottom, aperture_2d):
    """Body shared by every waterhouse-style stop: the two-semicircle paddle
    (small `inner_diameter` semicircle in -Y that seats in the bore, larger
    `outer_diameter` semicircle in +Y that fills the wall slot) with
    `aperture_2d` cut from its center, extruded `thk` along the lens axis and
    dropped so its lower face sits at `slot_bottom`."""
    upper = Sector(r=outer_diameter / 2, angles=(0, 180))
    lower = Sector(r=inner_diameter / 2, angles=(180, 360))
    paddle = difference(union(upper, lower), aperture_2d).linear_extrude(height=thk)
    return paddle.up(slot_bottom).red()


class SquareStop(Component):
    """Waterhouse stop with a square aperture, equal in area to a circle of
    `equiv_area_circle_dia`. Side = sqrt(area), so `side = equiv_area_circle_dia
    * sqrt(pi) / 2`. Axis-aligned. See WaterhouseStop for the paddle."""

    equations = _SHARED_STOP_EQS + """
        side = equiv_area_circle_dia * sqrt(pi) / 2
        side * sqrt(2) < inner_diameter        # diagonal fits the bore semicircle
    """

    def build(self):                                       # framework hook: required; returns the shape
        return _stop_paddle(
            self.inner_diameter, self.outer_diameter, self.thk,
            self.slot_bottom, square(self.side, center=True),
        )


class HexStop(Component):
    """Waterhouse stop with a regular-hexagon aperture, equal in area to a
    circle of `equiv_area_circle_dia`. Hexagon area = (3*sqrt(3)/2)*R^2 for
    circumradius R, so `hex_r = equiv_area_circle_dia * sqrt(pi / (6*sqrt(3)))`.
    Flat-top (rotated 30 deg from regular_polygon's vertex-on-+X default)."""

    equations = _SHARED_STOP_EQS + """
        hex_r = equiv_area_circle_dia * sqrt(pi / (6 * sqrt(3)))
        hex_r < inner_diameter / 2
    """

    def build(self):                                       # framework hook: required; returns the shape
        hexagon = regular_polygon(6, self.hex_r).rotate([0, 0, 30])
        return _stop_paddle(
            self.inner_diameter, self.outer_diameter, self.thk,
            self.slot_bottom, hexagon,
        )


class AlmondStop(Component):
    """Waterhouse stop with an almond (vesica) aperture, equal in area to a
    circle of `equiv_area_circle_dia`. The almond is two mirrored circular
    segments; equal area alone doesn't fix its shape, so `aspect` (long axis
    / short axis, default 2.5) sets the elongation and the size then follows
    from equal area. The long axis runs along Y — perpendicular to the line
    where the two paddle semicircles join — reaching from the bore semicircle
    toward the wall-slot semicircle.

    Sizing: at unit sag the segment radius is `(aspect^2 + 1)/2` and each
    segment subtends half-angle `asin(2*aspect/(aspect^2+1))`; the unit-sag
    area is `seg_r^2 * (2*half - sin 2*half)` (angle in radians). Area scales
    with sag^2, so `sag = sqrt(target_area / area_unit)` and `chord_r =
    aspect * sag`."""

    aspect = Param(float, default=2.5)     # long axis / short axis (> 1)

    equations = _SHARED_STOP_EQS + """
        aspect > 1
        almond_seg_r = (aspect**2 + 1) / 2
        almond_half = asin(2 * aspect / (aspect**2 + 1))
        almond_area_unit = almond_seg_r**2 * (radians(2 * almond_half) - sin(2 * almond_half))
        sag = sqrt(target_area / almond_area_unit)
        chord_r = aspect * sag
        chord_r < inner_diameter / 2
    """

    def build(self):                                       # framework hook: required; returns the shape
        # almond_profile spans the chord (long axis) along X; rotate 90 deg
        # so the long axis runs along Y, across the semicircle join line.
        almond = polygon(points=almond_profile(self.chord_r, self.sag)).rotate([0, 0, 90])
        return _stop_paddle(
            self.inner_diameter, self.outer_diameter, self.thk,
            self.slot_bottom, almond,
        )


class StopHolder(Component):
    """Two-sided clamp around a waterhouse stop. Upper and lower grippers
    flank the slot symmetrically along z; both pinch the stop's lower-ID
    semicircle from outside the slot. Each gripper has its own outer
    diameter so the holder can sit in a section where the local wall OD
    differs above vs. below the stop (e.g. when the stop lands inside the
    expansion funnel). When `od_top == od_bot` the result matches a
    uniform-OD section. ID is shared across both grippers since the disc
    they pinch is one diameter regardless of where it sits."""

    stop = Param(Stop)
    equations = """
        od_top, od_bot, grip_depth, grip_width, barrel_id > 0
        barrel_id > grip_depth
    """

    def build(self):                                       # framework hook: required; returns the shape
        s = self.stop
        slot_h = s.thk + s.clr
        half_gw = 0.5 * self.grip_width
        upper = Tube(
            h=half_gw, od=self.od_top, id=self.barrel_id - self.grip_depth,
        ).up(slot_h / 2)
        lower = Tube(
            h=half_gw, od=self.od_bot, id=self.barrel_id - self.grip_depth,
        ).down(slot_h / 2 + half_gw)
        return union(upper, lower).up(s.spacing).green()


class TruncFilletRing(Component):
    """A `FilletRing` with its sharp apex chopped to a flat rim of
    `rim_width` radial width."""

    equations = """
        id, od, base_angle, rim_width > 0
        base_angle < 90
        id < od
        rim_width < (od - id) / 2
        ?slant:str = ?slant or "outwards"
        slant in ("outwards", "inwards")
        h_full = tan(base_angle) * (od - id) / 2
        z_t = tan(base_angle) * ((od - id) / 2 - rim_width)
    """

    # Conical anchor for the inner slanted surface (slant="inwards" only:
    # the inner cone widens from id/2 at z=0 to (od/2 - rim_width) at z=z_t).
    # Lets add_text() wrap a label around the sloped face.
    slanted_face = anchor(
        at="(id + od - 2*rim_width) / 4.0, 0, z_t / 2",
        normal=(-1.0, 0.0, 0.0),
        kind="conical",
        surface_params={
            "axis": (0.0, 0.0, 1.0),
            "r1": "id / 2.0",
            "r2": "(od - 2*rim_width) / 2.0",
            "length": "z_t",
            "inner": True,
        },
    )

    def build(self):                                       # framework hook: required; returns the shape
        # Radial eps: the chopper's outer cylindrical surface sits at d=od, same
        # as the FilletRing's outer wall. through() handles planar-bbox coplanar
        # faces but not cylinder-on-cylinder coincidence, so nudge the chopper
        # outward by a hair.
        eps = default_eps()
        base = FilletRing(id=self.id, od=self.od, base_angle=self.base_angle, slant=self.slant)
        chopper = cylinder(
            h=self.h_full - self.z_t, d=self.od + 2 * eps,
        ).up(self.z_t).through(base, axis="z")
        return difference(base, chopper)

    def tight_bbox(self):                                  # framework hook: declare true post-truncation extents
        # Default tight_bbox would walk into our top-level Difference and
        # raise. We know the truth: the chopper removes the apex from
        # z_t to h_full, so the truncated rim's actual top is at z=z_t
        # (not h_full as the conservative bbox would say).
        r = self.od / 2
        return BBox(min=(-r, -r, 0), max=(r, r, self.z_t))

    def prefers_shift_at_anchor(self, anchor) -> bool:
        # Same rationale as the standard-library `FilletRing`: our bottom
        # annular cap exactly matches the host wall's top face, so the
        # framework's `cross_section_extend` slab is itself coplanar with
        # the host's outer cylinder and CGAL leaves non-manifold edges.
        # Opt the planar top/bottom anchors into `bond='shift'` so
        # `.attach(host, fuse=True)` resolves to the shift path. The
        # eps drift on the opposite face is invisible on a fillet rim.
        if anchor.kind != "planar":
            return False
        return abs(anchor.normal[2]) > 1.0 - 1e-9
