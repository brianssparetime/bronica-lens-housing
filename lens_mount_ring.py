"""Bronica S2 lens-mount ring.

The lens-side mating part for a `CameraMount`: a thick tube with
bayonet-channel cuts that mates with the camera's lugs. Generic
`LensMount` + `LensMountSupports` Components are framework-style
reusable bases; the Bronica concretes pull dimensions off the shared
bayonet spec.

The body-cap variant lives in `body_cap.py` (extends `LensMount` with
a plug disc). The locking variant lives in `lens_mount_locking_ring.py`
(adds pillow blocks + a pivot lever).

Run:
    python s2-evolving/lens_mount_ring.py
"""

import math

from scadwright import Component
from scadwright.api.tolerances import default_eps
from scadwright.boolops import difference, union
from scadwright.composition_helpers import arrange_on_bed
from scadwright.design import Design, run, variant
from scadwright.primitives import circle, polygon
from scadwright.shapes import Arc, Dome, Tube

from printer import BAMBU_X2D_PLATE
from rear_lens_cap import BronicaS2CameraMount
from spec import BronicaS2Bayonet


class LensMount(Component):
    """Lens-side mating part for a `CameraMount`.

    Coordinate convention: z=0 is the bottom face of the tube, z=h is
    the top face — the ENTRY RIM, the side the camera mount is pressed
    into. When fully inserted and rotated, the camera mount's lug-front
    plane sits at z = h - `insertion_depth` (the "rest plane").

    The cutter is the union of three pieces:

    1. BORE cutter (full tube height): a clearance-offset cylinder of
       `cam_barrel_od`. Lets the camera mount's body pass through the
       entire lens mount.
    2. ENTRY LUG CHANNELS (only `insertion_depth` tall, top of tube):
       lug-width arcs at the original lug angles, from the rest plane
       up to the entry rim. Each lug slides down its channel during
       press-in.
    3. ROTATION CHANNEL (only `lug_axial` tall, at the rest plane):
       lug arcs widened by `lock_angle_deg` on the CCW ("left") edge.
       The lug rotates through this channel after entry.

    Material between the rotation channel and the entry rim, at the
    swept angles outside the entry channels, is the bayonet lip —
    it catches each lug after the twist and prevents the mount from
    pulling back out. `clearance` is applied via 2D `offset` around
    each profile.

    Tube blank:
    - OD = `cam_barrel_od + 2*max_lug_proj + 2*wall_thk`
      (i.e. `wall_thk` of wall outside the circle around the lugs)
    - ID = `cam_barrel_id - 2`
    - h  = `h`
    """

    equations = """
        cam_barrel_od, cam_barrel_id, clearance, lock_angle_deg, lug_axial, wall_thk, mount_skirt, protrusion_len, entry_arc_clearance, lug_axial_clearance > 0
        cam_barrel_od > cam_barrel_id
        # Lug-tip radial channel gap — separate from the general `clearance`
        # so it can be 0 (channel cut flush to the lug tip).
        lug_radial_clearance >= 0
        # The camera-facing rim sits `mount_skirt` above the rotation
        # channel's camera-side face; the channel floor is at rest_z
        # (= protrusion_len) and the channel is lug_axial + lug_axial_clearance
        # tall. So the camera-side insertion depth is the channel height
        # plus the skirt — shrinking mount_skirt sands the rim toward the
        # channel without disturbing the channel or the lens-side stack.
        insertion_depth = lug_axial + lug_axial_clearance + mount_skirt
        lug_axial < insertion_depth
        h = protrusion_len + insertion_depth
        max_lug_proj = max(p for c, w, p in lugs:tuple)
        outer_od = cam_barrel_od + 2 * max_lug_proj + 2 * wall_thk
        # `inner_shrink` shrinks the blank's inner diameter (and the bore cut
        # below) by this amount, leaving the OD and channel walls untouched.
        # 0 = the bore matches `cam_barrel_od + 2*clearance`. Bumping it makes
        # the inner cylinder smaller and the cuts go that much deeper inward.
        # Concrete subclasses set the value (BronicaS2LensMount reads it
        # from the bayonet spec).
        inner_shrink >= 0
        inner_id = cam_barrel_id - 2 * wall_thk - inner_shrink
        body_or = cam_barrel_od / 2
        outer_or = outer_od / 2
        rest_z = h - insertion_depth
        # Bore and channel-cutter inner radii. The bore is a cylindrical
        # hole of radius `bore_inner_r`; channel cutters (entry + rotation)
        # must reach inward past it so the channels actually open into the
        # bore. Without `channel_breakthrough`, `inner_shrink` widens the
        # bore-side wall and seals the channels with a thin veil. The
        # value below is the desired POST-offset overlap into the bore;
        # `channel_inner_r` is set so that after `.offset(r=clearance)`
        # the cutter's inner edge sits `channel_breakthrough` mm inside
        # the bore inner wall.
        bore_inner_r = body_or + clearance - inner_shrink / 2
        channel_breakthrough = 0.5
        channel_inner_r = bore_inner_r + clearance - channel_breakthrough
        # Alignment groove: thin axial recess on the outer wall, centered
        # on the widest lug's center angle.
        groove_depth, groove_width > 0
        widest_w = max(lw for lc, lw, lp in lugs)
        widest_center = [lc for lc, lw, lp in lugs if lw == widest_w][0]
        # Groove ends up at the widest lug's world angle (the BRONICA
        # logo direction) after the lens rotates +lock_angle_deg in
        # camera coords into its locked position; in lens-coords the
        # groove therefore starts at widest_center - lock_angle_deg.
        groove_angle_deg = widest_center - lock_angle_deg
    """

    def tight_bbox(self):                                  # framework hook: declare true post-Difference extents
        # Default tight_bbox would walk into the bore/channel/groove
        # `difference()` and raise. The bore + channels are fully contained
        # within the blank, so the only thing past `outer_od/2` is the
        # alignment-dot cap (protrudes ~cap_height - cap_embed radially).
        # Round up to 1.5 mm so the layout helpers always have headroom.
        from scadwright import BBox
        cap_protrusion = 1.5
        r = self.outer_od / 2 + cap_protrusion
        return BBox(min=(-r, -r, 0.0), max=(r, r, self.h))

    @classmethod
    def from_camera_mount(cls, camera_mount, **extras):
        """Construct a `LensMount` (or subclass) sized to mate with the
        given `CameraMount`. The 4 bayonet-contract values
        (`cam_barrel_od`, `cam_barrel_id`, `lugs`, `lug_axial`) are
        plumbed automatically; `extras` is a passthrough for any
        subclass-specific kwargs the caller still wants to set."""
        return cls(
            cam_barrel_od=camera_mount.barrel_od,
            cam_barrel_id=camera_mount.barrel_id,
            lugs=camera_mount.lugs,
            lug_axial=camera_mount.lug_axial,
            **extras,
        )

    def _blank(self):
        return Tube(h=self.h, od=self.outer_od, id=self.inner_id)

    def _solid(self):
        """Material the cutters subtract from. Subclasses override to
        add bosses (e.g. pillow blocks) before the difference, so any
        boss-piercing cutters can act on them in the same pass."""
        return self._blank()

    def _bore_cutter(self):
        # Clearance-offset cylinder through the entire tube, shrunk
        # inward by `inner_shrink` (in diameter) so the bore cut reaches
        # the (extended-inward) blank inner edge. Cutter extends past
        # both ends of the tube by `default_eps()` to drop coplanar
        # bottom/top faces.
        eps = default_eps()
        bore_2d = circle(d=self.cam_barrel_od - self.inner_shrink).offset(r=self.clearance)
        return bore_2d.linear_extrude(height=self.h + 2 * eps).down(eps)

    def _channel_arc(self, p, angles):
        # Cross-section of one lug channel before the caller's
        # `.offset(r=clearance)`. Radially spans from `channel_inner_r`
        # (the breakthrough into the bore) out to a pre-offset outer
        # radius chosen so that AFTER the offset adds `clearance` back,
        # the channel's outer wall lands `lug_radial_clearance` past the
        # lug tip (`body_or + p`). This decouples the lug-tip radial gap
        # (`lug_radial_clearance`) from the general `clearance`, which the
        # offset still applies to the inner breakthrough and the
        # tangential ends.
        outer_r = self.body_or + p + self.lug_radial_clearance - self.clearance
        return Arc(
            r=(self.channel_inner_r + outer_r) / 2,
            width=outer_r - self.channel_inner_r,
            angles=angles,
        )

    def _entry_lug_cutter(self):
        # Lug-width arcs at the original lug angles, from the rest
        # plane to just past the entry rim. Each entry channel is
        # widened tangentially by `entry_arc_clearance` mm of arc
        # length on each side (converted per-lug to degrees by the
        # lug's mid-radius); only the entry channels are widened, not
        # the rotation channels, so the locked fit stays tight while
        # the initial drop-in is forgiving.
        entry_arcs = [
            self._channel_arc(
                p,
                angles=(
                    c - w / 2 - math.degrees(self.entry_arc_clearance / (self.body_or + p / 2)),
                    c + w / 2 + math.degrees(self.entry_arc_clearance / (self.body_or + p / 2)),
                ),
            )
            for c, w, p in self.lugs
        ]
        return (
            union(*entry_arcs).offset(r=self.clearance)
            .linear_extrude(height=self.insertion_depth + default_eps())
            .up(self.rest_z)
        )

    def _rot_cutter(self):
        # Lug arcs widened by `lock_angle_deg` on the CW edge,
        # `lug_axial` tall (+ lug_axial_clearance above for fit). The lug
        # front face rests on the bottom of this channel. The CW-edge
        # widening means the lens rotates CCW in camera coords
        # (CW from the viewer's perspective looking at the camera front)
        # to bring the indicator from its insertion angle (~10:30) into
        # the locked position at 12 o'clock.
        rot_arcs = [
            self._channel_arc(p, angles=(c - w / 2 - self.lock_angle_deg, c + w / 2))
            for c, w, p in self.lugs
        ]
        return (
            union(*rot_arcs).offset(r=self.clearance)
            .linear_extrude(height=self.lug_axial + self.lug_axial_clearance)
            .up(self.rest_z)
        )

    def _groove_cutter(self):
        # Long skinny triangular prism on the outer wall, apex pointing
        # inward, base on the outer surface, extruded the full tube
        # height. Placed so that AFTER the lens rotates into its locked
        # position, the groove ends up aligned with the widest lug's
        # world angle (toward the BRONICA logo).
        eps = default_eps()
        groove_2d = polygon(points=[
            (self.outer_or - self.groove_depth, 0.0),       # apex (inward)
            (self.outer_or + eps, self.groove_width / 2),    # base +Y (slight overshoot)
            (self.outer_or + eps, -self.groove_width / 2),   # base -Y
        ])
        return (
            groove_2d
            .linear_extrude(height=self.h + 2 * eps)
            .down(eps)
            .rotate([0, 0, self.groove_angle_deg])
        )

    def _cutters(self):
        """Generator of cutter Nodes for the main difference. Subclasses
        override to yield super()'s plus their own cuts, keeping all
        cuts in one Difference (avoids leaving thin veils of wall when
        a subclass cut would otherwise be in a separate pass)."""
        yield self._bore_cutter()
        yield self._entry_lug_cutter()
        yield self._rot_cutter()
        yield self._groove_cutter()

    def _alignment_dot_cap(self):
        # Small dome bump on the outer wall, centered on the same angle
        # as the alignment groove. The cap's flat base sits tangent to
        # the cylinder; embedded slightly inward (cap_embed) so the
        # union with the body stays a clean CSG (avoids tangent-line
        # contact between flat face and cylinder).
        cap_dia = 4.0
        cap_height = 1.5
        cap_inset = 0.2          # axial inset of cap's top edge from z=h
        cap_embed = default_eps()  # radial embed into the wall (CSG hygiene)
        return (
            Dome(cap_dia=cap_dia, cap_height=cap_height)
            .rotate([0, 90, 0])                        # dome from +z to +x
            .right(self.outer_or - cap_embed)
            .up(self.h - cap_inset - cap_dia / 2)
            .rotate([0, 0, self.groove_angle_deg])
        )

    def build(self):
        yield difference(self._solid(), *self._cutters())
        yield self._alignment_dot_cap()


class LensMountSupports(LensMount):
    """Modeled PETG supports for printing `LensMount` with a PLA part.

    Yields, per lug:
      - one central support arc — pillar in the bore region, spanning the
        swept-but-not-entry-channel arc of the lug's rotation channel and
        shifted radially inward by `support_inward_offset` so it isn't
        trapped under the bayonet lip during removal;
      - one rotation-channel filler — fits inside the rotation channel
        with `support_air_gap` of clearance above (under the channel roof)
        and below (above the bottom-flange top face), holding up the
        bayonet-lip ceiling during print. Inner edge extended inward by
        `support_inward_offset` so it overlaps the central pillar by
        ~0.1 mm radially — pillar + filler print as one connected part.

    Print orientation: bottom face on bed, entry rim pointing UP. Pair with
    the matching `LensMount` instance: print both as separate objects
    (PETG for supports, PLA for the mount, supports OFF on both objects).
    """

    support_air_gap = 0.1
    support_air_gap_deg = 0.5
    support_tube_wall_thk = 1.5
    support_inward_offset = 1.0
    # Axial floor where the central pillar starts. 0 = pillar sits on
    # the print bed. Subclasses with a plug disc in the bore (e.g.,
    # `BodyCapSupports`) raise this to the disc's top face so the PETG
    # pillar doesn't collide with the PLA disc.
    support_pillar_z_bottom = 0.0

    def tight_bbox(self):                                  # framework hook: walk the supports build tree
        # LensMount overrides tight_bbox to declare the lens-mount's
        # post-Difference extents, but our build yields only the
        # support pieces (no Difference). Fall back to the default
        # tree walk so the bbox reflects the supports themselves.
        return Component.tight_bbox(self)

    def _support_tube_radii(self):
        # Thin support-tube cross-section inside the bore: inner / outer
        # radii of the central pillar's arc.
        bore_inner_diameter = self.cam_barrel_od - self.inner_shrink + 2 * self.clearance
        outer_r = (
            (bore_inner_diameter - 2 * self.support_air_gap) / 2
            - self.support_inward_offset
        )
        inner_r = outer_r - self.support_tube_wall_thk
        return inner_r, outer_r

    def _support_seg_angles(self, c, w):
        # Per-lug angular extent of both the pillar and the filler:
        # CCW edge of the rotation channel minus a small gap, down to
        # the CW edge of the entry channel minus a small gap.
        seg_start = c - w / 2 - self.lock_angle_deg + self.support_air_gap_deg
        seg_end = c - w / 2 - self.support_air_gap_deg
        return seg_start, seg_end

    def _pillar(self, c, w, p, support_tube_inner_r, support_tube_outer_r):
        # Central pillar: bore region, from `support_pillar_z_bottom` up
        # to under the rotation-channel roof (offset by `support_air_gap`).
        seg_start, seg_end = self._support_seg_angles(c, w)
        z_bottom = self.support_pillar_z_bottom
        return (
            Arc(
                r=(support_tube_inner_r + support_tube_outer_r) / 2,
                width=support_tube_outer_r - support_tube_inner_r,
                angles=(seg_start, seg_end),
            )
            .linear_extrude(
                height=(
                    self.rest_z + self.lug_axial + self.lug_axial_clearance
                    - self.support_air_gap - z_bottom
                )
            )
            .up(z_bottom)
        )

    def _filler(self, c, w, p, support_tube_outer_r):
        # Rotation-channel filler: sits inside the rotation channel,
        # holding up its roof (the bayonet lip). Inner edge anchored to
        # the central pillar's outer edge minus `fuse_overlap` so the
        # two pieces always merge into one printed part — independent
        # of `inner_shrink` (which moves the pillar but not body_or).
        seg_start, seg_end = self._support_seg_angles(c, w)
        fuse_overlap = default_eps()
        filler_inner_r = support_tube_outer_r - fuse_overlap
        filler_outer_r = self.body_or + p - self.support_air_gap
        return (
            Arc(
                r=(filler_inner_r + filler_outer_r) / 2,
                width=filler_outer_r - filler_inner_r,
                angles=(seg_start, seg_end),
            )
            .linear_extrude(
                height=self.lug_axial + self.lug_axial_clearance - 2 * self.support_air_gap
            )
            .up(self.rest_z + self.support_air_gap)
        )

    def build(self):                                       # framework hook: required; returns the shape
        inner_r, outer_r = self._support_tube_radii()
        for c, w, p in self.lugs:
            yield self._pillar(c, w, p, inner_r, outer_r)
            yield self._filler(c, w, p, outer_r)


class BronicaS2LensMount(LensMount):
    """Concrete lens mount: lens-specific dimensions. The bayonet
    spec (cam_barrel_od, cam_barrel_id, lugs) is supplied at
    instantiation by reading them off the paired CameraMount instance.
    """

    # Bayonet-contract values come from the shared spec (single source
    # of truth shared with the lens housing in lens_housing.py).
    clearance      = BronicaS2Bayonet.mount_clearance
    lug_radial_clearance = BronicaS2Bayonet.lug_radial_clearance
    lug_axial_clearance  = BronicaS2Bayonet.lug_axial_clearance
    wall_thk       = BronicaS2Bayonet.mount_wall_thk
    inner_shrink   = BronicaS2Bayonet.mount_inner_shrink
    protrusion_len = BronicaS2Bayonet.mount_protrusion
    # Extra arc-length added to each side of the ENTRY channels (only)
    # so the lugs drop in without fiddling. Rotation channels stay tight.
    entry_arc_clearance = 1.0
    lock_angle_deg = BronicaS2Bayonet.lock_angle_deg
    # Bayonet-lip length above the rotation channel; the ring derives
    # `insertion_depth = lug_axial + lug_axial_clearance + mount_skirt` from
    # it (see the base equations). The camera-mount lug-front plane rests
    # at the channel floor, insertion_depth below the entry rim — the rim
    # must stay short enough not to impinge on the camera's face at
    # infinity focus. Cut to 2.0 to clear a second helicoid's collar
    # shoulder; see spec for history.
    mount_skirt = BronicaS2Bayonet.mount_skirt
    # Alignment groove on the outer wall, at the widest lug's center.
    groove_depth = BronicaS2Bayonet.alignment_groove_depth
    groove_width = BronicaS2Bayonet.alignment_groove_width
    # No part of the mount may extend more than these distances radially
    # past the OD wall — beyond them the feature collides with the focus
    # collar on the Bronica helicoid. The collar is stepped: features in
    # z=[h − focus_collar_narrow_axial, h] (camera-side end) must respect
    # `max_radial_extension_narrow`; features below that range may use
    # the wider `max_radial_extension_wide`.
    max_radial_extension_wide = (
        BronicaS2Bayonet.focus_collar_inner_dia_wide / 2
        - BronicaS2Bayonet.lens_mount_od / 2
    )
    max_radial_extension_narrow = (
        BronicaS2Bayonet.focus_collar_inner_dia_narrow / 2
        - BronicaS2Bayonet.lens_mount_od / 2
    )


class BronicaS2LensMountSupports(LensMountSupports, BronicaS2LensMount):
    """Bronica S2 modeled supports for the basic lens mount."""


class lens_mount_ring(Design):
    """Lens mount ring variants (basic, no body cap or locking lever).
    Body cap lives in `body_cap.py`; locking variant in
    `lens_mount_locking_ring.py`."""

    camera_mount = BronicaS2CameraMount()
    lens_mount = BronicaS2LensMount.from_camera_mount(camera_mount)
    lens_mount_supports = BronicaS2LensMountSupports.from_camera_mount(camera_mount)

    @variant(fn=96)
    def lens(self):
        return arrange_on_bed(self.lens_mount, plate=BAMBU_X2D_PLATE)

    @variant(fn=96)
    def lens_supports(self):
        # Modeled PETG supports for the lens part. Print as a SECOND object
        # alongside `lens` in Bambu Studio (PETG filament, support: off on
        # this object). Pre-aligned to the lens mount's world coords — load
        # both STLs without auto-arrange.
        return self.lens_mount_supports.red()

    @variant(fn=96, default=True)
    def lens_print(self):
        # Combined STL with the lens mount AND the modeled supports as
        # disconnected manifold solids in the same file. Workflow:
        # in Bambu Studio, Right-click → Split → To Objects, assign PLA to
        # the mount, PETG to the supports, support: off on both objects.
        return arrange_on_bed(
            union(self.lens_mount, self.lens_mount_supports.red()),
            plate=BAMBU_X2D_PLATE,
        )


if __name__ == "__main__":
    run()
