"""Bronica S2 camera-side bayonet artifacts.

Camera-side artifacts relative to the lens-mount ring:

- `CameraMount` — the open tube with three outward-projecting bayonet
  lugs at the top rim, matching the actual Bronica S/S2/S2A/EC/EC-TL
  camera-side bayonet (one large lug ~50° wide + two smaller ~40° lugs
  at roughly 120° spacing). Standalone artifact for verifying the lens-
  mount ring's bayonet engagement against printed dimensions.

- `RearLensCap` — a `CameraMount`-style bayonet stub on the lens side
  plus a cup that closes off the rear of the lens when no camera is
  attached. Mimics what the camera looks like from the lens's
  perspective — the lens-mount ring clicks onto the cap's lugs the
  same way it clicks onto the camera.

- `BronicaS2RearLensCapSymmetric` — a Bronica-sized rear cap with
  three identical small lugs at 120° spacing. Mates with the lens-
  mount ring in any of three rotational orientations, one of which
  visually aligns with the camera's bayonet.

Run:
    python s2-evolving/rear_lens_cap.py
"""

from scadwright import Component
from scadwright.api.tolerances import default_eps
from scadwright.composition_helpers import arrange_on_bed
from scadwright.design import Design, run, variant
from scadwright.primitives import cylinder
from scadwright.shapes import Arc, Tube

from printer import BAMBU_X2D_PLATE
from spec import BronicaS2Bayonet


class CameraMount(Component):
    """Open tube with a per-lug list of outward-projecting bayonet lugs
    at the top rim.

    `barrel_id` / `wall_thk` / `length` define the open cylindrical body
    (`barrel_od = barrel_id + 2 * wall_thk` is computed). `lug_axial` is
    the axial thickness of each lug; `lugs` is a tuple of
    `(center_deg, width_deg, radial_proj_mm)` triples, one per lug, so
    the bayonet pattern can be asymmetric.
    """

    equations = """
        barrel_id, length, wall_thk, lug_axial > 0
        barrel_od = barrel_id + 2 * wall_thk
        body_or = barrel_od / 2
        lug_axial <= length
        len(lugs:tuple) >= 1
    """

    def build(self):                                       # framework hook: required; returns the shape
        yield Tube(h=self.length, od=self.barrel_od, id=self.barrel_id)
        z_lug_bottom = self.length - self.lug_axial
        # Each lug: an annular sector (Arc) extruded vertically, rotated
        # to its center angle, lifted to the top of the barrel.
        for center_deg, width_deg, proj_mm in self.lugs:
            half = width_deg / 2
            yield (
                Arc(
                    r=self.body_or + proj_mm / 2,
                    width=proj_mm,
                    angles=(center_deg - half, center_deg + half),
                )
                .linear_extrude(height=self.lug_axial)
                .up(z_lug_bottom)
            )


class BronicaS2CameraMount(CameraMount):
    """Concrete mount sized to the Bronica S/S2/EC camera mount.
    All measurements live in `BronicaS2Bayonet` — single source of
    truth for the bayonet contract shared across files."""
    barrel_id = BronicaS2Bayonet.cam_barrel_id
    wall_thk  = BronicaS2Bayonet.cam_wall_thk
    length    = BronicaS2Bayonet.cam_length
    lug_axial = BronicaS2Bayonet.cam_lug_axial
    lugs      = BronicaS2Bayonet.lugs


class RearLensCap(Component):
    """Rear lens cap with a `CameraMount`-style bayonet stub on the
    lens side and a hollow cup with a closing disc on the camera side.

    The lug-bearing tube reuses `CameraMount` directly (same OD/ID,
    same lug pattern), with its `length` overridden to `lug_tube_len`
    (typically matching the lens housing's lh_outer segment so the
    bayonet engagement region is sized like a real camera's). The
    cup is a longer hollow tube below the lug section, closed at the
    far end by a flat disc.

    Tiny axial `*_axial_eps` overlaps at the disc–cup and cup–lug
    seams keep the CSG union preview-clean (same idiom as the body
    cap's radial disc-overlap).
    """

    equations = """
        cam_barrel_od, cam_barrel_id, lug_axial > 0
        cam_barrel_od > cam_barrel_id
        cup_od, cup_wall_thk > 0
        lug_tube_len, disc_thk, total_h > 0
        total_h > lug_tube_len + disc_thk
        cup_h = total_h - lug_tube_len
        cup_tube_len = cup_h - disc_thk
        cup_id = cup_od - 2 * cup_wall_thk
        # Cup interior must be wide enough to clear the lug-tube interior
        # so the hollow interior is continuous across the seam.
        cup_id >= cam_barrel_id
        # Lug pattern (center_deg, width_deg, radial_proj_mm) per lug.
        len(lugs:tuple) >= 1
    """

    # Axial eps overlaps for clean CSG at the disc-cup and cup-lug
    # interfaces — framework-tracked tolerance (same idiom as
    # `BodyCap.disc_radial_eps`).
    disc_axial_eps = default_eps()
    lug_tube_axial_eps = default_eps()

    def build(self):                                       # framework hook: required; returns the shape
        # Cup tube: hollow section between the closing disc and the lug
        # tube. OD matches the lug tube (per design choice), so the
        # cap's outer surface is uniform.
        yield (
            Tube(h=self.cup_tube_len, od=self.cup_od, id=self.cup_id)
            .up(self.disc_thk)
        )
        # Closing disc at z=0 (camera-side end). Extended by `disc_axial_eps`
        # into the cup's hollow interior so the union is preview-clean.
        yield cylinder(
            h=self.disc_thk + self.disc_axial_eps,
            r=self.cup_od / 2,
        )
        # Lug tube: a CameraMount with `length = lug_tube_len`,
        # geometrically identical to the camera-side bayonet stub.
        # Shifted up by `cup_h − eps` so the bottom of its tube
        # overlaps the cup tube's top by `lug_tube_axial_eps` for a
        # clean CSG seam.
        lug_section = CameraMount(
            barrel_id=self.cam_barrel_id,
            wall_thk=(self.cam_barrel_od - self.cam_barrel_id) / 2,
            length=self.lug_tube_len + self.lug_tube_axial_eps,
            lug_axial=self.lug_axial,
            lugs=self.lugs,
        )
        yield lug_section.up(self.cup_h - self.lug_tube_axial_eps)


class BronicaS2RearLensCap(RearLensCap):
    """Concrete rear lens cap sized to the Bronica S2 bayonet. Lug
    geometry comes from the shared bayonet spec; cup OD matches the
    lug-tube OD so the cap is a uniform 60.5 mm cylinder."""

    cam_barrel_od = BronicaS2Bayonet.cam_barrel_od
    cam_barrel_id = BronicaS2Bayonet.cam_barrel_id
    lug_axial     = BronicaS2Bayonet.cam_lug_axial
    lugs          = BronicaS2Bayonet.lugs
    # Lug-bearing tube length matches the lh_outer segment of the lens
    # housing — the axial extent the bayonet engagement region occupies
    # when a real lens is mounted on a real camera.
    lug_tube_len  = BronicaS2Bayonet.lower_housing_outer_len
    # Cup OD equal to the lug-tube OD: cap reads as a single uniform
    # cylinder rather than a stepped shoulder.
    cup_od        = BronicaS2Bayonet.cam_barrel_od
    cup_wall_thk  = 2.0
    disc_thk      = 2.0
    total_h       = 50.0


class BronicaS2RearLensCapSymmetric(BronicaS2RearLensCap):
    """Rear lens cap with three identical small lugs at 120° spacing
    instead of the spec's asymmetric pattern. Mates with the lens-
    mount ring in any of three rotational orientations; the
    orientation anchored on the spec's wide-lug angle is the one
    that visually aligns with the camera.

    Lug width is the largest that fits inside every spec entry
    channel under the 120° symmetric arrangement, minus a 1° fit
    margin (matching the spec's convention for measured widths).
    Lug projection equals the smallest spec projection.
    """

    # Symmetric lug pattern computed at class-definition time from
    # the bayonet spec. Anchor on the widest spec lug's centre so one
    # mounting orientation aligns visually with the camera. For each
    # k ∈ {0,1,2} the cap-lug at `anchor + 120·k` must fit inside
    # spec channel k; the limiting half-width per k is
    # `channel_half − |cap_lug_centre − channel_centre|` with the
    # angle delta normalised to [-180, 180]. The minimum over k,
    # doubled, gives the largest admissible symmetric width.
    def _compute_symmetric_lugs():
        spec_lugs = BronicaS2Bayonet.lugs
        anchor = spec_lugs[0][0]
        max_half = min(
            spec_lugs[k][1] / 2
            - abs(((anchor + 120 * k - spec_lugs[k][0] + 180) % 360) - 180)
            for k in range(3)
        )
        width = 2 * max_half - 1.0
        proj = min(p for _, _, p in spec_lugs)
        return tuple((anchor + 120 * k, width, proj) for k in range(3))

    lugs = _compute_symmetric_lugs()
    del _compute_symmetric_lugs


class rear_lens_cap(Design):
    """Camera-side artifacts for the Bronica S2 bayonet: the camera-
    mount test-fit stub, the rear lens cap, and the symmetric rear
    lens cap."""

    camera_mount = BronicaS2CameraMount()
    cap_part = BronicaS2RearLensCap()
    cap_part_sym = BronicaS2RearLensCapSymmetric()

    @variant(fn=96)
    def mount_testfit(self):
        # Standalone camera-mount stub for verifying the lens-mount
        # ring's bayonet engagement against printed dimensions.
        return arrange_on_bed(self.camera_mount, plate=BAMBU_X2D_PLATE)

    @variant(fn=96)
    def rear_cap(self):
        return arrange_on_bed(self.cap_part, plate=BAMBU_X2D_PLATE)

    @variant(fn=96, default=True)
    def rear_cap_sym(self):
        return arrange_on_bed(self.cap_part_sym, plate=BAMBU_X2D_PLATE)


if __name__ == "__main__":
    run()
