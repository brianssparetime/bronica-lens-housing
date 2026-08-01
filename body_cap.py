"""Bronica S2 body cap.

A `LensMount` ring (from `lens_mount_ring.py`) with a solid plug disc
filling the bore at the outward (lens-side) face. Worn on the camera
body when no lens is attached — the disc covers what would otherwise
be an open path into the mirror box.

Run:
    python s2-evolving/body_cap.py
"""

from scadwright.api.tolerances import default_eps
from scadwright.boolops import union
from scadwright.composition_helpers import arrange_on_bed
from scadwright.design import Design, run, variant
from scadwright.primitives import cylinder

from printer import BAMBU_X2D_PLATE
from rear_lens_cap import BronicaS2CameraMount
from lens_mount_ring import (
    BronicaS2LensMount,
    LensMount,
    LensMountSupports,
)


class BodyCap(LensMount):
    """Lens-mount ring with a solid plug disc filling the bore at the
    outward (lens-side) face. Worn on the camera body when no lens is
    attached — the disc covers what would otherwise be an open path
    into the mirror box.

    Geometry: the disc spans z=0 (lens-side rim, flush) to z=`disc_thk`
    in the ring's frame. The bayonet channels, alignment groove, and
    alignment dot bump on the ring are unchanged. Disc OD is sized
    slightly over the bore so the disc embeds ~`disc_radial_eps` mm
    radially into the ring's bore wall — same idiom the alignment-dot
    cap uses to keep the union CSG-clean.
    """

    disc_thk = 2.0
    # One-sided radial embed of the disc's outer wall into the ring's
    # bore wall. Diametral overlap is 2x this. Same idiom as
    # LensMount's alignment-dot `cap_embed`.
    disc_radial_eps = default_eps()

    def _disc(self):
        return cylinder(
            h=self.disc_thk,
            r=self.bore_inner_r + self.disc_radial_eps,
        )

    def build(self):                                       # framework hook: required; returns the shape
        yield from super().build()
        yield self._disc()


class BodyCapSupports(LensMountSupports, BodyCap):
    """Modeled PETG supports for printing a `BodyCap` with a PLA part.

    Same as `LensMountSupports` but the central pillar starts at
    `disc_thk` instead of z=0, so the PETG support doesn't collide
    with the PLA disc that fills the bore from z=0 to z=`disc_thk`.
    The pillar's bottom layer prints directly on the disc's top face
    — PETG-on-PLA forms a natural release boundary, no fusion.

    Print orientation and workflow are unchanged from `LensMountSupports`.
    """

    support_pillar_z_bottom = BodyCap.disc_thk


class BronicaS2BodyCap(BodyCap, BronicaS2LensMount):
    """Bronica S2 body cap: lens mount ring with a plug disc filling
    the bore at the outward face."""


class BronicaS2BodyCapSupports(BodyCapSupports, BronicaS2LensMount):
    """Bronica S2 modeled supports for the body cap. The pillar is
    lifted to sit on top of the plug disc instead of on the print bed."""


class body_cap(Design):
    """Body-cap variants."""

    camera_mount = BronicaS2CameraMount()
    body_cap = BronicaS2BodyCap.from_camera_mount(camera_mount)
    body_cap_supports = BronicaS2BodyCapSupports.from_camera_mount(camera_mount)

    @variant(fn=96, default=True)
    def cap(self):
        return arrange_on_bed(self.body_cap, plate=BAMBU_X2D_PLATE)

    @variant(fn=96)
    def cap_print(self):
        # Combined STL with the body cap AND the modeled supports as
        # disconnected manifold solids in the same file. Workflow:
        # in Bambu Studio, Right-click → Split → To Objects, assign PLA
        # to the cap, PETG to the supports, supports off on both.
        return arrange_on_bed(
            union(self.body_cap, self.body_cap_supports.red()),
            plate=BAMBU_X2D_PLATE,
        )


if __name__ == "__main__":
    run()
