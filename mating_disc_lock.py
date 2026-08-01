"""Shared spec for the lens-housing ↔ locking-ring tab/notch lock.

Both `lens_housing.py` (which builds the tabs onto the mating disc) and
`lens_mount_locking_ring.py` (which cuts the matching pockets in the ring's
bore wall) import from here so the two parts agree on dimensions and
angles without duplication. Kept in its own module so importing the
spec doesn't pull in either Design class (which collides on variant
names like `print`).
"""

from collections import namedtuple

from spec import BronicaS2Bayonet


# Lock between the mating disc and the locking lens-mount ring: a small
# number of tabs protrude radially OUTWARD from the disc's OD and seat
# into matching notches cut into the bore wall of the locking ring. The
# pair-up fixes the housing's rotational orientation against the ring
# (without this, the housing could rotate freely inside the disc bore).
# Asymmetric angles ensure the housing only mates one way.
#   - axial: tab height in z (mm)
#   - radial: radial extent past the disc OD (mm)
#   - circumferential: tangential extent at the disc OD (mm); the tab
#     is built as a parallel-sided box, not a wedge, so this is a
#     straight tangential dimension rather than an arc length
#   - z_top: top z of each tab in HOUSING-WORLD frame; tabs hang down
#     by `axial` from this. Typically z_top = MOUNT_OFFSET so the tab
#     top is flush with the disc top (= flange-flange bottom).
#   - angles_deg: tab center angles in HOUSING-FRAME degrees (the same
#     frame the alignment groove uses — housing groove sits at
#     HOUSING_GROOVE_ANGLE_DEG). The lens-mount ring converts these to
#     ring-frame by subtracting (HOUSING_GROOVE_ANGLE_DEG − ring's own
#     groove_angle_deg) so the two grooves line up at lock.
#   - clearance: print-fit gap (mm) the ring's notch leaves on every
#     face of the tab (radial, axial, tangential — all the same).
#   - housing_tab_offset_deg: angular offset (housing-frame degrees)
#     added to each tab angle when the housing builds the tab
#     geometry. The notches in the lens-mount ring are NOT shifted by
#     this offset — only the tabs on the housing's mating disc.
#     Purpose: when the housing prints in a flipped orientation (e.g.
#     the testfit's 180° X-rotation for lens-side-down bed adhesion),
#     the print mirrors the angular convention. Without compensation,
#     the natural mate-with-ring orientation puts the housing/ring
#     alignment grooves 60° apart. Negative values rotate tabs CW in
#     housing frame = CCW in the testfit's print frame.
MatingDiscTabs = namedtuple(
    "MatingDiscTabs",
    "axial radial circumferential z_top angles_deg clearance disc_od "
    "housing_tab_offset_deg",
)


# Housing's alignment groove sits at this angle in the housing frame
# (set by _alignment_groove_cutter in LensHousing). Shared with the
# locking lens-mount ring so its notches land at the right ring-frame
# angles for the two grooves to line up at lock.
HOUSING_GROOVE_ANGLE_DEG = 90.0


# Diametral slip-fit clearance for the mating disc inside the lens-mount
# ring's bore. 0.25 mm diametral = 0.125 mm radial per side — enough that
# the disc seats without binding on typical FDM over-extrusion/warp.
# Shared so both the housing (sizing the disc OD) and the ring (sizing
# the tab-notch cutout from the disc edge) agree on the same value.
MATING_DISC_CLEARANCE = 0.25


# Default tabs for the BronicaS2 housing/ring pair. 3.8 mm axial × 2.4 mm
# radial × 6 mm circumferential — enlarged from 2.0×1.2×3.0 so the keying
# is obvious by feel and the ring can't go on backwards. Top-flush with
# the disc top (= flange-flange bottom). Angles 55° and 185° in housing
# frame — asymmetric, in ring-frame these come out at 355° and 125°, each
# centered in a ~20° gap between lug rotation channels.
#   axial 3.8: the maximum that keeps the tab fully backed by the disc
#     (z=[1.2, 5.0]); a 4 mm tab would hang 0.2 mm below the disc and
#     force the disc's 1.2 mm clearance to the camera lugs to shrink.
#   radial 2.4: notch reaches r=32.4 of the 34.05 ring wall (1.7 mm left).
#   circumferential 6.0: ~3.9° (~2 mm) of solid wall to the nearest
#     rotation channel, plus the notch tops out a millimetre below the
#     channels — comfortable.
BRONICA_S2_MATING_DISC_TABS = MatingDiscTabs(
    axial=3.8,
    radial=2.4,
    circumferential=6.0,
    z_top=BronicaS2Bayonet.mount_protrusion,
    angles_deg=(55.0, 185.0),
    clearance=0.15,
    disc_od=BronicaS2Bayonet.lens_mount_bore - MATING_DISC_CLEARANCE,
    # Offset compensates for the testfit's 180° X-print rotation:
    # housing tabs at (55, 185) and notches at ring (-5, 125) naturally
    # mate at an orientation where the housing/ring alignment grooves
    # are 60° apart. Shifting housing tabs by -60° (= CW in housing,
    # CCW in the testfit's print view) puts the assembly at the
    # orientation where the grooves coincide.
    housing_tab_offset_deg=-60.0,
)


# Snap-fit retention bead around the mating disc's OD that seats into a
# matching groove cut into the locking ring's bore wall. Provides axial
# retention (housing won't slide out under axial tension once snapped
# in). The bead is a continuous 360° triangular ridge; the groove is a
# matched continuous 360° triangular pocket cut into the bore wall.
# Tabs (above) handle rotational lock; the bead is purely axial.
#   - height: radial protrusion past the disc OD (mm). Sized so the
#     bead's apex is slightly past the bore wall (interference), giving
#     a snap during insertion as the bead deflects past the bore lip.
#   - axial: total axial width of the bead (mm). Triangular cross-
#     section, so the chamfer angle from vertical is
#     atan(height / (axial/2)).
#   - z_center: bead center z in housing-world frame.
#   - groove_radial_clearance: extra radial depth of the groove past
#     the bead's apex; lets the bead seat without binding axially.
#   - groove_axial_clearance: extra axial width of the groove past the
#     bead; for the bead to sit centered with a hair of play.
MatingDiscBead = namedtuple(
    "MatingDiscBead",
    "height axial z_center groove_radial_clearance groove_axial_clearance",
)


# Default bead for the BronicaS2 housing/ring pair. Centered 1.25 mm
# from the lens-side face of the ring (= housing-frame z=3.75 = MOUNT_
# OFFSET − 1.25), well inside the disc's z range [1.2, 5].
#   height=0.20: with disc-bore radial clearance of 0.125 per side, the
#     bead protrudes 0.075 mm past the bore wall — gentle snap.
#   axial=1.0: chamfer angle from vertical ~22°, well inside the self-
#     supporting overhang limit. Bead/groove print without support.
#   groove_radial_clearance=0.05: bead apex at (disc_r + height),
#     groove apex at (bore_inner_r + height + clearance) — bead seats
#     with 0.05 mm of radial play past contact.
#   groove_axial_clearance=0.05: 0.10 mm wider than bead axially.
BRONICA_S2_MATING_DISC_BEAD = MatingDiscBead(
    height=0.20,
    axial=1.0,
    z_center=BronicaS2Bayonet.mount_protrusion - 1.25,
    groove_radial_clearance=0.05,
    groove_axial_clearance=0.05,
)
