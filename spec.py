"""Bronica S2 bayonet contract.

Single source of truth for the dimensions shared by the bayonet-
related modules — `rear_lens_cap.py` (camera mount + rear cap),
`lens_mount_ring.py` (basic lens mount ring), `body_cap.py` (body
cap), `lens_mount_locking_ring.py` (locking ring + lever),
and `lens_housing.py` (the lens housing). Splits cleanly into three
groups, all under one `Spec`:

  - Physical measurements of the camera-side bayonet (rear-cap STL +
    direct measurement of the camera lug). True regardless of which
    part reads them.

  - Design choices for the lens-mount ring (clearance to the camera
    barrel, wall thickness, bore-side wall reinforcement). The lens
    mount and lens housing both must agree on these to interface
    correctly.

  - Derived interface dimensions (lens-mount OD, lens-mount bore). Both
    sides read these directly — no risk of one side hand-computing a
    stale value when the underlying design choice shifts.
"""

from scadwright import Spec


class BronicaS2Bayonet(Spec):
    equations = """
        # === Camera-side bayonet (physical, measured) ===
        # From the rear-cap STL.
        cam_barrel_od = 60.5
        cam_barrel_id = 56.5            # also matches the M57 thread behind the bayonet
        cam_wall_thk = 2.0
        cam_length = 20.0

        # Lug axial thickness — measured directly on the camera lug.
        cam_lug_axial = 1.1
        cam_lug_axial += 0.05           # +0.05 after a test print showed the rotation channel too tight

        # Lug pattern: (center_deg, width_deg, radial_proj_mm) per lug.
        # Centers in camera coords (0° = +X, 90° = +Y = up toward BRONICA logo).
        # Widths shrunk 1° from measured (undersized lugs still mate, oversized jam).
        # One large lug + two smaller, centers ~120° apart with slight asymmetry.
        # Per-lug: large (measured 50°, scan 49.5°); small (measured 40°, scan 174.25°);
        # small (measured 41.5°, scan 283.75°).
        lugs = ((90.0, 49.0, 1.8), (214.75, 39.0, 1.2), (324.25, 40.5, 1.2))
        max_lug_proj = max(p for c, w, p in lugs)

        # === Lens-mount ring design choices ===
        # General radial clearance, applied as a 2D offset to the bore cut
        # (gap to the camera barrel) and to the lug channels EXCEPT their
        # radially-outward cut depth (the lug-tip gap), which is governed
        # separately by `lug_radial_clearance` below. So this covers the
        # bore fit, the channel inner breakthrough, and the channel
        # tangential gaps; it does NOT set the lug-tip radial play.
        mount_clearance = 0.20
        # Radial gap from the lug TIP to the channel's outer wall
        # (`channel_outer = body_or + p + lug_radial_clearance`). Split
        # out of `mount_clearance` so the lugs' radial play in the
        # channels can be tightened without also tightening the bore fit
        # or the entry/tangential gaps. 0 = channel cut exactly to the lug
        # tip (snug, relying on FDM tolerance); raise for more play.
        lug_radial_clearance = 0.0
        # Axial slop in the rotation channel (channel z-height = lug_axial +
        # lug_axial_clearance). Tightens lens-to-camera wobble independently
        # of the radial clearances.
        # Was 0.1 mm with Bambu's auto-generated supports. Bumped to 0.3 mm
        # for the modeled-PETG-support workflow: the air gap above the filler
        # lets the channel roof sag slightly, and the PETG/PLA boundary prints
        # rougher than a slicer-support face — both eat into the nominal slop.
        # Next, after test print (was .3), but that was just a tiny bit too loose, -> .25
        lug_axial_clearance = 0.25
        # Lens-mount tube wall thickness, measured outside the lugs.
        mount_wall_thk = 2.0
        # `inner_shrink` shrinks the blank's inner diameter (and the bore cut)
        # by this amount diametral, leaving the OD and channel walls untouched.
        # 0 = bore matches `cam_barrel_od + 2*clearance`. Bumping it makes the
        # bore-side wall thicker and the cuts reach that much deeper inward.
        mount_inner_shrink = 1.0
        # Axial extent of the lens-mount tube above the camera flange plane.
        # Was `protrusion_len` on the lens mount, `MOUNT_OFFSET` on the housing.
        mount_protrusion = 5.0
        # Axial length from the camera-facing rim of the lens-mount ring
        # down to the camera-side face of the rotation channel — the
        # bayonet lip the lug seats under. Shrinking it moves the rim
        # toward the channel (sands material off the rim) without touching
        # the channel itself or the lens-side stack; the ring derives
        # `insertion_depth = lug_axial + lug_axial_clearance + mount_skirt`.
        # Reduced 2.1 -> 1.7 (1.8 still bottomed out too tight on the
        # ECTL helicoid); see the note box below for the per-helicoid
        # rationale.
        mount_skirt = 1.7

        ##############################################################################
        #
        # BST note:  the s2a helicoid works with a skirt of 2.1 (where it bottoms
        #            out on the plasic focus ring), but the ECTL helicoid has a 
        #            shoulder that moves with with the lugs and may have protruding
        #            shim or screws, so it requires a shorter skirt 
        # 
        #    DO NOT DELETE OR ALTER THIS COMMENT BOX WITHOUT EXPLICITLY ASKING!!!!!
        #
        ##############################################################################

        # Angle the lens rotates from insertion to fully-locked position
        # (camera-frame, +CCW). Widens each rotation channel on its CCW
        # edge by this amount so the lug can sweep into the locked seat.
        # In lens-frame this is also the offset between the lens-mount
        # ring's printed groove and the BRONICA logo's 12-o'clock target
        # (`groove_angle_deg = widest_center - lock_angle_deg`).
        # Calibration history, two test fits:
        #   1. With lock_angle_deg=45, the printed indicator at MATING
        #      (before rotation) sat ~7.5 mm CW of where it should be
        #      relative to the BRONICA logo. Wanted to push the mating
        #      mark CCW.
        #   2. First correction lowered lock_angle_deg to 32.4 (the wrong
        #      way), which moved the mating mark CW instead of CCW; on
        #      the next test print it ended up about twice as far in
        #      that wrong direction.
        # Net: undo that wrong move AND apply the originally-intended
        # one in the right direction. 7.5 mm at the lens-mount outer
        # radius ~34.05 mm is ~12.6°, so lock_angle_deg = 45 + 12.6 =
        # 57.6. This elongates the rotation channel by 12.6° vs. the
        # original (and 25.2° vs. the wrong-way fix), shifting the
        # printed groove CCW in viewer frame so it lands at 12 o'clock
        # when locked.
        # 3. Further +2.4° on the locking-2 latch print — at 57.6 the
        #    indicator was still a hair short of the BRONICA logo at the
        #    locked stop. Total now 60° (lengthens the rotation channels
        #    by another 2.4° so the lens reaches the right rotation).
        lock_angle_deg = 60.0
        # Alignment groove cross-section (depth into the part, base
        # width on the surface). Shared by the lens-mount ring's
        # outer-wall groove and the lens housing's bottom-face groove
        # — when the ring is positioned correctly on the housing the
        # two grooves coincide, giving a visual cue for assembly.
        alignment_groove_depth = 0.3
        alignment_groove_width = 0.5

        # === Derived interface (read by both lens housing and lens mount) ===
        # OD of the lens-mount tube outside the lugs (incl. wall thickness).
        lens_mount_od = cam_barrel_od + 2*max_lug_proj + 2*mount_wall_thk
        # Inner bore of the lens-mount ring after the bore-cutter pass.
        # Mating disc on the housing fits inside this; previous bug had this
        # hand-mirrored on the housing side and drifted when
        # `mount_inner_shrink` changed.
        lens_mount_bore = cam_barrel_od - mount_inner_shrink + 2*mount_clearance

        # === Focus collar bore (camera-side constraint) ===
        # The Bronica helicoid's focus collar surrounds the lens-mount ring.
        # The collar has a stepped inner bore: a narrow constriction at
        # the camera-side end and a wider opening for the rest. Nothing
        # in the lens-mount assembly may extend past these radii.
        #
        # In the lens-mount frame z=0 is the lens-side bottom flange and
        # z=h is the camera-side entry rim. The collar's camera-side end
        # coincides with z=h, so the NARROW section constrains
        # z=[h − focus_collar_narrow_axial, h] and the WIDE section
        # constrains z below that.
        focus_collar_inner_dia_wide = 79.7      # mm; bore in the wider region
        focus_collar_inner_dia_narrow = 76.5    # mm; bore in the narrow constriction
        focus_collar_narrow_axial = 4.8         # mm; axial extent of the narrow constriction

        # === Helicoid focus slot (camera attribute) ===
        # A shallow axial groove in the mid-segment wall that clears a
        # focus-helicoid pin protruding from the camera body. Uniform
        # radial `depth` mm in from the OD across the slot's whole
        # angular extent — the pin rides in this groove throughout the
        # bayonet rotation. Two angular regions union into one cut:
        #   - LOCKED side: `width` mm circumferential at the mid OD,
        #     centered at `angle` (9 o'clock / -X axis). The pin sits
        #     here when the lens is locked.
        #   - SWEEP side: extends CW from `angle` by `lock_angle_deg`
        #     (toward the alignment groove at 12 o'clock / +Y) to cover
        #     the pin's path as the lens rotates from insertion to
        #     locked.
        # The slot's flange-side end sits `offset_from_flange` mm short
        # of z=0; the deep end stops at the mid/inner step (derived in
        # the housing as -(lower_housing_outer_len + lower_housing_mid_len))
        # so the slot never opens into the inner segment.
        helicoid_focus_slot_angle = 180              # deg; 180 = -X axis (locked-side center)
        helicoid_focus_slot_width = 8                # mm circumferential at OD (locked-side arc length)
        helicoid_focus_slot_depth = 0.8              # mm radial inward from OD, uniform across the slot
        helicoid_focus_slot_offset_from_flange = 11  # mm; slot z_top = -this in housing frame

        # === Aperture feeler slot (camera attribute) ===
        # A shallow groove in the inner-segment wall that clears a small
        # aperture-feeler protruding from the camera body. Same swept-
        # cut structure as the helicoid focus slot but in the inner
        # (deepest) segment, directly opposite the helicoid: at 3 o'clock
        # (+X axis; `angle = 0`) when the lens is locked. Sweep arc
        # equals `lock_angle_deg` so the feeler clears at every angle as
        # the lens rotates from insertion to locked. The slot's axial
        # extent spans the entire inner segment (mid/inner step at the
        # top, past the inner segment's deep end at the bottom) so the
        # feeler doesn't catch on either rim during insertion / removal —
        # the housing derives those z values from `lower_housing_*_len`.
        # Physical feeler extent on the camera body: ~23–29 mm below the
        # flange; the housing's slot is intentionally wider.
        #   angle: deg; 0 = +X axis (locked-side center).
        #   width: mm circumferential at the inner-segment OD.
        #   depth: mm radial inward from OD, uniform across the slot.
        aperture_feeler_slot_angle = 0               # deg; 0 = +X axis
        aperture_feeler_slot_width = 15              # mm circumferential at OD
        aperture_feeler_slot_depth = 0.5             # mm radial inward from OD

        # === Stepped throat fit (lower housing) ===
        # The M57 throat steps inward as the lower housing reaches deeper
        # into the camera body; each lower-housing segment matches one
        # throat step. Lengths measure below the flange plane (z=0) and
        # sum to the total insertion depth. Outer is the flange-side step
        # (continues above the flange through the mount-ring zone); mid
        # carries the helicoid focus slot; inner is the deepest, narrowest
        # step and stays clear of the slot.
        lower_housing_outer_od = 56.0
        lower_housing_outer_len = 7.3
        lower_housing_outer_thk = 1.5
        lower_housing_mid_od = 54.25
        lower_housing_mid_len = 13.0
        lower_housing_mid_thk = 1.75
        lower_housing_inner_od = 49.0
        lower_housing_inner_len = 9.7
        lower_housing_inner_thk = 1.5
        housing_insertion_depth = lower_housing_outer_len + lower_housing_mid_len + lower_housing_inner_len
    """
