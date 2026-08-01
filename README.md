#  Bronica programmatic 3D-printed lens housing and accessories

A working collection of 3D-printed parts for the Bronica S/S2/S2A/EC/EC-TL camera system, all built in [SCADwright](https://github.com/brianssparetime/SCADwright).


<p align="center">
  <img src="img-vid/IMG_2742%20Large%20angle.jpeg" alt="mounted" width="640">
</p>

*Completed lens mounted on my EC-TL.  I'm better at optical engineering and 3d modeling than at painting things*


## Goal

I wanted a swirly, roughly 100mm f/2.8 lens for the Bronica S2, built from $5-7 surplus elements. 

[My first attempt](https://www.brianssparetime.com/posts/my-first-attempt-at-designing-and-building-a-lens-from-scratch/) was trash, both optically and mechanically.

That first housing was a muzzle-loader:  the main part was a tube with a rear retaining ring.  This was loaded from the front with elements, spacer tubes, and a stop disc, all packed in behind a front snap-in bezel. Elements rattled, and a constant-width tube did little to keep them perpendicular to the light path. It mounted by friction fit into an M57-to-M42 adapter, which was sketchy at best.

The second housing set out to fix all of it:

- mount on the bayonet directly, with a latch, no adapter
- change aperture without full disassembly
- hold axial spacing accurately
- grip elements tightly enough to keep them centered and perpendicular
- stay independent of the optical design, so iterating the optics doesn't restart the housing

This design meets nearly all of that. Rather than loading from the muzzle, it splits down the optical axis into two halves that snap together. The locking mount ring then snaps on over the seam, adding a second layer of security and keeping supports minimal and easy to pull.

<p align="center">
  <img src="img-vid/IMG_2834%20Large%20all%20three%20hero.jpeg" alt="Three printed lens housings with mount rings, stops, and covers" width="640">
</p>

*Three builds of the housing, with the parts that go around them: locking mount rings, the C-clip aperture slot cover, stop discs and caps. The open halves show the element stack seated in its holders. The closed barrels show the build stamp on one side, design name over element codes over commit hash, and the engraved optical plan on the other.*

## What's in the collection

- **Lens housing with locking mount ring** — a full lens housing built from a JSON optical configuration (focal length, FOV, per-element diameters and spacings, stop position) in two halves, with:
  - a native Bronica bayonet mount, with locking lever, that snap-locks onto main housing halves.
  - waterhouse stop discs, one per aperture in the config, a full stop apart
  - aperture slot cover
  - snap-on front lens cap
  - snap-on lens hood
- **Rear lens cap** — closes off the back of a lens off-camera. Two flavours: a one-orientation cap matching the camera's asymmetric three-lug bayonet, and a symmetric three-orientation variant.  (still testing)
- **Body cap** — closes off the camera body when no lens is mounted.

<p align="center">
  <img src="img-vid/IMG_2743%20Large%2013f%20beauty.jpeg" alt="One housing taken apart: two halves, mount ring, aperture cover, and four waterhouse stops" width="640">
</p>

*One lens taken apart: the two housing halves with the element stack seated, the locking mount ring, the C-clip aperture cover, and four waterhouse stops. The stops show the paddle profile, a small semicircle for the bore and a large one for the wall slot, each with a different hole.*


## Features

- **Two halves, snap-fit and light-tight**, split along the optical axis by a stepped lap joint (`lap_split_x`).
- **Optical elements precisely positioned** in `ElementHolder`s sized to each element's diameter, edge thickness, and spacing.
- **Precise, stepped, recessed housing** that narrows in three steps (outer → mid → inner), with a bore-side fillet ring at each step, following the profile inside the S2's mount barrel.
- **Locking bayonet mount** through `lens_mount_locking_ring`: integrated pillow blocks hold a sewing-pin-pivoted curved lever whose tooth swings into the rotation channel. The torsion spring is the one part I haven't solved.
- **Fits both helicoids.** The S2A and EC-TL differ subtly, and the mount skirt in `spec.py` is sized to clear both.
- **Waterhouse stops**, one disc per aperture in the config, a full stop apart. Each disc is a paddle: the small semicircle sits in the bore as the aperture plate, the large one fills the wall slot, so stops swap through the slot without opening the housing.
- **Clip-on aperture cover** (`ApertureCover`) closes the stop slot against stray light. A C-clip wrapping past 180° snaps over center, with a pry tab at each end.
- **Clip-on hood** (`LensHood`) sized off the housing's hood-base angle and front-element diameter, so its flare follows the field of view in the optical plan.
- **Clip-on lens cap** (`LensCap`) shares the hood's coupler and snap groove, so the two seat identically, but has a flat lid in place of the funnel.
- **Funnel-widened upper barrel** when the front element is wider than the bayonet bore.
- **Tab/notch alignment** between the housing's mating disc and the locking ring, so once seated the housing can't rotate relative to the ring, plus a snap-fit bead that stops it pulling off.
- **Front fillet rim with text** (`TruncFilletRing`) wrapping the aperture, with the focal length and aperture engraved into its slanted face, e.g. `BRIOTAR 101mm f/2.9`.
- **Build identity on the barrel**: the design name, the code of every element in the stack, and the git short SHA it was built from, so any printed part traces back to the source that made it.
- **Optical-plan engraving** on the opposite side, the ray diagram for that design wrapped around the barrel with [`wrap_2d`](https://github.com/brianssparetime/SCADwright/blob/master/docs/wrap_2d.md).
- **Constraints that fail fast.** Every part declares its relationships and limits as equations, so a config that can't be manufactured errors at build time instead of quietly rendering something unprintable.
- **Print-bed-aware layouts.** Print variants run through [`arrange_on_bed`](https://github.com/brianssparetime/SCADwright/blob/master/docs/composition_helpers.md#arrange_on_bed) against the Bambu X2D plate, so a layout that won't fit fails before rendering.


## Building the parts

Every script is a SCADwright Design. Run it with no arguments to get its default variant, or pass `--variant=NAME` for any other. Output is written next to the script as `<name>-<variant>.scad`. For the housing, `<name>` is the `name` field in the JSON; for the caps and rings, it's the Design class name, which matches the filename. This path takes no `-o`.

The housing reads its optical configuration from JSON:

```
python lens_housing.py --from-json v13f.json                    # default: print_vertical
python lens_housing.py --from-json v13f.json --variant=display
python lens_housing.py --from-json v13f.json --variant=print_hood
python lens_housing.py --from-json v13f.json --variant=print_stops
python lens_housing.py --from-json v13f.json --variant=print_shape_stops
```

That JSON holds focal length, max aperture, FOV, per-element diameters, edge thicknesses and spacings, and the stop (depth plus hole diameters). Every dimension in the housing is derived from it. Without `--from-json` the script falls back to placeholder defaults, so always pass the config you want.

Because the barrel is stamped with the current commit, a build refuses on a dirty tree; pass `--allow_dirty` for throwaway test prints and the stamp gets a `-dirty` suffix.

Each config has a matching optical-plan SVG, a ray diagram of that lens. Pass it to engrave it on the barrel:

```
python lens_housing.py --from-json v13f.json --optical_plan v13f-optical-plan.svg
```

Author that SVG as filled paths. Open strokes don't import.

The caps and rings take no config:

```
python rear_lens_cap.py                             # default (rear_cap_sym): symmetric three-orientation rear cap
python rear_lens_cap.py --variant=rear_cap          # asymmetric rear cap matching the spec lugs
python rear_lens_cap.py --variant=mount_testfit     # lugs-only camera-mount test stub

python body_cap.py                                  # default (cap): body cap alone, packed on the bed
python body_cap.py --variant=cap_print              # cap + modeled PETG supports as separate solids in one file

python lens_mount_ring.py                           # default (lens_print): ring + modeled PETG supports, print layout
python lens_mount_ring.py --variant=lens            # ring only, no supports
python lens_mount_ring.py --variant=lens_supports   # PETG supports only (pre-aligned to the ring)

python lens_mount_locking_ring.py                   # default (print): locking ring with pivot lever
python lens_mount_locking_ring.py --variant=print_preview             # same, lower fn for fast preview
python lens_mount_locking_ring.py --variant=pivot_assembled_locked    # assembled, lever tooth engaged
python lens_mount_locking_ring.py --variant=pivot_assembled_unlocked  # assembled, lever swung clear
```

`cap_print` and `lens_print` emit the part and its supports as disconnected manifold solids in a single file. In Bambu Studio, right-click → Split → To Objects, then assign PLA to the part and PETG to the supports with supports off on both.

All of them share the bayonet contract in `spec.py`. Adjust a measurement there and every mating part follows.

For an STL, or to choose the output path, use the SCADwright [CLI](https://github.com/brianssparetime/SCADwright/blob/master/docs/cli_and_args.md), which does accept `-o`:

```
scadwright build  lens_housing.py --from-json v13f.json --variant=print_hood -o hood.scad
scadwright render lens_housing.py --from-json v13f.json --variant=print_hood -o hood.stl
scadwright preview lens_housing.py --from-json v13f.json --variant=display
```


## Variants

| Variant                      | Purpose                                                      |
| ------------------------------ | -------------------------------------------------------------- |
| `print_vertical` *(default)* | Housing halved, splayed flat, packed onto the print bed.     |
| `print_vertical_preview`     | Same layout, lower `fn` for fast preview. Skips the bed-fit check. |
| `print_hood`                 | The clip-on hood, print orientation.                         |
| `print_lens_cap`             | The clip-on lens cap, flat disc face-down on the bed.        |
| `print_aperture_cover`       | The clip-on aperture-slot cover, axis-vertical on the bed.   |
| `print_stops`                | The round iris stop set, packed onto the bed.                |
| `print_shape_stops`          | The non-circular stops (square, hexagon, almond), equal-area to a round stop. |
| `design_spread_out`          | Every sub-part laid out flat, spaced for design review.      |
| `display`                    | Assembled housing + hood, presentation orientation.          |

### Test-fit variants

I wasted a lot of filament printing full articles before I stopped and built small testers. Printing the whole mount ring makes it hard to see how far off the lug grooves are in depth or distance from the flange. A part that tests one fit answers that in twenty minutes instead of a whole plate.

These live in `test_fit.py`, not `lens_housing.py`, and read the same JSON config, so a fit verified here predicts a fit in the full housing. I took the approach further on the [Pentacon 6 mount](https://github.com/brianssparetime/SCADwright/tree/master/examples/pentacon-six-mount) in SCADwright's examples, which has a cleaner set of testers than this repo does:

```
python test_fit.py --from-json v13f.json --variant=lower_housing_testfit
python test_fit.py --from-json v13f.json --variant=lower_housing_testfit --no_locking_mech
```

`--no_locking_mech` drops the tab/notch and snap bead from the mating disc and prints a plain disc instead, which isolates whether the lock mechanism is causing a fit problem. The default keeps it, so the test piece exercises the same disc geometry the production housing does.

| Variant                 | Purpose                                                      |
| ------------------------- | -------------------------------------------------------------- |
| `lower_housing_testfit` | Truncated housing, stripped to the structural lower wall and mating disc. Slips into the camera throat to check stepped-OD and helicoid-pin clearance. |
| `element_grippers`      | Per-element half-grippers, the same three-tube stack as `ElementHolder` with inner diameters and axial extents matched to the housing exactly, for test-fitting one element's thickness and diameter without printing a full housing. |
| `half_housing`          | One splayed half (the -X half, lap-joint features visible) for inspecting a single shell in isolation. |
| `mount_ring_slice`      | A 1/12 (30°) wedge of the lens-mount ring, cut through a rotation channel so the exposed cross-section shows the whole axial stack for eyeballing anything that doesn't line up. |


## Files

Three groups: the shared bayonet contract, the small camera-side artifacts (caps and the bare mount ring), and the lens housing and everything it pulls in.

- **Bayonet contract & printer config** — read by every part downstream.

  - `spec.py` — the bayonet contract. Single source of truth for the camera-side bayonet (measured from a real Bronica) and the lens-mount ring's design choices.
  - `printer.py` — printer-side constants (the Bambu X2D print-bed footprint, used by `arrange_on_bed`).
- **Caps & the bare lens-mount ring** — small standalone Designs that mate with a real camera or lens.

  - `rear_lens_cap.py` — `CameraMount` (the lugs-only test-fit stub used to verify the lens-mount ring against printed dimensions), `RearLensCap` (cap with a bayonet stub on the lens side and a closing cup on the camera side), and `BronicaS2RearLensCapSymmetric` (a three-orientation symmetric variant).
  - `body_cap.py` — a `LensMount` plus a solid plug disc filling the bore. Worn on the camera body.
  - `lens_mount_ring.py` — generic `LensMount` Component, the lens-side bayonet ring that mates with `CameraMount`. Standalone as its own Design (`lens`, `lens_print`, `lens_supports` variants) and the base class extended by `body_cap.py` and `lens_mount_locking_ring.py`.
- **Lens housing** — the main artifact and everything it composes from.

  - `lens_housing.py` — the concrete lens-housing Design. Reads `v13f.json` (or any other optical config) and resolves every dimension from it. Mounts on the camera through `lens_mount_locking_ring`.
  - `lens_internals.py` — reusable optical Components, S2- and Bronica-agnostic: `HousingSection`, `FilletSection`, `ElementHolder`, `ElementTestGripper`, `WaterhouseStop`, `SquareStop`, `HexStop`, `AlmondStop`, `StopHolder`, `TruncFilletRing`, plus the `ElementClearances` and `SweptSlot` namedtuples.
  - `lens_mount_locking_ring.py` — `LensMount` extended with two integrated pillow-block bosses on its OD wall and a separate pivot lever pinned through them. The lever's chisel tooth drops into the bayonet rotation channel and locks the lens onto the camera. The lens-housing's mating disc snap-fits inside the ring's bore.
  - `mating_disc_lock.py` — shared spec for the interface between the lens housing and the locking ring's tab/notch. Kept in its own module so a part can import the spec without pulling in either Design class (avoiding `print`-variant collisions).
  - `lap_joint.py` — generic stepped lap-joint transforms (`lap_split_x`, `lap_splay`) used to halve the housing for printing. No project-specific dimensions.
  - `test_fit.py` — diagnostic Design holding the trial-print variants (`lower_housing_testfit`, `element_grippers`, `half_housing`, `mount_ring_slice`). Reads the same optical config as `lens_housing.py`, so a fit confirmed here holds in the full housing.


## Releases

`releases/` contains the files for versions that I've actually printed. Each folder has the SCAD and STL for whatever parts that version produced, plus the JSON config and optical-plan SVG they were built from, so an old print can be rebuilt without digging the settings out of history.

<p align="center">
  <img src="img-vid/IMG_2838%20Large%20%20spread%20out%20in%20parts.jpeg" alt="Three housing designs opened up, labeled 101mm f/2.9, 90mm f/3.1, and 98mm f/3.2" width="640">
</p>

*The three designs below, labeled on the front rim: 101mm f/2.9, 90mm f/3.1, and 98mm f/3.2. Each open half shows its element stack, with the achromats' cemented surfaces catching yellow. The barrel at top left shows the optical-plan engraving; the stops at top right sit ready to drop through the slot.*

- `releases/13f` and `releases/14a` — 101mm f/2.9 retrofocus
  - A negative (-174mm) singlet front with an interior (+130mm) achromat and a (+150mm) PCX rear, spaced so third-order spherical cancels for a sharp center under deep field curvature. 
  - The film testing showed that the negative singlet's coma beats the astigmatic sign, so the corners smear radially as chevron tie-dye rather than the tangential swirl the metric predicted.

- `releases/15a` — 90mm f/3.1 retrofocus
  - built entirely from achromats (-135, +125, +127), with a negative achromat in front to carry swirl direction without the coma a singlet brings. 
  - Film showed the swirl-type fix worked, but only partly. The field is three regimes: sharp center, tangential swirl ring, and non-chevron tiedye/radial corners going into the edges and corners.

- `releases/16c` — 98mm f/3.2 retrofocus
  - all-achromat design (-135, +130, +150), pulling both modeled levers at once: a longer focal length, and the stop carried far back for deep astigmatism, to push the swirl-type crossing out to an inscribed circle. 
  - The crossing has no closed form in thin-lens math, so what this teaches waits on a roll of film beside v16b as the astigmatism-only control.

- `releases/mount-ring` — the bayonet rings, in their own folder because they aren't tied to a lens. Their geometry comes from `spec.py`, not the optical config, so one ring fits every housing above.
  - `lens_mount_locking_ring-print` — the locking ring the housings mount with. Ring, pivot lever, and their PETG supports as separate solids.
  - `lens_mount_ring-lens` — the bare ring alone, no lever and no supports. Import this one to fuse a Bronica mount onto your own part.
  - `lens_mount_ring-lens_print` — the same bare ring with supports, laid out to print on its own.

The bare ring imports sitting on the bed rather than on the origin, so center it on the optical axis before fusing: it spans 68.1 mm square from a 1.5 mm corner, which is `translate([-35.55, -35.55, 0])`.


## Built with SCADwright

OpenSCAD got progressively annoying on the first attempt, less on features than on project management. Modules can't share attributes or read from each other, so mating parts either duplicate dimensions or rely on fragile workarounds to avoid it. [SCADwright](https://github.com/brianssparetime/SCADwright) is the OpenSCAD emitter I built to fix that, and this housing is its most demanding example: eleven source files using nearly every framework feature.

The ones that did the most work here:

- [`Component`](https://github.com/brianssparetime/SCADwright/blob/master/docs/components.md) with [`equations`](https://github.com/brianssparetime/SCADwright/blob/master/docs/components.md#parameters-equations) — each part declares its relationships and constraints once, and the framework solves for whatever you didn't supply
- [`Spec`](https://github.com/brianssparetime/SCADwright/blob/master/docs/specs_and_adjustments.md) with adjustments — the bayonet contract in `spec.py`, read by every mating part
- [`@variant`](https://github.com/brianssparetime/SCADwright/blob/master/docs/variants.md) — print, display, and diagnostic arrangements of one design
- [custom transforms](https://github.com/brianssparetime/SCADwright/blob/master/docs/custom_transforms.md) — the lap joint that halves the housing is reusable, not a one-off cut
- [`wrap_2d`](https://github.com/brianssparetime/SCADwright/blob/master/docs/wrap_2d.md), [`add_text()`](https://github.com/brianssparetime/SCADwright/blob/master/docs/add_text.md) and [`scad_import`](https://github.com/brianssparetime/SCADwright/blob/master/docs/scad_interop.md) — labels and the optical-plan SVG on curved walls
- [`from_json`](https://github.com/brianssparetime/SCADwright/blob/master/docs/cli_and_args.md#passing-complex-data-via-json-from_json) — the whole design driven from an optical config file
- [`attach(fuse=True)`](https://github.com/brianssparetime/SCADwright/blob/master/docs/attach.md) and [`through()`](https://github.com/brianssparetime/SCADwright/blob/master/docs/auto-eps_fuse_and_through.md) — positioning parts against each other without hand-computed offsets
- [clearances](https://github.com/brianssparetime/SCADwright/blob/master/docs/clearances.md) — snap-fit and material tolerances tunable apart from design dimensions
- [`arrange_on_bed`](https://github.com/brianssparetime/SCADwright/blob/master/docs/composition_helpers.md#arrange_on_bed) — print-bed fit checked before rendering
