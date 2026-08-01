"""Stepped lap-joint transforms for halving a 3D-printed assembly.

Generic transforms — no project-specific dimensions. `lap_split_x`
halves a host at X=0 with a stepped tongue/groove on each side so the
two halves self-align radially when pressed together; `lap_splay` lays
one half cut-face-down on the print bed.

These are framework-style transforms (registered as `Node.lap_split_x`
and `Node.lap_splay` via the `@transform` decorator at import time),
so importing this module is enough to make the methods callable on
any Node.
"""

from scadwright import BBox, bbox
from scadwright.api.tolerances import default_eps
from scadwright.boolops import difference, intersection, union
from scadwright.primitives import cube, cylinder
from scadwright.shapes import regular_polygon
from scadwright.transforms import transform


def _lap_split_x_tight_bbox(child, *, side, overlap, clearance=0.0, **_):
    """tight_bbox hook for `lap_split_x`. The transform's body builds a
    top-level Difference (groove subtracted from union(half, tongue)),
    which the framework's bbox visitor can't tighten on its own. We know
    the answer: the result keeps one X half of the housing plus a
    `tongue_w = overlap - clearance` tongue extending past the cut plane
    on the other side. Z and Y are unchanged from the child."""
    from scadwright import tight_bbox as _tb
    cb = _tb(child)
    tongue_w = overlap - clearance
    if side == "minus":
        return BBox(min=cb.min, max=(tongue_w, cb.max[1], cb.max[2]))
    return BBox(min=(-tongue_w, cb.min[1], cb.min[2]), max=cb.max)


# inline=True: the body sizes its auxiliary cutters from the child's real
# bbox. A hoisted module body is rendered once against the CHILDREN
# placeholder, which has no measurable extent — inline expansion is the
# framework's mechanism for transforms that must inspect their child.
@transform("lap_split_x", inline=True, tight_bbox=_lap_split_x_tight_bbox)
def lap_split_x(node, *, side, overlap, mid_radius, z_min=None, clearance=0.0,
                key_sides=None):
    """Halve the host shape at X=0 with a stepped lap joint of depth `overlap`
    at radial position `mid_radius`. `side` is 'minus' (keep X<=0) or
    'plus' (keep X>=0). The 'minus' and 'plus' results mate INVERSELY
    across the cut plane (not as mirror images): one carries the outer
    extension and the inner recess; the other carries the matching outer
    recess and inner extension.

    Specifically:
      - minus keeps X<=0; its OUTER layer (r >= mid_radius) is extended
        past X=0 into +X by `overlap` (tongue), and its INNER layer
        (r < mid_radius) is recessed from X=0 in -X by `overlap` (groove).
      - plus keeps X>=0; its OUTER layer is recessed (groove) at +X for
        `overlap` mm to receive minus's outer tongue, and its INNER layer
        is extended into -X by `overlap` (tongue) to fill minus's inner
        groove.

    Joint planes therefore land at X=+overlap (outer layer) and at
    X=-overlap (inner layer). Light path between bore and outer surface
    zigzags through the step — light-tight. The step constrains relative
    XY position — self-aligning. Mates by pure X-translation (radial
    press-fit), no axial sliding required.

    `mid_radius` must lie inside the wall everywhere along Z (between
    the bore and the outer wall in every section), or the layers will
    have zero thickness somewhere.

    `z_min` (optional) confines the lap features to z >= z_min. Below
    z_min the housing is just cleanly halved at X=0 with no lap geometry
    — useful when the wall is too thin to host the step (e.g. the M57-fit
    section of a lens housing).

    `clearance` (mm) shrinks the tongue (NOT the groove) so the press
    seats with some play. The shrink is applied two ways: tongue X-extent
    is shorter by `clearance` at the TIP (gives axial seating play); the
    tongue's step-face boundary is pulled away from `mid_radius` by
    `clearance` (gives radial play at the step inside the joint). The
    tongue's outer-cylinder surface (or bore surface for the inner
    tongue) is unchanged so the visible cylinder surface flushes at the
    joint. Default 0 (zero-clearance, may bind on tight prints).

    `key_sides` (optional int) makes the radial step a regular polygon of
    that many sides instead of a smooth circle. The polygon's flats and
    vertices, against `clearance`, vary the gap around the seam — local
    interference at the vertices, clearance at the flats — so the two
    halves snap/key together rather than sliding freely. This is a
    deliberate snap feature, not a low-resolution circle. The side count
    MUST divide the print `$fn` (e.g. 8 divides 128) so the polygon's
    vertices land on the wall's and the seam stays manifold on export; a
    count that doesn't divide `$fn` reintroduces T-junction slivers at the
    step. Default None = smooth circular step (free-sliding fit).
    """
    if side not in ("minus", "plus"):
        raise ValueError(f"side must be 'minus' or 'plus', got {side!r}")
    sign = -1 if side == "minus" else 1
    # All auxiliary cutters (halve kept-box, slabs, step masks) share one
    # size derived from the child's bbox — the same formula `halve()` uses
    # internally: origin-centered geometry of half-extent `big/2` covers
    # the child iff big/2 >= max |corner coordinate|, with a 2% margin.
    # One value keeps every cutter's reach mutually consistent; a cutter
    # shorter than the child silently truncates it (the halve box clips
    # the body while the tongue slab, anchored at z_min, does not — the
    # split then sheds everything above big/2 except a tongue-width
    # sliver).
    bb = bbox(node)
    r = max(abs(c) for corner in (bb.min, bb.max) for c in corner)
    big = max(2.0 * r * 1.02, 1.0)
    half = node.halve([sign, 0, 0], size=big)

    # Slabs need only enclose the housing, not be globe-sized. Intersection
    # clips them to the wall geometry, so the step automatically conforms
    # to varying OD/ID along Z (e.g. the funnel transition in wide
    # variants). The exact value matters because OpenSCAD's preview mode
    # draws the full auxiliary geometry every frame and very large slabs
    # cost real interactivity; the bbox-derived `big` stays proportional
    # to the part. When z_min is set, the slab is anchored at z=z_min
    # (not Z-centered) so only z >= z_min material is included.
    z_centered = z_min is None
    z_axes = "yz" if z_centered else "y"

    def _slab(width):
        c = cube([width, big, big], center=z_axes)
        return c if z_centered else c.up(z_min)

    tongue_w = overlap - clearance
    # `bond` buries the tongue this deep into the kept half across X=0 so
    # `union(half, tongue)` fuses instead of meeting on an exactly coplanar
    # X=0 face — the coplanar case survives CGAL STL export as a
    # non-manifold edge. The bond sits inside the half (the half owns all
    # of its X side at every radius), so it changes nothing visible: the
    # tongue's tip and outer cylinder are untouched.
    bond = default_eps()
    # Groove: full overlap on the slab, mid_radius for the radial split.
    # Tongue: shorter slab (clearance at the tip) and the radial split is
    # pulled away from mid_radius by clearance (further out for an outer-
    # layer tongue, further in for an inner-layer tongue), giving the
    # tongue play at both the step face and the tip when seated.
    #
    # Both groove and tongue slabs extend `bond` past X=0 into the kept
    # half. The tongue overlap fuses union(half, tongue); the groove
    # overlap drives the recess cut through X=0 so it doesn't leave a face
    # coplanar with the half's own cut face. Only the mouths (the X=0 ends)
    # move — the groove depth (`overlap`) and tongue tip (`tongue_w`) that
    # set the mating fit are untouched.
    pos_groove_slab = _slab(overlap + bond).left(bond)     # X in [-bond, overlap]
    neg_groove_slab = _slab(overlap + bond).left(overlap)  # X in [-overlap, bond]
    pos_tongue_slab = _slab(tongue_w + bond).left(bond)    # X in [-bond, tongue_w]
    neg_tongue_slab = _slab(tongue_w + bond).left(tongue_w)  # X in [-tongue_w, bond]

    # The step masks split the cross-section at `mid_radius` into the outer
    # layer (node − mask) and the inner layer (node & mask). The mask
    # surface at mid_radius becomes the lap STEP face between the
    # protruding tongue and the recessed groove — part of the final
    # geometry, not a throwaway mask.
    #
    # With `key_sides` set the step is a regular polygon of that many
    # sides: its flats and vertices, against `clearance`, key the two
    # halves together — that is the snap. It is deliberate; do not round it
    # off. The side count must divide the print `$fn` so the polygon's
    # vertices coincide with the wall's and the seam stays manifold on
    # export (8 divides 128). A circular step (`key_sides=None`) gives a
    # uniform gap instead — a free-sliding fit. `regular_polygon` and a
    # plain `cylinder` both place their first vertex on +X at radius r, so
    # the two forms differ only in side count.
    def _step_mask(r):
        if key_sides is None:
            return cylinder(r=r, h=big, center="z")
        return regular_polygon(key_sides, r).linear_extrude(height=big).down(big / 2)

    inner_volume = _step_mask(mid_radius)
    inner_volume_outer_tongue = _step_mask(mid_radius + clearance)
    inner_volume_inner_tongue = _step_mask(mid_radius - clearance)

    if side == "minus":
        # Outer-layer tongue at +X, inner-layer groove at -X.
        tongue = difference(intersection(node, pos_tongue_slab), inner_volume_outer_tongue)
        groove = intersection(intersection(node, neg_groove_slab), inner_volume)
    else:  # plus
        # Inner-layer tongue at -X, outer-layer groove at +X.
        tongue = intersection(intersection(node, neg_tongue_slab), inner_volume_inner_tongue)
        groove = difference(intersection(node, pos_groove_slab), inner_volume)

    return difference(union(half, tongue), groove)


@transform("lap_splay")
def lap_splay(node, *, side, lap_overlap=0.0):
    """Lay one housing half flat on the print bed: rotate so the cut
    face lies parallel to the bed (cut-face-down) and lift the result
    by `lap_overlap` so the lap-joint tongue (which extends past the
    cut plane and would otherwise dip below Z=0 after the rotation)
    rests on the bed.

    `side` selects the rotation direction so the kept material lands
    above the bed regardless of which half was cut. `lap_overlap` is
    the same value passed to `.lap_split_x()`; it's explicit here because
    the framework's `bbox()` returns a conservative AABB across `halve`,
    so the actual minimum Z can't be derived from the splayed shape.
    Pass 0.0 if the half has no lap joint.
    """
    angle = 90 if side == "minus" else -90
    return node.rotate([0, angle, 0]).up(lap_overlap)
