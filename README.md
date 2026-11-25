# Universal Curve Tracer (Spirograph-based)

College exhibition project: a machine + web simulator that draws 14 curve families (spiral, helix, hyperbola, parabola, ellipse, circle, line, cycloid, cardioid, toroid, flower, background pattern, regular polygon 3–12 sides, spiralling helix); spiral, helix, toroid and spiralling helix come in six cross-sections (circular, elliptical, squarish, rectangularish, triangular, hexagonal) with adjustable, scale-marked parameters.

## Deploy as a GitHub site
1. Create a repo, upload `index.html`, `.nojekyll`, `README.md`.
2. Settings → Pages → Source: *Deploy from a branch* → `main` / root.
3. Site goes live at `https://<username>.github.io/<repo>/`.

## Key maths
- Planetary core (hypotrochoid): `x=(R−r)cos t + d cos((R−r)t/r)`. With `r=R/2` it yields an ellipse (a=r+d, b=|r−d|); `d=0` circle; `d=r` straight line.
- Epicycloid with `R=r` → cardioid. Rolling on a rack → cycloid.
- Spiral `r=a+bθ`, rose `r=a cos(pθ/q)`, parabola `y=x²/4f`, hyperbola `x=a cosh s, y=b sinh s`.
- Helix and toroid are 3-D; the machine draws their projections.

Add photos, a video, circuit diagram, CAD files and firmware in `/docs` as you build.

## Repository layout
- `logo.svg`, `icon.svg` – project logo and favicon (square cross-section helix)
- `index.html` – live demo, physical model, BOM, circuit, video
- `gears.html` – gear / tooth calculator with preview
- `firmware/curve_tracer.ino` – Arduino polar-plotter firmware (all 14 curves, cross-sections via `sec asp sq`)
- `cad/gears.scad` – parametric OpenSCAD gears (planet with pen holes, ring gear)
- `model3d.html` – interactive 3-D models (machine + gear set) with exact dimensions

## Cross-sections
`ρ(φ)` is the outline radius: circle 1; ellipse `1/√(cos²φ+sin²φ/b²)`; squarish/rectangularish superellipse `(|cosφ|ⁿ+|sinφ/b|ⁿ)^(−1/n)`; triangle/hexagon `cos(π/n)/cos((φ mod 2π/n) − π/n)`. Spiral `r=(a+bθ)ρ(θ)`, helix `R·ρ(t)`, toroid tube profile `r·ρ(qt)`.
