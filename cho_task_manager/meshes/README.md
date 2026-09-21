# Bench meshes

Display-only bodies for the vessels named in `config/perception/*.yaml`. Nothing
in the control path reads them; `cho_object_pose` draws them at the detected
pose so an operator can see the glassware beside the robot in rviz.

## Where they came from

Converted from the lab assets in `sdl_project`
(`TAMP/tamp/content/assets/lab/{beaker,flask}.usd`) by `usd_to_stl.py`, which is
kept here so the conversion can be repeated rather than remembered:

```bash
python3 -m venv /tmp/usdvenv && /tmp/usdvenv/bin/pip install usd-core numpy
/tmp/usdvenv/bin/python usd_to_stl.py <asset>.usd <name>.stl
```

Two things about those assets, both measured rather than assumed:

- the `collision` prims are a convex decomposition — sixty-odd unit cubes — so
  only `visual` is taken;
- `metersPerUnit` is 0.01, but the xform scale already converts to metres.
  Applying both gives a 0.65 mm beaker; applying only the xform gives 65 mm,
  and the flask agrees (139 mm across, 132 mm tall). Both assets are sane only
  one way, so `metersPerUnit` is deliberately not applied.

## Normalised on purpose

Each file is centred on its bounding box and scaled to a unit box on every axis.
That makes `shape.size` in the object table mean the same thing for a mesh as it
does for a cylinder — the body's bounding box in metres — instead of a scale
factor derived from whatever units the mesh's author happened to use. The size
the assets were authored at is printed by the converter and recorded above; the
real glassware is what the config should say.
