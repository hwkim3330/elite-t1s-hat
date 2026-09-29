# LilyGo mechanical files, fetched on demand, and the script that reads them

Everything dimensional in `../GEOMETRY.md` traces back to three files LilyGo
publish for the T-ETH-Elite. **They are not stored in this repository:** LilyGo's
repository states no licence, so they are downloaded from upstream when needed
and ignored by git.

```bash
pip install --user ezdxf trimesh manifold3d py7zr
python3 measure.py --fetch     # downloads the three files below into this folder
python3 measure.py             # re-derives every number in GEOMETRY.md
```

| file | what it is | upstream |
|---|---|---|
| `T-ETH-ELite.dxf` | 2D mechanical drawing of the T-ETH-Elite | `shell/T-ETH-ELite.dxf` |
| `T-ETH-ELite.7z` | 3D model (STL inside) of the T-ETH-Elite | `shell/3D/T-ETH-ELite.7z` |
| `T-ETH-ELite-LoRa-Shield.7z` | 3D model of LilyGo's own stacking shield | `shell/3D/T-ETH-ELite-LoRa-Shield.7z` |

Upstream: [Xinyuan-LilyGO/LilyGO-T-ETH-Series](https://github.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series).
### The `-view.stl` files are for looking at, not for measuring

GitHub's 3D viewer stops at 10 MB and both originals are ~10.5 MB, so the
`-view.stl` pair is decimated (209k → 75k and 211k → 64k triangles, 3.8 MB and
3.2 MB). Decimation eats thin features: the base board's view model comes out
72.500 × 49.192 × **17.703** against the original's 72.500 × 49.352 × **18.822**,
i.e. about 1.1 mm of height is simply gone. Fine for seeing the shape, useless
for dimensions. `measure.py` reads the `.7z` originals and never touches these.

Upstream is [Xinyuan-LilyGO/LilyGO-T-ETH-Series](https://github.com/Xinyuan-LilyGO/LilyGO-T-ETH-Series),
fetched 2026-09-22. The `.7z` archives are kept compressed (≈1.3 MB each);
extracted they are ~10 MB STLs, which is not worth carrying in git.

## Reproducing GEOMETRY.md

```bash
pip install --user ezdxf trimesh manifold3d py7zr
python3 measure.py            # uses the files here
python3 measure.py --fetch    # or download them first
```

It prints, straight out of the vendor files:

- the four mounting holes, from the DXF *and* independently from the 3D model,
  and again from LilyGo's own shield — three sources for the same four holes
- the 40-pin grid, recovered by sectioning the header body above its plastic so
  the pins come out as 40 separate closed regions
- the board outline, simplified enough to show the antenna notch on the base
  board and the RJ45 notch on the shield
- every part standing more than 3.5 mm off the PCB — which is where the
  RJ45-vs-header-height problem that justifies the adapter comes from

It does not write anything or edit GEOMETRY.md. Compare by eye; if a number has
drifted, GEOMETRY.md is what needs updating, and every consumer of it
(here `../../hardware/make_t1s_hat.py`; in the adapter repo also its `pcb/` and `case/`) re-runs from
constants, not from hand-copied values.

## The one thing it cannot check

The TSN Lab 10Base-T1S HAT has no published CAD. Its 58 × 49 mounting pattern
is inferred from a product photo (see GEOMETRY.md for the method and the error
bars). Nothing in the adapter's fit to the *Elite* depends on that; only the
placement of the adapter's two far holes does.
