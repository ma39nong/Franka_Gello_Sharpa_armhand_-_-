# V10.1 L20 assets - migration notes (staged 2026-08-01)

Official Linker support package `L20_V10.1--0716` for the physical left
hand serial LHT20-010-502-L-B-1-D. Copied verbatim from
`/home/descfly/hsc/GeoRT/assets/linkerhand_L20_V10.1--0716/` (left URDF
sha256 2a668d0c...f6e0f - identical to what GeoRT checkpoints are bound
to). The old `assets/linkerhand_l20/` is measurably WRONG for this hand
and is kept only until the retargeting migration lands.

Wired into the optimization-based left G20 retargeter on 2026-08-01. The
unverified right V10.1 delivery remains unused; the deployed right hand is an
O30i. The old `assets/linkerhand_l20/` is retained for the right-G20 fallback
and historical comparison.

What changes vs the old model (all hardware-verified on the GeoRT side):

- `thumb_ip` renamed to `thumb_dip`; the thumb is fully re-parameterized.
  FK and optimization use `thumb_dip`, while packets retain the historical
  `thumb_ip` name through an explicit alias.
- mcp_pitch limit 1.40 -> 1.2217; pip 1.57 -> 1.7279.
- dip mimic ratio 0.8917 -> 0.7879; thumb mimic 1.0142.
- MCP mount offsets moved ~19 mm.

Driver/packet contract: the UDP hand bridge still expects the historical
21-name L20 packet order INCLUDING `thumb_ip` - keep packet names stable
and alias the URDF's `thumb_dip` when building packets (see
GeoRT/scripts/live_left_g20.py `packet_alias` for a working example, and
its `parse_mimics` for mimic expansion with pinocchio).

The recorded MANUS landmarks were replayed after the switch. Human-space
contact thresholds and curl gates remain applicable; the left index-middle
anchor was re-solved against V10.1. The labelled six-pose replay gives 0.6 mm
thumb-index, 3.9 mm thumb-middle, and 0.0 mm median index-middle FK gaps.
These are model-space results and still require physical acceptance. Right-hand
V10.1 is included but its serial has NOT been verified - check before trusting.
