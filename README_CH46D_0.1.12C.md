# CH-46D 0.1.12C source candidate

This branch brings the existing local 0.1.12C EFM source into the repository,
along with its matching DLL, build entry point, regression checks and eight
selectable saved liveries. Base: ch46d-slow at da1d27b8655d302813509216d728c98b8d09838e.
This is a source handoff for review; DCS appearance and flight validation are pending.

## Source and build

- `EFM Source Code/EFM`: editable C++ source. The local version uses separate
  front/aft rotor states, tandem control mixing and twin-engine state handling.
- `EFM Source Code/CH46D.sln`: Visual Studio solution; project targets C++17 and v143.
- `build_windows.py`: alternate Windows x64 build using Python and ziglang 0.13.0.
  Install that pinned compiler package, then run `python build_windows.py`.
- `tests`: existing focused C++ checks. `run_rate_tests.py` runs the existing
  restricted-model response tests; other test translation units can be built separately.
- `bin/CH46D.dll`: unchanged previously packaged 0.1.12C binary. SHA256:
  `75bc9c8f4998690d7bae03ef0a70a3b7b8073bdc6e494734c6b8fc2095065bd5`.

The source is copied from the existing local package, not newly tuned during this
handoff. Engine curve shape, temperature/fuel-flow calibration and compatibility
gauge scales remain provisional. The proposed inherited-resource cleanup is not applied.
See `docs/CH46D_0.1.12C_CHANGES.md` for the droop argument change from 36 to 1036.
The model must support argument 1036; this needs DCS validation.

## Liveries and installation

`Liveries` contains eight selectable schemes for both flyable aircraft types,
with color, DirectX normal and DCS RoughMet DDS maps and their `description.lua`
files. Copy that folder into the actual Saved Games DCS profile and select a
scheme through Mission Editor's Paint Scheme/Livery field. Keep shared-map folders.
This preserves the saved texture exports shown in the previews. Not all schemes
are Vietnam-era. See `docs/LIVERY_INSTALL.txt` for details.

The compatible complete VWV CH-46D base mod and flyable extension are required.
This source candidate does not reconstruct the absent base aircraft or its current
editable DCS animation bindings. Existing cockpit Lua/HUD and assets were retained
from the target branch; synchronize the tested installed baseline before flight testing.
The prior HUD/crew/door Lua patches are separate and have not been silently merged.

The Painter project is not included in Git (approximately 1.94 GB); the DDS files
are saved texture exports. No Blender geometry, UV or Painter changes are included.

## Validation

Source provenance, matching DLL hash and all local livery references are checked.
Focused compiler checks are recorded separately. These do not prove DCS handling,
cockpit indication, multiplayer behavior, rotor animation or material appearance.
The existing published test package remains available under GitHub Releases.
