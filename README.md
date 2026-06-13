# POST2 Lite

Minimal C++ trajectory prototype with a core library, shell CLI, Windows GUI, and a lightweight remote core server.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Shell

Run a local propagation and export both CSV and SVG:

```powershell
.\build\Release\post2_shell.exe simulate --mode local --csv trajectory.csv --svg trajectory.svg
```

Save a vehicle config:

```powershell
.\build\Release\post2_shell.exe simulate --save-vehicle-config vehicle.cfg --no-csv --no-svg
```

Load and override vehicle values:

```powershell
.\build\Release\post2_shell.exe simulate --vehicle-config vehicle.cfg --engine-enabled true --engine-max-thrust 4500 --engine-isp 310
```

Import a kOS vehicle/site export:

```powershell
.\build\Release\post2_shell.exe simulate --import-ksp-vehicle-site vehicle_launchsite.json --save-case imported.case.json --no-csv --no-svg
```

Run the core in remote server mode:

```powershell
.\build\Release\post2_core_server.exe --port 5050
```

Then request the trajectory from the shell:

```powershell
.\build\Release\post2_shell.exe simulate --mode remote --host 127.0.0.1 --port 5050
```

## GUI

On Windows, run:

```powershell
.\build\Release\post2_gui.exe
```

Use the `Mode` menu to switch between local core and remote core. Use `Mode > Remote endpoint...` to edit the remote host and port before connecting to `post2_core_server`. Use the `Vehicle` menu to edit the current vehicle or import a kOS vehicle/site JSON export.

## Current model

- Earth is a sphere with WGS84 equatorial radius and standard gravitational parameter.
- Vehicle lives in `include/post2/vehicle` and currently has config plus runtime state layers.
- Vehicle config has dry mass, engine config, and tank config.
- Vehicle runtime state has vehicle motion/mass state, engine state, and tank state.
- Default dynamics use J2 gravity, with point-mass gravity available as an explicit model option.
- If engine is enabled, the prototype applies a simple engine acceleration and consumes tank propellant.
- The ODE integrator is fixed-step RK4 in `include/post2/integrators`.
- Propagation passes a `StateLog` through the driver, event, propagator, and integrator path.
- Default initial state is a 200 km circular LEO at 28.5 degrees inclination.

## Vehicle Config Storage

Implemented now:

- `.cfg` / `.txt` key-value text: human editable, no dependency, used by shell and GUI.

Good future storage options:

- JSON: good for APIs and nested configs, easy to validate with schemas.
- YAML: good for hand-authored mission files, but needs careful parser choice.
- CSV: useful for tables like tank schedules or time actions, not ideal as the main vehicle file.
- SQLite: good once configs become many related tables with versioning and result metadata.

## Propagation Flow

The local core currently runs one `LaunchVehicleEvent`:

1. Initialize or truncate `StateLog`.
2. Initialize termination conditions. The only implemented condition is maximum time.
3. Execute the event.
4. Select the event propagator and RK4 ODE integrator.
5. Propagate with the gravity force model.
6. Update runtime vehicle/engine/tank state through the consumption propagation layer.
7. Append `LaunchVehicleStateLogEntry` values to `StateLog`.
8. Return the completed log to shell, GUI, or remote clients.
