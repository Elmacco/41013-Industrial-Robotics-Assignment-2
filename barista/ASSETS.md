# Barista cell - support package assets

Sizes are measured from the packaged meshes (W x D x H, metres). All parts are Z-up with
the origin at the bottom centre unless noted. Load with
`ir_support_extra_parts.part_mesh(name, pose=SE3(...))`.

## Robots (`ir_support`, verified core set)

| Model | Reach (horiz.) | Role here |
| --- | --- | --- |
| `UR3` | 0.59 m | Espresso: grind, tamp, portafilter, shot (same model as the lab robot) |
| `UR3e` | 0.60 m | Milk: steam, pour, place at pickup |
| `LinearUR3` | 0.59 m + 0.8 m rail | Runner: moves cups between stations |

Alternatives in core `ir_support`: `UR5` (0.95 m), `UR10`, `UR10e`, `DobotMagician` (0.36 m),
`DensoVS060`, `HansCute`, `Sawyer`. `ir_support_extra_robots` has ~80 more arms
(xArm6, Lite6, Kinova Gen3, ABB GoFa, ...), but they are marked *candidate, not verified*.

## Used in the scene

| Part | Size | Use |
| --- | --- | --- |
| `Workbench` | 2.50 x 0.75 x 0.93 | Counter (two back to back) |
| `PintCup` | 0.076 x 0.076 x 0.149 | Coffee cup |
| `MilkCarton` | 0.045 x 0.051 x 0.076 | Milk |
| `MilkPitcher` | 0.090 x 0.125 x 0.110 | Steaming jug |
| `BlueSyrupBottle` | 0.174 x 0.061 x 0.235 | Syrup |
| `Tray` | 0.380 x 0.250 x 0.040 | Pickup point |
| `LightTower` | 0.12 x 0.12 x 0.57 | Status light |
| `Camera` | 0.21 x 0.15 x 0.17 | Vision / monitoring |
| `emergencyStopButton` | 0.30 x 0.30 x 0.29 | E-stop |
| `SafetyLightCurtain` | 0.12 x 0.09 x 0.82 | Pickup guard |
| `SafetyRailing` | 2.57 x 0.40 x 1.05 | Cell boundary |
| `FireExtinguisher` | 0.17 x 0.38 x 0.41 | Safety |
| `WarningSign` | 0.35 x 0.25 x 0.61 | Safety signage |
| `personFemaleBusiness` | 0.58 x 0.67 x 1.85 | Customer |

## Other relevant parts

- **Drinkware:** `BeerGlass` (0.087 x 0.15), `ShotGlass`, `WineCup`, `GlassBottle`,
  `TakeawayCup` (oversized: 0.20 x 0.26, scale it down if used)
- **Ingredients:** `SauceBottle`, `CandyJar` (sugar / beans)
- **Furniture:** `BarTable` (2.2 x 1.0 x 1.17), `StandingBarTable`, `RobotTable` (0.45 x 0.45 x 0.62
  pedestal), `SimpleTable`, `WallShelf`, `tableRound0.3x0.3x0.3m` (origin off-centre)
- **Handling:** `FoodTrayBlue`, `Plate`, `PlateRack`, `DishRack`, `PlateStacker`, `SuctionCup`
- **Waste:** `GreenBin` / `RedBin` / `BlueBin` (0.17 x 0.17 x 0.5), good for a knock box or cup bin
- **Controls / sensing:** `ControlPanel`, `Monitor`, `PushButton`, `BarcodeScanner`, `ProximitySensor`,
  `SafetyLight`, `emergencyStopWallMounted` (origin off-centre)
- **Safety:** `SafetyGate`, `barrier1.5x0.2x1m`, `fenceFinal`, `CautionSign`, `WetFloorSign`,
  `FireBlanket`, `SafetyPerson`, `personMaleCasual` (origin off-centre)

## Not in the package (placeholders in `layout.PLACEHOLDERS`)

Espresso machine, grinder, tamp station, portafilter, steam wand. These are currently
grey `Cuboid` / `Cylinder` primitives and need custom meshes (DAE/STL).
