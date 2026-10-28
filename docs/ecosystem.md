# Sensors ecosystem map (for agents)

**Audience:** Cursor / coding agents organizing or changing code in a sensors multi-repo workspace.  
**Bootstrap (clone URLs, env deps, symlink layout):** [`../AGENTS.md`](../AGENTS.md) — start there on a **new machine**.  
**Canonical deep map:** this file (`sensors/docs/ecosystem.md`).  
**Core library:** `hik-sensors` (`sensors` on disk; import package name: `sensors`).  
**GitHub org/user:** [`fszhangYi`](https://github.com/fszhangYi) — see AGENTS.md for per-repo clone URLs.

When unsure where a change belongs, **default to extending `sensors`**, not forking drivers into an app.

---

## 1. One-sentence layout

`hww/hik_gello` (legacy MegaCollect) was refactored into **`sensors`**. Apps **`sensors-view` / `sensors-dcs` / `sensors-arm` / `rs-calibration`** consume that library via a local `./sensors` (or `vendor/sensors`) link. **`embody_model_eval`** embeds **sensors-view** over iframe only. **`office_skills`** documents porting/packaging contracts; it is not runtime code. **`mechanical/*`** holds CAD/FOV hardware assets, not the sensor SDK.

```text
                    ┌─────────────────────┐
                    │  hww/hik_gello      │  legacy reference only
                    │  (MegaCollect)      │
                    └──────────┬──────────┘
                               │ refactor
                               ▼
                    ┌─────────────────────┐
                    │  sensors            │  ★ single source of truth
                    │  (hik-sensors)      │  drivers · kinds · configs · CLI
                    └──────────┬──────────┘
           ┌───────────┬───────┼────────┬──────────────┐
           ▼           ▼       ▼        ▼              │
   sensors-view  sensors-dcs  sensors-arm  rs-calibration
   (probe UI)    (collect)    (arm only)   (eye-to-hand)
           ▲
           │ iframe embed + postMessage
           │
   embody_model_eval ─── Sensors page (guest = sensors-view)
           │
   office_skills ─── skills that encode the contracts above
```

---

## 2. Hard rules (do not violate)

1. **Drivers live only in `sensors`.** New device kinds: implement `@register_sensor(kind)` + YAML entry in `sensors`. Do not copy Dynamixel / RealSense / Elite / Modbus logic into view/dcs/arm/rs apps.
2. **Apps link the library; they do not vendor a fork.** Preferred layout:
   - `ln -sfn ../sensors ./sensors` (or absolute path)
   - runtime: insert `./sensors/src` at front of `sys.path` (see each app’s `paths` / `ensure_sensors_import`)
   - optional: `pip install -e ../sensors`
3. **`.gitignore` in apps already ignores `./sensors`.** The symlink is local; never commit the linked tree into an app repo.
4. **Collection vs probe vs control:**
   - Probe / status / rate → **sensors-view**
   - Episode record / export / teleop / multi-station → **sensors-dcs**
   - Multi-arm viz + gated write only → **sensors-arm**
   - Eye-to-hand extrinsic → **rs-calibration**
5. **Do not treat `hik_gello` as the live stack** for new features. Read it for archaeology; implement in `sensors` + the right app.
6. **Port collisions are real** (see §6). Do not assume “6006/6008 free” without checking what else is running.
7. **Desktop packaging (Scheme A)** is shared by sensors-view / sensors-dcs / rs-calibration (and some mechanical apps). Follow `office_skills/scheme-a-*` instead of inventing a new PyInstaller layout.
8. **Iframe embed contract** is fixed: host = embody (or any SPA); guest = sensors-view. See `sensors-view/docs/embed-host.md` and skill `sensors-view-iframe-embed`. Do not re-login / re-theme inside the guest when iframed.

---

## 3. Project catalog

### 3.1 `sensors` — core library ★

| | |
|---|---|
| **Path** | `/root/autodl-tmp/sensors` |
| **Pip name** | `hik-sensors` |
| **Role** | Config-driven sensor SDK: protocol, registry, drivers, `SensorManager`, CLI |
| **Depends on** | — (is the core) |
| **Entry** | CLI `sensors` (`list` / `probe` / `read` / `kinds`); Python `from sensors...` |
| **Kinds (typical)** | `bus`, `arm` / arm write, `gello`, `gripper` (DH AG95), `gripper_huiling` (慧灵 Z-EFG-C65), `realsense`, `pipeline`, `ft`, `tactile` |
| **Docs** | `README.md`, `docs/elite-cs-vs-ec.md`, `src/sensors/kinematics/`, `src/sensors/cs66_kinematics/` |
| **Must not** | Be reimplemented inside apps; receive app-only UI code |

Layout sketch:

```text
sensors/
  configs/           # machine YAML (default.yaml, …)
  src/sensors/
    core/            # Sensor protocol, HealthReport, Registry, Config
    drivers/         # bus, arm, gello, gripper, camera, pipeline, extensions
    runtime/         # SensorManager
    cli/
  tests/
```

**When to edit here:** new/changed hardware protocol, kind registration, shared kinematics, Elite EC/CS adapters, config schema used by multiple apps.

---

### 3.2 `sensors-view` — probe & status UI

| | |
|---|---|
| **Path** | `/root/autodl-tmp/sensors-view` |
| **Role** | FastAPI + web UI to list/probe/sample hik-sensors; Cookie auth; rate probes; **iframe guest** |
| **Depends on** | `./sensors` → `../sensors`; `sys.path` ← `sensors/src` |
| **Default URL** | `http://0.0.0.0:6008` |
| **Entry** | `python -m app.main`; desktop `app/desktop_main.py`; `./build.sh` |
| **Docs** | `README.md`, `docs/embed-host.md`, `docs/rate-probe*.md` |
| **Must not** | Own episode recording; fork drivers; persist host locale/theme when iframed |

**When to edit here:** status cards, probe UX, auth/users for the probe console, embed-guest behavior, desktop pack of the probe app.

---

### 3.3 `sensors-dcs` — data collection system

| | |
|---|---|
| **Path** | `/root/autodl-tmp/sensors-dcs` |
| **Role** | Agents + buffer + viz + record + export + Gello↔Arm sync/teleop + multi-station |
| **Depends on** | `./sensors` → `/root/autodl-tmp/sensors`; prefer `./sensors/src` over site-packages |
| **Default viz** | `http://127.0.0.1:7011/` (`viz_port`; helper scripts may mention 6006) |
| **Entry** | `sensors-dcs run -c configs/...`; desktop `python -m sensors_dcs.desktop_main`; `./build.sh` |
| **Docs** | `README.md`, `docs/hik-dataset*.md`, `docs/export-timeline.md`, `docs/gello-arm-*.md`, `docs/arm-write.md`, `docs/known-gaps.md`, `docs/plan.md` |
| **Must not** | Fork hik-sensors; use read-only arm agent to command motion; dual-open the same Elite endpoint carelessly |

**When to edit here:** recording pipeline, episode format, hik dataset export, teleop/sync, station peer, DCS viz, release/delta packaging filters.

Related packaging note: `packaging/bundle_filters.py` strips research bulk (checkpoints, teach-pendant media, etc.) from desktop datas / release deltas — keep filters in sync with what runtime actually needs.

---

### 3.4 `sensors-arm` — arm-only split from DCS

| | |
|---|---|
| **Path** | `/root/autodl-tmp/sensors-arm` |
| **Role** | Multi-arm visualization + gated `arm_write`; no cameras / gello / record |
| **Depends on** | `./sensors`; same `sys.path` pattern as DCS |
| **Default URL** | `http://127.0.0.1:6008/` — **collides with sensors-view**; change port when both run |
| **Entry** | `sensors-arm run -c configs/default.yaml`; `./start.sh` |
| **Docs** | `README.md`, `docs/split-from-dcs.md`, `docs/cs66.md`, `docs/ec66.md`, `docs/tcp-drag-ik.md` |
| **Must not** | Re-import full DCS collection stack; add a new arm brand without a hik-sensors kind |

**When to edit here:** multi-arm UI, URDF catalog, arm-only control UX. New Elite/CS protocol work still goes to **sensors**.

---

### 3.5 `rs-calibration` — eye-to-hand host

| | |
|---|---|
| **Path** | `/root/autodl-tmp/rs-calibration` |
| **Role** | Scene-camera Eye-to-Hand extrinsic calibration (PyQt + optional web) |
| **Depends on** | `vendor/sensors` → `sensors/src/sensors` + `PYTHONPATH=src:vendor` (or editable install) |
| **Default web** | `0.0.0.0:6006` |
| **Entry** | `python -m rs_calib` (PyQt); `python -m rs_calib web`; `scripts/build-desktop.sh` |
| **Docs** | `README.md`, `docs/需求报告-*.md`, `docs/概念设计-*.md`, `docs/详细设计-*.md` |
| **Must not** | Fork RealSense/Elite drivers into thin `infra` adapters beyond wrapping sensors |

---

### 3.6 `embody_model_eval` — model eval SPA (iframe host)

| | |
|---|---|
| **Path** | `/root/autodl-tmp/embody_model_eval` |
| **Role** | Policy/model eval visualization (Three.js, hub, chat, robots, π0.5); **Sensors page embeds sensors-view** |
| **Depends on sensors** | **No** hik-sensors symlink. Host→guest: iframe URL + `postMessage` + `GET /api/sensors-view/ping` |
| **Local arm cards** | Bundled `arm_kin` (from `demo_test` lineage), APIs under `/api/sensors/arm/*` — **not** hik-sensors |
| **Default** | `./serve.sh` → `0.0.0.0:6006` |
| **Docs** | `README.md`; embed skill `office_skills/sensors-view-iframe-embed` |
| **Must not** | Reimplement sensor drivers in the SPA; keep Sensors nav enabled when ping fails; write guest prefs into sensors-view localStorage from host |

`data/` is local eval datasets — ignored by git; do not commit episode dumps.

---

### 3.7 `office_skills` — agent skills (contracts, not apps)

| | |
|---|---|
| **Path** | `/root/autodl-tmp/office_skills` |
| **Role** | Cursor skills for reports, Embody UI ports, Scheme A desktop, sensors-view iframe, FOV, etc. |
| **Depends on sensors** | None as code |
| **Sensors-relevant skills** | `sensors-view-iframe-embed`, `scheme-a-desktop-compat`, `scheme-a-linux-to-linux-desktop`, `scheme-a-linux-to-windows-desktop`, `vendor-demo-to-structured-sensor`, `gripper-control-frequency-benchmark`, `camera-fov-analysis`, Embody/auth skills used by sensors-view |
| **Must not** | Drift from live docs (`embed-host.md`, app READMEs); skills describe contracts, apps implement them |

Install skills into a project’s `.cursor/skills/` as needed; many app-local skill copies are gitignored.

---

### 3.8 Ancestors & siblings (context only)

| Path | Relation |
|---|---|
| `hww/hik_gello` | Pre-refactor MegaCollect / teleop / calib. **Reference only.** |
| `demo_test` | FK/IK template; ideas flowed into `sensors` kinematics and `embody_model_eval/arm_kin`. Prefer fixing kinematics in **sensors** (or embody `arm_kin` if eval-only). |
| `mechanical/cad-browser` | CAD browser + Scheme A pack; **no** hik-sensors dependency. |
| `mechanical/wrist_d405`, `d405`, … | Hardware / FOV assets; use skill `camera-fov-analysis`. |
| `eoat-selector` | EOAT tooling; packaging sibling, not sensor SDK. |

---

## 4. Where should my change go?

| Intent | Put it in |
|---|---|
| New sensor kind / driver / shared config schema | **`sensors`** |
| Probe UI, rate probe, auth for status console, iframe guest | **`sensors-view`** |
| Record episode, export hik dataset, teleop, multi-station | **`sensors-dcs`** |
| Arm-only multi-robot UI / gated jog | **`sensors-arm`** |
| Eye-to-hand calibration workflow / UI | **`rs-calibration`** |
| Eval overlay, hub, chat, embed **host** wiring | **`embody_model_eval`** |
| Reusable agent procedure / porting recipe | **`office_skills`** (new `SKILL.md`) |
| Desktop PyInstaller / Wine onedir for a sensors app | That app’s `packaging/` + **scheme-a-*** skill |
| CAD / STEP / wrist FOV drawings | **`mechanical/`** |

If a feature needs both a driver and a UI: **land the driver in `sensors` first**, then thin agent/UI in the app.

---

## 5. Shared technical patterns

### 5.1 Local `sensors` link

```bash
# from sensors-view / sensors-dcs / sensors-arm
ln -sfn /root/autodl-tmp/sensors ./sensors
# rs-calibration uses vendor/ instead:
# ln -sfn /root/autodl-tmp/sensors/src/sensors vendor/sensors
```

Apps must keep importing the **linked tree**, not an outdated site-packages copy, when developing.

### 5.2 Scheme A desktop packaging

Used by **sensors-view**, **sensors-dcs**, **rs-calibration** (and some mechanical apps). Pattern: local uvicorn (or native UI) → pywebview/browser → PyInstaller **onedir** → Linux host builds Windows via Wine when needed → `release/<app>-desktop-{linux|windows}-x64-<ts>/`.

Follow: `office_skills/scheme-a-desktop-compat`, `scheme-a-linux-to-linux-desktop`, `scheme-a-linux-to-windows-desktop`.  
**sensors-arm** has no Scheme A desktop pack yet.

### 5.3 Embody ↔ sensors-view embed

- Host Settings stores sensors-view base URL; probes via `/api/sensors-view/ping`.
- Guest detects iframe: shows iframe badge; hides logout/users/locale/theme; applies host prefs via `postMessage` only (no guest localStorage write for those).
- Contract docs: `sensors-view/docs/embed-host.md`, skill `sensors-view-iframe-embed`.

### 5.4 UI / auth family

sensors-view and embody share visual/auth DNA (teal lab console, Cookie sessions). Port via Embody skills (`embody-ui-style`, `cookie-session-auth`, …) rather than one-off CSS.

---

## 6. Default ports (cheat sheet)

| Service | Default port | Notes |
|---|---|---|
| sensors-view | **6008** | Probe UI; iframe guest |
| sensors-arm | **6008** | Change if view also running |
| sensors-dcs viz | **7011** | Confirm `viz_port` in active YAML |
| rs-calibration web | **6006** | Also PyQt desktop |
| embody_model_eval | **6006** | Conflicts with rs-calib / cad-browser if concurrent |
| mechanical/cad-browser | **6006** | Hardware asset browser |
| Follower arm ZMQ (configs) | **6001** | In-sensors arm endpoint, not HTTP |

---

## 7. Repo hygiene for agents

- **Each app is its own git repo.** There is no monorepo root under `/root/autodl-tmp`.
- **Do not commit:** `./sensors` symlink targets, `data/` episode dumps (dcs/embody), `release/` / `packaging/dist*` build products, `config/users.json` / `.auth.json`, local `.cursor/skills/*` copies where gitignored.
- **Cross-repo PRs:** change `sensors` first when APIs shift; then bump app call sites in view/dcs/arm/rs.
- **Tests:** prefer `sensors/tests` for driver contracts; app tests for agents/export/UI.

---

## 8. Quick “start here” paths

```text
sensors/README.md
sensors/docs/ecosystem.md          ← this file
sensors-view/README.md + docs/embed-host.md
sensors-dcs/README.md + docs/known-gaps.md
sensors-arm/docs/split-from-dcs.md
rs-calibration/README.md
embody_model_eval/README.md
office_skills/README.md
office_skills/sensors-view-iframe-embed/SKILL.md
office_skills/scheme-a-desktop-compat/SKILL.md
```

---

## 9. Glossary

| Term | Meaning |
|---|---|
| hik-sensors / sensors | Core library repo and Python package |
| kind | Registered driver type string in YAML / registry |
| DCS | sensors-dcs data collection system |
| Scheme A | Shared desktop packaging recipe (PyInstaller onedir ± Wine) |
| MegaCollect | Legacy entry in hik_gello superseded by sensors |
| embed host / guest | SPA that iframes sensors-view / sensors-view inside iframe |
