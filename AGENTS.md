# AGENTS.md — sensors multi-repo workspace bootstrap

## Document purpose

This file is for **Cursor (or other coding agents) on a new machine**.

After reading it you should be able to:

1. Clone the related GitHub repositories into one workspace directory  
2. Wire the **`sensors`** core library into the apps via symlinks  
3. Install the minimum Python / optional hardware dependencies  
4. Know which repo owns which responsibility (so you do not put drivers in the wrong tree)

Canonical deep map (roles, must-not rules, ports, change placement):  
→ [`docs/ecosystem.md`](docs/ecosystem.md) (same repo, after clone)

**Owner GitHub user:** [`fszhangYi`](https://github.com/fszhangYi)  
**Suggested workspace root:** `~/sensors-workspace` (or any empty directory you choose)

---

## Architecture (one screen)

```text
hww/hik_gello (legacy, not required) ──refactor──► sensors (hik-sensors) ★
                                                      │
              ┌───────────────┬───────────────┬───────┴────────┐
              ▼               ▼               ▼                ▼
        sensors-view    sensors-dcs     sensors-arm      rs-calibration
        (probe UI)      (collect)       (arm-only)       (eye-to-hand)
              ▲
              │ iframe
              │
        embody_model_eval          office_skills (Cursor skills, not runtime)
```

**Hard rule:** all hardware drivers / sensor `kind`s live only in **`sensors`**. Apps consume it; they do not fork drivers.

---

## GitHub repositories

Clone with HTTPS or SSH. Public repos under `fszhangYi`:

| Local dir | GitHub | Default branch | Role | Required for minimal stack |
|-----------|--------|----------------|------|------------------------------|
| `sensors` | https://github.com/fszhangYi/sensors.git | `master` | Core SDK (`hik-sensors`) | **Yes** |
| `sensors-view` | https://github.com/fszhangYi/sensors-view.git | `master` | Probe / status UI (port **6008**); iframe guest | **Yes** (UI) |
| `sensors-dcs` | https://github.com/fszhangYi/sensors-dcs.git | `master` | Data collection / export / teleop | If collecting |
| `embody_model_eval` | https://github.com/fszhangYi/embody_model_eval.git | `main` (use **`dev`** for latest) | Model eval SPA; embeds sensors-view | If eval / embed host |
| `office_skills` | https://github.com/fszhangYi/office_skills.git | `main` | Agent skills (Scheme A, iframe, Embody UI…) | Recommended for agents |
| `sensors-arm` | https://github.com/fszhangYi/sensors-arm.git | `main` | Multi-arm viz/control (split from DCS) | If arm-only UI |
| `rs-calibration` | https://github.com/fszhangYi/rs-calibration.git | **`autodl`** | Eye-to-hand calibration host | If calibrating |
| `cad-browser` | https://github.com/fszhangYi/cad-browser.git | `master` | Mechanical CAD browser (optional) | No |

Legacy reference only (not required, often unpublished): `hww/hik_gello`, `demo_test`.

---

## Bootstrap: create the multi-repo workspace

Run on the new machine (bash). Adjust `WS` if needed.

```bash
set -euo pipefail
WS="${WS:-$HOME/sensors-workspace}"
mkdir -p "$WS"
cd "$WS"

clone() {
  local url="$1" dir="$2" branch="${3:-}"
  if [ -d "$dir/.git" ]; then
    echo "skip clone (exists): $dir"
    git -C "$dir" fetch --all --prune
  else
    git clone "$url" "$dir"
  fi
  if [ -n "$branch" ]; then
    git -C "$dir" checkout "$branch"
    git -C "$dir" pull --ff-only || true
  fi
}

# Core + apps that are published
clone https://github.com/fszhangYi/sensors.git            sensors
clone https://github.com/fszhangYi/sensors-view.git       sensors-view
clone https://github.com/fszhangYi/sensors-dcs.git        sensors-dcs
clone https://github.com/fszhangYi/embody_model_eval.git  embody_model_eval  dev
clone https://github.com/fszhangYi/office_skills.git      office_skills
clone https://github.com/fszhangYi/sensors-arm.git        sensors-arm
clone https://github.com/fszhangYi/rs-calibration.git   rs-calibration  autodl

# Optional
# clone https://github.com/fszhangYi/cad-browser.git      cad-browser

# Symlink core library into apps (required layout)
ln -sfn "$WS/sensors" "$WS/sensors-view/sensors"
ln -sfn "$WS/sensors" "$WS/sensors-dcs/sensors"
ln -sfn "$WS/sensors" "$WS/sensors-arm/sensors"
mkdir -p "$WS/rs-calibration/vendor"
ln -sfn "$WS/sensors/src/sensors" "$WS/rs-calibration/vendor/sensors"

# Cursor entrypoints: workspace AGENTS.md → sensors bootstrap copy
ln -sfn "$WS/sensors/AGENTS.md" "$WS/AGENTS.md" 2>/dev/null || cp -f "$WS/sensors/AGENTS.md" "$WS/AGENTS.md"

# Optional: install office_skills into Cursor project skills
mkdir -p "$WS/.cursor/skills"
for s in sensors-view-iframe-embed scheme-a-desktop-compat \
         scheme-a-linux-to-linux-desktop scheme-a-linux-to-windows-desktop \
         cookie-session-auth embody-ui-style; do
  [ -d "$WS/office_skills/$s" ] && cp -a "$WS/office_skills/$s" "$WS/.cursor/skills/$s"
done

echo "Workspace ready under: $WS"
echo "Next: install Python deps (see Environment below)."
```

SSH clone form (if keys are configured):

```text
git@github.com:fszhangYi/<repo>.git
```

---

## Environment dependencies

### Base (all machines)

| Item | Requirement |
|------|-------------|
| OS | Linux x86_64 recommended (AutoDL / Ubuntu 20.04+ / 22.04). Windows OK for some desktop packs / PyQt. |
| Git | 2.x |
| Python | **≥ 3.10** for sensors stack; **3.12 recommended** for `embody_model_eval` |
| pip / venv | Use a dedicated venv (or conda env) per machine; do not mix system Python blindly |
| Browser | Modern Chromium / Firefox for web UIs |

Create a venv at the workspace root (example):

```bash
cd "$WS"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip setuptools wheel
```

### Python packages — software-only (dry-run / UI without hardware)

```bash
# Core
pip install -e "./sensors[dev]"

# Probe UI
pip install -e "./sensors-view"
# or: pip install -r sensors-view/requirements.txt && pip install -e ./sensors-view

# Data collection (no cameras/arms yet)
pip install -e "./sensors-dcs"
# timeline export extras:
# pip install -e "./sensors-dcs[export]"

# Embody eval host (stdlib server + optional kin)
pip install -r embody_model_eval/requirements.txt
# Frontend rebuild only if you change React sources:
#   cd embody_model_eval/frontend && npm ci && npm run build
# Serving prebuilt pages does **not** require Node.
```

Pinned notes from app trees:

- Many apps pin **`numpy==1.23.5`** in requirements for desktop/RealSense compatibility; prefer the app’s `requirements.txt` when packaging.
- `sensors` extras (optional): `serial`, `zmq`, `realsense`, `dynamixel`, `kinematics`, `elite-ec`, `dev` — see `sensors/pyproject.toml`.

### Hardware extras (lab / robot PC)

Install only what the station has:

```bash
# sensors-view live probe
pip install -r sensors-view/requirements-hardware.txt
# → pyserial, pyzmq, dynamixel-sdk, pyrealsense2

# sensors-dcs live collect
pip install -r sensors-dcs/requirements-hardware.txt
# → serial, dynamixel-sdk, elirobots (Elite EC), loguru, pyrealsense2, opencv-headless

# Or via hik-sensors extras:
pip install -e "./sensors[serial,zmq,realsense,dynamixel,elite-ec,kinematics]"
```

| Hardware | Typical package / note |
|----------|-------------------------|
| USB serial / bus | `pyserial` |
| Gello (Dynamixel) | `dynamixel-sdk` |
| RealSense | `pyrealsense2` (+ Intel USB rules on Linux) |
| Elite EC arm | `elirobots` (`import elite`) |
| Elite CS arm | build/install Elite CS Python SDK separately (not on PyPI as `elirobots`) |
| DH AG95 gripper | Modbus via serial (covered by serial extras) |
| Force/torque, tactile | vendor-specific; see sensors drivers / docs |

### Node.js (optional)

- **Not** required to run `sensors-view` / `sensors-dcs` / stock `embody_model_eval` pages.  
- **Required** only to rebuild `embody_model_eval/frontend` (React + Vite). Use Node 20+ if you do.

### Desktop packaging (Scheme A)

Only if building onedir / Wine Windows packs: follow skills under `office_skills/scheme-a-*` and each app’s `packaging/` + `build.sh`. Needs PyInstaller; Windows cross-build from Linux needs Wine (see skills).

---

## Smoke-test after install

```bash
source "$WS/.venv/bin/activate"

# Core CLI
sensors kinds
sensors list -c sensors/configs/default.yaml

# Probe UI (http://127.0.0.1:6008) — often dry_run in configs/view.yaml
python -m app.main --host 127.0.0.1 --port 6008
# cwd: sensors-view

# DCS — see sensors-dcs/README.md (viz often :7011)
# sensors-dcs run -c configs/<your>.yaml

# Embody (http://127.0.0.1:6006); Sensors page iframes sensors-view when URL set in Settings
cd embody_model_eval && ./serve.sh
```

### Default ports

| Service | Port |
|---------|------|
| sensors-view | 6008 |
| sensors-arm (when present) | 6008 (conflicts with view) |
| sensors-dcs viz | 7011 (check YAML `viz_port`) |
| embody_model_eval | 6006 |
| rs-calibration web | 6006 |

---

## Where to put changes (short)

| Intent | Repo |
|--------|------|
| New driver / kind / shared kinematics | **`sensors`** |
| Probe UI, rate probe, iframe **guest** | **`sensors-view`** |
| Record / export / teleop / multi-station | **`sensors-dcs`** |
| Eval SPA / iframe **host** | **`embody_model_eval`** |
| Reusable agent recipes | **`office_skills`** |

Full rules: [`docs/ecosystem.md`](docs/ecosystem.md).

---

## Repo hygiene

- Each directory above is its **own git repo** (not a monorepo).  
- Do **not** commit the `./sensors` symlink target into app repos (already gitignored).  
- Do **not** commit `data/` episode dumps, `release/` build products, `config/users.json` / `.auth.json`.  
- Cross-repo API changes: land in **`sensors` first**, then update consumers.

---

## Publish checklist (maintainers)

If an agent is asked “is everything on GitHub?”:

1. Confirm remotes for the table above resolve (`git ls-remote`).  
2. Push any local-ahead branches on machines that have deploy/user keys.  
3. Keep default branches in sync with the table (`embody_model_eval` → `dev`, `rs-calibration` → `autodl`).
