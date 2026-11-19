#!/usr/bin/env bash
# Publish focused commits and repository topics. Never creates a release or tag.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
PREPARE_ONLY=false
if [[ "${1:-}" == --prepare-only ]]; then
    PREPARE_ONLY=true
elif [[ $# -ne 0 ]]; then
    echo "Usage: bash scripts/publish_github.sh [--prepare-only]" >&2; exit 2
fi
for program in git python3; do command -v "$program" >/dev/null || { echo "Missing $program" >&2; exit 2; }; done
REMOTE_HEAD=""
if [[ "$PREPARE_ONLY" == false ]]; then
    command -v gh >/dev/null || { echo 'GitHub CLI (gh) is required.' >&2; exit 2; }
    # Check connectivity/authentication before changing Git history.
    OWNER="$(gh api user --jq .login)"
    NAME="$(python3 -c 'import json; print(json.load(open(".github/repository.json"))["name"])')"
    REPOSITORY="$OWNER/$NAME"
    ORIGIN="$(git config --get remote.origin.url 2>/dev/null || true)"
    if [[ -n "$ORIGIN" && "$ORIGIN" != "https://github.com/$REPOSITORY.git" && "$ORIGIN" != "git@github.com:$REPOSITORY.git" ]]; then
        echo 'Origin points to another repository; refusing to replace it.' >&2; exit 2
    fi
    if [[ -n "$ORIGIN" ]]; then
        REMOTE_HEAD="$(git ls-remote origin refs/heads/main | cut -f1)"
        TRACKED_HEAD="$(git rev-parse refs/remotes/origin/main 2>/dev/null || true)"
        if [[ -n "$REMOTE_HEAD" && "$REMOTE_HEAD" != "$TRACKED_HEAD" ]]; then
            echo 'Remote main changed since the last fetch. Fetch and review it before retrying.' >&2; exit 2
        fi
        if [[ -n "$REMOTE_HEAD" ]] && ! git merge-base --is-ancestor "$REMOTE_HEAD" HEAD; then
            BACKUP="$(git rev-parse refs/backup/before-initial-split 2>/dev/null || true)"
            if [[ "$REMOTE_HEAD" != "$BACKUP" ]]; then
                echo 'Remote history is unrelated to the recorded initial commit; refusing to replace it.' >&2; exit 2
            fi
        fi
    fi
fi
if ! git rev-parse --git-dir >/dev/null 2>&1; then git init -b main; fi
if [[ "$(git rev-parse --show-toplevel)" != "$PROJECT_DIR" || "$(git branch --show-current)" != main ]]; then
    echo 'Run from a project checkout on main.' >&2; exit 2
fi
if ! git config user.name >/dev/null; then git config user.name "${OWNER:-rivindu02}"; fi
if ! git config user.email >/dev/null; then git config user.email "${OWNER:-rivindu02}@users.noreply.github.com"; fi
python3 - <<'PY'
"""Build a truthful component-based history without changing worktree contents."""
import os
from pathlib import Path
import subprocess
import tempfile

def git(*args,env=None):
    return subprocess.check_output(['git',*args],env=env,text=True).strip()

groups=[
 ('Set up project dependencies and Git housekeeping',['requirements.txt','.gitignore','.gitattributes']),
 ('Configure robot geometry, beacons and sensor models',['config']),
 ('Define versioned sensor events and configuration validation',['src/localization/__init__.py','src/localization/events.py']),
 ('Implement motion and RSSI models with analytic Jacobians',['src/localization/models.py']),
 ('Fit per-beacon RSSI calibration and held-out noise',['src/localization/calibration.py']),
 ('Implement gyro-bias EKF with gated RSSI corrections',['src/localization/ekf.py']),
 ('Generate paired synthetic routes and sensor impairments',['src/localization/simulation.py']),
 ('Add causal replay and six comparison baselines',['src/localization/replay.py']),
 ('Measure trajectory error, availability and filter consistency',['src/localization/evaluation.py']),
 ('Add calibration, simulation, replay and benchmark commands',['scripts/localization.py']),
 ('Test Jacobians, calibration, replay, outages and covariance',['tests']),
 ('Add self-contained differential-drive robot template',['gazebo/robot_base.sdf']),
 ('Generate the BLE beacon world and native IMU sensor',['scripts/build_localization_world.py','gazebo/localization_lab.world']),
 ('Bridge Gazebo sensors to fusion and separate truth evaluation',['gazebo/localization_nodes.py']),
 ('Configure RViz paths, beacons and covariance displays',['gazebo/localization.rviz']),
 ('Inspect Python, ROS and Gazebo runtime availability',['scripts/check_environment.py']),
 ('Launch bounded headless and GUI Gazebo demos',['gazebo/run_localization.sh']),
 ('Validate native sensors, BLE corrections and saved trajectories',['scripts/validate_localization_run.py']),
 ('Validate paired benchmark coverage and covariance',['scripts/validate_experiments.py']),
 ('Record fitted simulated beacon calibration',['data/simulated/calibration/fit.json']),
 ('Record aggregate metrics for 260 paired trials',['results/localization/summary.json','results/localization/metrics.csv','results/localization/trials.json']),
]
for name in ['nominal_rectangle_7','nominal_stationary_7','nominal_straight_7','nominal_figure_eight_7',
             'noisy_rssi_rectangle_7','nlos_rectangle_7','beacon_outage_rectangle_7','weak_geometry_rectangle_7',
             'gyro_bias_rectangle_7','wheel_slip_rectangle_7','miscalibration_rectangle_7',
             'delayed_packets_rectangle_7','overconfident_rectangle_7']:
    groups.append(('Record '+name.replace('_',' ')+' experiment',['results/localization/'+name]))
groups += [
 ('Preserve the validated 80-second Gazebo outage recording',['results/localization/gazebo_outage']),
 ('Render reproducible figures and animations from saved data',['scripts/render_media.py']),
 ('Animate the nominal EKF localization trajectory',['docs/assets/localization.gif']),
 ('Animate radio dropout and recovery in the Gazebo recording',['docs/assets/gazebo-outage.gif']),
 ('Plot nominal trajectory comparisons',['docs/assets/trajectory.png']),
 ('Plot paired scenario errors and failure cases',['docs/assets/scenarios.png']),
 ('Plot calibrated RSSI models and validation samples',['docs/assets/calibration.png']),
 ('Plot the recorded Gazebo outage trajectory and errors',['docs/assets/gazebo-outage.png']),
 ('Plot RSSI innovation gates and outlier diagnostics',['docs/assets/innovations.png']),
 ('Document state equations, Jacobians and covariance updates',['docs/LOCALIZATION_MODEL.md']),
 ('Document Gazebo setup, ROS topics and troubleshooting',['docs/LOCALIZATION_GAZEBO.md']),
 ('Document benchmark methodology and measured limitations',['docs/EXPERIMENTS.md']),
 ('Document the development and validation workflow',['docs/CONTRIBUTING.md']),
 ('Write step-by-step README with animations and measured results',['README.md']),
 ('Configure GitHub description and robotics topic tags',['.github/repository.json']),
 ('Run numerical and fixture checks across Python versions in CI',['.github/workflows']),
 ('Publish focused commits without creating releases',['scripts/publish_github.sh']),
]
old=subprocess.run(['git','rev-parse','--verify','HEAD'],capture_output=True,text=True)
head=old.stdout.strip() if old.returncode==0 else None
split=head is None or int(git('rev-list','--count','HEAD'))==1
if split and head:
    backup='refs/backup/before-initial-split'
    existing=subprocess.run(['git','rev-parse','--verify',backup],capture_output=True,text=True)
    if existing.returncode==0 and existing.stdout.strip()!=head:
        raise SystemExit('A different history backup already exists; review it before splitting')
    git('update-ref',backup,head)
allowlist=['README.md','requirements.txt','.gitignore','.gitattributes','.github','config','src','tests','gazebo','scripts','docs','results','data']
with tempfile.TemporaryDirectory(prefix='ble-imu-history-') as tmp:
    env=dict(os.environ,GIT_INDEX_FILE=str(Path(tmp)/'index'))
    git('read-tree',head if head else '--empty',env=env)
    git('add','-A','--',*allowlist,env=env)
    desired=git('write-tree',env=env)
    desired_paths=set(git('ls-tree','-r','--name-only',desired).splitlines())
    old_paths=set(git('ls-tree','-r','--name-only',head).splitlines()) if head and not split else set()
    paths=desired_paths|old_paths
    git('read-tree','--empty' if split else head,env=env)
    parent=None if split else head
    last_tree=git('write-tree',env=env); covered=set(); created=0
    for title,prefixes in groups+[('Preserve remaining project files and remove obsolete publishing notes',[''])]:
        selected=sorted(p for p in paths-covered if any(not prefix or p==prefix or p.startswith(prefix+'/') for prefix in prefixes))
        if not selected: continue
        git('add','-A','--',*selected,env=env); covered.update(selected)
        tree=git('write-tree',env=env)
        if tree==last_tree: continue
        args=['commit-tree',tree]
        if parent: args+=['-p',parent]
        parent=git(*args,'-m',title); last_tree=tree; created+=1
        print(f'{created:02d} {parent[:8]} {title}',flush=True)
    if last_tree!=desired: raise SystemExit('History construction changed the final tree; branch left untouched')
    if parent and parent!=head:
        git('update-ref','refs/heads/main',parent,head or '0'*40)
        git('read-tree',desired)
# Remove the release tag locally, retaining its object under a backup ref.
tag=subprocess.run(['git','rev-parse','--verify','refs/tags/v1.0.0'],capture_output=True,text=True)
if tag.returncode==0:
    git('update-ref','refs/backup/removed-release-tag',tag.stdout.strip())
    git('update-ref','-d','refs/tags/v1.0.0')
print(f'Prepared {created} focused commits; final file tree verified unchanged.')
PY
if [[ "$PREPARE_ONLY" == true ]]; then
    echo 'Local history prepared. Remote repository was not changed.'; exit 0
fi
mapfile -t META < <(python3 - <<'PY'
import json
m=json.load(open('.github/repository.json'))
for key in ['visibility','description']: print(m[key])
for topic in m['topics']: print(topic)
PY
)
if [[ -z "${ORIGIN:-}" ]]; then
    gh repo create "$REPOSITORY" "--${META[0]}" --description "${META[1]}" --source "$PROJECT_DIR" --remote origin
fi
TOPIC_ARGS=()
for topic in "${META[@]:2}"; do TOPIC_ARGS+=(--add-topic "$topic"); done
gh repo edit "$REPOSITORY" --description "${META[1]}" --enable-issues --enable-wiki=false "${TOPIC_ARGS[@]}"
# Only replace the exact remote head checked above; never overwrite newer work.
if [[ -n "$REMOTE_HEAD" ]] && ! git merge-base --is-ancestor "$REMOTE_HEAD" HEAD; then
    git -c credential.helper= -c 'credential.helper=!gh auth git-credential' push "--force-with-lease=refs/heads/main:$REMOTE_HEAD" -u origin main
else
    git -c credential.helper= -c 'credential.helper=!gh auth git-credential' push -u origin main
fi
ERROR_FILE="$(mktemp)"
trap 'rm -f "$ERROR_FILE"' EXIT
if gh api "repos/$REPOSITORY/releases/tags/v1.0.0" --silent 2>"$ERROR_FILE"; then
    gh release delete v1.0.0 --repo "$REPOSITORY" --cleanup-tag --yes
elif ! python3 -c 'import sys; sys.exit(0 if "HTTP 404" in open(sys.argv[1]).read() else 1)' "$ERROR_FILE"; then
    cat "$ERROR_FILE" >&2; exit 1
fi
if [[ -n "$(git ls-remote origin refs/tags/v1.0.0)" ]]; then
    git -c credential.helper= -c 'credential.helper=!gh auth git-credential' push origin :refs/tags/v1.0.0
fi
gh repo view "$REPOSITORY" --json url,description,repositoryTopics --jq '{url, description, topics: [.repositoryTopics[].name]}'
echo 'Published focused history; v1.0.0 release and tag removed.'
