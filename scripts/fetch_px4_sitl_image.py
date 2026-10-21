#!/usr/bin/env python3
"""
Fetch a PX4 SIH SITL container image (px4io/px4-sitl) and unpack its PX4 install
so the test harness can run it directly — no Docker needed.

The image is PX4's own pre-built SIH package (the `px4` .deb, /opt/px4) on an
Ubuntu 24.04 base; the binary runs natively on a matching host. This pulls the
image's layers from Docker Hub, extracts /opt/px4, and adds the
`build/px4_sitl_default` layout `--px4-sitl` expects (a symlink to it):

    python3 scripts/fetch_px4_sitl_image.py                 # px4io/px4-sitl:latest
    python3 scripts/fetch_px4_sitl_image.py --tag v1.18.0-rc1

    pytest tests/command/nav_land/ --drone-address=udp://:14540 \\
        --px4-sitl=~/px4_sitl_images/latest --px4-model=sihsim_quadx \\
        --vehicle-type=quadcopter --autopilot=px4

Any SIH model in the image works (`sihsim_quadx`, `sihsim_standard_vtol`,
`sihsim_airplane`, ...). What this does NOT reproduce is the container's
entrypoint, which only redirects MAVLink to the Docker host — irrelevant when
PX4 runs on the host itself. Unlike `docker run --rm`, parameters persist
between runs, in <dest>/root/opt/px4/fs/ (delete it for a factory-fresh start).

Needs `curl` and `tar`; amd64 or arm64 Linux with the image's glibc or newer.
"""

import argparse
import json
import platform
import subprocess
import sys
import urllib.request
from pathlib import Path

REPO = "px4io/px4-sitl"
REGISTRY = "https://registry-1.docker.io/v2"
INDEX_TYPES = ("application/vnd.oci.image.index.v1+json,"
               "application/vnd.docker.distribution.manifest.list.v2+json")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--tag", default="latest", help="image tag (default: latest)")
    ap.add_argument("--dest", default=None, help="destination (default: ~/px4_sitl_images/<tag>)")
    args = ap.parse_args()

    arch = {"x86_64": "amd64", "aarch64": "arm64"}.get(platform.machine())
    if arch is None:
        sys.exit(f"unsupported architecture: {platform.machine()}")
    dest = Path(args.dest).expanduser() if args.dest else Path.home() / "px4_sitl_images" / args.tag
    layers_dir, root = dest / "layers", dest / "root"
    layers_dir.mkdir(parents=True, exist_ok=True)
    root.mkdir(exist_ok=True)

    token = json.load(urllib.request.urlopen(
        f"https://auth.docker.io/token?service=registry.docker.io&scope=repository:{REPO}:pull"))["token"]

    def fetch(path: str, accept: str, out: Path | None = None) -> bytes:
        # curl, not urllib: blob downloads redirect to a CDN that rejects the bearer
        # token, and curl drops the Authorization header on a cross-host redirect.
        cmd = ["curl", "-sSfL", "-H", f"Authorization: Bearer {token}", "-H", f"Accept: {accept}",
               f"{REGISTRY}/{REPO}/{path}"]
        if out is not None:
            subprocess.run(cmd + ["-o", str(out)], check=True)
            return b""
        return subprocess.run(cmd, check=True, capture_output=True).stdout

    index = json.loads(fetch(f"manifests/{args.tag}", INDEX_TYPES))
    entry = next(m for m in index["manifests"]
                 if m["platform"]["architecture"] == arch and m["platform"]["os"] == "linux")
    manifest = json.loads(fetch(f"manifests/{entry['digest']}", entry["mediaType"]))
    config = json.loads(fetch(f"blobs/{manifest['config']['digest']}", "*/*"))
    (dest / "image.json").write_text(json.dumps(
        {"repo": REPO, "tag": args.tag, "arch": arch, "digest": entry["digest"],
         "created": config.get("created"), "config": config.get("config")}, indent=1))

    for i, layer in enumerate(manifest["layers"]):
        blob = layers_dir / f"layer{i}.tar.gz"
        fetch(f"blobs/{layer['digest']}", "*/*", blob)
        # Only PX4's own install; later layers overwrite earlier ones, as in the image.
        subprocess.run(["tar", "-xzf", str(blob), "-C", str(root), "--wildcards", "opt/px4*"],
                       stderr=subprocess.DEVNULL, check=False)

    px4 = root / "opt/px4/bin/px4"
    if not px4.exists():
        sys.exit(f"no /opt/px4/bin/px4 in {REPO}:{args.tag} — not a SIH image?")
    link = dest / "build/px4_sitl_default"
    link.parent.mkdir(exist_ok=True)
    if link.is_symlink():
        link.unlink()
    link.symlink_to("../root/opt/px4")

    missing = [line.strip() for line in subprocess.run(["ldd", str(px4)], capture_output=True, text=True).stdout.splitlines()
               if "not found" in line]
    print(f"{REPO}:{args.tag} ({arch}, {entry['digest'][:19]}) unpacked to {dest}")
    if missing:
        print("WARNING: missing shared libraries on this host:\n  " + "\n  ".join(missing))
    print(f"Use: --px4-sitl={dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
