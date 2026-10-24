#!/usr/bin/env python3
# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""Download and verify the Go2 model_manifest in config/policies.yaml."""

import argparse
import hashlib
from http.client import HTTPException
from pathlib import Path, PurePosixPath
import shutil
import sys
import time
from urllib.error import HTTPError
from urllib.parse import quote, urljoin
from urllib.request import urlopen

import yaml


def application_root():
    root = Path(__file__).resolve().parents[1]
    if (root / "config/policies.yaml").is_file():
        return root
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("unitree_go2"))


def matches(path, entry):
    if not path.is_file() or path.stat().st_size != entry["bytes"]:
        return False
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest() == entry["sha256"]


def download_model(url, target, entry):
    temporary = target.with_name(target.name + ".part")
    attempts = 3
    for attempt in range(1, attempts + 1):
        print(f"Downloading {url} (attempt {attempt}/{attempts})", flush=True)
        try:
            with urlopen(url, timeout=30) as response, temporary.open("wb") as stream:
                shutil.copyfileobj(response, stream)
            received = temporary.stat().st_size
            if received != entry["bytes"]:
                raise ConnectionError(f"incomplete download: expected {entry['bytes']} bytes, received {received}")
            if not matches(temporary, entry):
                raise ValueError(f"SHA-256 mismatch: {url}")
            temporary.replace(target)
            return
        except (OSError, HTTPException) as error:
            retryable = not isinstance(error, HTTPError) or error.code in (408, 429, 500, 502, 503, 504)
            if not retryable or attempt == attempts:
                raise RuntimeError(f"{url} (attempt {attempt}/{attempts}): {error}") from error
            delay = 2 ** (attempt - 1)
            print(f"Download attempt failed: {error}; retrying {url} in {delay}s", file=sys.stderr, flush=True)
            time.sleep(delay)
        finally:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", help="Override the model archive URL")
    parser.add_argument("--output", type=Path, help="Model directory; default: this application's policy/")
    parser.add_argument("--training-repo", type=Path, help="Local checkout containing self-trained model archives")
    parser.add_argument("--check", action="store_true", help="Verify local files without downloading")
    args = parser.parse_args()
    root = application_root()
    manifest = yaml.safe_load((root / "config/policies.yaml").read_text())["model_manifest"]
    destination = args.output or root / "policy"
    base_url = (args.base_url or manifest["archive"]["base_url"]).rstrip("/") + "/"
    missing = []
    for name, entry in manifest["files"].items():
        relative = PurePosixPath(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"Invalid model asset path: {name}")
        target = destination / relative
        if matches(target, entry):
            continue
        if args.check:
            missing.append(name)
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        if entry.get("training_path") and args.training_repo is not None:
            relative_source = PurePosixPath(entry["training_path"])
            if relative_source.is_absolute() or ".." in relative_source.parts:
                raise ValueError(f"Invalid training asset path: {relative_source}")
            source = args.training_repo / relative_source
            if not matches(source, entry):
                raise ValueError(f"Missing or invalid training archive: {source}")
            shutil.copyfile(source, target)
            print(f"Copied {name} from training archive", flush=True)
            continue
        remote = PurePosixPath(entry.get("remote_path", name))
        if remote.is_absolute() or ".." in remote.parts:
            raise ValueError(f"Invalid model archive path: {remote}")
        download_model(urljoin(base_url, quote(str(remote))), target, entry)
    if missing:
        print("Missing or invalid assets:\n" + "\n".join(missing), file=sys.stderr)
        return 1
    print(f"Verified {len(manifest['files'])} files in {destination}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, KeyError, yaml.YAMLError) as error:
        print(f"Model download failed: {error}", file=sys.stderr)
        sys.exit(1)
