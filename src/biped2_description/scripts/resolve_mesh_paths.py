#!/usr/bin/env python3
"""Replace package:// mesh URIs with absolute file:// paths for Gazebo."""

from __future__ import annotations

import sys
from pathlib import Path


def main() -> int:
    urdf_path = Path(sys.argv[1])
    mesh_dir = Path(sys.argv[2]).resolve()
    text = urdf_path.read_text(encoding='utf-8')
    text = text.replace(
        'package://biped2_description/meshes/',
        f'file://{mesh_dir}/',
    )
    urdf_path.write_text(text, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
