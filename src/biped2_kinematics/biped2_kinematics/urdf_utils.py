"""Utilities to prepare URDF files for Pinocchio."""

from __future__ import annotations

import os
import re

from xacro import process as xacro_process


def xacro_to_pinocchio_urdf(xacro_path: str, package_share_dir: str, output_path: str) -> str:
    """Expand xacro and produce a Pinocchio-compatible URDF file."""
    urdf_dir = os.path.join(package_share_dir, 'urdf')
    xml = xacro_process(xacro_path, mappings={'urdf_dir': urdf_dir})
    xml = _sanitize_for_pinocchio(xml, package_share_dir)
    with open(output_path, 'w', encoding='utf-8') as urdf_file:
        urdf_file.write(xml)
    return output_path


def _sanitize_for_pinocchio(xml: str, package_share_dir: str) -> str:
    xml = re.sub(r'<ros2_control[\s\S]*?</ros2_control>', '', xml)
    xml = re.sub(r'<gazebo[\s\S]*?</gazebo>', '', xml)
    xml = xml.replace('package://biped2_description', package_share_dir)
    return xml
