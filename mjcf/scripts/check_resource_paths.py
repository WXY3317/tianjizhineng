# 检查 MJCF 引用的模型和网格路径，定位缺失资源，便于打包和迁移。
from __future__ import annotations

import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


MJCF_ROOT = Path(__file__).resolve().parents[1]
ABSOLUTE_WINDOWS = re.compile(r"^[A-Za-z]:[\\/]|^\\\\")
FORBIDDEN_TEXT = ("package://", "Gento Luna_URDF")


def main() -> int:
    errors: list[str] = []
    xml_files = list(MJCF_ROOT.rglob("*.xml"))
    resource_files = {
        path.resolve()
        for path in MJCF_ROOT.rglob("*")
        if path.is_file()
    }

    for xml_file in xml_files:
        text = xml_file.read_text(encoding="utf-8")
        for marker in FORBIDDEN_TEXT:
            if marker in text:
                errors.append(f"{xml_file}: forbidden text {marker!r}")
        try:
            root = ET.fromstring(text)
        except ET.ParseError as exc:
            errors.append(f"{xml_file}: XML parse error: {exc}")
            continue
        for element in root.iter():
            for attr_name, value in element.attrib.items():
                if attr_name not in {"file", "mesh", "texture", "material"}:
                    continue
                if ABSOLUTE_WINDOWS.match(value) or value.startswith("/"):
                    errors.append(f"{xml_file}: absolute resource path {value!r}")
                if attr_name == "file" and element.tag in {"mesh", "texture"}:
                    target = (xml_file.parent / value).resolve()
                    if target not in resource_files:
                        errors.append(f"{xml_file}: missing resource {value!r}")
        include = root.find("include")
        if include is not None:
            include_path = (xml_file.parent / include.get("file", "")).resolve()
            if include_path not in resource_files:
                errors.append(f"{xml_file}: missing include {include.get('file')!r}")

    for path in MJCF_ROOT.rglob("*"):
        if path.is_file() and MJCF_ROOT.resolve() not in path.resolve().parents and path.resolve() != MJCF_ROOT.resolve():
            errors.append(f"resource escapes mjcf root: {path}")

    if errors:
        print("RESOURCE PATH CHECK FAILED")
        print("\n".join(f"- {error}" for error in errors))
        return 1
    print(f"RESOURCE PATH CHECK PASSED ({len(xml_files)} XML files, {len(resource_files)} files inside mjcf)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
