#!/usr/bin/env python3
"""Verify the exported source snapshot without importing robot packages."""

import argparse
import ast
import hashlib
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SECRET_PATTERNS = {
    "private_key": re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    "github_token": re.compile(rb"(?:gh[pousr]_[A-Za-z0-9]{20,}|github_pat_[A-Za-z0-9_]{20,})"),
    "gitlab_token": re.compile(rb"glpat-[A-Za-z0-9_-]{12,}"),
    "credential_url": re.compile(rb"https?://[^\s/@:]+:[^\s/@]+@"),
    "aws_access_key": re.compile(rb"AKIA[A-Z0-9]{16}"),
}


def verify():
    manifest = json.loads((ROOT / "docs/source_manifest.json").read_text())
    errors = []
    counts = {"source_files": 0, "python_files": 0, "xml_files": 0,
              "markdown_links": 0, "scanned_files": 0}
    source_paths = set()
    for record in manifest["files"]:
        relative = Path(record["destination"])
        if relative.is_absolute() or ".." in relative.parts:
            errors.append({"path": str(relative), "check": "unsafe_manifest_path"})
            continue
        if str(relative) in source_paths:
            errors.append({"path": str(relative), "check": "duplicate_manifest_path"})
        source_paths.add(str(relative))
        path = ROOT / relative
        if not path.is_file() or path.is_symlink():
            errors.append({"path": str(relative), "check": "missing_source"})
            continue
        data = path.read_bytes()
        if len(data) != record["bytes"] or hashlib.sha256(data).hexdigest() != record["sha256"]:
            errors.append({"path": str(relative), "check": "source_hash"})
        counts["source_files"] += 1

    for path in sorted(ROOT.rglob("*")):
        relative = path.relative_to(ROOT)
        if ".git" in relative.parts or "__pycache__" in relative.parts:
            continue
        if path.is_symlink():
            errors.append({"path": str(relative), "check": "symlink"})
            continue
        if not path.is_file():
            continue
        data = path.read_bytes()
        counts["scanned_files"] += 1
        if path.suffix in {".so", ".whl", ".deb", ".pyc", ".key"} or ".so." in path.name or path.name.startswith(".env"):
            errors.append({"path": str(relative), "check": "excluded_artifact"})
        if len(data) >= 100 * 1024 * 1024:
            errors.append({"path": str(relative), "check": "oversize_file"})
        for name, pattern in SECRET_PATTERNS.items():
            if pattern.search(data):
                errors.append({"path": str(relative), "check": name})
        if path.suffix == ".py":
            counts["python_files"] += 1
            try:
                ast.parse(data, filename=str(relative))
            except SyntaxError as exc:
                errors.append({"path": str(relative), "check": "python_syntax", "line": exc.lineno})
        if path.suffix in {".xml", ".urdf"}:
            counts["xml_files"] += 1
            try:
                ET.fromstring(data)
            except ET.ParseError:
                errors.append({"path": str(relative), "check": "xml_syntax"})
        if path.suffix == ".md":
            # Validate authored repository docs; copied upstream docs may refer to
            # assets and pages outside the deliberately selected source snapshot.
            if str(relative) in source_paths:
                continue
            for target in re.findall(r"\]\(([^\s)]+)\)", data.decode("utf-8")):
                if "://" in target or target.startswith(("#", "mailto:")):
                    continue
                target = target.split("#", 1)[0]
                counts["markdown_links"] += 1
                if not (path.parent / target).exists():
                    errors.append({"path": str(relative), "check": "markdown_link", "target": target})

    return {"status": "passed" if not errors else "failed",
            "python_version": sys.version.split()[0], "checks": counts,
            "algorithm_tests_run": False, "cpp_build_run": False,
            "robot_connections_used": False, "errors": errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, help="Optionally save the JSON result")
    args = parser.parse_args()
    result = verify()
    output = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.report:
        report = args.report if args.report.is_absolute() else ROOT / args.report
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(output, encoding="utf-8")
    print(output, end="")
    raise SystemExit(0 if result["status"] == "passed" else 1)


if __name__ == "__main__":
    main()
