#!/usr/bin/env python3
"""
Engineering audit script for repository-level software quality checks.

It performs lightweight static analysis focused on maintainability,
reproducibility, and operational safety, then outputs a prioritized report.

Usage:
  python3 src/scripts/engineering_audit.py --root /path/to/repo
  python3 src/scripts/engineering_audit.py --root . --format json
"""

from __future__ import annotations

import argparse
import json
import re
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional


SEVERITY_ORDER = {"HIGH": 0, "MEDIUM": 1, "LOW": 2}

TEXT_EXTENSIONS = {
    ".py",
    ".sh",
    ".md",
    ".txt",
    ".yaml",
    ".yml",
    ".xml",
    ".toml",
    ".cfg",
    ".ini",
    ".xacro",
    ".launch",
}

IGNORE_DIRS = {
    ".git",
    ".idea",
    ".vscode",
    "__pycache__",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "build",
    "install",
    "log",
    ".venv",
    "venv",
    "node_modules",
}
NON_FIRST_PARTY_DIRS = {"third_party", "data", "archive"}

REQUIREMENT_OPERATOR_RE = re.compile(r"(==|>=|<=|~=|>|<|!=)")
ABSOLUTE_PATH_RE = re.compile(
    r"(/home/[A-Za-z0-9._-]+/[^\s\"'`]+|/Users/[A-Za-z0-9._-]+/[^\s\"'`]+)"
)
TODO_RE = re.compile(r"\b(TODO|FIXME|HACK|XXX)\b")
SUBPROCESS_SHELL_RE = re.compile(r"subprocess\.(run|Popen)\(.*shell\s*=\s*True")
BARE_EXCEPT_RE = re.compile(r"^\s*except\s*:\s*$")
GENERIC_EXCEPTION_RE = re.compile(r"^\s*except\s+Exception(\s+as\s+\w+)?\s*:\s*$")


@dataclass
class Finding:
    severity: str
    category: str
    message: str
    recommendation: str
    file: Optional[str] = None
    line: Optional[int] = None


class EngineeringAnalyzer:
    def __init__(self, root: Path, include_non_first_party: bool = False):
        self.root = root.resolve()
        self.ignore_dirs = set(IGNORE_DIRS)
        if not include_non_first_party:
            self.ignore_dirs |= NON_FIRST_PARTY_DIRS
        self.findings: list[Finding] = []
        self.ext_counter: Counter[str] = Counter()
        self.file_counter = 0
        self.python_files: list[Path] = []
        self.shell_files: list[Path] = []
        self.test_files: list[Path] = []
        self.package_xml_files: list[Path] = []

    def add_finding(
        self,
        *,
        severity: str,
        category: str,
        message: str,
        recommendation: str,
        file: Optional[Path] = None,
        line: Optional[int] = None,
    ) -> None:
        rel = None
        if file is not None:
            rel = str(file.relative_to(self.root))
        self.findings.append(
            Finding(
                severity=severity,
                category=category,
                message=message,
                recommendation=recommendation,
                file=rel,
                line=line,
            )
        )

    def iter_files(self) -> Iterable[Path]:
        for path in self.root.rglob("*"):
            if not path.is_file():
                continue
            if any(part in self.ignore_dirs for part in path.parts):
                continue
            yield path

    def read_text(self, path: Path) -> Optional[str]:
        try:
            return path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            return None
        except OSError:
            return None

    def analyze(self) -> None:
        files = list(self.iter_files())
        self.file_counter = len(files)

        for path in files:
            ext = path.suffix.lower()
            self.ext_counter[ext] += 1

            lower_name = path.name.lower()
            if lower_name.startswith("test_") and ext == ".py":
                self.test_files.append(path)
            if ext == ".py":
                self.python_files.append(path)
            if ext == ".sh":
                self.shell_files.append(path)
            if path.name == "package.xml":
                self.package_xml_files.append(path)

        self.check_repo_basics()
        self.check_requirements()
        self.check_python_files()
        self.check_shell_files()
        self.check_ros_packages()
        self.check_test_strategy()
        self.check_orphan_docs_vs_structure()
        self.check_log_files_in_source()

    def check_repo_basics(self) -> None:
        if not (self.root / "README.md").exists():
            self.add_finding(
                severity="MEDIUM",
                category="documentation",
                message="README.md is missing.",
                recommendation="Add a README with setup, run, and validation instructions.",
            )
        if not (self.root / ".gitignore").exists():
            self.add_finding(
                severity="MEDIUM",
                category="repo-hygiene",
                message=".gitignore is missing.",
                recommendation="Create a .gitignore tailored for ROS, Python, and generated artifacts.",
            )

        workflows = list((self.root / ".github" / "workflows").glob("*.yml"))
        workflows += list((self.root / ".github" / "workflows").glob("*.yaml"))
        if not workflows:
            self.add_finding(
                severity="LOW",
                category="automation",
                message="No CI workflow detected in .github/workflows.",
                recommendation="Add a minimal CI pipeline for linting and smoke tests.",
            )

    def check_requirements(self) -> None:
        req_path = self.root / "requirements.txt"
        if not req_path.exists():
            return
        text = self.read_text(req_path)
        if text is None:
            return

        for idx, raw in enumerate(text.splitlines(), start=1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("-r "):
                continue
            if not REQUIREMENT_OPERATOR_RE.search(line):
                self.add_finding(
                    severity="MEDIUM",
                    category="dependency-management",
                    message=f"Unpinned dependency '{line}' in requirements.txt.",
                    recommendation="Pin versions (e.g., package==x.y.z) for reproducible environments.",
                    file=req_path,
                    line=idx,
                )

    def check_python_files(self) -> None:
        for path in self.python_files:
            text = self.read_text(path)
            if text is None:
                continue

            lines = text.splitlines()
            for i, line in enumerate(lines, start=1):
                if BARE_EXCEPT_RE.search(line):
                    self.add_finding(
                        severity="HIGH",
                        category="error-handling",
                        message="Bare 'except:' swallows all errors.",
                        recommendation="Catch specific exceptions and log actionable context.",
                        file=path,
                        line=i,
                    )
                if GENERIC_EXCEPTION_RE.search(line):
                    self.add_finding(
                        severity="MEDIUM",
                        category="error-handling",
                        message="Generic 'except Exception' used.",
                        recommendation="Narrow exception types where possible and preserve traceability.",
                        file=path,
                        line=i,
                    )
                if "sys.path.append(" in line:
                    sev = "HIGH" if ABSOLUTE_PATH_RE.search(line) else "MEDIUM"
                    self.add_finding(
                        severity=sev,
                        category="packaging",
                        message="Runtime sys.path modification detected.",
                        recommendation="Package code as installable modules and use normal imports.",
                        file=path,
                        line=i,
                    )
                if ABSOLUTE_PATH_RE.search(line):
                    self.add_finding(
                        severity="HIGH",
                        category="portability",
                        message="Hardcoded absolute filesystem path detected.",
                        recommendation="Use config/env vars and pathlib-based relative path resolution.",
                        file=path,
                        line=i,
                    )
                if TODO_RE.search(line):
                    self.add_finding(
                        severity="LOW",
                        category="maintainability",
                        message="Outstanding TODO/FIXME marker found.",
                        recommendation="Track unresolved work in issues and keep code comments current.",
                        file=path,
                        line=i,
                    )
                if len(line) > 140:
                    self.add_finding(
                        severity="LOW",
                        category="readability",
                        message="Very long line (>140 chars) reduces readability.",
                        recommendation="Wrap long expressions/logs for easier review and diffs.",
                        file=path,
                        line=i,
                    )

            if SUBPROCESS_SHELL_RE.search(text):
                self.add_finding(
                    severity="MEDIUM",
                    category="security",
                    message="subprocess shell=True detected.",
                    recommendation="Prefer list-arg subprocess calls; avoid shell=True unless essential.",
                    file=path,
                )

    def check_shell_files(self) -> None:
        for path in self.shell_files:
            text = self.read_text(path)
            if text is None:
                continue

            if "set -e" not in text:
                self.add_finding(
                    severity="LOW",
                    category="shell-robustness",
                    message="Shell script does not enable fail-fast mode (set -e).",
                    recommendation="Use set -euo pipefail (or equivalent safeguards) when safe.",
                    file=path,
                )

            for i, line in enumerate(text.splitlines(), start=1):
                if re.search(r"\bpkill\s+-9\b", line):
                    self.add_finding(
                        severity="HIGH",
                        category="operational-safety",
                        message="Force-kill command (pkill -9) detected.",
                        recommendation="Prefer graceful shutdown first, then targeted fallback kill.",
                        file=path,
                        line=i,
                    )
                if re.search(r"\brm\s+-rf\s+", line):
                    self.add_finding(
                        severity="MEDIUM",
                        category="operational-safety",
                        message="Potentially destructive rm -rf usage detected.",
                        recommendation="Add path guards and explicit checks before recursive deletion.",
                        file=path,
                        line=i,
                    )
                if ABSOLUTE_PATH_RE.search(line):
                    self.add_finding(
                        severity="MEDIUM",
                        category="portability",
                        message="Hardcoded absolute filesystem path detected in shell script.",
                        recommendation="Use script-relative paths or environment variables.",
                        file=path,
                        line=i,
                    )

    def check_ros_packages(self) -> None:
        for package_xml in self.package_xml_files:
            text = self.read_text(package_xml)
            if text is None:
                continue

            try:
                root = ET.fromstring(text)
            except ET.ParseError:
                self.add_finding(
                    severity="HIGH",
                    category="ros-packaging",
                    message="package.xml is not valid XML.",
                    recommendation="Fix XML syntax to avoid build/metadata failures.",
                    file=package_xml,
                )
                continue

            all_dep_tags = [
                "depend",
                "build_depend",
                "buildtool_depend",
                "exec_depend",
                "test_depend",
            ]
            seen = defaultdict(list)
            for tag in all_dep_tags:
                for el in root.findall(tag):
                    if el.text:
                        dep = el.text.strip()
                        seen[dep].append(tag)

            for dep, tags in seen.items():
                if len(tags) > 1 and len(set(tags)) == 1:
                    self.add_finding(
                        severity="LOW",
                        category="ros-packaging",
                        message=f"Duplicate dependency '{dep}' appears multiple times as <{tags[0]}>.",
                        recommendation="Remove duplicate entries from package.xml.",
                        file=package_xml,
                    )

        for cmake in self.root.rglob("CMakeLists.txt"):
            if any(part in self.ignore_dirs for part in cmake.parts):
                continue
            text = self.read_text(cmake)
            if text is None:
                continue
            if "if(BUILD_TESTING)" in text and "ament_lint_auto_find_test_dependencies" in text:
                if "ament_add" not in text and "add_rostest" not in text and "add_test(" not in text:
                    self.add_finding(
                        severity="LOW",
                        category="testing",
                        message="BUILD_TESTING enabled but no explicit package tests detected in CMakeLists.",
                        recommendation="Add at least one package-level smoke/integration test target.",
                        file=cmake,
                    )

    def check_test_strategy(self) -> None:
        if not self.test_files:
            self.add_finding(
                severity="MEDIUM",
                category="testing",
                message="No Python test files detected.",
                recommendation="Add tests covering critical navigation/manipulation behaviors.",
            )
            return

        has_pytest = (self.root / "pytest.ini").exists() or (self.root / "pyproject.toml").exists()
        if not has_pytest:
            self.add_finding(
                severity="LOW",
                category="testing",
                message="Test scripts exist but no standard test runner configuration detected.",
                recommendation="Adopt pytest/colcon test configuration for repeatable automated execution.",
            )

    def check_orphan_docs_vs_structure(self) -> None:
        readme = self.root / "README.md"
        text = self.read_text(readme)
        if text is None:
            return

        expected = ["third_party", "data", "docs", "archive"]
        missing = [name for name in expected if not (self.root / name).exists()]
        if missing:
            self.add_finding(
                severity="LOW",
                category="documentation",
                message=f"README describes directories not present in repository: {', '.join(missing)}.",
                recommendation="Update README tree to reflect current state or add missing directories.",
                file=readme,
            )

    def check_log_files_in_source(self) -> None:
        for path in self.iter_files():
            if path.suffix.lower() == ".log":
                self.add_finding(
                    severity="LOW",
                    category="repo-hygiene",
                    message="Log artifact tracked in repository.",
                    recommendation="Move runtime logs under ignored directories and update .gitignore.",
                    file=path,
                )

    def sorted_findings(self) -> list[Finding]:
        return sorted(
            self.findings,
            key=lambda f: (SEVERITY_ORDER.get(f.severity, 99), f.category, f.file or "", f.line or 0),
        )

    def render_markdown(self) -> str:
        findings = self.sorted_findings()
        by_severity: dict[str, list[Finding]] = {"HIGH": [], "MEDIUM": [], "LOW": []}
        for finding in findings:
            by_severity[finding.severity].append(finding)

        top_ext = ", ".join(
            f"{ext or '[no-ext]'}={count}" for ext, count in self.ext_counter.most_common(8)
        )
        lines = [
            "# Engineering Audit Report",
            "",
            f"- Root: `{self.root}`",
            f"- Files scanned: **{self.file_counter}**",
            f"- Python files: **{len(self.python_files)}**",
            f"- Shell files: **{len(self.shell_files)}**",
            f"- Detected test files: **{len(self.test_files)}**",
            f"- Top extensions: {top_ext if top_ext else 'n/a'}",
            "",
            "## Findings by severity",
        ]

        for sev in ("HIGH", "MEDIUM", "LOW"):
            group = by_severity[sev]
            lines.append(f"### {sev} ({len(group)})")
            if not group:
                lines.append("- None")
                continue
            for idx, f in enumerate(group, start=1):
                loc = ""
                if f.file:
                    loc = f"`{f.file}`"
                    if f.line:
                        loc += f":{f.line}"
                location_suffix = f" — {loc}" if loc else ""
                lines.append(f"{idx}. **[{f.category}]** {f.message}{location_suffix}")
                lines.append(f"   - Fix: {f.recommendation}")

        return "\n".join(lines) + "\n"

    def render_json(self) -> str:
        payload = {
            "root": str(self.root),
            "files_scanned": self.file_counter,
            "python_files": len(self.python_files),
            "shell_files": len(self.shell_files),
            "test_files": len(self.test_files),
            "extensions": dict(self.ext_counter),
            "findings": [asdict(f) for f in self.sorted_findings()],
        }
        return json.dumps(payload, indent=2)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Static engineering analyzer for repositories.")
    parser.add_argument("--root", type=Path, default=Path("."), help="Repository root path.")
    parser.add_argument(
        "--include-non-first-party",
        action="store_true",
        help="Include third_party, data, and archive directories in analysis.",
    )
    parser.add_argument(
        "--format",
        choices=("markdown", "json"),
        default="markdown",
        help="Report output format.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    analyzer = EngineeringAnalyzer(
        args.root,
        include_non_first_party=args.include_non_first_party,
    )
    analyzer.analyze()
    if args.format == "json":
        print(analyzer.render_json())
    else:
        print(analyzer.render_markdown())


if __name__ == "__main__":
    main()
