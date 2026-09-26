#!/usr/bin/env python3
"""Exit nonzero unless a headless RGB-D capture report passed validation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path, help="capture output/validation_report.json")
    parser.add_argument("--min-usable-fraction", type=float, default=None,
                        help="override minimum usable/attempted frame ratio, including older reports")
    args = parser.parse_args()
    if args.min_usable_fraction is not None and not 0 <= args.min_usable_fraction <= 1:
        parser.error("--min-usable-fraction must be between 0 and 1")
    try:
        report = json.loads(args.report.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        print(f"capture report unavailable: {error}")
        return 1
    status = report.get("status")
    episodes = report.get("episodes", [])
    threshold = (args.min_usable_fraction if args.min_usable_fraction is not None else
                 report.get("run", {}).get("min_usable_frame_fraction") or 0.0)
    quality_passed = bool(episodes)
    for item in episodes:
        usable = item.get("usable_frames", 0)
        attempted = item.get("saved_frames", 0) + item.get("skipped_frames", 0)
        fraction = usable / max(1, attempted)
        quality_passed = quality_passed and fraction >= threshold
        print(f"episode {item.get('episode')}: usable {usable}/{attempted}, "
              f"fraction={fraction:.1%}, stop={item.get('stop_reason')}")
    verdict = "PASS" if status == "PASS" and quality_passed else "FAIL"
    print(f"capture verdict: {verdict}; report status: {status}; "
          f"usable threshold: {threshold:.1%}; report: {args.report}")
    if report.get("error"):
        print(f"capture error: {report['error']}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
