#!/usr/bin/env python3
"""Extract and group BlueFrog/JURA 0x26 frames from decoded Jutta logs."""

from __future__ import annotations

import argparse
import csv
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path


HEX_RE = re.compile(r"\bhex=(?:\"([0-9A-Fa-f ]+)\"|([0-9A-Fa-f ]+))")
LEN_RE = re.compile(r"\blen=(\d+)")
TS_RE = re.compile(r"^\[?(\d{2}:\d{2}:\d{2}(?:[.,]\d{1,6})?)\]?")


@dataclass
class Frame:
    index: int
    line_no: int
    timestamp: str
    time_ms: int | None
    direction: str
    length: int
    hex_text: str
    ascii_preview: str
    cluster_key: str
    cluster_id: int = 0
    sequence_id: int = 0
    sequence_role: str = ""
    delta_ms: int | None = None


def parse_time_ms(text: str) -> int | None:
    match = TS_RE.search(text)
    if not match:
        return None
    ts = match.group(1).replace(",", ".")
    hh, mm, ss = ts.split(":")
    sec = float(ss)
    return int(hh) * 3_600_000 + int(mm) * 60_000 + int(sec * 1000)


def ascii_preview(raw: bytes) -> str:
    out = []
    for byte in raw:
        if 32 <= byte <= 126:
            out.append(chr(byte))
        elif byte in (9, 10, 13):
            out.append(".")
        else:
            out.append(".")
    return "".join(out)


def direction_from_line(line: str) -> str | None:
    if "DONGLE_TO_MACHINE" in line:
        return "dongle_to_machine"
    if "MACHINE_TO_DONGLE" in line:
        return "machine_to_dongle"
    return None


def parse_hex(match: re.Match[str]) -> bytes:
    value = match.group(1) or match.group(2) or ""
    return bytes(int(part, 16) for part in value.split())


def cluster_key_for(raw: bytes) -> str:
    prefix = raw[:10].hex(" ").upper()
    return f"len={len(raw)} prefix={prefix}"


def extract_frames(path: Path) -> list[Frame]:
    frames: list[Frame] = []
    for line_no, line in enumerate(path.read_text(errors="replace").splitlines(), start=1):
        direction = direction_from_line(line)
        if direction is None:
            continue
        hex_match = HEX_RE.search(line)
        if not hex_match:
            continue
        raw = parse_hex(hex_match)
        if not raw or raw[0] != 0x26:
            continue
        length_match = LEN_RE.search(line)
        frames.append(
            Frame(
                index=len(frames) + 1,
                line_no=line_no,
                timestamp=TS_RE.search(line).group(1) if TS_RE.search(line) else "",
                time_ms=parse_time_ms(line),
                direction=direction,
                length=int(length_match.group(1)) if length_match else len(raw),
                hex_text=raw.hex(" ").upper(),
                ascii_preview=ascii_preview(raw),
                cluster_key=cluster_key_for(raw),
            )
        )
    return frames


def assign_clusters(frames: list[Frame]) -> None:
    cluster_ids: dict[str, int] = {}
    for frame in frames:
        if frame.cluster_key not in cluster_ids:
            cluster_ids[frame.cluster_key] = len(cluster_ids) + 1
        frame.cluster_id = cluster_ids[frame.cluster_key]


def assign_sequences(frames: list[Frame], response_window_ms: int) -> None:
    current_sequence = 0
    request_time: int | None = None
    request_seen = False
    last_time: int | None = None

    for frame in frames:
        if last_time is not None and frame.time_ms is not None and frame.time_ms < last_time:
            # Midnight/log rollover: keep ordering stable and drop delta-based grouping for that boundary.
            request_time = None
        last_time = frame.time_ms if frame.time_ms is not None else last_time

        if frame.direction == "dongle_to_machine":
            current_sequence += 1
            request_seen = True
            request_time = frame.time_ms
            frame.sequence_id = current_sequence
            frame.sequence_role = "request_or_write"
            frame.delta_ms = 0
            continue

        if request_seen and request_time is not None and frame.time_ms is not None:
            delta = frame.time_ms - request_time
            if 0 <= delta <= response_window_ms:
                frame.sequence_id = current_sequence
                frame.delta_ms = delta
                frame.sequence_role = "machine_response_or_status_update" if delta <= 500 else "post_request_status"
                continue

        frame.sequence_id = 0
        frame.sequence_role = "periodic_or_unsolicited_machine_frame"
        frame.delta_ms = None


def write_csv(frames: list[Frame], path: Path) -> None:
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "index",
                "line_no",
                "timestamp",
                "direction",
                "length",
                "hex",
                "ascii_preview",
                "cluster_id",
                "cluster_key",
                "sequence_id",
                "sequence_role",
                "delta_ms",
            ],
        )
        writer.writeheader()
        for frame in frames:
            writer.writerow(
                {
                    "index": frame.index,
                    "line_no": frame.line_no,
                    "timestamp": frame.timestamp,
                    "direction": frame.direction,
                    "length": frame.length,
                    "hex": frame.hex_text,
                    "ascii_preview": frame.ascii_preview,
                    "cluster_id": frame.cluster_id,
                    "cluster_key": frame.cluster_key,
                    "sequence_id": frame.sequence_id,
                    "sequence_role": frame.sequence_role,
                    "delta_ms": "" if frame.delta_ms is None else frame.delta_ms,
                }
            )


def write_summary(frames: list[Frame], path: Path) -> None:
    clusters = Counter(frame.cluster_id for frame in frames)
    by_direction = Counter(frame.direction for frame in frames)
    by_role = Counter(frame.sequence_role for frame in frames)
    examples: dict[int, Frame] = {}
    for frame in frames:
        examples.setdefault(frame.cluster_id, frame)

    with path.open("w") as handle:
        handle.write("# Jutta 0x26 Sequence Analysis\n\n")
        handle.write(f"frames_total={len(frames)}\n\n")
        handle.write("## Direction Counts\n\n")
        for key, count in sorted(by_direction.items()):
            handle.write(f"- {key}: {count}\n")
        handle.write("\n## Sequence Role Counts\n\n")
        for key, count in sorted(by_role.items()):
            handle.write(f"- {key}: {count}\n")
        handle.write("\n## Clusters\n\n")
        for cluster_id, count in clusters.most_common():
            sample = examples[cluster_id]
            handle.write(
                f"- cluster={cluster_id} count={count} {sample.cluster_key} "
                f"sample_direction={sample.direction} sample_line={sample.line_no}\n"
            )
        handle.write("\n## Notes\n\n")
        handle.write(
            "- `request_or_write` marks Dongle-to-Machine 0x26 frames.\n"
            "- Machine frames within the response window after a request are grouped as possible ACK/status updates.\n"
            "- Machine frames outside request windows are marked periodic/unsolicited candidates.\n"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="Decoded Jutta log, e.g. jura_jutta_decoded_resync.log")
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--summary", type=Path, default=None)
    parser.add_argument("--response-window-ms", type=int, default=1500)
    args = parser.parse_args()

    if not args.log.exists():
        raise SystemExit(f"log not found: {args.log}")

    frames = extract_frames(args.log)
    assign_clusters(frames)
    assign_sequences(frames, args.response_window_ms)

    csv_path = args.csv or args.log.with_name(args.log.stem + "_26_frames.csv")
    summary_path = args.summary or args.log.with_name(args.log.stem + "_26_sequences.md")
    write_csv(frames, csv_path)
    write_summary(frames, summary_path)
    print(f"frames={len(frames)} csv={csv_path} summary={summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
