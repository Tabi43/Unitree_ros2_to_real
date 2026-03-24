#!/usr/bin/env python3
"""
Build a Kalibr camera-only dataset directly from the custom UDP camera stream used in the
provided udp_camera_interface package.

The script listens for UDP datagrams carrying chunked JPEG frames with the header format
implemented in the provided C++ receiver/sender. Each completed frame is decoded, split as a
side-by-side stereo image, downsampled in time according to a user-provided target rate, and
saved as:

    <output_dir>/cam0/<timestamp_ns>.png
    <output_dir>/cam1/<timestamp_ns>.png

This directory layout is intentionally compatible with Kalibr's bagcreater workflow.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import os
import shutil
import signal
import socket
import select
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

HEADER_FMT = "!I H H Q I I I I I I Q"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAGIC = 0x55494D47  # 'UIMG'
VERSION = 1
MAX_JPEG_SIZE = 50 * 1024 * 1024

STOP = False

def _signal_handler(signum, frame):
    del signum, frame
    global STOP
    STOP = True


@dataclasses.dataclass
class FrameAssembly:
    frame_id: int
    stream_id: int
    width: int
    height: int
    chunk_count: int
    jpeg_size: int
    stamp_ns: int
    first_seen_ns: int
    chunks: List[Optional[bytes]]
    received: List[bool]
    received_count: int = 0

    @classmethod
    def create(
        cls,
        *,
        frame_id: int,
        stream_id: int,
        width: int,
        height: int,
        chunk_count: int,
        jpeg_size: int,
        stamp_ns: int,
        now_ns: int,
    ) -> "FrameAssembly":
        return cls(
            frame_id=frame_id,
            stream_id=stream_id,
            width=width,
            height=height,
            chunk_count=chunk_count,
            jpeg_size=jpeg_size,
            stamp_ns=stamp_ns,
            first_seen_ns=now_ns,
            chunks=[None] * chunk_count,
            received=[False] * chunk_count,
            received_count=0,
        )

    def complete(self) -> bool:
        return self.chunk_count > 0 and self.received_count == self.chunk_count


@dataclasses.dataclass
class Stats:
    datagrams_rx: int = 0
    frames_completed: int = 0
    frames_decoded: int = 0
    frames_saved: int = 0
    frames_skipped_rate: int = 0
    frames_dropped_timeout: int = 0
    frames_dropped_inflight: int = 0
    frames_dropped_malformed: int = 0
    frames_dropped_decode: int = 0


class TimestampMapper:
    def __init__(self, mode: str):
        self.mode = mode
        self.offset_ns: Optional[int] = None
        self.last_out_ns: Optional[int] = None

    def map(self, sender_stamp_ns: int, receive_time_ns: int) -> int:
        if self.mode == "sender":
            out = sender_stamp_ns
        elif self.mode == "receive":
            out = receive_time_ns
        elif self.mode == "sender_to_wall":
            if self.offset_ns is None:
                self.offset_ns = receive_time_ns - sender_stamp_ns
            out = sender_stamp_ns + self.offset_ns
        else:
            raise ValueError(f"Unsupported timestamp mode: {self.mode}")

        # Ensure strictly increasing and unique filenames.
        if self.last_out_ns is not None and out <= self.last_out_ns:
            out = self.last_out_ns + 1
        self.last_out_ns = out
        return out


class KalibrDatasetWriter:
    def __init__(
        self,
        output_dir: Path,
        left_dir_name: str,
        right_dir_name: str,
        png_compression: int,
        overwrite: bool,
        writer_threads: int = 4,
    ) -> None:
        self.output_dir = output_dir
        self.left_dir = output_dir / left_dir_name
        self.right_dir = output_dir / right_dir_name
        self.png_params = [cv2.IMWRITE_PNG_COMPRESSION, int(png_compression)]
        self._pool = concurrent.futures.ThreadPoolExecutor(max_workers=writer_threads)
        self._pending: list[concurrent.futures.Future] = []
        self._lock = threading.Lock()
        self._write_errors = 0

        self._prepare_dirs(overwrite=overwrite)

    def _prepare_dirs(self, overwrite: bool) -> None:
        if self.output_dir.exists() and overwrite:
            shutil.rmtree(self.output_dir)

        self.left_dir.mkdir(parents=True, exist_ok=True)
        self.right_dir.mkdir(parents=True, exist_ok=True)

        if not overwrite:
            left_nonempty = any(self.left_dir.iterdir())
            right_nonempty = any(self.right_dir.iterdir())
            if left_nonempty or right_nonempty:
                raise RuntimeError(
                    f"Output directory '{self.output_dir}' already contains data. "
                    "Use --overwrite to replace it."
                )

    def _write_pair_sync(
        self, timestamp_ns: int, left_img: np.ndarray, right_img: np.ndarray
    ) -> None:
        left_path = str(self.left_dir / f"{timestamp_ns}.png")
        right_path = str(self.right_dir / f"{timestamp_ns}.png")
        if not cv2.imwrite(left_path, left_img, self.png_params):
            with self._lock:
                self._write_errors += 1
            print(f"warning: failed to save {left_path}", file=sys.stderr, flush=True)
            return
        if not cv2.imwrite(right_path, right_img, self.png_params):
            with self._lock:
                self._write_errors += 1
            print(f"warning: failed to save {right_path}", file=sys.stderr, flush=True)

    def save_pair(self, timestamp_ns: int, left_img: np.ndarray, right_img: np.ndarray) -> None:
        # Prune completed futures periodically to avoid unbounded list growth.
        if len(self._pending) > 256:
            self._pending = [f for f in self._pending if not f.done()]
        fut = self._pool.submit(self._write_pair_sync, timestamp_ns, left_img, right_img)
        self._pending.append(fut)

    @property
    def pending_count(self) -> int:
        return sum(1 for f in self._pending if not f.done())

    @property
    def write_errors(self) -> int:
        with self._lock:
            return self._write_errors

    def drain(self, timeout: float = 30.0) -> None:
        """Wait for all pending writes to finish, then shut down the pool."""
        concurrent.futures.wait(self._pending, timeout=timeout)
        self._pool.shutdown(wait=True)


class UdpKalibrDatasetBuilder:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.stats = Stats()
        self.inflight: Dict[int, FrameAssembly] = {}
        self.last_saved_sender_stamp_ns: Optional[int] = None
        self.timestamp_mapper = TimestampMapper(args.timestamp_source)
        self.writer = KalibrDatasetWriter(
            output_dir=Path(args.output_dir),
            left_dir_name=args.left_dir_name,
            right_dir_name=args.right_dir_name,
            png_compression=args.png_compression,
            overwrite=args.overwrite,
            writer_threads=args.writer_threads,
        )
        self.sample_period_ns = 0 if args.sample_rate_hz <= 0 else int(round(1e9 / args.sample_rate_hz))
        self.sock = self._open_socket()
        self.last_stats_print_ns = time.monotonic_ns()
        self.show_frame_enabled = bool(args.show_frame)
        self.show_window_name = "udp_kalibr_full_frame"
        self.show_window_initialized = False
        self.recording_started = False

    def _open_socket(self) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(self.args.recv_buffer_bytes))
        sock.bind((self.args.bind_ip, int(self.args.port)))
        sock.settimeout(float(self.args.poll_timeout_s))
        return sock

    def run(self) -> None:
        print(
            f"Listening on {self.args.bind_ip}:{self.args.port} "
            f"stream_id={self.args.stream_id} -> {self.writer.output_dir}",
            flush=True,
        )
        if self.sample_period_ns > 0:
            print(
                f"Temporal downsampling enabled: {self.args.sample_rate_hz:.6g} Hz "
                f"(period {self.sample_period_ns} ns)",
                flush=True,
            )
        else:
            print("Temporal downsampling disabled: every completed frame will be saved.", flush=True)

        try:
            self._print_startup_instructions()
            while not STOP:
                self._poll_terminal_start()
                self._recv_batch()
                self._cleanup_inflight()
                self._maybe_print_stats()
                if self.args.max_frames > 0 and self.stats.frames_saved >= self.args.max_frames:
                    print(f"Reached max saved frame pairs: {self.args.max_frames}", flush=True)
                    break
        finally:
            self.sock.close()
            pending = self.writer.pending_count
            if pending > 0:
                print(f"Draining {pending} pending writes...", flush=True)
            self.writer.drain()
            if self.show_window_initialized:
                try:
                    cv2.destroyAllWindows()
                except cv2.error:
                    pass
            self._print_final_stats()


    def _print_startup_instructions(self) -> None:
        print(
            "Recording is armed but paused. Press Enter, space, or 'r' to start saving stereo pairs.",
            flush=True,
        )
        if self.show_frame_enabled:
            print(
                f"Give focus to the '{self.show_window_name}' window and press Enter/space/r to start. "
                "Press q or Esc to quit.",
                flush=True,
            )
        else:
            print("Press Enter in this terminal to start. Press Ctrl+C to quit.", flush=True)

    def _start_recording(self, source: str) -> None:
        if self.recording_started:
            return
        self.recording_started = True
        print(f"Recording started from {source}.", flush=True)

    def _poll_terminal_start(self) -> None:
        if self.recording_started or self.show_frame_enabled:
            return

        try:
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        except (ValueError, OSError):
            return

        if ready:
            try:
                _ = sys.stdin.readline()
            except Exception:
                return
            self._start_recording("terminal")

    def _recv_batch(self) -> None:
        """Drain all immediately available datagrams from the socket."""
        batch_limit = 512
        for _ in range(batch_limit):
            try:
                packet, _addr = self.sock.recvfrom(65536)
            except socket.timeout:
                return

            receive_time_ns = time.time_ns()
            now_mono_ns = time.monotonic_ns()
            self.stats.datagrams_rx += 1

            if len(packet) < HEADER_SIZE:
                self.stats.frames_dropped_malformed += 1
                continue

            try:
                (
                    magic,
                    version,
                    stream_id,
                    frame_id,
                    chunk_idx,
                    chunk_count,
                    payload_size,
                    jpeg_size,
                    width,
                    height,
                    stamp_ns,
                ) = struct.unpack_from(HEADER_FMT, packet, 0)
            except struct.error:
                self.stats.frames_dropped_malformed += 1
                continue

            if magic != MAGIC or version != VERSION:
                self.stats.frames_dropped_malformed += 1
                continue
            if stream_id != self.args.stream_id:
                continue
            if chunk_count == 0 or chunk_idx >= chunk_count:
                self.stats.frames_dropped_malformed += 1
                continue
            if jpeg_size <= 0 or jpeg_size > MAX_JPEG_SIZE:
                self.stats.frames_dropped_malformed += 1
                continue
            if HEADER_SIZE + payload_size > len(packet):
                self.stats.frames_dropped_malformed += 1
                continue

            if len(self.inflight) > self.args.max_inflight:
                oldest_frame_id = min(self.inflight.keys())
                del self.inflight[oldest_frame_id]
                self.stats.frames_dropped_inflight += 1

            assembly = self.inflight.get(frame_id)
            if assembly is None:
                assembly = FrameAssembly.create(
                    frame_id=frame_id,
                    stream_id=stream_id,
                    width=width,
                    height=height,
                    chunk_count=chunk_count,
                    jpeg_size=jpeg_size,
                    stamp_ns=stamp_ns,
                    now_ns=now_mono_ns,
                )
                self.inflight[frame_id] = assembly
            else:
                if (
                    assembly.chunk_count != chunk_count
                    or assembly.jpeg_size != jpeg_size
                    or assembly.width != width
                    or assembly.height != height
                ):
                    del self.inflight[frame_id]
                    self.stats.frames_dropped_malformed += 1
                    continue

            if not assembly.received[chunk_idx]:
                start = HEADER_SIZE
                end = HEADER_SIZE + payload_size
                assembly.chunks[chunk_idx] = packet[start:end]
                assembly.received[chunk_idx] = True
                assembly.received_count += 1

            if assembly.complete():
                self.stats.frames_completed += 1
                del self.inflight[frame_id]
                self._process_complete_frame(assembly, receive_time_ns)

    def _process_complete_frame(self, assembly: FrameAssembly, receive_time_ns: int) -> None:
        jpeg = b"".join(chunk for chunk in assembly.chunks if chunk is not None)
        if len(jpeg) != assembly.jpeg_size:
            self.stats.frames_dropped_malformed += 1
            return

        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if img is None or img.size == 0:
            self.stats.frames_dropped_decode += 1
            return

        self.stats.frames_decoded += 1
        self._maybe_show_frame(img, assembly)

        if not self.recording_started:
            return

        if self.sample_period_ns > 0:
            if self.last_saved_sender_stamp_ns is not None:
                if assembly.stamp_ns - self.last_saved_sender_stamp_ns < self.sample_period_ns:
                    self.stats.frames_skipped_rate += 1
                    return

        left_img, right_img = self._split_stereo_sbs(img)
        timestamp_ns = self.timestamp_mapper.map(assembly.stamp_ns, receive_time_ns)
        self.writer.save_pair(timestamp_ns, left_img, right_img)

        self.last_saved_sender_stamp_ns = assembly.stamp_ns
        self.stats.frames_saved += 1

        if self.args.verbose_each_save:
            print(
                f"saved pair #{self.stats.frames_saved}: frame_id={assembly.frame_id} "
                f"sender_stamp_ns={assembly.stamp_ns} dataset_stamp_ns={timestamp_ns}",
                flush=True,
            )

    def _maybe_show_frame(self, img: np.ndarray, assembly: FrameAssembly) -> None:
        if not self.show_frame_enabled:
            return

        global STOP

        try:
            if not self.show_window_initialized:
                cv2.namedWindow(self.show_window_name, cv2.WINDOW_NORMAL)
                self.show_window_initialized = True

            display_img = img
            if img.ndim == 2:
                display_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

            overlay = display_img.copy()
            text = (
                f"frame_id={assembly.frame_id} stamp_ns={assembly.stamp_ns} "
                f"saved={self.stats.frames_saved} decoded={self.stats.frames_decoded}"
            )
            cv2.putText(
                overlay,
                text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(self.show_window_name, overlay)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):
                STOP = True
            elif key in (13, 10, 32, ord('r'), ord('R')):
                self._start_recording('window keyboard')
        except cv2.error as exc:
            self.show_frame_enabled = False
            print(
                f"warning: disabling --show-frame because OpenCV HighGUI is unavailable: {exc}",
                file=sys.stderr,
                flush=True,
            )

    def _split_stereo_sbs(self, img: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        if img.ndim not in (2, 3):
            raise RuntimeError(f"Unsupported image shape: {img.shape}")

        height, width = img.shape[:2]
        single_width = int(self.args.single_width) if self.args.single_width > 0 else width // 2

        if single_width <= 0:
            raise RuntimeError(f"Invalid single_width={single_width} for image width={width}")
        if 2 * single_width > width:
            raise RuntimeError(
                f"Configured single_width={single_width} requires at least {2 * single_width} px, "
                f"but received frame width={width}"
            )
        if self.args.single_width <= 0 and width % 2 != 0:
            raise RuntimeError(
                f"Received odd full-frame width={width}; cannot infer equal left/right halves automatically. "
                "Pass --single-width explicitly."
            )

        first_half = img[:, 0:single_width]
        second_half = img[:, single_width:2 * single_width]

        if self.args.split_order == "left-right":
            left_img = first_half
            right_img = second_half
        elif self.args.split_order == "right-left":
            left_img = second_half
            right_img = first_half
        else:
            raise RuntimeError(f"Unsupported split order: {self.args.split_order}")

        # Clone before saving so the slices own their memory independently.
        return left_img.copy(), right_img.copy()

    def _cleanup_inflight(self) -> None:
        if not self.inflight:
            return
        now_ns = time.monotonic_ns()
        timeout_ns = int(self.args.frame_timeout_ms) * 1_000_000
        to_delete: List[int] = []
        for frame_id, assembly in self.inflight.items():
            if now_ns - assembly.first_seen_ns > timeout_ns:
                to_delete.append(frame_id)
        for frame_id in to_delete:
            del self.inflight[frame_id]
            self.stats.frames_dropped_timeout += 1

    def _maybe_print_stats(self) -> None:
        if self.args.stats_period_s <= 0:
            return
        now_ns = time.monotonic_ns()
        period_ns = int(self.args.stats_period_s * 1e9)
        if now_ns - self.last_stats_print_ns < period_ns:
            return
        self.last_stats_print_ns = now_ns
        print(
            "stats: "
            f"datagrams_rx={self.stats.datagrams_rx} "
            f"frames_completed={self.stats.frames_completed} "
            f"frames_decoded={self.stats.frames_decoded} "
            f"frames_saved={self.stats.frames_saved} "
            f"recording_started={int(self.recording_started)} "
            f"frames_skipped_rate={self.stats.frames_skipped_rate} "
            f"drop_timeout={self.stats.frames_dropped_timeout} "
            f"drop_inflight={self.stats.frames_dropped_inflight} "
            f"drop_malformed={self.stats.frames_dropped_malformed} "
            f"drop_decode={self.stats.frames_dropped_decode} "
            f"inflight={len(self.inflight)} "
            f"pending_writes={self.writer.pending_count}",
            flush=True,
        )

    def _print_final_stats(self) -> None:
        print(
            "final stats: "
            f"datagrams_rx={self.stats.datagrams_rx}, "
            f"frames_completed={self.stats.frames_completed}, "
            f"frames_decoded={self.stats.frames_decoded}, "
            f"frames_saved={self.stats.frames_saved}, "
            f"recording_started={int(self.recording_started)}, "
            f"frames_skipped_rate={self.stats.frames_skipped_rate}, "
            f"drop_timeout={self.stats.frames_dropped_timeout}, "
            f"drop_inflight={self.stats.frames_dropped_inflight}, "
            f"drop_malformed={self.stats.frames_dropped_malformed}, "
            f"drop_decode={self.stats.frames_dropped_decode}, "
            f"write_errors={self.writer.write_errors}",
            flush=True,
        )

def positive_int(value: str) -> int:
    ivalue = int(value)
    if ivalue <= 0:
        raise argparse.ArgumentTypeError(f"Expected a positive integer, got {value}")
    return ivalue


def nonnegative_int(value: str) -> int:
    ivalue = int(value)
    if ivalue < 0:
        raise argparse.ArgumentTypeError(f"Expected a non-negative integer, got {value}")
    return ivalue


def nonnegative_float(value: str) -> float:
    fvalue = float(value)
    if fvalue < 0:
        raise argparse.ArgumentTypeError(f"Expected a non-negative float, got {value}")
    return fvalue


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "Receive full side-by-side stereo frames over the udp_camera_interface UDP protocol "
            "and save a Kalibr-compatible camera-only dataset (cam0/cam1 PNG files)."
        )
    )

    p.add_argument("--bind-ip", default="0.0.0.0", help="Local IP address to bind (default: 0.0.0.0)")
    p.add_argument("--port", type=positive_int, default=5000, help="UDP port to bind (default: 5000)")
    p.add_argument(
        "--stream-id",
        type=nonnegative_int,
        default=0,
        help="Expected stream id in the UDP header (default: 0)",
    )

    p.add_argument(
        "--output-dir",
        required=True,
        help="Output Kalibr dataset directory. The script will create cam0/ and cam1/ inside it.",
    )
    p.add_argument(
        "--sample-rate-hz",
        type=nonnegative_float,
        default=0,
        help=(
            "Target saved stereo-pair rate in Hz. 0 means save every completed frame. "
            "Default: 0 (save all frames)"
        ),
    )
    p.add_argument(
        "--single-width",
        type=nonnegative_int,
        default=0,
        help=(
            "Width of one eye inside the full side-by-side frame. "
            "Default: auto = full_width/2"
        ),
    )
    p.add_argument(
        "--split-order",
        choices=("left-right", "right-left"),
        default="right-left",
        help=(
            "Order of the two halves inside the received side-by-side frame. "
            "Default: right-left"
        ),
    )
    p.add_argument(
        "--left-dir-name",
        default="cam0",
        help="Directory name for the left camera inside output-dir (default: cam0)",
    )
    p.add_argument(
        "--right-dir-name",
        default="cam1",
        help="Directory name for the right camera inside output-dir (default: cam1)",
    )
    p.add_argument(
        "--timestamp-source",
        choices=("sender", "receive", "sender_to_wall"),
        default="sender_to_wall",
        help=(
            "Timestamp source used for the saved PNG filenames. "
            "'sender' uses the packet stamp_ns directly, 'receive' uses local wall-clock receive time, "
            "'sender_to_wall' maps the sender monotonic stamp to wall time using the first received frame. "
            "Default: sender_to_wall"
        ),
    )
    p.add_argument(
        "--frame-timeout-ms",
        type=positive_int,
        default=80,
        help="Discard incomplete frames older than this timeout (default: 80)",
    )
    p.add_argument(
        "--max-inflight",
        type=positive_int,
        default=16,
        help="Maximum number of incomplete frames tracked simultaneously (default: 16)",
    )
    p.add_argument(
        "--recv-buffer-bytes",
        type=positive_int,
        default=4 * 1024 * 1024,
        help="Socket receive buffer size in bytes (default: 4194304)",
    )
    p.add_argument(
        "--poll-timeout-s",
        type=nonnegative_float,
        default=0.01,
        help="Socket poll timeout in seconds (default: 0.01)",
    )
    p.add_argument(
        "--png-compression",
        type=nonnegative_int,
        default=1,
        help="PNG compression level passed to OpenCV (0..9, default: 1)",
    )
    p.add_argument(
        "--writer-threads",
        type=positive_int,
        default=4,
        help="Number of threads for async PNG writing (default: 4)",
    )
    p.add_argument(
        "--max-frames",
        type=nonnegative_int,
        default=0,
        help="Stop after saving this many stereo pairs. 0 means run until interrupted.",
    )
    p.add_argument(
        "--stats-period-s",
        type=nonnegative_float,
        default=2.0,
        help="How often to print aggregate statistics. 0 disables periodic stats. Default: 2.0",
    )
    p.add_argument(
        "--show-frame",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Show the decoded full side-by-side frame in an OpenCV window while acquiring. "
            "Use --no-show-frame to disable. Default: enabled"
        ),
    )
    p.add_argument(
        "--verbose-each-save",
        action="store_true",
        help="Print one line for every saved stereo pair.",
    )
    p.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete output-dir first if it already exists.",
    )

    return p

def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.png_compression < 0 or args.png_compression > 9:
        parser.error("--png-compression must be in [0, 9]")

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        builder = UdpKalibrDatasetBuilder(args)
        builder.run()
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0

if __name__ == "__main__":
    sys.exit(main())
