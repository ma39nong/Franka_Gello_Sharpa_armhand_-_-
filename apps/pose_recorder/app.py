#!/usr/bin/env python3
"""Simple web UI for read-only FR3 + Wuji Hand 2 pose snapshots."""

from __future__ import annotations

import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from apps.pose_recorder.readers import PoseRecorder


INDEX_HTML = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Pose Recorder</title>
  <style>
    :root {
      --bg: #111827;
      --panel: #1f2937;
      --text: #f3f4f6;
      --muted: #9ca3af;
      --accent: #38bdf8;
      --ok: #34d399;
      --bad: #f87171;
      --line: #374151;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "IBM Plex Sans", "Noto Sans SC", sans-serif;
      background: radial-gradient(circle at top, #1e293b, var(--bg) 55%);
      color: var(--text);
      min-height: 100vh;
    }
    main {
      max-width: 960px;
      margin: 0 auto;
      padding: 32px 20px 48px;
    }
    h1 {
      margin: 0 0 8px;
      font-size: 1.75rem;
      letter-spacing: 0.02em;
    }
    .sub { color: var(--muted); margin-bottom: 24px; }
    .panel {
      background: color-mix(in srgb, var(--panel) 92%, black);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 18px;
      margin-bottom: 16px;
    }
    label {
      display: block;
      font-size: 0.85rem;
      color: var(--muted);
      margin-bottom: 8px;
    }
    textarea {
      width: 100%;
      min-height: 72px;
      resize: vertical;
      border-radius: 10px;
      border: 1px solid var(--line);
      background: #0b1220;
      color: var(--text);
      padding: 12px;
      font: inherit;
    }
    .row {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
      margin-top: 12px;
      align-items: center;
    }
    button {
      appearance: none;
      border: 0;
      border-radius: 999px;
      padding: 12px 20px;
      font: inherit;
      font-weight: 600;
      cursor: pointer;
      background: var(--accent);
      color: #082f49;
    }
    button.secondary {
      background: transparent;
      color: var(--text);
      border: 1px solid var(--line);
    }
    button:disabled { opacity: 0.5; cursor: wait; }
    .meta { color: var(--muted); font-size: 0.9rem; }
    .path { font-family: ui-monospace, SFMono-Regular, Menlo, monospace; color: var(--accent); }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(260px, 1fr));
      gap: 12px;
    }
    .side h2 {
      margin: 0 0 8px;
      font-size: 1rem;
    }
    .badge {
      display: inline-block;
      border-radius: 999px;
      padding: 2px 10px;
      font-size: 0.75rem;
      margin-left: 8px;
    }
    .badge.ok { background: color-mix(in srgb, var(--ok) 25%, transparent); color: var(--ok); }
    .badge.bad { background: color-mix(in srgb, var(--bad) 25%, transparent); color: var(--bad); }
    pre {
      margin: 8px 0 0;
      padding: 10px;
      border-radius: 8px;
      background: #0b1220;
      overflow: auto;
      font-size: 0.78rem;
      line-height: 1.35;
      max-height: 180px;
    }
    .toast {
      margin-top: 10px;
      color: var(--ok);
      min-height: 1.2em;
    }
    .toast.err { color: var(--bad); }
  </style>
</head>
<body>
  <main>
    <h1>关节位姿记录</h1>
    <p class="sub">只读抓取当前 FR3 / Wuji Hand 2 关节；点击记录追加到 JSONL，可写备注。</p>

    <section class="panel">
      <div class="meta">输出文件：<span class="path" id="outputPath">-</span></div>
      <div class="meta">已记录条数：<span id="recordCount">0</span></div>
      <div class="meta">侧：<span id="sides">-</span></div>
      <label for="note" style="margin-top:16px">备注</label>
      <textarea id="note" placeholder="例如：右手抓杯预抓姿态"></textarea>
      <div class="row">
        <button id="recordBtn" type="button">记录当前姿态</button>
        <button id="refreshBtn" class="secondary" type="button">刷新预览</button>
      </div>
      <div class="toast" id="toast"></div>
    </section>

    <section class="panel">
      <h2 style="margin:0 0 12px;font-size:1.05rem">实时预览</h2>
      <div class="grid" id="preview"></div>
    </section>
  </main>
  <script>
    const toast = document.getElementById("toast");
    const preview = document.getElementById("preview");
    const recordBtn = document.getElementById("recordBtn");
    const refreshBtn = document.getElementById("refreshBtn");

    function fmtPositions(values) {
      if (!values || !values.length) return "(empty)";
      return values.map((v) => Number(v).toFixed(4)).join(", ");
    }

    function sideCard(title, sample) {
      if (!sample) {
        return `<div class="side"><h2>${title}<span class="badge bad">off</span></h2><pre>disabled</pre></div>`;
      }
      const ok = !!sample.ok;
      const badge = ok ? "ok" : "bad";
      const label = ok ? "ok" : "fault";
      const body = ok
        ? fmtPositions(sample.positions)
        : (sample.fault || "unavailable");
      const names = sample.names ? sample.names.join(", ") : "";
      return `<div class="side"><h2>${title}<span class="badge ${badge}">${label}</span></h2>
        <div class="meta">${names}</div>
        <pre>${body}</pre></div>`;
    }

    function render(live) {
      document.getElementById("outputPath").textContent = live.output || "-";
      document.getElementById("recordCount").textContent = live.record_count ?? 0;
      document.getElementById("sides").textContent = (live.sides || []).join(", ");
      const sides = live.sides || [];
      const cards = [];
      for (const side of sides) {
        const arm = live.arms && live.arms.sides ? live.arms.sides[side] : null;
        const hand = live.hands && live.hands.sides ? live.hands.sides[side] : null;
        cards.push(sideCard(`${side} arm`, arm || (live.arms ? {ok:false, fault: live.arms.fault} : null)));
        cards.push(sideCard(`${side} hand`, hand || (live.hands ? {ok:false, fault: live.hands.fault} : null)));
      }
      preview.innerHTML = cards.join("") || "<div class='meta'>no sides</div>";
    }

    async function refresh() {
      const res = await fetch("/api/live");
      const live = await res.json();
      render(live);
      return live;
    }

    async function record() {
      recordBtn.disabled = true;
      toast.textContent = "";
      toast.className = "toast";
      try {
        const res = await fetch("/api/record", {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({ note: document.getElementById("note").value || "" }),
        });
        const data = await res.json();
        if (!res.ok || !data.ok) {
          throw new Error(data.error || "record failed");
        }
        document.getElementById("note").value = "";
        toast.textContent = `已写入第 ${data.record_count} 条 → ${data.output}`;
        await refresh();
      } catch (error) {
        toast.className = "toast err";
        toast.textContent = String(error.message || error);
      } finally {
        recordBtn.disabled = false;
      }
    }

    recordBtn.addEventListener("click", record);
    refreshBtn.addEventListener("click", () => refresh().catch(console.error));
    refresh().catch(console.error);
    setInterval(() => refresh().catch(() => {}), 1000);
  </script>
</body>
</html>
"""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read-only web UI that snapshots FR3 and Wuji Hand 2 joints into JSONL."
        )
    )
    parser.add_argument(
        "--sides",
        choices=("left", "right", "both"),
        default="both",
        help="which sides to record (default: both)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "apps" / "pose_recorder" / "records" / "poses.jsonl",
        help="JSONL path for recorded snapshots",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument(
        "--no-arms",
        action="store_true",
        help="skip FR3 ROS state (hands only)",
    )
    parser.add_argument(
        "--no-hands",
        action="store_true",
        help="skip Wuji SDK state (arms only)",
    )
    for side in ("left", "right"):
        parser.add_argument(
            f"--wuji-{side}-address",
            default="",
            help=f"{side} Wuji Hand 2 address IP:PORT",
        )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    sides = ("left", "right") if args.sides == "both" else (args.sides,)
    if args.no_arms and args.no_hands:
        parser.error("need at least one of arms or hands")
    addresses = {
        side: getattr(args, f"wuji_{side}_address")
        for side in sides
    }
    if not args.no_hands:
        missing = [side for side in sides if not addresses[side]]
        if missing:
            parser.error(
                "Wuji Hand 2 requires --wuji-<side>-address for: "
                + ", ".join(missing)
            )

    recorder = PoseRecorder(
        sides=sides,
        output=args.output.expanduser().resolve(),
        wuji_addresses=addresses,
        record_arms=not args.no_arms,
        record_hands=not args.no_hands,
    )

    try:
        recorder.start()
    except Exception as error:
        print(f"failed to start readers: {error}", file=sys.stderr)
        recorder.close()
        return 1

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args_msg) -> None:
            print(f"[http] {self.address_string()} {format % args_msg}")

        def _send(self, code: int, body: bytes, content_type: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, payload: object) -> None:
            raw = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self._send(code, raw, "application/json; charset=utf-8")

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._send(200, INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if path == "/api/live":
                self._json(200, recorder.live())
                return
            self._json(404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            if path != "/api/record":
                self._json(404, {"ok": False, "error": "not found"})
                return
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8") or "{}")
            except json.JSONDecodeError:
                self._json(400, {"ok": False, "error": "invalid JSON body"})
                return
            note = str(payload.get("note") or "")
            try:
                result = recorder.record(note)
            except Exception as error:
                self._json(500, {"ok": False, "error": str(error)})
                return
            self._json(200, result)

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(
        f"pose recorder on http://{args.host}:{args.port}\n"
        f"JSONL -> {recorder.output}\n"
        f"sides={','.join(sides)} arms={not args.no_arms} hands={not args.no_hands}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        server.server_close()
        recorder.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
