#!/usr/bin/env python3
"""离线消费遥测痕迹文件（WP1b「合成回放」）——不需要硬件、不需要在跑的那台机器。

ADR-001 README 引的 `tools/offline_checks.py summarize examples/synthetic_trace.ndjson`
此前是一句引用不存在的话。这一支把它变成真的，并把 2026-09-28 那次现场教训焊进去：
**第一个真跳闸文件不是合法 JSON**（少一个收尾引号），而当时所有断言都过了——因为它们是
`find(键名)`。**这里的每一条线都必须真解析**，否则工具就是个装饰。

跑法：
  python3 tools/offline_checks.py make-example --out examples/synthetic_trace.ndjson
  python3 tools/offline_checks.py summarize <file.ndjson>
  python3 tools/offline_checks.py check      <file.ndjson> [--source Firmware目录]
  python3 tools/offline_checks.py --selftest
"""
from __future__ import annotations

import argparse
import datetime
import json
import pathlib
import re
import sys

KIND = "trip_trace"
# ns 级时间戳必须是十进制字符串：墙上钟 ns 超过 2^53，JSON number 会掉低位。
NS_FIELDS = ("frozen_t_ns", "wall_t_ns", "mono_to_wall_ns", "mono_to_wall_err_ns")
# clock_epoch / clock_mapping_id 是 ADR-001 契约里的词（docs/04_CONTRACTS.md:22）：
# 偏移的有效期与身份。跨 epoch / 跨 boot 拼接统计是被契约禁止的，所以这里查。
HEADER_KEYS = ("clock", "boot_id", "clock_epoch") + NS_FIELDS


# ---------------------------------------------------------------- 例子（合成的）
def make_rows(t0: int = 53_951_250_656_072, n: int = 8, period: int = 5_000_000):
    """合成一段：AUTO_ROAM 里 yaw 被要求 -10 deg/s 却一动不动（09-28 06:59 那次的形状）。"""
    rows = []
    for i in range(n):
        stalled = i >= 2
        rows.append({
            "t": str(t0 + i * period),
            "ack": str(557 + i),
            "mode": "AUTO_ROAM",
            "track": "search",
            "phase": "hold",
            "temp_raw": [-1, 27],                     # pitch 无读数：缺席是 -1，不是 0
            "q": [-0.8127, 0.7256],
            "ref": [-0.8131, 0.4250 - 0.0005 * i],    # 参考点在走
            "vref": [0.0, 0.0 if stalled else -0.17],
            "cmd": [0.0, 0.0],                        # mixed yaw 这条路不填 v_command
            "effort": [0.29, None],                   # GM6020 无电流回读：缺席是 null
            "vest": [-0.005, 0.0 if stalled else -0.05],
            "omega": 0,
            "safety": 0,
            "period_us": 5000,
            "goal": [0.0, 0.0],
        })
    return rows


def make_example(out: pathlib.Path, wall_ns: int = 1_790_533_381_711_056_000,
                 off_ns: int = 1_790_478_027_063_700_826) -> None:
    head = {"kind": KIND, "rows": len(make_rows()), "frozen_t_ns": "53951250656072",
            "clock": "CLOCK_MONOTONIC", "boot_id": "00000000-0000-4000-8000-000000000001",
            "wall_t_ns": str(wall_ns), "mono_to_wall_ns": str(off_ns),
            "mono_to_wall_err_ns": "412", "clock_epoch": 1}
    lines = [json.dumps(head, ensure_ascii=False)]
    lines += [json.dumps(r, ensure_ascii=False) for r in make_rows()]
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")


# ---------------------------------------------------------------- 读（真解析）
def read_ndjson(path: pathlib.Path):
    """返回 (header, [(行号, 记录)])。任何一行不是合法 JSON 都当场带行号报错。"""
    out, head = [], None
    for no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError as e:
            raise SystemExit(f"第 {no} 行不是合法 JSON：{e}\n  {line[:120]}")
        if no == 1:
            if rec.get("kind") != KIND:
                raise SystemExit(f"第 1 行不是 {KIND} 头部：{list(rec)[:6]}")
            head = rec
        else:
            out.append((no, rec))
    if head is None:
        raise SystemExit("空文件：连头部都没有")
    return head, out


def vocabulary_from_source(firmware: pathlib.Path):
    """从 C++ 源里抽相位/模式/跟踪状态的词表——防止"文件里的词"和"代码里的词"各走各的。"""
    vocab = {"phase": set(), "mode": set(), "track": set()}
    wanted = {"phase": "control/phase.hpp", "mode": "mode/operating_mode.hpp",
              "track": "tracking/tracking_state_machine.hpp"}
    for key, rel in wanted.items():
        p = firmware / "src" / rel
        if not p.exists():
            continue
        vocab[key] = set(re.findall(r'return "([A-Za-z_][A-Za-z0-9_]*)"', p.read_text()))
    return vocab


# ---------------------------------------------------------------- 子命令
def cmd_summarize(path: pathlib.Path) -> int:
    head, rows = read_ndjson(path)
    ts = [int(r["t"]) for _, r in rows if str(r.get("t", "")).isdigit()]
    print(f"{path.name}：{len(rows)} 行，跨度 {(max(ts) - min(ts)) / 1e9:.3f} s，"
          f"clock={head.get('clock')}，frozen={head.get('frozen_t_ns')}")
    for key in ("phase", "mode", "track"):
        seen = {}
        for _, r in rows:
            seen[r.get(key)] = seen.get(r.get(key), 0) + 1
        print(f"  {key:<6} {dict(sorted(seen.items(), key=lambda kv: -kv[1]))}")
    nul = sum(1 for _, r in rows if any(v is None for v in r.get("effort", [])))
    abs_sentinel = sum(1 for _, r in rows if -1 in r.get("temp_raw", []))
    print(f"  effort 含 null 的行 {nul}；temp_raw 用 -1 表缺席的行 {abs_sentinel}"
          f"（缺席是 -1/null，不是 0）")
    where = place_on_wall_clock(head, min(ts) if ts else None, max(ts) if ts else None)
    if where:
        print(f"  墙上钟位置：{where[0]} → {where[1]}（用头部锚点换算）")
    else:
        print("  墙上钟位置：**算不出来**——头部没带锚点，这文件活得比一次开机久就没法放 time 轴上")
    return 0


def place_on_wall_clock(head, t_first, t_last):
    try:
        off = int(head["mono_to_wall_ns"])
    except (KeyError, TypeError, ValueError):
        return None
    def fmt(t):
        return datetime.datetime.fromtimestamp(
            (t + off) / 1e9, datetime.timezone.utc).isoformat(timespec="milliseconds")
    return fmt(t_first), fmt(t_last)


def cmd_check(paths, firmware: pathlib.Path | None) -> int:
    """多个文件一起查：契约（08_ACCEPTANCE.md:42）**禁止跨 boot 拼接统计**，
    所以同一批文件必须同一 boot、同一 epoch，否则拒绝——拒绝要现在拒，别等复盘。"""
    if not isinstance(paths, (list, tuple)):
        paths = [paths]
    heads = [read_ndjson(p)[0] for p in paths]
    rows_all = [read_ndjson(p)[1] for p in paths]
    if len(paths) > 1:
        boots = {h.get("boot_id") for h in heads}
        epochs = {h.get("clock_epoch") for h in heads}
        if len(boots) > 1:
            print(f"✗ 这批文件跨了 boot（{sorted(boots)}）：契约禁止跨 boot 拼接统计")
            return 1
        if len(epochs) > 1:
            print(f"✗ 这批文件跨了 clock_epoch（{sorted(epochs)}）：跨 epoch 拼接要写明换算")
            return 1
    head, rows = heads[0], rows_all[0]
    path = paths[0]
    bad = []
    for k in HEADER_KEYS:
        if not head.get(k):
            bad.append(f"头部缺 {k}（活得比一次开机久的东西必须自带钟名与开机身份）")
    for k in NS_FIELDS:
        v = head.get(k)
        if isinstance(v, (int, float)):
            bad.append(f"头部 {k} 是 number：ns 级超过 2^53 会掉低位，必须是十进制字符串")
        elif v is None:
            bad.append(f"头部缺 {k}，行内 t 无法换算到墙上钟")
    try:
        if int(head["clock_epoch"]) < 1:
            raise ValueError
    except (KeyError, TypeError, ValueError):
        bad.append("头部 clock_epoch 不是 ≥1 的整数：映射身份缺失，跨文件拼统计无从判断")
    try:
        bound = int(head["mono_to_wall_err_ns"])
        if not 0 < bound < 100_000_000:
            bad.append(f"头部 mono_to_wall_err_ns={bound}：界必须是正数且小于 100 ms，"
                       f"否则这个偏移等于没有界")
    except (KeyError, TypeError, ValueError):
        bad.append("头部 mono_to_wall_err_ns 不是整数：偏移没带误差界就等于没带偏移")
    prev_t = prev_ack = None
    for no, r in rows:
        if not isinstance(r.get("t"), str) or not r["t"].isdigit():
            bad.append(f"第 {no} 行 t 不是十进制字符串")
            continue
        t = int(r["t"])
        if prev_t is not None and t < prev_t:
            bad.append(f"第 {no} 行 t 比上一行小 {prev_t - t} ns：单调钟不许倒流")
        prev_t = t
        ack = r.get("ack")
        if isinstance(ack, str) and ack.isdigit():
            if prev_ack is not None and int(ack) < prev_ack:
                bad.append(f"第 {no} 行 ack 倒退")
            prev_ack = int(ack)
        for axis, tr in enumerate(r.get("temp_raw", [])):
            if tr == 0:
                bad.append(f"第 {no} 行 temp_raw[{axis}] 是 0：0 是合法读数，缺席请用 -1")
    if firmware is not None:
        vocab = vocabulary_from_source(firmware)
        for key, names in vocab.items():
            if not names:
                continue
            for _, r in rows:
                v = r.get(key)
                if v is not None and v not in names:
                    bad.append(f"文件里的 {key}={v!r} 不在源码词表里（{sorted(names)[:4]}…）")
                    break
    if bad:
        for b in bad:
            print("✗ " + b)
        return 1
    print(f"✓ {path.name}：{len(rows)} 行全部可解析、锚点齐、时间单调、词表一致")
    return 0


# ---------------------------------------------------------------- 自检（可跑验收）
def cmd_selftest() -> int:
    tmp = pathlib.Path("/tmp/ota-offline-selftest")
    tmp.mkdir(parents=True, exist_ok=True)
    good = tmp / "good.ndjson"
    make_example(good)
    text = good.read_text(encoding="utf-8")
    cases = [
        ("好文件通过", text, True),
        # 2026-09-28 现场真实发生过的那次：少一个收尾引号。
        ("少收尾引号被抓住", text.replace('"track":"search,"phase"', '"track":"search,"phase"')
         .replace('"track": "search", "phase"', '"track":"search,"phase"'), False),
        ("锚点是 number 被抓住", text.replace('"mono_to_wall_ns": "', '"mono_to_wall_ns": ', 1)
         .replace('063700826"}', '063700826}'), False),
        ("t 倒退被抓住", text.replace('53951255656072', '53951240656072', 1), False),
        ("temp_raw 拿 0 当缺席被抓住", text.replace('"temp_raw": [-1, 27]',
                                                  '"temp_raw": [0, 27]', 1), False),
        ("头部没 boot_id 被抓住", re.sub(r'"boot_id": "[^"]*", ', '', text, count=1), False),
        ("没 clock_epoch 被抓住", re.sub(r'"clock_epoch": 1, ', '', text.replace(
            '"mono_to_wall_err_ns": "412", "clock_epoch": 1',
            '"mono_to_wall_err_ns": "412"'), count=1), False),
        ("界是 0 被抓住", text.replace('"mono_to_wall_err_ns": "412"',
                                     '"mono_to_wall_err_ns": "0"', 1), False),
    ]
    failed = 0
    total = len(cases)
    for i, (name, body, want_ok) in enumerate(cases):
        p = tmp / f"case{i}.ndjson"
        p.write_text(body, encoding="utf-8")
        try:
            ok = cmd_check(p, None) == 0
        except SystemExit:
            ok = False
        verdict = "✓" if ok == want_ok else "✗"
        failed += 0 if ok == want_ok else 1
        print(f"  {verdict} {name}（期望{'通过' if want_ok else '被拒'}，实际{'通过' if ok else '被拒'}）")
    other = tmp / "other_boot.ndjson"
    other.write_text(text.replace("00000000-0000-4000-8000-000000000001",
                                  "00000000-0000-4000-8000-000000000002"), encoding="utf-8")
    refused = cmd_check([good, other], None) != 0
    print(f"  {'✓' if refused else '✗'} 跨 boot 的两枚被拒绝拼接（期望被拒，实际{'被拒' if refused else '通过'}）")
    failed += 0 if refused else 1
    total += 1
    # summarize 也要能跑（它比 check 宽松，但同样必须真解析）
    cmd_summarize(good)
    print(("✓ 自检 %d/%d" % (total - failed, total)) if not failed
          else ("✗ 自检 %d/%d 失败" % (failed, total)))
    return 0 if not failed else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", nargs="?", choices=["summarize", "check", "make-example"])
    ap.add_argument("file", nargs="?", type=pathlib.Path)
    ap.add_argument("files", nargs="*", type=pathlib.Path)
    ap.add_argument("--source", type=pathlib.Path, help="Firmware 目录，用源码词表校验文件里的词")
    ap.add_argument("--out", type=pathlib.Path)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return cmd_selftest()
    if not a.cmd:
        ap.print_help()
        return 2
    if a.cmd == "make-example":
        out = a.out or a.file or pathlib.Path("examples/synthetic_trace.ndjson")
        make_example(out)
        print(f"✓ 写了合成的 {out}（{len(make_rows())} 行 + 头部）")
        return 0
    if a.file is None:
        print("✗ 要一个 ndjson 路径", file=sys.stderr)
        return 2
    return cmd_check(a.files, a.source) if a.cmd == "check" else cmd_summarize(a.file)


if __name__ == "__main__":
    sys.exit(main())
