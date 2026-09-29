"""HQ 电子变倍档位 ↔ 视场角的唯一一处查询（表在 `config/hq_zoom_fov.json`）。

这颗镜头**没有光学变倍**：能拧的是对焦，对焦不动几何。会动 FOV 的是**电子变倍**（居中裁剪），
目前停在最低档，而且主人 09-29 明确「暂时也没有用电子变倍的需求」。所以本模块今天**不被任何控制
路径调用**——它存在的原因和那张表一样：等架构师决定「Hailo 喂哪一路、要不要变倍」的时候，
"这一档 FOV 是多少"必须有一个能查的地方，而不是有人凭印象报一个数。

规则写在表里，也在这里演一遍：档位没实测 ⇒ 要么按裁剪律派生并**说明派生自哪个未实测的基档**，
要么直接拒绝回答。**空值是有位置的空值，不是 0，也不是"大概是广角那个数"。**
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TABLE = ROOT / "config" / "hq_zoom_fov.json"


class ZoomFovUnknown(RuntimeError):
    """表答不了这个档位。消息里必须说清缺的是哪一档、为什么。"""


@dataclass(frozen=True)
class ZoomFov:
    digital_zoom: float
    fov_h_deg: float
    status: str          # measured | derived_from_measured_base
    base_zoom: float


def derive_fov_h(base_fov_h_deg: float, zoom: float) -> float:
    """线性居中裁剪下的水平 FOV。推导，不是实测。"""
    if zoom <= 0:
        raise ValueError(f"digital zoom must be positive, got {zoom}")
    return 2.0 * math.degrees(math.atan(math.tan(math.radians(base_fov_h_deg / 2.0)) / zoom))


def load_table(path: Path = DEFAULT_TABLE) -> dict[str, Any]:
    table = json.loads(Path(path).read_text(encoding="utf-8"))
    if "levels" not in table or "base" not in table:
        raise ValueError(f"{path}: a zoom/FOV table needs a base and levels")
    return table


def fov_at(zoom: float, table: dict[str, Any] | None = None,
           path: Path = DEFAULT_TABLE) -> ZoomFov:
    table = table or load_table(path)
    for level in table["levels"]:
        if abs(float(level["digital_zoom"]) - float(zoom)) < 1e-9:
            if isinstance(level.get("fov_h_deg"), (int, float)):
                return ZoomFov(float(zoom), float(level["fov_h_deg"]),
                               str(level.get("status", "measured")), 1.0)
            break
    base = table.get("base", {})
    base_fov = base.get("fov_h_deg")
    if not isinstance(base_fov, (int, float)):
        raise ZoomFovUnknown(
            f"digital zoom {zoom} of {table.get('sensor', '?')} has no measured FOV, and the "
            f"base档 ({base.get('digital_zoom')}) is itself {base.get('status', 'not_measured')}: "
            "deriving from an unmeasured base would print a number with no measurement under it"
        )
    return ZoomFov(float(zoom), derive_fov_h(float(base_fov), float(zoom)),
                   "derived_from_measured_base", float(base.get("digital_zoom", 1.0)))


def _selftest() -> int:
    # 用广角那个已实测的 79.27° 当"假如基档测过了"的输入，专测派生与拒绝这两条路。
    derived = derive_fov_h(79.27, 2.0)
    assert 44.0 < derived < 46.0, derived
    table = {"sensor": "imx477", "base": {"digital_zoom": 1.0, "fov_h_deg": 79.27,
                                          "status": "measured"},
             "levels": [{"digital_zoom": 1.0, "fov_h_deg": None, "status": "measured"}]}
    got = fov_at(2.0, table=table)
    assert got.status == "derived_from_measured_base" and abs(got.fov_h_deg - derived) < 1e-9
    try:
        fov_at(4.0, path=DEFAULT_TABLE)          #  shipped table: base is NOT measured yet
    except ZoomFovUnknown as exc:
        assert "4.0" in str(exc) and "not_measured" in str(exc), exc
    else:
        raise AssertionError("an unmeasured base must refuse, not derive a number")
    print("camera zoom/FOV selftest: 3/3 passed（ shipped 表基档未实测，所以派生必须拒绝）")
    return 0


if __name__ == "__main__":
    raise SystemExit(_selftest())
