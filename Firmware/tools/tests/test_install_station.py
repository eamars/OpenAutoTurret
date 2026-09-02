"""Unit rendering + the silent-failure checks, without a station or root.

P11 deploys by editing paths in templates and typing `cp` + `systemctl`. The
failures it produces are the quiet kind: `vision_connected=false` forever, a
preview that disagrees with the tracker about orientation, an "active" service
that runs nothing. Those are text properties of text files, so they are testable
here, with fake units, long before anyone stands at the turret.

One test exists specifically because `render_unit`'s first version captured only
the `(ExecStart)` group and returned it — erasing every command it rewrote. The
staged unit said `ExecStart=` and nothing else.
"""
from __future__ import annotations

import os
import tempfile
import unittest

from tools.install_station import (FAIL, INFO, PASS, Report, check_layout,
                                   check_units, exec_command, render_unit,
                                   stage)

VISION_UNIT = """\
[Unit]
Description=test
# explain the flag: pass --socket COMMENT_MUST_NOT_MATCH to visiond
[Service]
User=eamars
Group=eamars
SupplementaryGroups=video
WorkingDirectory=/opt/open_auto_turret
ExecStart=/usr/bin/python3 -m vision.visiond --real --socket /tmp/ota_vision.sock \\
    --orientation rotate_180 --detector none
Restart=on-failure
"""

WEB_UNIT = """\
[Service]
User=eamars
Group=eamars
WorkingDirectory=/opt/open_auto_turret
ExecStart=/usr/bin/python3 -m web.webd.app
Environment=OTA_WEB_SOCKET=/run/ota/controld-web.sock
Environment=OTA_VIDEO_ORIENTATION=rotate_180
"""

CTRL_UNIT = """\
[Unit]
After=can0.service
Wants=can0.service

[Service]
User=eamars
Group=eamars
WorkingDirectory=/opt/open_auto_turret
ExecStart=/opt/open_auto_turret/build/control/controld /opt/open_auto_turret/config/turret.yaml
Environment=OTA_WEB_SOCKET=/run/ota/controld-web.sock
ExecStartPre=/usr/bin/mkdir -p /run/ota
ExecStartPre=/usr/bin/chown eamars:eamars /run/ota
KillSignal=SIGINT
Restart=on-failure
TimeoutStartSec=infinity
"""


def write_tree(root: str, *, controld: bool = True, config: bool = True,
               units: dict | None = None) -> str:
    os.makedirs(os.path.join(root, "build", "control"), exist_ok=True)
    os.makedirs(os.path.join(root, "vision"), exist_ok=True)
    os.makedirs(os.path.join(root, "web"), exist_ok=True)
    os.makedirs(os.path.join(root, "config", "payload_profiles"),
                exist_ok=True)
    os.makedirs(os.path.join(root, "systemd"), exist_ok=True)
    if controld:
        p = os.path.join(root, "build", "control", "controld")
        with open(p, "w") as f:
            f.write("#!/bin/sh\n")
        os.chmod(p, 0o755)
    if config:
        with open(os.path.join(root, "config", "turret.yaml"), "w") as f:
            f.write("payload:\n  profile_dir: config/payload_profiles\n")
    for name, text in (units or {}).items():
        with open(os.path.join(root, "systemd", name), "w") as f:
            f.write(text)
    return root


class RenderTest(unittest.TestCase):

    def test_root_and_identity_are_substituted(self):
        out = render_unit(CTRL_UNIT, "/srv/turret", "turret", "turret")
        self.assertIn("WorkingDirectory=/srv/turret", out)
        self.assertIn("ExecStart=/srv/turret/build/control/controld "
                      "/srv/turret/config/turret.yaml", out)
        self.assertIn("User=turret", out)
        self.assertIn("Group=turret", out)
        self.assertIn("/usr/bin/chown turret:turret /run/ota", out)

    def test_rendering_keeps_the_command_it_rewrites(self):
        """The regression that motivated this file: capturing only the key
        group turned every ExecStart into an empty value."""
        out = render_unit(VISION_UNIT, "/srv/turret", "turret", "turret")
        self.assertTrue(exec_command(out), "ExecStart was erased")
        self.assertIn("vision.visiond", exec_command(out))
        self.assertIn("--detector none", exec_command(out))

    def test_rendering_leaves_reviewed_directives_alone(self):
        out = render_unit(CTRL_UNIT, "/srv/turret", "turret", "turret")
        self.assertIn("TimeoutStartSec=infinity", out)
        self.assertIn("Restart=on-failure", out)
        self.assertIn("KillSignal=SIGINT", out)
        self.assertIn("After=can0.service",
                      render_unit(CTRL_UNIT, "/x", "a", "b"))


class ExecCommandTest(unittest.TestCase):

    def test_continuations_are_joined(self):
        cmd = exec_command(VISION_UNIT)
        self.assertIn("--orientation rotate_180", cmd)
        self.assertIn("--socket /tmp/ota_vision.sock", cmd)

    def test_comments_do_not_count_as_flags(self):
        cmd = exec_command(VISION_UNIT)
        self.assertNotIn("COMMENT_MUST_NOT_MATCH", cmd)

    def test_empty_when_erased(self):
        self.assertEqual(exec_command("[Service]\nExecStart=\n"), "")


class CheckTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.mkdtemp(prefix="ota_install_")

    def _check(self, units) -> Report:
        write_tree(self.dir, units=units)
        rep = Report()
        check_layout(self.dir, rep)
        check_units(self.dir, os.path.join(self.dir, "systemd"), rep)
        return rep

    def _levels(self, rep: Report, needle: str):
        return [(lvl, detail) for lvl, what, detail in rep.rows
                if needle in what]

    def _rendered(self, root: str) -> dict:
        """The units a real install would have: rendered for THIS root."""
        return {name: render_unit(text, root, "eamars", "eamars")
                for name, text in (("turret-control.service", CTRL_UNIT),
                                   ("turret-vision.service", VISION_UNIT),
                                   ("turret-web.service", WEB_UNIT))}

    def test_a_consistent_install_passes(self):
        rep = self._check(self._rendered(self.dir))
        self.assertFalse(rep.failed, rep.rows)

    def test_can0_unit_absent_is_not_a_failure_on_the_yousee_station(self):
        # This station's primary CAN is the yousee USB-CAN adapter; the can0
        # unit is the MCP2515 HAT. A check that FAILs on the station it was
        # written for gets ignored, which is worse than no check.
        rep = self._check(self._rendered(self.dir))
        rows = self._levels(rep, "can0.service not found")
        self.assertEqual(rows[0][0], INFO, rows)

    def test_vision_socket_mismatch_is_a_failure_not_a_note(self):
        vis = VISION_UNIT.replace("/tmp/ota_vision.sock", "/run/ota/v.sock")
        rep = self._check({"turret-control.service": CTRL_UNIT,
                           "turret-vision.service": vis,
                           "turret-web.service": WEB_UNIT})
        rows = self._levels(rep, "vision socket agrees")
        self.assertEqual(rows[0][0], FAIL, rows)
        self.assertIn("vision_connected=false", rows[0][1])

    def test_orientation_disagreement_is_caught(self):
        web = WEB_UNIT.replace("OTA_VIDEO_ORIENTATION=rotate_180",
                               "OTA_VIDEO_ORIENTATION=none")
        rep = self._check({"turret-control.service": CTRL_UNIT,
                           "turret-vision.service": VISION_UNIT,
                           "turret-web.service": web})
        self.assertEqual(self._levels(rep, "orientation agrees")[0][0], "WARN")

    def test_a_finite_timeout_would_kill_homing(self):
        ctrl = CTRL_UNIT.replace("TimeoutStartSec=infinity",
                                 "TimeoutStartSec=60")
        rep = self._check({"turret-control.service": ctrl,
                           "turret-vision.service": VISION_UNIT,
                           "turret-web.service": WEB_UNIT})
        self.assertEqual(
            self._levels(rep, "TimeoutStartSec lets homing finish")[0][0], FAIL)

    def test_missing_binary_is_a_failure(self):
        write_tree(self.dir, controld=False,
                   units={"turret-control.service": CTRL_UNIT,
                          "turret-vision.service": VISION_UNIT,
                          "turret-web.service": WEB_UNIT})
        rep = Report()
        check_layout(self.dir, rep)
        self.assertTrue(rep.failed)

    def test_wrong_working_directory_is_caught(self):
        ctrl = CTRL_UNIT.replace("WorkingDirectory=/opt/open_auto_turret",
                                 "WorkingDirectory=/somewhere/else")
        rep = self._check({"turret-control.service": ctrl,
                           "turret-vision.service": VISION_UNIT,
                           "turret-web.service": WEB_UNIT})
        self.assertTrue(any(lvl == FAIL and "WorkingDirectory" in what
                            for lvl, what, _ in rep.rows), rep.rows)

    def test_empty_execstart_is_caught(self):
        web = "[Service]\nUser=root\nGroup=root\n" \
              "WorkingDirectory=" + self.dir + "\nExecStart=\n"
        rep = self._check({"turret-control.service": CTRL_UNIT,
                           "turret-vision.service": VISION_UNIT,
                           "turret-web.service": web})
        self.assertTrue(any(lvl == FAIL and "ExecStart is empty" in what
                            for lvl, what, _ in rep.rows), rep.rows)


class StageTest(unittest.TestCase):

    def test_dry_run_writes_nothing(self):
        src = tempfile.mkdtemp(prefix="ota_src_")
        dst = tempfile.mkdtemp(prefix="ota_dst_")
        write_tree(src, units={"turret-control.service": CTRL_UNIT,
                               "turret-vision.service": VISION_UNIT,
                               "turret-web.service": WEB_UNIT,
                               "can0.service": "[Unit]\n"})
        rep = Report()
        stage(src, dst, "turret", "turret", apply=False, rep=rep)
        self.assertEqual(os.listdir(dst), [])
        self.assertTrue(any(lvl == INFO and "dry run" in what
                            for lvl, what, _ in rep.rows))

    def test_staging_carries_the_binary_and_rendered_units(self):
        src = tempfile.mkdtemp(prefix="ota_src2_")
        dst = os.path.join(tempfile.mkdtemp(prefix="ota_dst2_"), "stage")
        write_tree(src, units={"turret-control.service": CTRL_UNIT,
                               "turret-vision.service": VISION_UNIT,
                               "turret-web.service": WEB_UNIT,
                               "can0.service": "[Unit]\n"})
        rep = Report()
        stage(src, dst, "turret", "turret", apply=True, rep=rep)
        self.assertTrue(os.access(
            os.path.join(dst, "build", "control", "controld"), os.X_OK))
        rendered = open(os.path.join(dst, "systemd",
                                     "turret-vision.service")).read()
        self.assertIn(f"WorkingDirectory={dst}", rendered)
        self.assertIn("vision.visiond", exec_command(rendered))
        self.assertFalse(rep.failed, rep.rows)

    def test_refuses_to_stage_onto_its_own_source(self):
        src = tempfile.mkdtemp(prefix="ota_src3_")
        write_tree(src)
        rep = Report()
        stage(src, src, "a", "b", apply=True, rep=rep)
        self.assertTrue(rep.failed)


if __name__ == "__main__":
    unittest.main()
