"""Regressions for the multi-second live storage stalls found during takeover."""
import json
import threading

from perception.pipeline import LatestJsonPublisher
from perception.protocol.jsonio import AsyncJsonlWriter
from perception.tests.support import track_set_of


def test_snapshot_writer_keeps_latest_without_waiting_for_storage(tmp_path):
    writer = LatestJsonPublisher(str(tmp_path))
    entered, release = threading.Event(), threading.Event()
    write = writer._write
    def slow(*pair):
        entered.set()
        assert release.wait(2)
        write(*pair)
    writer._write = slow
    try:
        writer.publish(track_set_of([], sequence=1), None)
        assert entered.wait(2)
        for sequence in range(2, 20):
            writer.publish(track_set_of([], sequence=sequence), None)
        assert writer.overwritten == 17
    finally:
        release.set()
        writer.close()
    assert json.loads((tmp_path/'track_set.json').read_text())['track_set_sequence'] == 19
    assert writer.stats()['written'] == 2


def test_recording_overflow_is_reported_and_accepted_lines_are_drained(tmp_path):
    path = tmp_path/'recording.jsonl'
    writer = AsyncJsonlWriter(str(path), capacity=2)
    entered, release = threading.Event(), threading.Event()
    write = writer._writer.write
    def slow(line):
        entered.set()
        assert release.wait(2)
        write(line)
    writer._writer.write = slow
    try:
        writer.write({'n':0})
        assert entered.wait(2)
        for n in range(1, 5):
            writer.write({'n':n})
    finally:
        release.set()
        writer.close()
    assert writer.stats()['dropped'] == 2
    assert writer.stats()['written'] == 3
    assert [json.loads(line)['n'] for line in path.read_text().splitlines()] == [0,1,2]
