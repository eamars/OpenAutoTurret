"""Review a saved read-only inventory locally. Never connects or permits motion.

Collection completeness, successful commands and hardware qualification are
distinct. Even a clean inventory is not an acquisition/stop certificate.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

REQUIRED = frozenset({
    'utc', 'account', 'kernel', 'os', 'processes', 'head', 'status', 'dirty_diff',
    'links', 'runtime/launcher.pid', 'runtime/shutdown.result', 'root_listing',
    'release_listing', 'runtime_listing', 'launcher_status',
})


def review(raw: bytes) -> dict:
    """Fail closed on failed, duplicate, missing or incomplete evidence sections."""
    sections = {}
    failures = []
    notes = []
    current = None
    body = []
    completed = False
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError:
        return _result(raw, {}, ['INVALID_UTF8'], [], False)
    for line in text.splitlines():
        if completed:
            if line.strip():
                failures.append('TRAILING_CONTENT')
            continue
        if line.startswith('@@BEGIN '):
            if current is not None:
                failures.append('UNCLOSED_SECTION:' + current)
            current = line[8:]
            if not current or current in sections:
                failures.append('DUPLICATE_OR_EMPTY_SECTION:' + current)
            body = []
        elif line.startswith('@@END '):
            match = re.fullmatch(r'@@END rc=(\d+)', line)
            if current is None or match is None:
                failures.append('INVALID_SECTION_END')
                continue
            code = int(match[1])
            if code:
                failures.append(f'COMMAND_FAILED:{current}:rc={code}')
            # Keep the first record on duplicates; do not overwrite failure evidence.
            sections.setdefault(current, {'returncode': code, 'text': '\n'.join(body)})
            current, body = None, []
        elif re.fullmatch(r'@@INVENTORY_COMPLETE(?: errors=\d+)?', line):
            if current is not None:
                failures.append('UNCLOSED_SECTION:' + current)
            completed = True
            if 'errors=' in line and int(line.rsplit('=', 1)[1]):
                failures.append('COLLECTOR_REPORTED_ERRORS')
        elif current is not None:
            body.append(line)
        elif line.startswith('@@STATUS_NOT_EXECUTED'):
            notes.append(line)
        elif line.strip():
            failures.append('UNFRAMED_CONTENT')
    if not completed:
        failures.append('INCOMPLETE_CAPTURE')
    if current is not None:
        failures.append('UNCLOSED_SECTION:' + current)
    for name in sorted(REQUIRED - sections.keys()):
        failures.append('MISSING_SECTION:' + name)
    return _result(raw, sections, failures, notes, completed)


def _result(raw, sections, failures, notes, completed):
    def value(name):
        row = sections.get(name)
        return row['text'] if row and row['returncode'] == 0 else None

    links = []
    if value('links') is not None:
        try:
            data = json.loads(value('links'))
            if not isinstance(data, list):
                raise ValueError('links must be a JSON array')
            for row in data:
                if not isinstance(row, dict):
                    raise ValueError('invalid link entry')
                if row.get('ifname') in ('can0', 'can1'):
                    links.append(row)
        except (ValueError, TypeError):
            failures.append('INVALID_LINK_DATA')
    power = None
    match = re.search(r'\bthrottled=0x([0-9a-fA-F]+)\b', value('power') or '')
    if match:
        bits = int(match[1], 16)
        power = {'raw': hex(bits), 'current_flags': bits & 15,
                 'historical_flags': (bits >> 16) & 15,
                 'loaded_power_qualified': False}
    shutdown = value('runtime/shutdown.result')
    previous_stop = ('FAILED' if shutdown and ('STOP FAILED' in shutdown or 'PARK FAILED' in shutdown)
                     else 'UNQUALIFIED')
    return {
        'version': 'adr0022.inventory-review/1',
        'capture_sha256': hashlib.sha256(raw).hexdigest(),
        'capture_completed': completed,
        'collection_integrity': 'PASS' if completed and not failures else 'FAIL',
        'failures': sorted(set(failures)), 'notes': notes,
        'section_count': len(sections),
        'observed_utc': value('utc'), 'checkout_revision': value('head'),
        'process_ownership_verified': False,
        'process_listing_collected': value('processes') is not None,
        'launcher_status': value('launcher_status'),
        'last_recorded_stop': previous_stop,
        'can_links': links, 'power': power,
        'motion_allowed': False,
        'motion_gate_reason': 'Inventory cannot certify owner handoff, live adapters, calibration or stop capability',
        'physical_acquisition': 'NOT_RUN',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = review(args.input.read_bytes())
    # An existing review is evidence, not a scratch file to silently overwrite.
    with args.output.open('x', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write('\n')
    print(json.dumps({k: result[k] for k in ('collection_integrity', 'failures', 'motion_allowed')}))
    return 0 if result['collection_integrity'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
