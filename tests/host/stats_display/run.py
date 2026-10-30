#!/usr/bin/env python3
"""Exercise the receiver's detailed-stats state using extracted C source.

The stats-thread fragment is delimited by its display_expired declaration and
its if (display_expired) log block. State accesses are instrumented to model an
interrupt attempting to observe the pair while the display spinlock is held.
"""

import argparse
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent / 'harness'))
from c_extract import _code_mask, extract_block

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source', type=Path, default=ROOT / 'src/connection/esb.c')
args = parser.parse_args()
source = args.source.read_text()


def unique(pattern, text):
    matches = list(re.finditer(pattern, text, re.MULTILINE))
    if len(matches) != 1:
        raise ValueError(f'Expected one match for {pattern}, found {len(matches)}')
    return matches[0]


def declaration(name):
    pattern = (r'^static[ \t]+(?:volatile[ \t]+)?'
               r'(?:struct[ \t]+\w+|\w+)[ \t]+'
               rf'{re.escape(name)}\b[^;]*;')
    return unique(pattern, source).group()


def function(name):
    return extract_block(
        source,
        rf'^(?:static\s+)?(?:bool|void)\s+{re.escape(name)}\([^;{{]*\)\s*\{{',
    )


def instrument(body, required):
    names = {
        'stats_detailed_enabled': 'instrument_enabled',
        'stats_detailed_end_time': 'instrument_deadline',
    }
    mask = _code_mask(body)
    matches = list(re.finditer(r'\b(?:stats_detailed_enabled|stats_detailed_end_time)\b', mask))
    found = {match.group() for match in matches}
    if not required <= found:
        raise ValueError(f'Missing instrumented state accesses: {required - found}')
    for match in reversed(matches):
        replacement = f'(*{names[match.group()]}())'
        body = body[:match.start()] + replacement + body[match.end():]
    return body


thread = function('esb_stats_thread')
start = unique(r'^[ \t]*bool\s+display_expired\s*=\s*false\s*;', thread)
log_block = extract_block(thread, r'^[ \t]*if\s*\(\s*display_expired\s*\)\s*\{')
end = thread.find(log_block, start.end())
if end < 0:
    raise ValueError('Expiration log block must follow the display_expired declaration')
expiration = thread[start.start():end + len(log_block)]

enabled = 'stats_detailed_enabled'
deadline = 'stats_detailed_end_time'
declarations = '\n'.join(declaration(name) for name in (
    'stats_display_lock', enabled, deadline,
))
functions = '\n\n'.join(instrument(function(name), required) for name, required in (
    ('esb_toggle_stats_detailed', {enabled, deadline}),
    ('esb_set_stats_detailed', {enabled, deadline}),
    ('esb_get_stats_detailed_enabled', {enabled}),
))
fixture = (HERE / 'fixture.c').read_text()
for marker, replacement in (
    ('/* PRODUCTION DECLARATIONS */', declarations),
    ('/* PRODUCTION FUNCTIONS */', functions),
    ('/* PRODUCTION EXPIRATION */', instrument(expiration, {enabled, deadline})),
):
    if fixture.count(marker) != 1:
        raise ValueError(f'Expected one fixture marker: {marker}')
    fixture = fixture.replace(marker, replacement)

with tempfile.TemporaryDirectory(prefix='receiver-stats-display-') as directory:
    directory = Path(directory)
    generated = directory / 'fixture.c'
    generated.write_text(fixture)
    executable = directory / 'fixture'
    compiler = shlex.split(os.environ.get('CC', 'cc'))
    subprocess.run(compiler + [
        '-std=c11', '-g', '-O1', '-Wall', '-Wextra',
        '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
        '-fno-pie', '-no-pie', str(generated), '-o', str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True, timeout=20)
