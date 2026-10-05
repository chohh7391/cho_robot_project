#!/usr/bin/env python3

# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check the headers ament_copyright cannot see: every CMakeLists.txt outside extern/.

ament_copyright picks files by extension, so CMakeLists.txt is never one of
them, and its --add-missing would comment a header with `//` there. This checks
those files with ament_copyright's own parser, under the same rule as every
other source (NOTICE, "File headers"), and can add the header with `#`:

    tools/ci/check_copyright_headers.py [PATH ...]
    tools/ci/check_copyright_headers.py --add-missing "Hyunho Cho" apache2 [PATH ...]

A PATH is a CMakeLists.txt or a directory to search; the default is the
repository this script is in. setup.py needs nothing here: ament_copyright
checks it when it is named on the command line, which the Lint workflow does.
"""

import argparse
import os
import sys

from ament_copyright import get_licenses, UNKNOWN_IDENTIFIER
from ament_copyright.parser import parse_file

REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Top-level directories never searched: vendored and generated trees.
SKIPPED_DIRECTORIES = ('extern', 'build', 'install', 'log')


def cmake_lists(paths):
    """Return the CMakeLists.txt named or found under paths; never extern/, hidden or build directories."""
    found = []
    for path in paths:
        if os.path.isfile(path):
            if os.path.basename(path) == 'CMakeLists.txt':
                found.append(path)
            continue
        for dirpath, dirnames, filenames in os.walk(path):
            top = os.path.realpath(dirpath) == os.path.realpath(REPOSITORY)
            dirnames[:] = sorted(d for d in dirnames
                                 if not d.startswith(('.', '_')) and not (top and d in SKIPPED_DIRECTORIES))
            if 'CMakeLists.txt' in filenames:
                found.append(os.path.join(dirpath, 'CMakeLists.txt'))
    return found


def problem(path):
    """Return why path's header is not acceptable, or None."""
    descriptor = parse_file(path)
    if not descriptor.content:
        return 'file empty'
    if not descriptor.copyright_identifiers:
        return 'could not find copyright notice'
    if descriptor.license_identifier == UNKNOWN_IDENTIFIER:
        return 'unknown license'
    return None


def add_header(path, holder, license_, year):
    """Prepend the header as `#` comments, then one blank line."""
    text = license_.file_headers[0].format(
        copyright=f'Copyright {year} {holder}', copyright_holder=holder)
    comment = ''.join(f'# {line}\n' if line else '#\n' for line in text.splitlines())
    with open(path, encoding='utf-8') as stream:
        content = stream.read()
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write(comment + '\n' + content.lstrip('\n'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('paths', nargs='*', default=[REPOSITORY],
                        help='CMakeLists.txt files or directories (default: the repository)')
    parser.add_argument('--add-missing', nargs=2, metavar=('HOLDER', 'LICENSE'),
                        help='add a header to each file that has none (e.g. "Hyunho Cho" apache2)')
    parser.add_argument('--year', type=int, default=2026, help='year in an added header')
    args = parser.parse_args(argv)

    files = cmake_lists([os.path.abspath(p) for p in args.paths])
    if args.add_missing:
        holder, name = args.add_missing
        licenses = get_licenses()
        if name not in licenses:
            parser.error(f"unknown license '{name}'; one of {sorted(licenses)}")
        for path in files:
            if problem(path) == 'could not find copyright notice':
                add_header(path, holder, licenses[name], args.year)
                print(f'* {os.path.relpath(path, REPOSITORY)}')
        return 0

    failures = [(path, reason) for path in files for reason in [problem(path)] if reason]
    for path, reason in failures:
        print(f'{os.path.relpath(path, REPOSITORY)}: {reason}')
    print(f'{len(files)} CMakeLists.txt checked, {len(failures)} without an acceptable header')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
