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

"""Import matplotlib.pyplot for the plot tools."""

import site
import sys


def import_pyplot(backend=None):
    """matplotlib.pyplot (on *backend*, if given), skipping a user-site matplotlib built for a newer numpy.

    The user site comes first on sys.path, so its matplotlib shadows the system's even when the
    system's numpy is too old for it and the import raises ImportError. Only then is the import
    retried without the user site; when that fails too, the first error is raised. Before the retry
    only matplotlib's own modules are dropped: dependencies the failed attempt already loaded from
    the user site (pyparsing, dateutil, ...) are reused.
    """
    try:
        return _import(backend)
    except ImportError as error:
        user_site = site.getusersitepackages()
        positions = [i for i, entry in enumerate(sys.path) if entry == user_site]
        if not positions:
            raise
        first_error = error
    try:
        sys.path[:] = [entry for entry in sys.path if entry != user_site]
        _forget_matplotlib()
        return _import(backend)
    except ImportError:
        _forget_matplotlib()
        raise first_error from None
    finally:
        # Whatever ended the retry, even Ctrl-C, the user site goes back where it was.
        sys.path[:] = [entry for entry in sys.path if entry != user_site]
        for position in positions:
            sys.path.insert(position, user_site)


def _import(backend):
    import matplotlib
    if backend is not None:
        matplotlib.use(backend)
    import matplotlib.pyplot as plt
    return plt


def _forget_matplotlib():
    """Drop matplotlib, including a half-imported one, from sys.modules so the next import starts over."""
    for name in list(sys.modules):
        if name.split('.')[0] in ('matplotlib', 'mpl_toolkits'):
            del sys.modules[name]
