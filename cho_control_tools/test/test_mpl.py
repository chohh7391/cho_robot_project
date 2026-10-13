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

"""import_pyplot() retries without the user site only after an ImportError, and always restores sys.path.

The matplotlib import and the user site are faked, so none of this depends on what ~/.local holds.
"""

import sys

import pytest

from cho_control_tools.plotting import mpl

USER_SITE = '/fake/home/.local/lib/python3.10/site-packages'
SYSTEM_PATH = ['', '/usr/lib/python3.10', USER_SITE, '/usr/lib/python3/dist-packages']
PYPLOT = object()


class _FakeImport:
    """Stands in for mpl._import: one outcome per call (an exception is raised), the path seen by each."""

    def __init__(self, *outcomes):
        self.outcomes = list(outcomes)
        self.backends = []
        self.paths = []

    def __call__(self, backend):
        self.backends.append(backend)
        self.paths.append(list(sys.path))
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome


@pytest.fixture
def fake(monkeypatch):
    """A sys.path with a fake user site on it; (set the outcomes, the forget calls) for the import."""
    monkeypatch.setattr(sys, 'path', list(SYSTEM_PATH))
    monkeypatch.setattr(mpl.site, 'getusersitepackages', lambda: USER_SITE)
    forgets = []
    monkeypatch.setattr(mpl, '_forget_matplotlib', lambda: forgets.append(list(sys.path)))

    def install(*outcomes):
        fake_import = _FakeImport(*outcomes)
        monkeypatch.setattr(mpl, '_import', fake_import)
        return fake_import, forgets

    return install


def test_a_first_import_that_works_is_returned_without_a_retry(fake):
    fake_import, forgets = fake(PYPLOT)
    assert mpl.import_pyplot('Agg') is PYPLOT
    assert fake_import.backends == ['Agg']
    assert forgets == []
    assert sys.path == SYSTEM_PATH


def test_an_import_error_is_retried_without_the_user_site_and_the_path_comes_back(fake):
    fake_import, forgets = fake(ImportError('numpy too old'), PYPLOT)
    assert mpl.import_pyplot('Agg') is PYPLOT
    # The retry sees no user site, with matplotlib forgotten first; the same backend is asked for.
    assert fake_import.backends == ['Agg', 'Agg']
    assert USER_SITE in fake_import.paths[0]
    assert USER_SITE not in fake_import.paths[1]
    assert forgets == [[entry for entry in SYSTEM_PATH if entry != USER_SITE]]
    assert sys.path == SYSTEM_PATH
    assert sys.path.index(USER_SITE) == 2


def test_when_the_retry_fails_too_the_first_error_is_raised_and_the_path_comes_back(fake):
    first = ImportError('first: numpy too old')
    fake_import, forgets = fake(first, ImportError('second'))
    with pytest.raises(ImportError) as error_info:
        mpl.import_pyplot()
    assert error_info.value is first
    assert len(fake_import.backends) == 2
    # Forgotten before the retry and again after it failed, so no half-imported matplotlib is left.
    assert len(forgets) == 2
    assert sys.path == SYSTEM_PATH


def test_an_error_that_is_not_an_import_error_in_the_retry_propagates_and_the_path_comes_back(fake):
    fake_import, _ = fake(ImportError('numpy too old'), RuntimeError('broken backend'))
    with pytest.raises(RuntimeError, match='broken backend'):
        mpl.import_pyplot()
    assert len(fake_import.backends) == 2
    assert sys.path == SYSTEM_PATH


def test_the_path_comes_back_when_the_retry_is_interrupted(fake):
    fake(ImportError('numpy too old'), KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        mpl.import_pyplot()
    assert sys.path == SYSTEM_PATH


def test_without_the_user_site_on_the_path_the_first_error_is_raised_with_no_retry(fake, monkeypatch):
    first = ImportError('numpy too old')
    fake_import, forgets = fake(first, PYPLOT)
    monkeypatch.setattr(sys, 'path', [entry for entry in SYSTEM_PATH if entry != USER_SITE])
    with pytest.raises(ImportError) as error_info:
        mpl.import_pyplot()
    assert error_info.value is first
    assert len(fake_import.backends) == 1
    assert forgets == []
    assert USER_SITE not in sys.path
