"""run.py start_companion(): when UNDS/UNDS.exe is beside the port it starts with the game (stand-ins for
the Windows-only subprocess pieces, so this runs anywhere)."""
from paths import ROOT
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(ROOT))
import run  # noqa: E402


class FakeSubprocess:
    CREATE_NO_WINDOW, CREATE_NEW_CONSOLE, STARTF_USESHOWWINDOW = 0x08000000, 0x10, 1
    SubprocessError, DEVNULL = Exception, -3

    class STARTUPINFO:
        def __init__(self):
            self.dwFlags = self.wShowWindow = 0

    def __init__(self, out, code=0):
        self.out, self.code, self.queries, self.started = out, code, [], []

    def run(self, command, **kwargs):
        self.queries.append(command)
        return types.SimpleNamespace(returncode=self.code, stdout=self.out)

    def Popen(self, command, **kwargs):
        self.started.append((command, kwargs))


class StartCompanionTests(unittest.TestCase):
    def setUp(self):
        self.port = Path(tempfile.mkdtemp())
        (self.port / 'UNDS').mkdir()
        (self.port / 'UNDS' / 'UNDS.exe').write_bytes(b'')
        self.saved = run.PORT, run.subprocess, run.os.name
        run.PORT, run.os.name = self.port, 'nt'

    def tearDown(self):
        run.PORT, run.subprocess, run.os.name = self.saved

    def start(self, env, out='UNDS_EVENTS=127.0.0.1:47111\nUNDS_TRIGGER=30,95\nnoise\nOTHER=1\n', code=0):
        run.subprocess = FakeSubprocess(out, code)
        run.start_companion(env, self.port / 'user')
        return run.subprocess

    def test_starts_attached_without_overriding_the_player(self):
        env = {'UNDS_TRIGGER': '30,78'}
        fake = self.start(env)
        self.assertEqual((env['UNDS_EVENTS'], env['UNDS_TRIGGER'], env['UNDS_AUTO_STARTED']),
                         ('127.0.0.1:47111', '30,78', '1'))
        self.assertNotIn('OTHER', env)
        command, kwargs = fake.started[0]
        self.assertEqual(command[1:3], ['--attach', str(os.getpid())])
        self.assertEqual(command[command.index('--log') + 1], str(self.port / 'user' / 'last_run.log'))
        self.assertEqual((kwargs['creationflags'], kwargs['startupinfo'].wShowWindow),
                         (FakeSubprocess.CREATE_NEW_CONSOLE, 7))       # minimized console by default
        # its own console's handles, never run.py's (a pipe the launcher reads into last_run.log)
        self.assertFalse({'stdin', 'stdout', 'stderr'} & set(kwargs))

    def test_hidden_when_asked(self):
        env = {}
        fake = self.start(env, out='UNDS_EVENTS=127.0.0.1:47111\nUNDS_WINDOW=hidden\n')
        _command, kwargs = fake.started[0]
        self.assertEqual((kwargs['creationflags'], kwargs['stdout']), (FakeSubprocess.CREATE_NO_WINDOW, FakeSubprocess.DEVNULL))
        self.assertNotIn('UNDS_WINDOW', env)

    def test_someone_else_runs_a_companion(self):
        fake = self.start({'UNDS_EVENTS': '127.0.0.1:47111'})
        self.assertEqual((fake.queries, fake.started), ([], []))

    def test_apply_and_restart_starts_a_new_one(self):
        fake = self.start({'UNDS_EVENTS': '127.0.0.1:47111', 'UNDS_AUTO_STARTED': '1'})
        self.assertEqual(len(fake.started), 1)

    def test_turned_off(self):
        env = {}
        fake = self.start(env, out='', code=1)
        self.assertEqual((fake.started, env), ([], {}))
        fake = self.start({'UNDS_AUTO': '0'})
        self.assertEqual(fake.queries, [])

    def test_without_unds(self):
        (self.port / 'UNDS' / 'UNDS.exe').unlink()
        self.assertEqual(self.start({}).queries, [])


if __name__ == '__main__':
    unittest.main()
