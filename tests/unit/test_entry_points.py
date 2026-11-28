"""Every script and program must be reachable from ./g1 or named in the three guides.

A file nobody calls and no guide mentions is dead weight that only its author knows about.
"""
import subprocess
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CODE_DIRS = ('shared', 'sim', 'robot', 'tests')
CODE_SUFFIXES = ('.sh', '.py', '.cpp', '.yaml', '.rviz', '.json', '.html')
TEXT_SUFFIXES = CODE_SUFFIXES + ('.txt', '.md')
GUIDES = ('README.md', 'docs/setup.md', 'docs/simulation.md', 'docs/robot.md')


def tracked():
    names = subprocess.check_output(['git', 'ls-files'], cwd=ROOT, text=True).split('\n')
    return [name for name in names if name]


def references(text, candidates):
    """Files named by path or file name, imported as a module, or inside a named source directory."""
    found = set()
    for path in candidates:
        name = Path(path).name
        stem = Path(path).stem
        folder = str(Path(path).parent)
        if path in text or name in text:
            found.add(path)
        elif folder.count('/') >= 1 and folder in text:
            found.add(path)
        elif path.endswith('.py') and (f'import {stem}' in text or f'from {stem} import' in text):
            found.add(path)
    return found


class EntryPoints(unittest.TestCase):
    def test_every_file_is_reachable(self):
        files = tracked()
        code = [f for f in files if f.startswith(CODE_DIRS) and f.endswith(CODE_SUFFIXES)]
        roots = ['g1', *[g for g in GUIDES if (ROOT / g).is_file()]]
        # pytest collects the unit tests by directory, not by file name.
        reached = {f for f in code if f.startswith('tests/unit/')}
        queue = [*roots, *reached]
        while queue:
            current = ROOT / queue.pop()
            if current.suffix not in TEXT_SUFFIXES and current.name != 'g1':
                continue
            text = current.read_text(errors='ignore')
            for path in references(text, code):
                if path not in reached:
                    reached.add(path)
                    queue.append(path)
        orphans = sorted(set(code) - reached)
        self.assertEqual(orphans, [], 'not reachable from ./g1 or the guides')


if __name__ == '__main__':
    unittest.main()
