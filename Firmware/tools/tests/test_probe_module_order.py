"""Round 71: a script whose __main__ guard precedes a top-level def cannot run, yet imports fine.

Round 66 appended `oscillation_verdict` to the end of `tools/probe_track_loop.py`, not knowing the file's tail was
its main guard calling `main()`. Running the script therefore executed main() before the def statement had ever
run: NameError. Importing it under pytest skipped the guard body, defined the function normally, and 81 tests
passed against a tool that could not run. "All tests green" was true of the module and false of the program - the
most misleading kind of green, because it is the green that stops people looking.

This test asserts the shape of the file, not its semantics. An earlier draft also tried to resolve every name
main() loads against the top-level defs; it reported `zip` as unresolved, because `__builtins__` is a dict inside
a test module so the builtin set came out empty - the checker was measuring its own bookkeeping, the same failure
as round 51's arity checker, and it was deleted rather than tuned.
"""
import ast
import pathlib
import unittest

PROBE = pathlib.Path(__file__).resolve().parents[1] / "probe_track_loop.py"


def _is_main_guard(node):
    return (isinstance(node, ast.If)
            and getattr(node.test, "left", None) is not None
            and getattr(node.test.left, "id", "") == "__name__")


class ModuleOrder(unittest.TestCase):
    def setUp(self) -> None:
        self.tree = ast.parse(PROBE.read_text(), filename=str(PROBE))
        self.body = self.tree.body
        self.guards = [i for i, n in enumerate(self.body) if _is_main_guard(n)]

    def test_probe_has_a_main_guard(self) -> None:
        self.assertTrue(self.guards, "probe lost its main guard entirely")

    def test_main_guard_is_the_last_top_level_statement(self) -> None:
        # Anything after the guard - a def, an assignment - is dead code when the file is run as a script,
        # because main() has already been called by then. That is precisely round 69's NameError.
        self.assertEqual(self.guards[-1], len(self.body) - 1,
                         "something follows the main guard at top level; a def there never executes when "
                         "the script runs, though importing it would look fine")

    def test_guard_body_is_a_call_not_a_definition(self) -> None:
        guard = self.body[self.guards[-1]]
        self.assertTrue(guard.body, "empty main guard")
        self.assertIsInstance(guard.body[0], (ast.Raise, ast.Expr, ast.Assign),
                              "the main guard's first statement is neither a call nor an exit")


if __name__ == "__main__":
    unittest.main()
