"""``python -m mapkit`` command line."""
from __future__ import annotations

import argparse
import os
import sys
from typing import List, Optional

from mapkit import gen_docs, gen_glossary, gen_index, import_issues, itemcli, tickets


def default_repo() -> str:
    env = os.environ.get("JUGGLEBOT_REPO")
    if env:
        return env
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _cmd_gen(args: argparse.Namespace) -> int:
    root = os.path.abspath(args.repo or default_repo())
    changed: List[str] = []
    changed += gen_index.write_indexes(root, check=args.check)
    changed += gen_glossary.write_term_block(root, check=args.check)
    changed += gen_docs.write_docs(root, check=args.check)
    for path in changed:
        print(("would change " if args.check else "wrote ") + os.path.relpath(path, root))
    return 1 if (args.check and changed) else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="mapkit")
    sub = parser.add_subparsers(dest="command")
    gen = sub.add_parser("gen", help="regenerate INDEX boards and map outputs")
    gen.add_argument("--repo", default=None, help="repo root (default: $JUGGLEBOT_REPO or parent of mapkit/)")
    gen.add_argument("--check", action="store_true", help="write nothing; exit 1 if anything is stale")
    gen.set_defaults(func=_cmd_gen)
    tickets.register(sub)
    itemcli.register(sub)
    import_issues.register(sub)
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    func = getattr(args, "func", None)
    if func is None:
        parser.print_help()
        return 2
    return int(func(args) or 0)


if __name__ == "__main__":
    sys.exit(main())
