"""TEMPORARY compatibility shim — ``controller.teensy_link`` moved to ``teensy_link``.

**Delete after 2026-09.** New code MUST import the top-level package directly::

    from teensy_link import TeensyLinkClient, RpcClient, RpcError   # ✅
    from controller.teensy_link import TeensyLinkClient             # ❌ deprecated

The package was relocated from ``controller/teensy_link/`` to the repo root on
2026-08-01 (``plans/parked/refactor-2026-07.md`` Phase 4): it is the repo's
hottest production code and it was living inside its most dormant subsystem —
``controller/`` is the parked MPC runtime, and importing through it dragged
CasADi into the can-bridge process for no reason.

WHY THIS FILE IS ``sys.modules`` ALIASING AND NOT A RE-EXPORT
-------------------------------------------------------------
The obvious shim — ``from teensy_link import *`` — would give the old path a
SECOND module identity for every submodule, and that is silently dangerous
here. ``teensy_link.rpc`` defines the exception classes ``RpcError`` and
``RpcTimeout``. Under a re-export shim, ``controller.teensy_link.rpc.RpcError``
and ``teensy_link.rpc.RpcError`` would be two distinct class objects, so a
``raise`` from one identity would sail straight through an
``except RpcError:`` written against the other. On this codebase that handler
is what stops an RPC failure from escaping into the bridge's callback path.
Module-level state has the same problem in the quiet direction: two copies of
the protocol module means two copies of every module-level cache.

So this module imports the real package, aliases ITSELF to it, and then aliases
EVERY submodule explicitly. After that, ``controller.teensy_link.rpc`` and
``teensy_link.rpc`` are the same object — ``is``-identical, one class identity
each, one copy of module state.

The explicit submodule loop is not optional. Aliasing only the package would
leave ``import controller.teensy_link.rpc`` to the normal finder, which would
locate ``rpc.py`` via the (aliased) parent's ``__path__`` and build a fresh
module object under the old name — reintroducing exactly the dual identity this
file exists to prevent.
"""

from __future__ import annotations

import importlib as _importlib
import pkgutil as _pkgutil
import sys as _sys

import teensy_link as _pkg

_CANONICAL = "teensy_link"

# Alias every submodule FIRST, so nothing can be reached through the finder.
for _info in _pkgutil.iter_modules(_pkg.__path__):
    _sys.modules[f"{__name__}.{_info.name}"] = _importlib.import_module(
        f"{_CANONICAL}.{_info.name}"
    )
del _info

# Then alias the package itself. Replacing ``sys.modules[__name__]`` during
# module execution is supported: importlib re-reads the entry after
# ``exec_module`` and binds whatever is there, so ``import
# controller.teensy_link`` and ``from controller.teensy_link import X`` both
# resolve against the real package.
_sys.modules[__name__] = _pkg
