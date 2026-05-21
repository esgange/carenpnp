"""Tiny async-test runner for environments without pytest-asyncio."""

from __future__ import annotations

import asyncio
import inspect


def pytest_configure(config) -> None:
    config.addinivalue_line("markers", "asyncio: run async test function")


def pytest_pyfunc_call(pyfuncitem):
    test_func = pyfuncitem.obj
    if not inspect.iscoroutinefunction(test_func):
        return None

    kwargs = {
        name: pyfuncitem.funcargs[name]
        for name in pyfuncitem._fixtureinfo.argnames
    }
    asyncio.run(test_func(**kwargs))
    return True
