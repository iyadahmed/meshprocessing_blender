from contextlib import contextmanager
from timeit import default_timer


@contextmanager
def scoped_timer(msg: str):
    t0 = default_timer()
    yield
    t1 = default_timer()
    print(f"{msg} finished in {t1 - t0} seconds.")
