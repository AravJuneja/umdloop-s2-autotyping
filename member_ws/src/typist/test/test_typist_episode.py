import threading

from typist.typist import Typist


def test_result_completes_attempt_even_after_new_launch_key_copy():
    typist = Typist.__new__(Typist)
    typist._attempted_episode = 7
    typist._episode_id = 8
    typist._result_seen_for = -1
    typist._episode_over = threading.Event()
    typist._handle_lock = threading.Lock()
    typist._active_handle = None

    Typist._on_result(typist, object())

    assert typist._result_seen_for == 7
    assert typist._episode_over.is_set()
