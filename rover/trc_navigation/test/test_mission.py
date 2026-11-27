import math

import pytest

from trc_navigation import mission as ms


def test_parse_markers_like_rulebook_table():
    # 競技説明編 表A-2 の書き方（見出し・括弧・空白あり）
    text = 'id, 座標(緯度, 経度)\n0, 35.537067, 134.210578\n1,35.536932,134.210631\n\n8, "35.536784, 134.210739"\n'
    m = ms.parse_markers(text)
    assert m == {0: (35.537067, 134.210578), 1: (35.536932, 134.210631), 8: (35.536784, 134.210739)}


def test_parse_markers_rejects_bad_values():
    with pytest.raises(ValueError):
        ms.parse_markers('1, 95.0, 134.2\n')
    with pytest.raises(ValueError):
        ms.parse_markers('1, 35.5, 134.2\n1, 35.6, 134.2\n')


def test_parse_route():
    markers = {2: (35.1, 134.1), 5: (35.2, 134.2)}
    steps = ms.parse_route([
        {'checkpoint': [2, 5]},
        {'checkpoint': {'a': [35.3, 134.3], 'b': [35.4, 134.4]}, 'name': 'CP2'},
        {'latlon': [35.5, 134.5], 'name': '最後'},
    ], markers)
    assert [s.kind for s in steps] == ['checkpoint', 'checkpoint', 'latlon']
    assert steps[0].a == (35.1, 134.1) and steps[0].ids == (2, 5)
    assert steps[1].name == 'CP2' and steps[2].point == (35.5, 134.5)
    with pytest.raises(ValueError):
        ms.parse_route([{'checkpoint': [2, 9]}], markers)       # 対応表にない ID
    with pytest.raises(ValueError):
        ms.parse_route([{'go': 1}], markers)


def test_crossing_points_from_either_side():
    a, b = (8.0, -1.5), (8.0, 1.5)                              # x = 8 の縦の線分
    pre, post, gap = ms.crossing_points(a, b, (0.0, 0.0), 2.0, 1.6)
    assert gap == 3.0
    assert pre == pytest.approx((6.0, 0.0)) and post == pytest.approx((9.6, 0.0))
    pre, post, _ = ms.crossing_points(a, b, (15.0, 0.0), 2.0, 1.6)   # 反対側から来たとき
    assert pre == pytest.approx((10.0, 0.0)) and post == pytest.approx((6.4, 0.0))


def test_crossed_fully():
    a, b = (8.0, -1.5), (8.0, 1.5)
    through = [(x * 0.5, 0.0) for x in range(0, 22)]            # 0 → 10.5 m まっすぐ
    assert ms.crossed_fully(through, a, b, 0.6)
    stop_on_line = [(x * 0.5, 0.0) for x in range(0, 17)] + [(8.3, 0.0)]   # 0.3 m しか抜けていない
    assert not ms.crossed_fully(stop_on_line, a, b, 0.6)
    outside = [(x * 0.5, 3.0) for x in range(0, 22)]            # マーカーの外側を通った
    assert not ms.crossed_fully(outside, a, b, 0.6)
    back = [(x * 0.5, 0.0) for x in range(0, 17)] + [(8.4, 0.0), (7.0, 0.0), (9.0, 0.0)]
    assert ms.crossed_fully(back, a, b, 0.6)                    # 戻ってもう一度跨ぎ切った
