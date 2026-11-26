import math

import pytest

from trc_localization import heading_math as hm


def test_latlon_to_local_en_one_metre_north():
    lat0, lon0 = 35.5, 134.2          # 鳥取砂丘付近
    dlat = math.degrees(1.0 / hm.EARTH_RADIUS)
    east, north = hm.latlon_to_local_en(lat0, lon0, lat0 + dlat, lon0)
    assert east == pytest.approx(0.0, abs=1e-6)
    assert north == pytest.approx(1.0, rel=1e-6)


def test_rotation_when_robot_starts_facing_north():
    # ロボットはオドメトリ上 +x（前）に 5 m 進んだが、GNSS では北に 5 m 動いた
    # → オドメトリの x 軸は北（ENU で +90 度）を向いている
    rot = hm.estimate_rotation((0.0, 5.0), (5.0, 0.0), min_distance=3.0, max_length_mismatch=0.5)
    assert rot == pytest.approx(math.pi / 2)


def test_rotation_works_on_curved_path():
    # オドメトリ座標を 30 度回すと ENU になる場合（曲がって進んでも始点→終点の向きで比べる）
    true_rot = math.radians(30)
    odom = (4.0, 3.0)
    gnss = (odom[0] * math.cos(true_rot) - odom[1] * math.sin(true_rot),
            odom[0] * math.sin(true_rot) + odom[1] * math.cos(true_rot))
    rot = hm.estimate_rotation(gnss, odom, 3.0, 0.5)
    assert rot == pytest.approx(true_rot)


def test_accepts_odometry_slightly_shorter_than_threshold():
    # GNSS 3.01 m / オドメトリ 2.97 m（配信タイミングの差）は受け入れる
    rot = hm.estimate_rotation((0.0, 3.01), (2.97, 0.0), 3.0, 0.5)
    assert rot == pytest.approx(math.pi / 2)


def test_rejects_short_or_inconsistent_motion():
    assert hm.estimate_rotation((1.0, 0.0), (1.0, 0.0), 3.0, 0.5) is None   # 短すぎる
    assert hm.estimate_rotation((5.0, 0.0), (3.5, 0.0), 3.0, 0.5) is None   # 距離が合わない（空転など）


def test_quaternion_roundtrip():
    for deg in (-170, -90, 0, 45, 179):
        yaw = math.radians(deg)
        assert hm.yaw_from_quaternion(*hm.quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_body_velocity():
    import math
    from trc_localization.odom_to_twist_node import body_velocity
    # 北（+y）を向いて北へ 0.5 m 進み、左に 0.1 rad 回った（0.1 秒）
    vx, vy, w, dt = body_velocity((0.0, 0.0, 0.0, math.pi / 2), (0.1, 0.0, 0.05, math.pi / 2 + 0.01))
    assert abs(vx - 0.5) < 1e-9 and abs(vy) < 1e-9 and abs(w - 0.1) < 1e-9 and abs(dt - 0.1) < 1e-12
    assert body_velocity((1.0, 0, 0, 0), (1.0, 1, 0, 0)) is None
