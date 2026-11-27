import math

import pytest

from trc_logging import history


@pytest.mark.parametrize('yaw_deg, compass', [
    (90.0, 0.0),      # 北
    (0.0, 90.0),      # 東
    (-90.0, 180.0),   # 南
    (180.0, 270.0),   # 西
    (45.0, 45.0),     # 北東
])
def test_enu_yaw_to_compass(yaw_deg, compass):
    assert history.enu_yaw_to_compass_deg(math.radians(yaw_deg)) == pytest.approx(compass)


def test_row_has_all_columns_and_blanks_for_missing():
    row = history.format_row(
        time_iso='2027-03-21T10:30:00.000+09:00', ros_time=1.5,
        latitude=None, longitude=None, map_x=None, map_y=None,
        yaw_enu=None, speed=None, gnss_quality=None,
        cmd_linear=None, cmd_angular=None, cmd_age=None, control_mode=None)
    assert len(row) == len(history.COLUMNS)
    assert row[0] == '2027-03-21T10:30:00.000+09:00'
    assert row[1] == '1.500'
    assert all(v == '' for v in row[2:])


def test_row_formats_values():
    row = history.format_row(
        time_iso='t', ros_time=10.0,
        latitude=35.540012345678, longitude=134.23, map_x=1.23456, map_y=-2.0,
        yaw_enu=math.radians(90.0), speed=0.5, gnss_quality=4,
        cmd_linear=0.3, cmd_angular=-0.1, cmd_age=0.05, control_mode='auto')
    d = dict(zip(history.COLUMNS, row))
    assert d['latitude'] == '35.540012346'
    assert d['map_x'] == '1.235'
    assert d['heading_deg'] == '0.0'
    assert d['gnss_quality'] == '4'
    assert d['cmd_angular'] == '-0.100'
    assert d['control_mode'] == 'auto'


def test_mission_status_column():
    row = history.format_row(
        time_iso='2027-03-20T10:00:00.000+09:00', ros_time=1.0, latitude=None, longitude=None,
        map_x=None, map_y=None, yaw_enu=None, speed=None, gnss_quality=None,
        cmd_linear=None, cmd_angular=None, cmd_age=None, control_mode='auto',
        mission_status='CP1 の奥（通過） に着きました（2/5）')
    d = dict(zip(history.COLUMNS, row))
    assert d['mission_status'] == 'CP1 の奥（通過） に着きました（2/5）'
