import math

from sensor_msgs.msg import NavSatFix, NavSatStatus

from trc_localization.gnss_adapter_node import checked_covariance, quality_of


def fix(status, cov=None, kind=NavSatFix.COVARIANCE_TYPE_UNKNOWN):
    m = NavSatFix()
    m.status.status = status
    if cov is not None:
        m.position_covariance = cov
        m.position_covariance_type = kind
    return m


def test_quality_routes():
    S = NavSatStatus
    assert quality_of('fix', S.STATUS_GBAS_FIX) == 4
    assert quality_of('clas', S.STATUS_GBAS_FIX) == 4
    assert quality_of('gnss', S.STATUS_GBAS_FIX) == 5        # RTK FLOAT
    assert quality_of('gnss', S.STATUS_SBAS_FIX) == 2
    assert quality_of('gnss', S.STATUS_FIX) == 1
    assert quality_of('gnss', S.STATUS_NO_FIX) == 0


def test_unknown_covariance_is_filled():
    # GGA だけのとき：共分散 0・UNKNOWN。単独測位なら 3 m とみなす（0 のまま渡さない）
    cov, kind, filled = checked_covariance(fix(NavSatStatus.STATUS_FIX), 1, 0.01)
    assert filled and kind == NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    assert math.isclose(math.sqrt(cov[0]), 3.0) and math.isclose(math.sqrt(cov[8]), 6.0)
    # RTK Fix で精度不明なら 5 cm（向きの初期化の「5 cm 以下」をぎりぎり満たす）
    cov, _, _ = checked_covariance(fix(NavSatStatus.STATUS_GBAS_FIX), 4, 0.01)
    assert math.isclose(math.sqrt(cov[0]), 0.05)


def test_known_covariance_is_kept_with_floor():
    known = [0.0004, 0.0001, 0.0, 0.0001, 0.0004, 0.0, 0.0, 0.0, 0.0016]
    cov, kind, filled = checked_covariance(fix(NavSatStatus.STATUS_GBAS_FIX, known, NavSatFix.COVARIANCE_TYPE_KNOWN),
                                           4, 0.01)
    assert not filled and kind == NavSatFix.COVARIANCE_TYPE_KNOWN and cov == known
    # 楽観的すぎる値（1 mm）は 1 cm にする
    tiny = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]
    cov, _, _ = checked_covariance(fix(NavSatStatus.STATUS_GBAS_FIX, tiny, NavSatFix.COVARIANCE_TYPE_KNOWN), 4, 0.01)
    assert math.isclose(cov[0], 1e-4) and math.isclose(cov[8], 4e-4)


def test_zero_diagonal_with_known_type_is_filled():
    zeros = [0.0] * 9
    _, _, filled = checked_covariance(fix(NavSatStatus.STATUS_FIX, zeros, NavSatFix.COVARIANCE_TYPE_KNOWN), 1, 0.01)
    assert filled
