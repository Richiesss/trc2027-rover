import pytest

from trc_gnss import nmea


def with_checksum(body: str) -> str:
    """`$` と `*` の間の文字列から、チェックサム付きのセンテンスを作る。"""
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    return f'${body}*{calc:02X}'


def test_checksum_known_sentence():
    # NMEA の説明でよく使われる例文（チェックサム 47）
    assert nmea.checksum_ok('$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47')
    assert not nmea.checksum_ok('$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48')


def test_parse_single_fix():
    fix = nmea.parse_gga('$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47')
    assert fix is not None
    assert fix.quality == nmea.QUALITY_SINGLE
    assert fix.latitude == pytest.approx(48 + 7.038 / 60)
    assert fix.longitude == pytest.approx(11 + 31.0 / 60)
    assert fix.num_satellites == 8
    assert fix.ellipsoid_height == pytest.approx(545.4 + 46.9)


def test_parse_rtk_fixed_southwest():
    s = with_checksum('GNGGA,031500.00,3530.12345,S,13945.67890,W,4,24,0.6,10.0,M,36.7,M,1.0,0000')
    fix = nmea.parse_gga(s)
    assert fix.quality == nmea.QUALITY_RTK_FIXED
    assert fix.latitude == pytest.approx(-(35 + 30.12345 / 60))
    assert fix.longitude == pytest.approx(-(139 + 45.67890 / 60))


def test_no_fix_and_garbage_return_none():
    assert nmea.parse_gga(with_checksum('GNGGA,031500.00,,,,,0,00,99.99,,,,,,')) is None
    assert nmea.parse_gga(with_checksum('GNRMC,031500.00,A,3530.1,N,13945.6,E,0.0,,261126,,,A')) is None
    assert nmea.parse_gga('not a sentence') is None


def test_gga_quality_reports_no_fix():
    assert nmea.gga_quality(with_checksum('GNGGA,031500.00,,,,,0,00,99.99,,,,,,')) == 0
    assert nmea.gga_quality(
        with_checksum('GNGGA,031500.00,3530.1,N,13945.6,E,4,24,0.6,10.0,M,36.7,M,1.0,0000')) == 4
    assert nmea.gga_quality(with_checksum('GNRMC,031500.00,V,,,,,,,261126,,,N')) is None
    assert nmea.gga_quality('$GNGGA,031500.00,,,,,0,00,99.99,,,,,,*00') is None   # チェックサム不一致


def test_covariance_depends_on_quality():
    base = dict(utc='0', latitude=0.0, longitude=0.0, num_satellites=20,
                altitude=0.0, geoid_separation=0.0)
    fixed = nmea.GgaFix(quality=nmea.QUALITY_RTK_FIXED, hdop=0.6, **base)
    single = nmea.GgaFix(quality=nmea.QUALITY_SINGLE, hdop=2.0, **base)
    cov_fixed = nmea.position_covariance(fixed, nmea.DEFAULT_SIGMA)
    cov_single = nmea.position_covariance(single, nmea.DEFAULT_SIGMA)
    assert cov_fixed[0] == pytest.approx(0.02 ** 2)
    assert cov_fixed[8] == pytest.approx((0.02 * 2.0) ** 2)
    # 単独測位は HDOP 2.0 を掛けて 5.0 m
    assert cov_single[0] == pytest.approx(5.0 ** 2)
    assert cov_single[0] > cov_fixed[0]
