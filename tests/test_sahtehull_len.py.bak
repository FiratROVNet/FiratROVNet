import pytest
import numpy as np

from FiratROVNet.kutuphane.helper import gnc_helper


@pytest.mark.skipif(not getattr(gnc_helper, 'SHAPELY_AVAILABLE', False), reason="Shapely not available")
def test_sahtehull_len_returns_number_of_points():
    class DummyHullManager:
        def hull(self, offset=40.0):
            points = [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]]
            center = (5.0, 5.0, 0.0)
            return {'hull': None, 'points': np.array(points), 'center': center}

    class DummyFilo:
        def __init__(self):
            self.hull_manager = DummyHullManager()
            self.ortam_ref = None

    helper = gnc_helper.FiloHelper(DummyFilo())
    result = helper.yeni_hull(yasakli_noktalar=[])

    assert isinstance(result, dict)
    hull = result.get('hull')
    points = result.get('points')
    assert hull is not None
    assert points is not None
    # SahteHull should support len()
    assert len(hull) == len(points)
    # Also indexing and iteration should work
    assert np.allclose(hull[0], points[0])
    assert list(iter(hull))[0] is not None
