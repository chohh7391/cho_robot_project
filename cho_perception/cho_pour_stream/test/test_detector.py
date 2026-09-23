"""
The stream detector against synthetic frames.

What is pinned here is the behaviour that makes the detector useful to a pour,
not its arithmetic: that it sees the start quickly, that it keeps seeing a
stream that has been running for a while, that things which are not streams do
not read as one, and that it says "no information" rather than guessing when the
view is blocked.
"""
import numpy as np
import pytest

from cho_pour_stream.detector import Band, DetectorConfig, StreamDetector

HEIGHT, WIDTH = 200, 320


def scene(value=100):
    return np.full((HEIGHT, WIDTH), value, np.uint8)


def noisy(image, rng, spread=3):
    jitter = rng.integers(-spread, spread + 1, image.shape)
    return np.clip(image.astype(int) + jitter, 0, 255).astype(np.uint8)


def stream_frame(base, x0=158, x1=163, value=200):
    """Draw a vertical line crossing every row of the band, as free fall does."""
    frame = base.copy()
    frame[:, x0:x1] = value
    return frame


@pytest.fixture
def detector():
    return StreamDetector(DetectorConfig(band=Band(0.05, 0.40, 0.95, 0.55)))


@pytest.fixture
def rng():
    return np.random.default_rng(0)


def settle(detector, rng, frames=25):
    state = None
    for _ in range(frames):
        state = detector.update(noisy(scene(), rng))
    return state


def test_warmup_reports_no_information_rather_than_no_flow(rng):
    detector = StreamDetector(DetectorConfig(band=Band(0.05, 0.40, 0.95, 0.55)))
    state = detector.update(noisy(scene(), rng))
    assert not state.valid
    assert not state.flowing
    assert 'background' in state.status


def test_quiet_scene_is_valid_and_not_flowing(detector, rng):
    state = settle(detector, rng)
    assert state.valid
    assert not state.flowing


def test_a_stream_is_seen_within_open_frames(detector, rng):
    settle(detector, rng)
    frame = stream_frame(scene())
    for _ in range(detector.config.open_frames):
        state = detector.update(noisy(frame, rng))
    assert state.flowing
    assert state.coverage == pytest.approx(1.0)


def test_a_running_stream_is_not_absorbed_into_the_background(detector, rng):
    # The failure this guards is specific and quiet: a background model that
    # keeps updating during a pour dissolves the stream into itself within a
    # second or two, after which a pour in full flow reads as stopped.
    settle(detector, rng)
    frame = stream_frame(scene())
    for _ in range(200):
        state = detector.update(noisy(frame, rng))
    assert state.flowing
    assert state.coverage == pytest.approx(1.0)


def test_the_stream_is_released_after_it_stops(detector, rng):
    settle(detector, rng)
    frame = stream_frame(scene())
    for _ in range(10):
        detector.update(noisy(frame, rng))
    for _ in range(detector.config.close_frames + 2):
        state = detector.update(noisy(scene(), rng))
    assert not state.flowing


def test_a_gap_shorter_than_close_frames_does_not_read_as_a_stop(detector, rng):
    settle(detector, rng)
    frame = stream_frame(scene())
    for _ in range(10):
        detector.update(noisy(frame, rng))
    for _ in range(detector.config.close_frames - 1):
        state = detector.update(noisy(scene(), rng))
    assert state.flowing, 'sensor noise must not be able to break a stream'


def test_a_splash_does_not_read_as_a_stream(detector, rng):
    # Many changed pixels, but only a few rows: free fall crosses the band, a
    # splash does not. This is the discriminator that earns its place.
    settle(detector, rng)
    splash = scene()
    splash[80:84, :] = 200
    for _ in range(8):
        state = detector.update(noisy(splash, rng))
    assert not state.flowing
    assert state.coverage < detector.config.min_row_coverage


def test_an_occlusion_reports_no_information_not_a_torrent(detector, rng):
    settle(detector, rng)
    state = detector.update(scene(15))
    assert not state.valid
    assert not state.flowing
    assert 'occlusion' in state.status or 'lighting' in state.status


def test_a_lasting_change_is_relearned_rather_than_reported_forever(detector, rng):
    settle(detector, rng)
    for _ in range(detector.config.reset_after_invalid + 1):
        state = detector.update(noisy(scene(15), rng))
    assert 'background' in state.status
    for _ in range(40):
        state = detector.update(noisy(scene(15), rng))
    assert state.valid, 'the detector has to recover from someone switching a light'
    assert not state.flowing


def test_band_outside_the_image_is_refused():
    with pytest.raises(ValueError):
        StreamDetector(DetectorConfig(band=Band(0.0, 0.4, 1.4, 0.5)))


def test_a_band_taller_than_half_the_image_is_refused():
    # A band that tall inevitably contains the gripper or the receiving rim, and
    # both of them move.
    with pytest.raises(ValueError) as excinfo:
        StreamDetector(DetectorConfig(band=Band(0.1, 0.1, 0.9, 0.9)))
    assert 'free-fall' in str(excinfo.value)


def test_a_band_narrower_than_the_lip_arc_is_refused():
    with pytest.raises(ValueError) as excinfo:
        StreamDetector(DetectorConfig(band=Band(0.50, 0.40, 0.51, 0.55)))
    assert 'arc' in str(excinfo.value)


def test_band_slices_are_clamped_into_the_image():
    rows, cols = Band(0.0, 0.0, 1.0, 1.0).slice_for(HEIGHT, WIDTH)
    assert rows.start == 0 and rows.stop == HEIGHT
    assert cols.start == 0 and cols.stop == WIDTH
