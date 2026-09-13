"""A successful Vivado build must still produce a usable board image and map."""
import hashlib
import importlib.util
from pathlib import Path
import subprocess

import pytest

spec = importlib.util.spec_from_file_location('wr_bench_build', Path(__file__).resolve().parents[1] / 'tools/build_wr_bench.py')
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


@pytest.fixture
def generated(tmp_path, monkeypatch):
    monkeypatch.setattr(build, 'ROOT', tmp_path)
    (tmp_path / 'test').mkdir()
    (tmp_path / 'test/csr.csv').write_text('csr_register,refclk_dac_current,0xf000a80c,1,ro\n')
    output = tmp_path / 'output'
    (output / 'gateware').mkdir(parents=True)
    (output / 'gateware/board_timing.rpt').write_text('All user specified timing constraints are met.')
    return output


@pytest.mark.parametrize('board', ['acorn', 'spec', 'hyvision'])
def test_build_only_outputs_become_programmable(board, generated, monkeypatch):
    image = generated / 'gateware' / Path(build.BOARDS[board][2]).with_suffix('.bit')
    image.write_bytes(b'Vivado output')
    def convert(command, **kwargs):
        assert board == 'spec'
        assert Path(command[-2]) == image
        assert Path(command[-1]) == image.with_suffix('.bin')
        assert kwargs['check'] is True
        Path(command[-1]).write_bytes(b'Converted 35T image')
    monkeypatch.setattr(build.subprocess, 'run', convert)
    hashes = build.finish_build(board, generated)
    expected_image = image.with_suffix('.bin') if board == 'spec' else image
    assert str(expected_image.relative_to(generated)) in hashes
    assert (generated / 'csr.csv').read_bytes() == (build.ROOT / 'test/csr.csv').read_bytes()
    assert hashes['csr.csv'] == hashlib.sha256((generated / 'csr.csv').read_bytes()).hexdigest()


def test_converter_failure_is_not_a_successful_build(generated, monkeypatch):
    def fail(command, **kwargs):
        raise subprocess.CalledProcessError(1, command)
    monkeypatch.setattr(build.subprocess, 'run', fail)
    with pytest.raises(subprocess.CalledProcessError):
        build.finish_build('spec', generated)


def test_missing_programming_image_is_rejected(generated):
    with pytest.raises(RuntimeError, match='programming image'):
        build.finish_build('acorn', generated)
