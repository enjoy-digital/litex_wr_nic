"""Run WRPC regression cases against the pinned, patched C functions."""
import importlib.util
from pathlib import Path
import re
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'litex_wr_nic/firmware/wrpc-sw'


def function(text, name):
    start = re.search(r'^(?:static )?(?:int|void) ' + name + r'\(', text, re.M).start()
    brace = text.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


@pytest.fixture
def firmware(tmp_path, monkeypatch):
    if not SOURCE.exists():
        pytest.skip('Initialize pinned WRPC sources first')
    spec = importlib.util.spec_from_file_location('wr_build', ROOT / 'litex_wr_nic/firmware/build.py')
    build = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build)
    for name in ('include/board.h', 'dev/sfp.c', 'dev/spi_flash.c', 'dev/storage-cal.c'):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(subprocess.check_output(['git', '-C', str(SOURCE), 'show', build.COMMIT_HASH + ':' + name]))
    monkeypatch.setattr(build, 'CLONE_DIR', str(tmp_path))
    return build, tmp_path


def run_c(tmp_path, source):
    if shutil.which('cc') is None:
        pytest.skip('A host C compiler is required')
    path = tmp_path / 'regression.c'
    path.write_text(source)
    subprocess.run(['cc', '-std=c99', '-Werror=implicit-function-declaration', str(path), '-o', str(tmp_path / 'regression')], check=True, capture_output=True)
    return subprocess.run([str(tmp_path / 'regression')], capture_output=True, text=True)


def test_sfp_storage_error_is_not_a_calibration_match(firmware):
    build, path = firmware
    before = function((path / 'dev/sfp.c').read_text(), 'sfp_match')
    build.configure_source_fixes()
    after = function((path / 'dev/sfp.c').read_text(), 'sfp_match')
    # Stub only I2C and storage, exercising the actual sfp_match function.
    prefix = r'''
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#define HAS_SFP_DOM 0
#define SFP_DIAG_IMPLEMENTED 1
#define I2C_SFP_ADDRESS 0x50
#define I2C_SFP_DOM_ADDRESS 0x51
#define SFP_PN_LEN 16
#define SFP_NOT_MATCHED 0
#define SFP_MATCHED 1
struct shw_sfp_header { int diagnostic_monitoring_type; char vendor_pn[16]; } sfp_header;
struct shw_sfp_dom { char bytes[96]; } sfp_dom;
struct { struct shw_sfp_header *sfp_header; struct { char pn[16]; } sfp_params; int sfp_in_db; } sfp_info = { .sfp_header = &sfp_header };
static int result;
int sfp_present(void) { return 1; }
void sfp_read_i2c(int a, uint8_t *b, int c, int d) {}
int verify_checksum(uint8_t *a, int b, int c) { return 0; }
int pp_printf(const char *fmt, ...) { return 0; }
int storage_match_sfp(void *params) { return result; }
'''
    checks = r'''
int main(void) {
    result = 1; assert(sfp_match(1) == 0 && sfp_info.sfp_in_db == SFP_MATCHED);
    result = 0; assert(sfp_match(1) == -ENXIO && sfp_info.sfp_in_db == SFP_NOT_MATCHED);
    result = 1; sfp_match(1);
    result = -EIO; assert(sfp_match(1) == -EIO && sfp_info.sfp_in_db == SFP_NOT_MATCHED);
    result = -ENODEV; assert(sfp_match(1) == -ENODEV && sfp_info.sfp_in_db == SFP_NOT_MATCHED);
    return 0;
}
'''
    assert run_c(path, prefix + before + checks).returncode != 0
    result = run_c(path, prefix + after + checks)
    assert result.returncode == 0, result.stderr


def test_read_only_firmware_retains_calibration_without_writes(firmware):
    build, path = firmware
    build.configure_source_fixes(read_only_storage=True)
    contents = {p: p.read_text() for p in path.rglob('*.c')}
    build.configure_source_fixes(read_only_storage=True)
    assert all(p.read_text() == text for p, text in contents.items())
    flash = (path / 'dev/spi_flash.c').read_text()
    cal = (path / 'dev/storage-cal.c').read_text()
    prefix = r'''
#include <stdint.h>
#include <errno.h>
#include <assert.h>
struct spi_flash_device { void *bus; int use_4byte_addr; int sector_size; };
static int spi_accesses, saved, set_result;
static uint32_t parameter, value;
void bb_spi_cs(void *bus, int level) { spi_accesses++; }
void bb_spi_write(void *bus, int val, int count) { spi_accesses++; }
void bb_spi_delay(void *bus) { spi_accesses++; }
void spi_flash_write_addr(struct spi_flash_device *dev, uint32_t addr) { spi_accesses++; }
int spi_flash_rsr(struct spi_flash_device *dev) { spi_accesses++; return 0; }
int storage_set_calibration_parameter(uint32_t id, uint32_t val) { parameter=id; value=val; return set_result; }
int storage_save_calibration(void) { saved++; return 0; }
'''
    code = '\n'.join(function(flash, name) for name in ('spi_flash_write', 'spi_flash_erase_sector', 'spi_flash_erase'))
    code += function(cal, 'storage_set_calibration_parameter_and_save')
    checks = r'''
int main(void) {
    struct spi_flash_device dev = {.sector_size = 65536}; uint8_t byte = 0;
    assert(spi_flash_write(&dev, 0, &byte, 1) == -EROFS);
    assert(spi_flash_erase(&dev, 0, 65536) == -EROFS);
    spi_flash_erase_sector(&dev, 0);
    assert(spi_accesses == 0);
    assert(storage_set_calibration_parameter_and_save(7, 1234) == 0);
    assert(parameter == 7 && value == 1234 && saved == 0);
    set_result = -1;
    assert(storage_set_calibration_parameter_and_save(7, 5678) == -1);
    assert(saved == 0);
    return 0;
}
'''
    result = run_c(path, prefix + code + checks)
    assert result.returncode == 0, result.stderr


def test_diagnostics_use_cpu_map_not_host_map(firmware):
    build, path = firmware
    build.configure_source_fixes()
    board = (path / 'include/board.h').read_text()
    cpu_map = (SOURCE / 'include/hw/wrc_devices_map.h').read_text()
    expected = int(re.search(r'#define WRC_DEVICES_MAP_WDIAG (0x[0-9a-f]+)', cpu_map).group(1), 16)
    actual = int(re.search(r'#define BASE_WDIAGS_PRIV\s+\(DEV_BASE \+ (0x[0-9a-f]+)\)', board).group(1), 16)
    assert actual == expected == 0x800
