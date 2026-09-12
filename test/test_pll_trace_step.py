"""Exercise the actual optional injection helper without an FPGA."""
from pathlib import Path
import subprocess


def test_phase_step_bounds_atomicity_and_polarity(tmp_path):
    header = Path(__file__).resolve().parents[1] / 'litex_wr_nic/firmware/pll_trace_step.h'
    source = r'''
#include <assert.h>
#include <errno.h>
struct spll_main_state { int locked, phase_shift_target, phase_shift_current, adder_ref; };
struct { struct spll_main_state mpll; } softpll;
unsigned spll_trace_burst;
int reverse_spll = 1, irq_disabled;
void disable_irq(void) { assert(!irq_disabled); irq_disabled=1; }
void enable_irq(void) { assert(irq_disabled); irq_disabled=0; }
void mpll_set_phase_shift(struct spll_main_state *s, int ps) { s->phase_shift_target=ps*16384/16000; }
''' + '#include "' + str(header) + '"\n' + r'''
int main(void) {
    assert(spll_debug_step(256)==-EBUSY && !irq_disabled);
    softpll.mpll.locked=1;
    assert(spll_debug_step(256)==0 && !irq_disabled);
    assert(softpll.mpll.phase_shift_current==262);
    assert(softpll.mpll.phase_shift_target==262 && softpll.mpll.adder_ref==-262);
    assert(spll_trace_burst==640);
    assert(spll_debug_step(0)==-EBUSY && !irq_disabled);
    spll_trace_burst=0;
    assert(spll_debug_step(2000)==-EINVAL && !irq_disabled);
    assert(softpll.mpll.phase_shift_target==262 && softpll.mpll.adder_ref==-262);
    assert(spll_debug_step(0)==0 && softpll.mpll.adder_ref==0);
    spll_trace_burst=0; reverse_spll=0;
    assert(spll_debug_step(-256)==0 && softpll.mpll.adder_ref==-262);
    assert(spll_debug_step(2147483647)==-EINVAL && !irq_disabled);
    return 0;
}
'''
    c = tmp_path / 'step.c'
    c.write_text(source)
    executable = tmp_path / 'step'
    subprocess.run(['cc', '-std=c99', '-Wall', '-Werror', str(c), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
