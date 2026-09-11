/* Optional bench instrumentation, included after softpll in softpll_ng.c.
 * Copyright (c) 2026 Enjoy-Digital. SPDX-License-Identifier: BSD-2-Clause
 */
extern unsigned spll_trace_burst;
extern int reverse_spll;
int spll_debug_step(int phase_ps);

int spll_debug_step(int phase_ps)
{
	struct spll_main_state *s = (struct spll_main_state *)&softpll.mpll;
	int delta, old_target;

	/* Bound conversion arithmetic and reject injections during acquisition. */
	if (phase_ps < -16000 || phase_ps > 16000)
		return -EINVAL;
	disable_irq();
	if (!s->locked || spll_trace_burst) {
		enable_irq();
		return -EBUSY;
	}
	old_target = s->phase_shift_target;
	mpll_set_phase_shift(s, phase_ps);
	delta = s->phase_shift_target - s->phase_shift_current;
	/* At 62.5 MHz, limit each step to 500 ps, below the lock threshold. */
	if (delta < -512 || delta > 512) {
		s->phase_shift_target = old_target;
		enable_irq();
		return -EINVAL;
	}
	s->adder_ref += reverse_spll ? -delta : delta;
	s->phase_shift_current = s->phase_shift_target;
	/* At decimation 16, 640 main samples plus helper records fit the FIFO.
	 * Drain the FIFO before each injection. Decimation resumes automatically. */
	spll_trace_burst = 640;
	enable_irq();
	return 0;
}
