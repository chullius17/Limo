"""Translation of mpc_timing.cpp: synchronous, bounded timing diagnostics."""
import math
import time
from ...include.python_ver.mpc_timing import TIMING_FIELDS, Window, CriticWindow
from ...include.python_ver.ros_compat import DiagnosticArray, DiagnosticStatus, Header, KeyValue, read_parameter


class MpcTimingPublisher:
    def __init__(self, node, name, budget_ms, now=time.monotonic):
        self.node_, self.name_, self.now_, self.budget_ms_ = node, name, now, budget_ms
        prefix = name + '.MPC.Timing.'
        enabled = read_parameter(node, prefix + 'enabled', False)
        rate = read_parameter(node, prefix + 'publish_rate', 1.0)
        topic = read_parameter(node, prefix + 'topic', '/limo/control/mpc_timing')
        self.log_summary_ = read_parameter(node, prefix + 'log_summary', False)
        if (not math.isfinite(rate) or not 0 < rate <= 10 or not topic or
                not math.isfinite(budget_ms) or budget_ms <= 0 or not callable(now)):
            raise ValueError('Invalid MPC.Timing publish_rate, topic, clock or cycle budget')
        self.period_ = 1.0 / rate
        self.publisher_ = node.create_publisher(topic, qos=1) if enabled else None
        self.active_, self.critic_names_, self.critic_windows_ = False, [], []
        self.reset()

    def enabled(self):
        return self.publisher_ is not None

    def setCriticNames(self, names):
        if self.active_:
            raise RuntimeError('Cannot change MPC timing critic names while active')
        if any(not name for name in names) or len(set(names)) != len(names):
            raise ValueError('MPC timing critic names must be nonempty and unique')
        self.critic_names_ = list(names)
        self.reset()

    def reset(self):
        self.window_ = Window()
        self.critic_windows_ = [CriticWindow() for _ in self.critic_names_]
        self.window_start_ = None

    def activate(self):
        self.reset()
        if self.publisher_ is not None:
            self.publisher_.on_activate()
            self.active_ = True

    def deactivate(self):
        self.active_ = False
        self.reset()
        if self.publisher_ is not None:
            self.publisher_.on_deactivate()

    def report(self, now):
        w = self.window_
        status = DiagnosticStatus(name=self.name_ + '/mpc_timing', hardware_id=self.node_.get_name(),
            level=1 if w.failures or w.deadline_misses else 0,
            message='MPC cycles failed' if w.failures else
                'MPC cycle budget exceeded' if w.deadline_misses else 'MPC within cycle budget')

        def add(key, value):
            status.values.append(KeyValue(key, str(value)))

        for key, value in (('cycles', w.cycles), ('failures', w.failures), ('deadline_misses', w.deadline_misses),
                ('candidate_generated', w.generated), ('candidate_evaluated', w.evaluated), ('candidate_rejected', w.rejected),
                ('detour_cycles', w.detour_cycles), ('grid_updates', w.grid_updates), ('budget_ms', self.budget_ms_),
                ('window_ms', (now - self.window_start_) * 1000.0)):
            add(key, value)
        for name, metric in zip(TIMING_FIELDS, w.times):
            add(name + '_mean', metric.mean())
            add(name + '_max', metric.maximum)
        for name, critic in zip(self.critic_names_, self.critic_windows_):
            wall, cpu = 'critic_prepare/' + name, 'critic_prepare_cpu/' + name
            add(wall + '_ms_mean', critic.prepare.mean())
            add(wall + '_ms_max', critic.prepare.maximum)
            add(cpu + '_ms_mean', critic.prepare_cpu.mean())
            add(cpu + '_ms_max', critic.prepare_cpu.maximum)
            add(wall + '_calls', critic.calls)
        return DiagnosticArray(Header(stamp=self.node_.now()), [status])

    def finish(self, sample):
        if self.publisher_ is None or not self.active_:
            return
        try:
            now, w = self.now_(), self.window_
            if not w.cycles:
                self.window_start_ = now
            for name, metric in zip(TIMING_FIELDS, w.times):
                metric.add(getattr(sample, name))
            if len(sample.critic_prepare) == len(self.critic_windows_):
                for source, dest in zip(sample.critic_prepare, self.critic_windows_):
                    if source.called:
                        dest.prepare.add(source.prepare_ms)
                        dest.prepare_cpu.add(source.prepare_cpu_ms)
                        dest.calls += 1
            w.cycles += 1
            w.failures += not sample.success
            w.deadline_misses += sample.cycle_ms > self.budget_ms_
            w.generated += sample.generated_count
            w.evaluated += sample.evaluated_count
            w.rejected += sample.rejected_count
            w.detour_cycles += sample.detouring
            w.grid_updates += sample.grid_updated
            if now - self.window_start_ < self.period_:
                return
            self.publisher_.publish(self.report(now))
            if self.log_summary_:
                self.node_.get_logger().info(
                    'MPC timing: cycles=%d wall mean/max=%.2f/%.2f ms CPU=%.2f ms '
                    'solve=%.2f scoring=%.2f prepare=%.2f grid=%.2f debug=%.2f ms '
                    'lock=%.2f plan=%.2f critics=%.2f ms overruns=%d/%d failures=%d',
                    w.cycles, w.times[0].mean(), w.times[0].maximum, w.times[1].mean(),
                    w.times[4].mean(), w.times[5].mean(), w.times[2].mean(), w.times[3].mean(),
                    w.times[6].mean(), w.times[9].mean(), w.times[7].mean(), w.times[10].mean(),
                    w.deadline_misses, w.cycles, w.failures)
            self.reset()
        except Exception:
            self.reset()  # Diagnostics cannot replace a command or a control error.
