"""
over temp monitor w timer latch
dropout detection: range checks
    probe dc (adc == 4095)
    thermocouple dc (tc == [-1,0] and internal == [-1,0])
sensor fault: if fault (oc | scg | scv) persistent == dropout
event detection markers for overtemp (servo safe shutdown)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import math

# output struct, copied into Properties dataclass
@dataclass
class FaultFlags:
    # tc fault flags, if persistent
    fault: int = -1
    fault_oc: int = -1
    fault_scg: int = -1
    fault_scv: int = -1

    # dropout
    fault_drop: int = -1
    fault_probe_dc: int = -1
    fault_tc_dc: int = -1

    warn_overtemp: int = -1

    # non-critical fault indicator - switch to timer mode in fault_handler.py
    timer_mode: bool = False

    # timekeeping and debugging flags
    dt_s: float = math.nan
    overtemp_timer_s: float = math.nan
    probe_timer_s: float = math.nan
    tc_timer_s: float = math.nan
    sensor_fault_timer_s: float = math.nan

    def clear(self):
        self.fault = -1
        self.fault_oc = -1
        self.fault_scg = -1
        self.fault_scv = -1

        self.fault_drop = -1
        self.fault_probe_dc = -1
        self.fault_tc_dc = -1

        self.warn_overtemp = -1
        self.timer_mode = False

        self.dt_s = math.nan
        self.overtemp_timer_s = math.nan
        self.probe_timer_s = math.nan
        self.tc_timer_s = math.nan
        self.sensor_fault_timer_s = math.nan


# threshold config
@dataclass(frozen=True)
class ThresholdConfig:
    overtemp_set: float = 260.0 # flag as overtemp at 260ºC (500ºF)
    overtemp_clr: float = 255.0 # indicate overtemp handled at 255ºC (491ºF)
    overtemp_trip_time: float = 3.0 # trip flag at 3s in overtemp

    dropout_persist: float = 0.5 # adc==4095 or (tc==0 & internal==0) must persist for 0.5s or 50 samples
    fault_persist: float = 0.5 # og|scg|scv must persist for 0.5s

    probe_dc_adc: int = 4095 # when probe disconnects, reads constant 4095

    gate_overtemp: bool = True # if sensor dc, ignore temps
    dt_clamp: float = 0.25 # serial data loss, so it doesnt keep using old overtemp for long periods of time





class FaultMonitor:
    def __init__(self, cfg: ThresholdConfig = ThresholdConfig()):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self._last_t_us: Optional[int] = None
        self._probe_timer_s: float = 0.0
        self._tc_timer_s: float = 0.0
        self._sensor_fault_timer_s: float = 0.0

        # if using overtemp state (once it hits overtemp, latched into state -> shutdown?) -- with _update_overtemp_latch
        # self._overtemp_timer_s: float = 0.0 # how long its been overtemp for
        # self._overtemp_armed: bool = False # has measurement overtemp
        # self._overtemp_latched: bool = False # has enough measurements overtemp according to overtemp_trip_time in config

        self._overtemp_timer_s = 0.0
        self._overtemp_active = False

    def update(self, t_us: int, adc: int, tc_c: float, internal_c: float, fault: int, oc: int, scg: int, scv: int):
        out = FaultFlags()
        dt_s = self._compute_dt_s(t_us) # time since last signal received
        out.dt_s = dt_s

        # 1. probe dropout detection
        probe_dc_bool = (adc == self.cfg.probe_dc_adc) # if the current probe value is == value indicating disconnect (4095) -- def in config
        self._probe_timer_s = self._persistence_coutner(self._probe_timer_s, probe_dc_bool, dt_s) # updates the counter of how long fault active, if active
        out.probe_timer_s = self._probe_timer_s
        
        probe_dc_persist = (self._probe_timer_s >= self.cfg.dropout_persist)
        if probe_dc_persist:
            out.fault_drop = 1 # indicator for dc true
            out.fault_probe_dc = 1 # specifies dc source
            out.timer_mode = True # indicator for start timer controlled
        else:
            out.fault_drop = 0
            out.fault_probe_dc = 0


        # 2. thermocouple dropout detection
        tc_dc_bool = (-1.0 <= tc_c <= 0.0) and (-1.0 <= internal_c <= 0.0)
        self._tc_timer_s = self._persistence_coutner(self._tc_timer_s, tc_dc_bool, dt_s)
        out.tc_timer_s = self._tc_timer_s

        tc_dc_persist = (self._tc_timer_s >= self.cfg.dropout_persist)
        if tc_dc_persist:
            out.fault_tc_dc = 1
            out.fault_drop = 1
            out.timer_mode = True
        else:
            out.fault_tc_dc = 0
            # do not override out.fault_drop
            if out.fault_drop != 1:
                out.fault_drop = 0


        # 3. persistent fault flags on thermocouple
        tc_fault_bool = bool(oc or scg or scv)
        self._sensor_fault_timer_s = self._persistence_coutner(self._sensor_fault_timer_s, tc_fault_bool, dt_s)
        out.sensor_fault_timer_s = self._sensor_fault_timer_s

        tc_fault_persist = (self._sensor_fault_timer_s >= self.cfg.fault_persist)
        if tc_fault_persist:
            out.fault = 1
            out.fault_oc = 1 if oc else 0
            out.fault_scg = 1 if scg else 0
            out.fault_scv = 1 if scv else 0
            out.timer_mode = True
        else:
            out.fault = 0
            out.fault_oc = 0
            out.fault_scg = 0
            out.fault_scv = 0


        # 4. overtemp timer latch marker
        tc_valid_for_overtemp = True
        if self.cfg.gate_overtemp:
            # if in persistent fault mode, dont let tc control anything
            if probe_dc_persist or tc_dc_persist or tc_fault_persist:
                tc_valid_for_overtemp = False

        if tc_valid_for_overtemp:
            # self._update_overtemp_latch(tc_c, dt_s, out)
            self._update_overtemp_state(tc_c, dt_s, out)
        else:
            # if tc dropout, just report if overtemp already latched before dc
            # out.warn_overtemp = 1 if self._overtemp_latched else 0
            out.warn_overtemp = 1 if self._overtemp_active else 0

        out.overtemp_timer_s = self._overtemp_timer_s
        return out
                
            


    # def _update_overtemp_latch(self, tc_c: float, dt_s: float, out: FaultFlags):
    #     # already latched
    #     if self._overtemp_latched:
    #         out.warn_overtemp = 1
    #         return
        
    #     # arm if a measurement > threshold
    #     if (not self._overtemp_armed) and (tc_c >= self.cfg.overtemp_set):
    #         self._overtemp_armed = True

    #     # if armed
    #     if self._overtemp_armed:
    #         # update timer of how long armed
    #         if tc_c >= self.cfg.overtemp_set:
    #             self._overtemp_timer_s += dt_s

    #         # if drop below temp, clr armed
    #         elif tc_c <= self.cfg.overtemp_clr:
    #             self._overtemp_armed = False
    #             self._overtemp_timer_s = 0.0

    #         # if exceed threshold time, warn for overtemp (set marker)
    #         if self._overtemp_timer_s >= self.cfg.overtemp_trip_time:
    #             self._overtemp_latched = True
    #             out.warn_overtemp = 1
    #             return


    #     out.warn_overtemp = 0


    def _update_overtemp_state(self, tc_c: float, dt_s: float, out: FaultFlags) -> None:
        # if currently active, clear when we cooled down
        if self._overtemp_active:
            if tc_c <= self.cfg.overtemp_clr:
                self._overtemp_active = False
                self._overtemp_timer_s = 0.0
            out.warn_overtemp = 1 if self._overtemp_active else 0
            return

        # not active yet: only arm/enter if we are above setpoint
        if tc_c >= self.cfg.overtemp_set:
            self._overtemp_timer_s += dt_s
            if self._overtemp_timer_s >= self.cfg.overtemp_trip_time:
                self._overtemp_active = True
        else:
            self._overtemp_timer_s = 0.0

        out.warn_overtemp = 1 if self._overtemp_active else 0





    def _compute_dt_s(self, t_us: int):
        # if no last received time
        if self._last_t_us is None:
            self._last_t_us = t_us
            return 0.0
        
        # calculate change in time since last received signal
        dt_us = t_us - self._last_t_us
        # update new last time
        self._last_t_us = t_us

        # edge case/error handling
        if dt_us <= 0:
            return 0.0
        
        dt_s = dt_us * 1e-6

        # if exceed sensor/serial dropout time limit, clamp time to designated max time
        if dt_s > self.cfg.dt_clamp:
            dt_s = self.cfg.dt_clamp

        return dt_s
    
    # if fault active, start counting how long fault is active for
        # after called, will compare against max time fault needs to be active for to consider a real fault and set marker/flag
    @staticmethod
    def _persistence_coutner(timer_s: float, condition: bool, dt_s: float):
        if condition:
            return timer_s + dt_s
        return 0.0
    


def apply_fault_flags(props, flags: FaultFlags):
    props.fault = flags.fault
    props.fault_oc = flags.fault_oc
    props.fault_scg = flags.fault_scg
    props.fault_scv = flags.fault_scv

    props.fault_drop = flags.fault_drop
    props.fault_probe_dc = flags.fault_probe_dc
    props.fault_tc_dc = flags.fault_tc_dc

    props.warn_overtemp = flags.warn_overtemp

    if flags.timer_mode:
        props.timer_mode = True