# variance from filter and derive for stability
# timer latch for sensor disconnect
# temp-state transitions

from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, Optional, Tuple, Any

class State(Enum):
    IDLE = auto()
    PREHEAT = auto()
    SEAR1 = auto()
    SEAR2 = auto()
    REST = auto()
    DONE = auto()
    ERROR = auto() # critical fault only: tc/probe dc or fault, servo shutdown

class Event(Enum):
    START = auto()
    CANCEL = auto() # user exit

    TEMP_REACHED = auto() # preheat temp reached
    FLIP = auto() # user flip command
    TIMER_EXPIRED = auto() # timer finished
    TARGET_MET = auto() # internal temp met

    FAULT = auto()
    RESET = auto() # recover from fault state

@dataclass(frozen=True)
class EventMsg:
    kind: Event
    payload: Dict[str, Any] = field(default_factory = dict)

Action = Callable[["FSM", EventMsg], None]
Guard = Callable[["FSM", EventMsg], bool]
Transition = Tuple[State, Optional[Guard], Optional[Action]]

@dataclass
class FSM:
    state: State = State.IDLE
    target_pan_temp_c: float = 210.0
    last_pan_temp_c: float = 0.0
    fault_code: Optional[int] = None

    transitions: Dict[Tuple[State, Event], Transition] = field(init=False, default_factory=dict)

    def __post_init__(self):
        self.transitions = {
            # (CURR_STATE, EVENT (dispatched when condition met)): (NEXT_STATE, GUARD (secondary condition), (resulting) ACTION)

            # standard path
            (State.IDLE, Event.START): (State.PREHEAT, None, self._act_start),
            (State.PREHEAT, Event.TEMP_REACHED): (State.SEAR1, self._guard_temp_reached, self._act_begin_sear1),
            (State.SEAR1, Event.FLIP): (State.SEAR2, None, self._act_flip_and_begin_sear2),
            (State.SEAR2, Event.TARGET_MET): (State.REST, None, self._act_begin_rest),
            (State.REST, Event.TIMER_EXPIRED): (State.DONE, None, self._act_done),

            # user cancel
            (State.PREHEAT, Event.CANCEL): (State.IDLE, None, self._act_cancel),
            (State.SEAR1, Event.CANCEL): (State.IDLE, None, self._act_cancel),
            (State.SEAR2, Event.CANCEL): (State.IDLE, None, self._act_cancel),
            (State.REST, Event.CANCEL): (State.IDLE, None, self._act_cancel),

            # fault handling
            (State.IDLE, Event.FAULT): (State.ERROR, None, self._act_fault),
            (State.PREHEAT, Event.FAULT): (State.ERROR, None, self._act_fault),
            (State.SEAR1, Event.FAULT): (State.ERROR, None, self._act_fault),
            (State.SEAR2, Event.FAULT): (State.ERROR, None, self._act_fault),
            (State.REST, Event.FAULT): (State.ERROR, None, self._act_fault),
            (State.DONE, Event.FAULT): (State.ERROR, None, self._act_fault),

            # fault recovery and reset
            (State.ERROR, Event.RESET): (State.IDLE, None, self._act_reset),
            (State.DONE, Event.RESET): (State.IDLE, None, self._act_reset),
        }
    
    def dispatch(self, msg: EventMsg):
        key = (self.state, msg.kind)
        t = self.transitions.get(key)

        if t is None:
            self._log(f"IGNORED {msg.kind.name} in {self.state.name}")
            return False
        
        next_state, guard, action = t

        # if guard not met, but event dispatched
        if guard is not None and not guard(msg):
            self._log(f"BLOCKED {msg.kind.name} in {self.state.name} (guard false)")
            return False

        if action is not None:
            action(msg)

        old = self.state
        self.state = next_state
        self._log(f"STATE {old.name} -> {self.state.name} on {msg.kind.name}")
        return True
    

    # update
    def update_pan_temp(self, temp_c: float):
        self.last_pan_temp_c = temp_c

    # guards
    def _guard_temp_reached(self, msg: EventMsg):
        return self.last_pan_temp_c >= self.target_pan_temp_c
    


    # actions
    def _act_start(self, msg: EventMsg) -> None:
        self._log("Start button pressed")

    def _act_begin_sear1(self, msg: EventMsg) -> None:
        self._log("Pan temperature reached")

    def _act_flip_and_begin_sear2(self, msg: EventMsg) -> None:
        self._log("Flip food")

    def _act_begin_rest(self, msg: EventMsg) -> None:
        self._log("Food has reached internal temperature, remove to rest")

    def _act_done(self, msg: EventMsg) -> None:
        self._log("Serve temp reached, done resting")

    def _act_cancel(self, msg: EventMsg) -> None:
        self._log("Canceled")

    def _act_fault(self, msg: EventMsg) -> None:
        self.fault_code = msg.payload.get("code")
        self._log(f"fault -> code={self.fault_code}; safe shutdown")

    def _act_reset(self, msg: EventMsg) -> None:
        self.fault_code = None
        self._log("Reset, returned to idle")

    def _log(self, s: str) -> None:
        print(s)