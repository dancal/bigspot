"""Pure remappable button mapping, shared by ROS Joy and message-level tests."""
import numpy as np
from .dance import DANCES
from .expressions import CUTES
from .poses import BODY_POSES

# All grounded routines are selectable without adding buttons. Unvalidated
# jumping/vertical acrobatics retain separate, experiment-gated shortcuts.
SELECTABLE=('dance_funk','dance_steps','dance_show','dance_rhythm','dance_big','dance','dance_sway','dance_bounce',
            'cute','cute_tilt','cute_wave','cute_bow',*BODY_POSES,'seated_clap','turn_left','turn_right')

DEFAULTS={'start_button':0,'stop_button':1,'jump_button':2,'run_button':3,
          'boost_button':4,'deadman_button':5,'bow_button':6,'crouch_button':7,
          'seated_clap_button':8,'rear_stand_button':-1,'handstand_button':9,
          'turn_left_button':10,'turn_right_button':11,'dance_button':12,'cute_button':13,
          'dance_action':'dance_funk','cute_action':'cute',
          'selection_actions':','.join(SELECTABLE),
          'forward_axis':1,'lateral_axis':0,'yaw_axis':3,
          'forward_speed':.06,'lateral_speed':.05,'yaw_speed':.4,
          'boost_multiplier':2.5,'run_speed':.18}


class JoyMapping:
    def __init__(self,settings=None):
        self.settings={**DEFAULTS,**(settings or {})}
        if self.settings['dance_action'] not in DANCES or self.settings['cute_action'] not in CUTES:
            raise ValueError('Invalid dance/cute button action')
        self.catalog=tuple(self.settings['selection_actions'].split(','))
        if not self.catalog or len(set(self.catalog))!=len(self.catalog) or any(a not in SELECTABLE for a in self.catalog):
            raise ValueError('Selection must contain unique supported grounded actions')
        self.selected_index=0
        self.previous={}
        self.running=False
        self.dance_requested=False

    @property
    def selected_action(self):
        return self.catalog[self.selected_index]

    def status(self):
        return {'selected_action':self.selected_action,'selection_index':self.selected_index,
                'selection_count':len(self.catalog),'catalog':list(self.catalog),
                'dance_action':self.settings['dance_action'],'cute_action':self.settings['cute_action'],
                'physical_button_mapping_verified':False}

    def button(self,buttons,key):
        i=self.settings[key]
        return 0<=i<len(buttons) and bool(buttons[i])

    def axis(self,axes,key):
        i=self.settings[key]
        v=float(axes[i]) if 0<=i<len(axes) else 0.
        return 0. if abs(v)<.1 else float(np.clip(v,-1,1))

    def events(self,axes,buttons):
        if not np.isfinite(axes).all():
            return [],None
        active={key:self.button(buttons,key) for key in DEFAULTS if key.endswith('_button')}
        previous=self.previous
        edge=lambda key: active[key] and not previous.get(key,False)
        self.previous=active
        if active['stop_button']:
            self.running=False
            self.dance_requested=False
            return (['stop'] if edge('stop_button') else []),False
        enable=True if edge('start_button') else None
        actions=[]
        if active['deadman_button']:
            selecting=active['boost_button']
            if selecting:
                # Boost + dance/cute cycles; boost + bow executes. Selection
                # alone publishes no robot action and never starts a routine.
                if edge('dance_button'):
                    self.selected_index=(self.selected_index-1)%len(self.catalog)
                elif edge('cute_button'):
                    self.selected_index=(self.selected_index+1)%len(self.catalog)
                if edge('bow_button'):
                    actions.append(self.selected_action)
                    self.dance_requested=True
            if edge('run_button'):
                self.running=not self.running
                actions.append('run' if self.running else 'walk')
            if edge('jump_button'):
                x=self.axis(axes,'forward_axis')
                y=self.axis(axes,'lateral_axis')
                actions.append(('jump_left' if y>0 else 'jump_right') if abs(y)>.5
                               else ('jump_forward' if x>0 else 'jump_back') if abs(x)>.5 else 'jump')
            for action in ['bow','crouch','seated_clap','rear_stand','handstand','turn_left','turn_right','dance','cute']:
                if selecting and action in ('bow','dance','cute'):
                    continue
                if edge(action+'_button'):
                    actions.append(self.settings[action+'_action'] if action in ('dance','cute') else
                                   action+'_experimental' if action in ('rear_stand','handstand') else action)
                    if action in ('bow','crouch','seated_clap','turn_left','turn_right','dance','cute'):
                        self.dance_requested=True
        return actions,enable

    def cancel_dance_if_unsafe(self,axes,buttons,fresh=True):
        if self.dance_requested and (not fresh or not np.isfinite(axes).all() or
                not self.button(buttons,'deadman_button')):
            self.dance_requested=False
            return True
        return False

    def velocity(self,axes,buttons,fresh=True):
        if not fresh or not np.isfinite(axes).all() or self.button(buttons,'stop_button') or not self.button(buttons,'deadman_button'):
            return np.zeros(3)
        boost=self.button(buttons,'boost_button')
        if boost and any(self.button(buttons,k) for k in ('dance_button','cute_button','bow_button')):
            return np.zeros(3)
        forward=self.settings['run_speed'] if self.running else self.settings['forward_speed']*(self.settings['boost_multiplier'] if boost else 1)
        lateral=self.settings['lateral_speed']*(1.5 if boost else 1.)
        yaw=self.settings['yaw_speed']*(1.5 if boost else 1.)
        return np.array([self.axis(axes,'forward_axis')*forward,
                         self.axis(axes,'lateral_axis')*lateral,self.axis(axes,'yaw_axis')*yaw])
