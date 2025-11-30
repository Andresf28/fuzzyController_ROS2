import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np

class FuzzyNavigator:
    def __init__(self):

        self.dist_goal = ctrl.Antecedent(np.arange(0, 21, 0.1), 'dist_goal')
        self.angle_error = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.01), 'angle_error')
        self.v_nav = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'v_nav')
        self.w_nav = ctrl.Consequent(np.arange(-1, 1.1, 0.01), 'w_nav')

        self.dist_goal['Very_near'] = fuzz.trimf(self.dist_goal.universe, [0.0,  0.0,  0.5])
        self.dist_goal['Near']      = fuzz.trimf(self.dist_goal.universe, [0.0,  2.0, 4.0])
        self.dist_goal['Medium']    = fuzz.trimf(self.dist_goal.universe, [2.0, 6.0, 10.0])
        self.dist_goal['Far']       = fuzz.trimf(self.dist_goal.universe, [6.0, 10.0, 14.0])
        self.dist_goal['Very_far']  = fuzz.trimf(self.dist_goal.universe, [10.0, 14.0, 20.0])

        self.angle_error['Left_big']   = fuzz.trimf(self.angle_error.universe, [-3.14, -3.14, -1.5])
        self.angle_error['Left_small'] = fuzz.trimf(self.angle_error.universe, [-1.6, -0.4,  0.00])
        self.angle_error['Zero']       = fuzz.trimf(self.angle_error.universe, [-0.17,  0.00,  0.17])
        self.angle_error['Right_small']= fuzz.trimf(self.angle_error.universe, [ 0.00,  0.4,  1.6])
        self.angle_error['Right_big']  = fuzz.trimf(self.angle_error.universe, [ 1.5,  3.14,  3.14])

        self.v_nav['Zero']   = fuzz.trimf(self.v_nav.universe, [0.0,   0.0,   0.333])
        self.v_nav['Low']    = fuzz.trimf(self.v_nav.universe, [0.0,   0.333, 0.666])
        self.v_nav['Medium'] = fuzz.trimf(self.v_nav.universe, [0.333, 0.666, 1.0])
        self.v_nav['High']   = fuzz.trimf(self.v_nav.universe, [0.666, 1.0,   1.0])

        self.w_nav['Left_big'] = fuzz.trimf(self.w_nav.universe, [-1.0,   -1.0,   -0.666])
        self.w_nav['Left']     = fuzz.trimf(self.w_nav.universe, [-1.0,   -0.666, -0.333])
        self.w_nav['Left_small']=fuzz.trimf(self.w_nav.universe, [-0.666, -0.333,  0.0])
        self.w_nav['Zero']      = fuzz.trimf(self.w_nav.universe, [-0.01,  0.0,    0.01])
        self.w_nav['Right_small']=fuzz.trimf(self.w_nav.universe, [ 0.0,    0.333,  0.666])
        self.w_nav['Right']     = fuzz.trimf(self.w_nav.universe, [ 0.333,  0.666,  1.0])
        self.w_nav['Right_big'] = fuzz.trimf(self.w_nav.universe, [ 0.666,  1.0,    1.0])

        self.rules_nav = [
            ctrl.Rule(self.dist_goal['Very_far']   & self.angle_error['Zero'],        (self.v_nav['High'],   self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Far']        & self.angle_error['Zero'],        (self.v_nav['High'],   self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Medium']     & self.angle_error['Zero'],        (self.v_nav['Medium'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Near']       & self.angle_error['Zero'],        (self.v_nav['Low'],    self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Very_near']  & self.angle_error['Zero'],        (self.v_nav['Zero'],   self.w_nav['Zero'])),

            ctrl.Rule(self.dist_goal['Very_far']   & self.angle_error['Right_big'],   (self.v_nav['Zero'],   self.w_nav['Right_big'])),
            ctrl.Rule(self.dist_goal['Far']        & self.angle_error['Right_big'],   (self.v_nav['Zero'],   self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Medium']     & self.angle_error['Right_big'],   (self.v_nav['Zero'],   self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Near']       & self.angle_error['Right_big'],   (self.v_nav['Zero'],   self.w_nav['Right_small'])),
            ctrl.Rule(self.dist_goal['Very_near']  & self.angle_error['Right_big'],   (self.v_nav['Zero'],   self.w_nav['Right_small'])),

            ctrl.Rule(self.dist_goal['Very_far']   & self.angle_error['Right_small'], (self.v_nav['Zero'],   self.w_nav['Right_big'])),
            ctrl.Rule(self.dist_goal['Far']        & self.angle_error['Right_small'], (self.v_nav['Zero'],   self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Medium']     & self.angle_error['Right_small'], (self.v_nav['Zero'],   self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Near']       & self.angle_error['Right_small'], (self.v_nav['Zero'],    self.w_nav['Right_small'])),
            ctrl.Rule(self.dist_goal['Very_near']  & self.angle_error['Right_small'], (self.v_nav['Zero'],   self.w_nav['Right_small'])),

            ctrl.Rule(self.dist_goal['Very_far']   & self.angle_error['Left_big'],    (self.v_nav['Zero'],   self.w_nav['Left_big'])),
            ctrl.Rule(self.dist_goal['Far']        & self.angle_error['Left_big'],    (self.v_nav['Zero'],   self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Medium']     & self.angle_error['Left_big'],    (self.v_nav['Zero'],   self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Near']       & self.angle_error['Left_big'],    (self.v_nav['Zero'],   self.w_nav['Left_small'])),
            ctrl.Rule(self.dist_goal['Very_near']  & self.angle_error['Left_big'],    (self.v_nav['Zero'],   self.w_nav['Left_small'])),
            
            ctrl.Rule(self.dist_goal['Very_far']   & self.angle_error['Left_small'],  (self.v_nav['Zero'],   self.w_nav['Left_big'])),
            ctrl.Rule(self.dist_goal['Far']        & self.angle_error['Left_small'],  (self.v_nav['Zero'],   self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Medium']     & self.angle_error['Left_small'],  (self.v_nav['Zero'],   self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Near']       & self.angle_error['Left_small'],  (self.v_nav['Zero'],    self.w_nav['Left_small'])),
            ctrl.Rule(self.dist_goal['Very_near']  & self.angle_error['Left_small'],  (self.v_nav['Zero'],   self.w_nav['Left_small'])),
        ]
    
    def build_with_mask(self, mask):
        active_nav_rules = [self.rules_nav[i] for i in range(len(mask)) if mask[i] == 1]

        if len(active_nav_rules) == 0:
            active_nav_rules = [self.rules_nav[0]]  
        
        nav_sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(active_nav_rules))
        return nav_sim

    def compute(self, sim, dist_goal, angle_error):
        sim.input['dist_goal'] = dist_goal
        sim.input['angle_error'] = angle_error
        sim.compute()
        return sim.output['v_nav'], sim.output['w_nav']
    
# ===================================================================
class FuzzyAvoider:
    def __init__(self):
        self.d_front = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_front')
        self.d_left  = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_left')
        self.d_right = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_right')

        self.v_avoid = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'v_avoid')
        self.w_avoid = ctrl.Consequent(np.arange(-1, 1.0, 0.05), 'w_avoid')

        for d in [self.d_front, self.d_left, self.d_right]:
            d['Very_near'] = fuzz.trimf(d.universe, [0, 0, 3.25])
            d['Near']      = fuzz.trimf(d.universe, [0, 3.25, 6.5])
            d['Medium']    = fuzz.trimf(d.universe, [3.25, 6.5, 9.75])
            d['Far']       = fuzz.trimf(d.universe, [6.5, 9.75, 13])
            d['Very_far']  = fuzz.trimf(d.universe, [9.75, 13, 13])

        # SALIDA LINEAL
        self.v_avoid['Zero']   = fuzz.trimf(self.v_avoid.universe, [0.0, 0.0, 0.25])
        self.v_avoid['Low']    = fuzz.trimf(self.v_avoid.universe, [0.0, 0.25, 0.5])
        self.v_avoid['Medium'] = fuzz.trimf(self.v_avoid.universe, [0.25, 0.5, 0.75])
        self.v_avoid['High']   = fuzz.trimf(self.v_avoid.universe, [0.5, 1.0, 1.0])

        self.w_avoid['Left_big'] = fuzz.trimf(self.w_avoid.universe, [-1.0, -1.0, -0.5])
        self.w_avoid['Left']     = fuzz.trimf(self.w_avoid.universe, [-1.0, -0.5,  0.0])
        self.w_avoid['Zero']     = fuzz.trimf(self.w_avoid.universe, [-0.01,  0.0,  0.01])
        self.w_avoid['Right']    = fuzz.trimf(self.w_avoid.universe, [ 0.0,  0.5,  1.0])
        self.w_avoid['Right_big']= fuzz.trimf(self.w_avoid.universe, [ 0.5,  1.0,  1.0])

        self.rules_avoid = [

            # ---- VN - very far
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_far'] & self.d_right['Very_far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far']      & self.d_right['Very_far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near']& self.d_right['Very_far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near']     & self.d_right['Very_far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium']   & self.d_right['Very_far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),

            # ---- VN - Far
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_far'] & self.d_right['Far'],      (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far']      & self.d_right['Far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near']& self.d_right['Far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near']     & self.d_right['Far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium']   & self.d_right['Far'],      (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),

            # --- VN - Medium
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_far'] & self.d_right['Medium'],      (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far']      & self.d_right['Medium'],   (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near']& self.d_right['Medium'],   (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near']     & self.d_right['Medium'],   (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium']   & self.d_right['Medium'],   (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),

            # --- VN - Near
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_far'] & self.d_right['Near'],      (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far']      & self.d_right['Near'],     (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near']& self.d_right['Near'],     (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near']     & self.d_right['Near'],     (self.v_avoid['Zero'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium']   & self.d_right['Near'],     (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),

            # --- VN - Very_near
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_far'] & self.d_right['Very_near'],      (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far']      & self.d_right['Very_near'], (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near']& self.d_right['Very_near'], (self.v_avoid['Zero'],   self.w_avoid['Zero'])),  # caso especial
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near']     & self.d_right['Very_near'], (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium']   & self.d_right['Very_near'], (self.v_avoid['Zero'],   self.w_avoid['Right_big'])),

            # ---- N - very far
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_far'] & self.d_right['Very_far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far']      & self.d_right['Very_far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near']& self.d_right['Very_far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near']     & self.d_right['Very_far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium']   & self.d_right['Very_far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            
            # ---- N - Far
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_far'] & self.d_right['Far'],      (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far']      & self.d_right['Far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near']& self.d_right['Far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near']     & self.d_right['Far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium']   & self.d_right['Far'],      (self.v_avoid['Low'],   self.w_avoid['Left_big'])),

            # ---- N - Medium
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_far'] & self.d_right['Medium'],      (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far']      & self.d_right['Medium'],   (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near']& self.d_right['Medium'],   (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near']     & self.d_right['Medium'],   (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium']   & self.d_right['Medium'],   (self.v_avoid['Low'],   self.w_avoid['Left_big'])),

            # ---- N - Near
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_far'] & self.d_right['Near'],      (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far']      & self.d_right['Near'],     (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near']& self.d_right['Near'],     (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near']     & self.d_right['Near'],     (self.v_avoid['Low'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium']   & self.d_right['Near'],     (self.v_avoid['Low'],   self.w_avoid['Right_big'])),

            # ---- N - Very_near
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_far'] & self.d_right['Very_near'],      (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far']      & self.d_right['Very_near'], (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near']& self.d_right['Very_near'], (self.v_avoid['Low'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near']     & self.d_right['Very_near'], (self.v_avoid['Low'],   self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium']   & self.d_right['Very_near'], (self.v_avoid['Low'],   self.w_avoid['Right_big'])),

            # ---- Medium - very far
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_far'] & self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far']      & self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near']& self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near']     & self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium']   & self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),

            # ---- Medium - Far
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_far'] & self.d_right['Far'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far']      & self.d_right['Far'],      (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near']& self.d_right['Far'],      (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near']     & self.d_right['Far'],      (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium']   & self.d_right['Far'],      (self.v_avoid['Medium'], self.w_avoid['Zero'])),

            # ---- Medium - Medium
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_far'] & self.d_right['Medium'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far']      & self.d_right['Medium'],   (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near']& self.d_right['Medium'],   (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near']     & self.d_right['Medium'],   (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium']   & self.d_right['Medium'],   (self.v_avoid['Medium'], self.w_avoid['Zero'])),

            # ---- Medium - Near
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_far'] & self.d_right['Near'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far']      & self.d_right['Near'],     (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near']& self.d_right['Near'],     (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near']     & self.d_right['Near'],     (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium']   & self.d_right['Near'],     (self.v_avoid['Medium'], self.w_avoid['Zero'])),

            # ---- Medium - Very_near
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_far'] & self.d_right['Very_near'],      (self.v_avoid['Medium'],   self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far']      & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near']& self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near']     & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium']   & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),

            # ---- Far - very far
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_far'] & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far']      & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near']& self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near']     & self.d_right['Very_far'],      (self.v_avoid['Medium'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium']   & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),

            # ---- Far - Far
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_far'] & self.d_right['Far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far']      & self.d_right['Far'],        (self.v_avoid['High'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near']& self.d_right['Far'],        (self.v_avoid['Medium'], self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near']     & self.d_right['Far'],        (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium']   & self.d_right['Far'],        (self.v_avoid['High'], self.w_avoid['Zero'])),

            # ---- Far - Medium
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_far'] & self.d_right['Medium'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far']      & self.d_right['Medium'],     (self.v_avoid['High'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near']& self.d_right['Medium'],     (self.v_avoid['Medium'], self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near']     & self.d_right['Medium'],     (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium']   & self.d_right['Medium'],     (self.v_avoid['High'], self.w_avoid['Zero'])),

            # ---- Far - Near
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_far'] & self.d_right['Near'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far']      & self.d_right['Near'],       (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near']& self.d_right['Near'],       (self.v_avoid['Medium'], self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near']     & self.d_right['Near'],       (self.v_avoid['High'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium']   & self.d_right['Near'],       (self.v_avoid['Medium'], self.w_avoid['Zero'])),

            # ---- Far - Very_near
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_far'] & self.d_right['Very_near'],      (self.v_avoid['High'],   self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far']      & self.d_right['Very_near'],  (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near']& self.d_right['Very_near'],  (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near']     & self.d_right['Very_near'],  (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium']   & self.d_right['Very_near'],  (self.v_avoid['Medium'], self.w_avoid['Right'])),

            # ---- very far - very far
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_far'] & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Far']      & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_near']& self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Near']     & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Medium']   & self.d_right['Very_far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),

            # ---- very far - far
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_far'] & self.d_right['Far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Far']      & self.d_right['Far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_near']& self.d_right['Far'],      (self.v_avoid['High'],   self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Near']     & self.d_right['Far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Medium']   & self.d_right['Far'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),

            # ---- very far - Medium
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_far'] & self.d_right['Medium'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Far']      & self.d_right['Medium'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_near']& self.d_right['Medium'],      (self.v_avoid['High'],   self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Near']     & self.d_right['Medium'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Medium']   & self.d_right['Medium'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),

            # ---- very far - Near
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_far'] & self.d_right['Near'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Far']      & self.d_right['Near'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_near']& self.d_right['Near'],      (self.v_avoid['High'],   self.w_avoid['Left'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Near']     & self.d_right['Near'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Medium']   & self.d_right['Near'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),

            # ---- very far - Very_near
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_far'] & self.d_right['Very_near'],      (self.v_avoid['High'],   self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Far']      & self.d_right['Very_near'],      (self.v_avoid['High'],   self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Very_near']& self.d_right['Very_near'],      (self.v_avoid['High'],   self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Near']     & self.d_right['Very_near'],      (self.v_avoid['High'],   self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Very_far'] & self.d_left['Medium']   & self.d_right['Very_near'],      (self.v_avoid['High'],   self.w_avoid['Right'])),
        ]

    def build_with_mask(self, mask):
        active_avoid_rules = [self.rules_avoid[i] for i in range(len(mask)) if mask[i] == 1]

        if len(active_avoid_rules) == 0:
            active_avoid_rules = [self.rules_avoid[0]] 
        
        avoid_sim = ctrl.ControlSystemSimulation(ctrl.ControlSystem(active_avoid_rules))

        return avoid_sim

    def compute(self, sim, d_front, d_left, d_right):
        sim.input['d_front'] = d_front
        sim.input['d_left']  = d_left
        sim.input['d_right'] = d_right
        sim.compute()
        return sim.output['v_avoid'], sim.output['w_avoid']
