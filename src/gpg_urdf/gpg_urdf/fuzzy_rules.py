import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

class FuzzyNavigator:
    def __init__(self):
        # Definición de universos
        self.dist_goal = ctrl.Antecedent(np.arange(0, 21, 0.1), 'dist_goal')
        self.angle_error = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.01), 'angle_error')
        self.v_nav = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'v_nav')
        self.w_nav = ctrl.Consequent(np.arange(-1, 1.1, 0.01), 'w_nav')

        # Funciones de membresía
        self.dist_goal['Very_near'] = fuzz.trimf(self.dist_goal.universe, [0, 0, 0.3])
        self.dist_goal['Near'] = fuzz.trimf(self.dist_goal.universe, [0.2, 1, 2])
        self.dist_goal['Medium'] = fuzz.trimf(self.dist_goal.universe, [1.5, 6, 10])
        self.dist_goal['Far'] = fuzz.trimf(self.dist_goal.universe, [8, 14, 20])

# ----------------CAMBIOS AQUI-------------------
        self.angle_error['Left_big'] = fuzz.trimf(self.angle_error.universe, [-np.pi, -(3*np.pi/4), -np.pi/2])
        self.angle_error['Left_small'] = fuzz.trimf(self.angle_error.universe, [-np.pi/2, -np.pi/8, -np.pi/24])
        self.angle_error['Zero'] = fuzz.trimf(self.angle_error.universe, [-np.pi/18, 0, np.pi/18])
        self.angle_error['Right_small'] = fuzz.trimf(self.angle_error.universe, [np.pi/24, np.pi/8, np.pi/2])
        self.angle_error['Right_big'] = fuzz.trimf(self.angle_error.universe, [np.pi/2, 3*np.pi/4, np.pi])
#-----------------------------------------------

        self.v_nav['Zero'] = fuzz.trimf(self.v_nav.universe, [0, 0, 0.1])
        self.v_nav['Low'] = fuzz.trimf(self.v_nav.universe, [0, 0.1, 0.3])
        self.v_nav['Medium'] = fuzz.trimf(self.v_nav.universe, [0.3, 0.5, 0.7])
        self.v_nav['High'] = fuzz.trimf(self.v_nav.universe, [0.7, 1.0, 1.0])

        self.w_nav['Left'] = fuzz.trimf(self.w_nav.universe, [-1, -0.7, -0.4])
        self.w_nav['Left_small'] = fuzz.trimf(self.w_nav.universe, [-0.5, -0.25, 0])
        self.w_nav['Zero'] = fuzz.trimf(self.w_nav.universe, [-0.01, 0, 0.01])
        self.w_nav['Right_small'] = fuzz.trimf(self.w_nav.universe, [0, 0.25, 0.5])
        self.w_nav['Right'] = fuzz.trimf(self.w_nav.universe, [0.4, 0.7, 1])

        # Reglas
        rules_nav = [
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Zero'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Zero'], (self.v_nav['Medium'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Zero'], (self.v_nav['Low'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Zero'], (self.v_nav['Zero'], self.w_nav['Zero'])),
            #------------------
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Right_big'], (self.v_nav['Zero'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Right_big'], (self.v_nav['Zero'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Right_big'], (self.v_nav['Zero'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Right_big'], (self.v_nav['Zero'], self.w_nav['Right'])),
            #------------------
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Right_small'], (self.v_nav['Zero'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Right_small'], (self.v_nav['Zero'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Right_small'], (self.v_nav['Low'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Right_small'], (self.v_nav['Zero'], self.w_nav['Right_small'])),
            #------------------
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Left_big'], (self.v_nav['Zero'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Left_big'], (self.v_nav['Zero'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Left_big'], (self.v_nav['Zero'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Left_big'], (self.v_nav['Zero'], self.w_nav['Left'])),
            #------------------
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Left_small'], (self.v_nav['Zero'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Left_small'], (self.v_nav['Zero'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Left_small'], (self.v_nav['Low'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Left_small'], (self.v_nav['Zero'], self.w_nav['Left_small'])),
        ]

        self.nav_ctrl = ctrl.ControlSystem(rules_nav)
        self.nav_sim = ctrl.ControlSystemSimulation(self.nav_ctrl)

    def compute(self, dist_goal_val, angle_error_val):
        self.nav_sim_local = ctrl.ControlSystemSimulation(self.nav_ctrl)
        self.nav_sim.input['dist_goal'] = dist_goal_val
        self.nav_sim.input['angle_error'] = angle_error_val
        self.nav_sim.compute()
        return self.nav_sim.output['v_nav'], self.nav_sim.output['w_nav']


class FuzzyAvoider:
    def __init__(self):
        self.d_front = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_front')
        self.d_left = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_left')
        self.d_right = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_right')
        self.v_avoid = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'v_avoid')
        self.w_avoid = ctrl.Consequent(np.arange(-1.0, 1.0, 0.05), 'w_avoid')

        for d in [self.d_front, self.d_left, self.d_right]:
            d['Very_near'] = fuzz.trimf(d.universe, [0, 0.3, 0.6])
            d['Near'] = fuzz.trimf(d.universe, [0.3, 2, 4])
            d['Medium'] = fuzz.trimf(d.universe, [2, 5, 7])
            d['Far'] = fuzz.trimf(d.universe, [6, 9, 13])

        self.v_avoid['Zero'] = fuzz.trimf(self.v_avoid.universe, [0, 0, 0.1])
        self.v_avoid['Low'] = fuzz.trimf(self.v_avoid.universe, [0, 0.1, 0.3])
        self.v_avoid['Medium'] = fuzz.trimf(self.v_avoid.universe, [0.3, 0.5, 0.7])
        self.v_avoid['High'] = fuzz.trimf(self.v_avoid.universe, [0.7, 1.0, 1.0])

        self.w_avoid['Left_big'] = fuzz.trimf(self.w_avoid.universe, [-1, -0.6, -0.3])
        self.w_avoid['Left'] = fuzz.trimf(self.w_avoid.universe, [-0.4, -0.2, -0.1])
        self.w_avoid['Zero'] = fuzz.trimf(self.w_avoid.universe, [-0.1, 0, 0.1])
        self.w_avoid['Right'] = fuzz.trimf(self.w_avoid.universe, [0.1, 0.2, 0.4])
        self.w_avoid['Right_big'] = fuzz.trimf(self.w_avoid.universe, [0.3, 0.6, 1])

        rules_avoid = [
            #----VN - Far
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far'] & self.d_right['Far'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near'] & self.d_right['Far'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near'] & self.d_right['Far'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium'] & self.d_right['Far'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            #---VN - Medium
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far'] & self.d_right['Medium'], (self.v_avoid['Zero'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near'] & self.d_right['Medium'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near'] & self.d_right['Medium'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium'] & self.d_right['Medium'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            #---VN - Near
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far'] & self.d_right['Near'], (self.v_avoid['Zero'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near'] & self.d_right['Near'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near'] & self.d_right['Near'], (self.v_avoid['Zero'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium'] & self.d_right['Near'], (self.v_avoid['Zero'], self.w_avoid['Right_big'])),
            #---VN - Very_near
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Far'] & self.d_right['Very_near'], (self.v_avoid['Zero'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Very_near'] & self.d_right['Very_near'], (self.v_avoid['Zero'], self.w_avoid['Zero'])), #PENSAR EN QUE PASA EN ESTE CASO
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Near'] & self.d_right['Very_near'], (self.v_avoid['Zero'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Very_near'] & self.d_left['Medium'] & self.d_right['Very_near'], (self.v_avoid['Zero'], self.w_avoid['Right_big'])),
            #----N - Far
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far'] & self.d_right['Far'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near'] & self.d_right['Far'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near'] & self.d_right['Far'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium'] & self.d_right['Far'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            #---N - Medium
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far'] & self.d_right['Medium'], (self.v_avoid['Low'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near'] & self.d_right['Medium'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near'] & self.d_right['Medium'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium'] & self.d_right['Medium'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            #---N - Near
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far'] & self.d_right['Near'], (self.v_avoid['Low'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near'] & self.d_right['Near'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near'] & self.d_right['Near'], (self.v_avoid['Low'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium'] & self.d_right['Near'], (self.v_avoid['Low'], self.w_avoid['Right_big'])),
            #---N - Very_near
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far'] & self.d_right['Very_near'], (self.v_avoid['Low'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Very_near'] & self.d_right['Very_near'], (self.v_avoid['Low'], self.w_avoid['Zero'])), #PENSAR EN QUE PASA EN ESTE CASO
            ctrl.Rule(self.d_front['Near'] & self.d_left['Near'] & self.d_right['Very_near'], (self.v_avoid['Low'], self.w_avoid['Right_big'])),
            ctrl.Rule(self.d_front['Near'] & self.d_left['Medium'] & self.d_right['Very_near'], (self.v_avoid['Low'], self.w_avoid['Right_big'])),
            #----Med - Far
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far'] & self.d_right['Far'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near'] & self.d_right['Far'], (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near'] & self.d_right['Far'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium'] & self.d_right['Far'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            #---Med - Medium
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far'] & self.d_right['Medium'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near'] & self.d_right['Medium'], (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near'] & self.d_right['Medium'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium'] & self.d_right['Medium'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            #---Med - Near
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            #---Med - Very_near
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Far'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Very_near'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])), #PENSAR EN QUE PASA EN ESTE CASO
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Near'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Medium'] & self.d_left['Medium'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            #----Far - Far
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far'] & self.d_right['Far'], (self.v_avoid['High'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near'] & self.d_right['Far'], (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near'] & self.d_right['Far'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium'] & self.d_right['Far'], (self.v_avoid['High'], self.w_avoid['Zero'])),
            #---Far - Medium
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far'] & self.d_right['Medium'], (self.v_avoid['High'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near'] & self.d_right['Medium'], (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near'] & self.d_right['Medium'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium'] & self.d_right['Medium'], (self.v_avoid['High'], self.w_avoid['Zero'])),
            #---Far - Near
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Left_big'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near'] & self.d_right['Near'], (self.v_avoid['High'], self.w_avoid['Zero'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium'] & self.d_right['Near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])),
            #---Far - Very_near
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Very_near'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Zero'])), #PENSAR EN QUE PASA EN ESTE CASO
            ctrl.Rule(self.d_front['Far'] & self.d_left['Near'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Medium'] & self.d_right['Very_near'], (self.v_avoid['Medium'], self.w_avoid['Right'])),
        ]

        self.avoid_ctrl = ctrl.ControlSystem(rules_avoid)
        self.avoid_sim = ctrl.ControlSystemSimulation(self.avoid_ctrl)

    def compute(self, d_front_val, d_left_val, d_right_val):
        self.avoid_sim = ctrl.ControlSystemSimulation(self.avoid_ctrl)
        self.avoid_sim.input['d_front'] = d_front_val
        self.avoid_sim.input['d_left'] = d_left_val
        self.avoid_sim.input['d_right'] = d_right_val
        self.avoid_sim.compute()
        return self.avoid_sim.output['v_avoid'], self.avoid_sim.output['w_avoid']
