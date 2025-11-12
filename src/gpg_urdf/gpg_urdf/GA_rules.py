import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

V_NAV_LABELS = ['Zero', 'Low', 'Medium', 'High']
W_NAV_LABELS = ['Left', 'Left_small', 'Zero', 'Right_small', 'Right']

NAV_RULE_CONDITIONS = [
    ('Far', 'Zero'),
    ('Medium', 'Zero'),
    ('Near', 'Zero'),
    ('Very_near', 'Zero'),
    ('Far', 'Right_big'),
    ('Medium', 'Right_big'),
    ('Near', 'Right_big'),
    ('Very_near', 'Right_big'),
    ('Far', 'Right_small'),
    ('Medium', 'Right_small'),
    ('Near', 'Right_small'),
    ('Very_near', 'Right_small'),
    ('Far', 'Left_big'),
    ('Medium', 'Left_big'),
    ('Near', 'Left_big'),
    ('Very_near', 'Left_big'),
    ('Far', 'Left_small'),
    ('Medium', 'Left_small'),
    ('Near', 'Left_small'),
    ('Very_near', 'Left_small'),
]

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
    def build_nav_rules_ga(self, individual):
        rules_nav = []
        
        for i, cond in enumerate(NAV_RULE_CONDITIONS):
            dist_label, ang_label = cond
            
            # Obtener los genes de la regla actual
            v_idx, w_idx, active = get_rule_genes(individual, i)
            
            if active < 0.5:
                # regla desactivada
                continue
                
            v_name = V_NAV_LABELS[int(v_idx)]
            w_name = W_NAV_LABELS[int(w_idx)]
            
            rule = ctrl.Rule(
                self.dist_goal[dist_label] & self.angle_error[ang_label],
                (self.v_nav[v_name], self.w_nav[w_name])
            )
            rules_nav.append(rule)

        nav_ctrl = ctrl.ControlSystem(rules_nav)
        return ctrl.ControlSystemSimulation(nav_ctrl)
# ===================================================================
V_AVD_LABELS = ['Zero', 'Low', 'Medium', 'High']
W_AVD_LABELS = ['Left_big', 'Left', 'Zero', 'Right', 'Right_big']

AVOID_RULE_CONDITIONS = [
    ('Very_near', 'Far', 'Far'),
    ('Very_near', 'Very_near', 'Far'),
    ('Very_near', 'Near', 'Far'),
    ('Very_near', 'Medium', 'Far'),
    ('Very_near', 'Far', 'Medium'),
    ('Very_near', 'Very_near', 'Medium'),
    ('Very_near', 'Near', 'Medium'),
    ('Very_near', 'Medium', 'Medium'),
    ('Very_near', 'Far', 'Near'),
    ('Very_near', 'Very_near', 'Near'),
    ('Very_near', 'Near', 'Near'),
    ('Very_near', 'Medium', 'Near'),
    ('Very_near', 'Far', 'Very_near'),
    ('Very_near', 'Very_near', 'Very_near'),
    ('Very_near', 'Near', 'Very_near'),
    ('Very_near', 'Medium', 'Very_near'),
    #
    ('Near', 'Far', 'Far'),
    ('Near', 'Very_near', 'Far'),
    ('Near', 'Near', 'Far'),
    ('Near', 'Medium', 'Far'),
    ('Near', 'Far', 'Medium'),
    ('Near', 'Very_near', 'Medium'),
    ('Near', 'Near', 'Medium'),
    ('Near', 'Medium', 'Medium'),
    ('Near', 'Far', 'Near'),
    ('Near', 'Very_near', 'Near'),
    ('Near', 'Near', 'Near'),
    ('Near', 'Medium', 'Near'),
    ('Near', 'Far', 'Very_near'),
    ('Near', 'Very_near', 'Very_near'),
    ('Near', 'Near', 'Very_near'),
    ('Near', 'Medium', 'Very_near'),
    #
    ('Medium', 'Far', 'Far'),
    ('Medium', 'Very_near', 'Far'),
    ('Medium', 'Near', 'Far'),
    ('Medium', 'Medium', 'Far'),
    ('Medium', 'Far', 'Medium'),
    ('Medium', 'Very_near', 'Medium'),
    ('Medium', 'Near', 'Medium'),
    ('Medium', 'Medium', 'Medium'),
    ('Medium', 'Far', 'Near'),
    ('Medium', 'Very_near', 'Near'),
    ('Medium', 'Near', 'Near'),
    ('Medium', 'Medium', 'Near'),
    ('Medium', 'Far', 'Very_near'),
    ('Medium', 'Very_near', 'Very_near'),
    ('Medium', 'Near', 'Very_near'),
    ('Medium', 'Medium', 'Very_near'),
    #
    ('Far', 'Far', 'Far'),
    ('Far', 'Very_near', 'Far'),
    ('Far', 'Near', 'Far'),
    ('Far', 'Medium', 'Far'),
    ('Far', 'Far', 'Medium'),
    ('Far', 'Very_near', 'Medium'),
    ('Far', 'Near', 'Medium'),
    ('Far', 'Medium', 'Medium'),
    ('Far', 'Far', 'Near'),
    ('Far', 'Very_near', 'Near'),
    ('Far', 'Near', 'Near'),
    ('Far', 'Medium', 'Near'),
    ('Far', 'Far', 'Very_near'),
    ('Far', 'Very_near', 'Very_near'),
    ('Far', 'Near', 'Very_near'),
    ('Far', 'Medium', 'Very_near'),
]

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

    def build_avoid_rules_ga(self, individual):
        rules_avoid = []
        
        for i, cond in enumerate(AVOID_RULE_CONDITIONS):
            front, left, right = cond
            
            # Obtener los genes de la regla actual
            v_a_idx, w_a_idx, active_a = get_rule_genes(individual, i)
            
            if active_a < 0.5:
                continue
                
            v_name = V_AVD_LABELS[int(v_a_idx)]
            w_name = W_AVD_LABELS[int(w_a_idx)]
            
            rule = ctrl.Rule(
                self.d_front[front] & self.d_left[left] & self.d_right[right],
                (self.v_avoid[v_name], self.w_avoid[w_name])
            )
            rules_avoid.append(rule)

        avoid_ctrl = ctrl.ControlSystem(rules_avoid)
        return ctrl.ControlSystemSimulation(avoid_ctrl)


def get_rule_genes(individual, rule_idx):
    start_idx = rule_idx * 3
    return individual[start_idx:start_idx+3]