from definitions.definitions_base import *
from utils.ws_client import call_method
from xmlrpc.client import ServerProxy

class InsertionFactory(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "insertion", learn_skills=[("TaxInsertion", "insertion", "insertion")],
                         setup_skills=[("MoveToPoseJoint", "move", "move_joint")],
                         reset_skills=[("TaxExtraction", "extraction", "extraction"),
                          ("MoveToPoseJoint", "move_approach", "move_joint")], 
                          rescue_skils=[("MoveToPoseJoint", "move_back", "move_joint")],
                          termination_skills=[], cost_function=cost_function, objects=objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p0_offset_x": (-0.005, 0.005),
            "p0_offset_y": (-0.005, 0.005),
            "p0_offset_phi": (-10, 10),
            "p0_offset_chi": (-10, 10),
            "p1_dx_d": (0, 0.1),
            "p1_dphi_d": (0, 0.5),
            "p1_ddx_d": (0, 0.5),
            "p1_ddphi_d": (0, 1),
            "p1_K_x": (0, 2000),
            "p1_K_phi": (0, 200),
            "p2_dx_d": (0, 0.1),
            "p2_dphi_d": (0, 0.5),
            "p2_ddx_d": (0, 0.5),
            "p2_ddphi_d": (0, 1),
            "p2_wiggle_a_x": (0, 10),
            "p2_wiggle_a_y": (0, 10),
            # "p2_wiggle_a_z": (0, 10),
            "p2_wiggle_a_phi": (0, 2),
            "p2_wiggle_a_chi": (0, 2),
            "p2_wiggle_f_x": (0, 3),
            "p2_wiggle_f_y": (0, 3),
            # "p2_wiggle_f_z": (0, 3),
            "p2_wiggle_f_phi": (0, 1),
            "p2_wiggle_f_chi": (0, 1),
            "p2_wiggle_phi_x": (0, 6.28),
            "p2_wiggle_phi_y": (0, 6.28),
            # "p2_wiggle_phi_z": (0, 6.28),
            "p2_wiggle_phi_phi": (0, 6.28),
            "p2_wiggle_phi_chi": (0, 6.28),
            "p2_K_x": (0, 2000),
            "p2_K_y": (0, 2000),
            "p2_K_z": (0, 2000),
            "p2_K_phi": (0, 200),
            "p2_K_chi": (0, 200),
            "p2_K_psi": (0, 200),
            "p2_f_push_x": (-10, 10),
            "p2_f_push_y": (-10, 10),
            "p2_f_push_z": (0, 20)
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p0_offset_x": ["skills.insertion.skill.p0.DeltaX-1"],
            "p0_offset_y": ["skills.insertion.skill.p0.DeltaX-2"],
            "p0_offset_phi": ["skills.insertion.skill.p0.DeltaX-4"],
            "p0_offset_chi": ["skills.insertion.skill.p0.DeltaX-5"],
            "p1_dx_d": ["skills.insertion.skill.p1.dX_d-1"],
            "p1_dphi_d": ["skills.insertion.skill.p1.dX_d-2"],
            "p1_ddx_d": ["skills.insertion.skill.p1.ddX_d-1"],
            "p1_ddphi_d": ["skills.insertion.skill.p1.ddX_d-2"],
            "p1_K_x": ["skills.insertion.skill.p1.K_x-1", "skills.insertion.skill.p1.K_x-2",
                       "skills.insertion.skill.p1.K_x-3"],
            "p1_K_phi": ["skills.insertion.skill.p1.K_x-4", "skills.insertion.skill.p1.K_x-5",
                         "skills.insertion.skill.p1.K_x-6"],
            "p2_dx_d": ["skills.insertion.skill.p2.dX_d-1"],
            "p2_dphi_d": ["skills.insertion.skill.p2.dX_d-2"],
            "p2_ddx_d": ["skills.insertion.skill.p2.ddX_d-1"],
            "p2_ddphi_d": ["skills.insertion.skill.p2.ddX_d-2"],
            "p2_wiggle_a_x": ["skills.insertion.skill.p2.search_a-1"],
            "p2_wiggle_a_y": ["skills.insertion.skill.p2.search_a-2"],
            # "p2_wiggle_a_z": ["skills.insertion.skill.p2.search_a-3"],
            "p2_wiggle_a_phi": ["skills.insertion.skill.p2.search_a-4"],
            "p2_wiggle_a_chi": ["skills.insertion.skill.p2.search_a-5"],
            "p2_wiggle_f_x": ["skills.insertion.skill.p2.search_f-1"],
            "p2_wiggle_f_y": ["skills.insertion.skill.p2.search_f-2"],
            # "p2_wiggle_f_z": ["skills.insertion.skill.p2.search_f-3"],
            "p2_wiggle_f_phi": ["skills.insertion.skill.p2.search_f-4"],
            "p2_wiggle_f_chi": ["skills.insertion.skill.p2.search_f-5"],
            "p2_wiggle_phi_x": ["skills.insertion.skill.p2.search_phi-1"],
            "p2_wiggle_phi_y": ["skills.insertion.skill.p2.search_phi-2"],
            # "p2_wiggle_phi_z": ["skills.insertion.skill.p2.search_phi-3"],
            "p2_wiggle_phi_phi": ["skills.insertion.skill.p2.search_phi-4"],
            "p2_wiggle_phi_chi": ["skills.insertion.skill.p2.search_phi-5"],
            "p2_K_x": ["skills.insertion.skill.p2.K_x-1"],
            "p2_K_y": ["skills.insertion.skill.p2.K_x-2"],
            "p2_K_z": ["skills.insertion.skill.p2.K_x-3"],
            "p2_K_phi": ["skills.insertion.skill.p2.K_x-4"],
            "p2_K_chi": ["skills.insertion.skill.p2.K_x-5"],
            "p2_K_psi": ["skills.insertion.skill.p2.K_x-6"],
            "p2_f_push_x": ["skills.insertion.skill.p2.f_push-1"],
            "p2_f_push_y": ["skills.insertion.skill.p2.f_push-2"],
            "p2_f_push_z": ["skills.insertion.skill.p2.f_push-3"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        x_0["p0_offset_x"] = 0.5
        x_0["p0_offset_y"] = 0.5
        x_0["p0_offset_phi"] = 0.5
        x_0["p0_offset_chi"] = 0.5
        x_0["p2_f_push_x"] = 0.5
        x_0["p2_f_push_y"] = 0.5
        return x_0

    def ground_skills(self):
        #print(self.setup_instructions)
        self.learn_context["skills"]["insertion"]["skill"]["objects"]["Approach"] = self.objects["Approach"]
        self.learn_context["skills"]["insertion"]["skill"]["objects"]["Container"] = self.objects["Container"]
        self.learn_context["skills"]["insertion"]["skill"]["objects"]["Insertable"] = self.objects["Insertable"]

        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]
        
        self.reset_instructions[0]["parameters"]["skills"]["extraction"]["skill"]["objects"]["ExtractTo"] = \
        self.objects["Approach"]
        self.reset_instructions[0]["parameters"]["skills"]["extraction"]["skill"]["objects"]["Container"] = \
        self.objects["Container"]
        self.reset_instructions[0]["parameters"]["skills"]["extraction"]["skill"]["objects"]["Extractable"] = \
        self.objects["Insertable"]
        
        self.reset_instructions[0]["preconditions"] = {"grasped_object":self.objects["Insertable"]}

        self.rescue_instructions[0]["parameters"]["skills"]["move_back"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]
        
        self.reset_instructions[0]["parameters"]["skills"]["move_approach"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]

        

    def run_setup(self) -> bool:
        for r in self.robots:
            result = call_method(r, self.mios_port, "set_grasped_object", {"object": self.objects["Insertable"]})
        return True

    def modify_contexts(self):
        self.learn_context["skills"]["insertion"]["user"]["env_X"] = [0.003, 0.003, 0.002, 0.1, 0.1, 0.1]
        self.learn_context["skills"]["insertion"]["skill"]["ROI_x"] = [-0.03, 0.03, -0.03, 0.03, -1, 1]
        self.learn_context["skills"]["insertion"]["user"]["F_ext_contact"] = [12, 5]
        self.learn_context["skills"]["insertion"]["limits"] = {"cartesian_space":{"dX_max":[0.5,1]}}


class ExtractionFactory(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "extraction", learn_skills=[("TaxExtraction", "extraction", "extraction")],
                         setup_skills=[("MoveToPoseJoint", "move", "move_joint"), ("TaxInsertion", "insertion", "insertion")],
                         reset_skills=[("MoveToPoseJoint", "move", "move_joint"), ("TaxInsertion", "insertion", "insertion")],
                         rescue_skils=[],
                         termination_skills=[], cost_function=cost_function, objects=objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p0_dx_d": (0, 0.3),
            "p0_dphi_d": (0, 1),
            "p0_ddx_d": (0, 1),
            "p0_ddphi_d": (0, 4),
            "p0_K_x": (0, 2000),
            "p0_K_y": (0, 2000),
            "p0_K_z": (0, 2000),
            "p0_K_phi": (0, 200),
            "p0_K_chi": (0, 200),
            "p0_K_psi": (0, 200),
            "p0_wiggle_a_x": (0, 10),
            "p0_wiggle_a_y": (0, 10),
            "p0_wiggle_a_phi": (0, 2),
            "p0_wiggle_a_chi": (0, 2),
            "p0_wiggle_f_x": (0, 3),
            "p0_wiggle_f_y": (0, 3),
            "p0_wiggle_f_phi": (0, 1),
            "p0_wiggle_f_chi": (0, 1)
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p0_dx_d": ["skills.extraction.skill.p0.dX_d-1"],
            "p0_dphi_d": ["skills.extraction.skill.p0.dX_d-2"],
            "p0_ddx_d": ["skills.extraction.skill.p0.ddX_d-1"],
            "p0_ddphi_d": ["skills.extraction.skill.p0.ddX_d-2"],
            "p0_K_x": ["skills.extraction.skill.p0.K_x-1"],
            "p0_K_y": ["skills.extraction.skill.p0.K_x-2"],
            "p0_K_z": ["skills.extraction.skill.p0.K_x-3"],
            "p0_K_phi": ["skills.extraction.skill.p0.K_x-4"],
            "p0_K_chi": ["skills.extraction.skill.p0.K_x-5"],
            "p0_K_psi": ["skills.extraction.skill.p0.K_x-6"],
            "p0_wiggle_a_x": ["skills.extraction.skill.p0.search_a-1"],
            "p0_wiggle_a_y": ["skills.extraction.skill.p0.search_a-2"],
            "p0_wiggle_a_phi": ["skills.extraction.skill.p0.search_a-4"],
            "p0_wiggle_a_chi": ["skills.extraction.skill.p0.search_a-5"],
            "p0_wiggle_f_x": ["skills.extraction.skill.p0.search_f-1"],
            "p0_wiggle_f_y": ["skills.extraction.skill.p0.search_f-2"],
            "p0_wiggle_f_phi": ["skills.extraction.skill.p0.search_f-4"],
            "p0_wiggle_f_chi": ["skills.extraction.skill.p0.search_f-5"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        return x_0

    def ground_skills(self):
        self.learn_context["skills"]["extraction"]["skill"]["objects"]["ExtractTo"] = self.objects["ExtractTo"]
        self.learn_context["skills"]["extraction"]["skill"]["objects"]["Container"] = self.objects["Container"]
        self.learn_context["skills"]["extraction"]["skill"]["objects"]["Extractable"] = self.objects["Extractable"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "ExtractTo"]
        self.setup_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["objects"]["Approach"] = \
            self.objects["ExtractTo"]
        self.setup_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["objects"]["Container"] = \
            self.objects["Container"]
        self.setup_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["objects"]["Insertable"] = \
            self.objects["Extractable"]
        self.reset_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["objects"]["Approach"] = \
            self.objects["ExtractTo"]
        self.reset_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["objects"]["Container"] = \
            self.objects["Container"]
        self.reset_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["objects"]["Insertable"] = \
            self.objects["Extractable"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "ExtractTo"]

    def run_setup(self) -> bool:
        for r in self.robots:
            result = call_method(r, self.mios_port, "set_grasped_object", {"object": self.objects["Extractable"]})
        return True

    def modify_contexts(self):
        self.reset_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["time_max"] = 1000
        self.setup_instructions[0]["parameters"]["skills"]["insertion"]["skill"]["time_max"] = 1000



class TipFactory(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "tip", learn_skills=[("TaxTip", "tip", "tip")],
                        setup_skills= [("MoveToPoseJoint", "move", "move_joint")],
                         reset_skills=[("MoveToPoseJoint", "move", "move_joint")], 
                         rescue_skils=[],
                         termination_skills=[], cost_function=cost_function, objects=objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p1_dx_d": (0, 0.1),
            "p1_dphi_d": (0, 0.5),
            "p1_ddx_d": (0, 0.5),
            "p1_ddphi_d": (0, 1),
            "p1_K_x": (0, 2000),
            "p1_K_phi": (0, 200),
            "p1_f_tip": (0, 20),
            "p2_dx_d": (0, 0.1),
            "p2_dphi_d": (0, 0.5),
            "p2_ddx_d": (0, 0.5),
            "p2_ddphi_d": (0, 1),
            "p2_K_x": (0, 2000),
            "p2_K_phi": (0, 200)
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p1_dx_d": ["skills.tip.skill.p1.dX_d-1"],
            "p1_dphi_d": ["skills.tip.skill.p1.dX_d-2"],
            "p1_ddx_d": ["skills.tip.skill.p1.ddX_d-1"],
            "p1_ddphi_d": ["skills.tip.skill.p1.ddX_d-2"],
            "p1_K_x": ["skills.tip.skill.p1.K_x-1", "skills.tip.skill.p1.K_x-2",
                       "skills.tip.skill.p1.K_x-3"],
            "p1_K_phi": ["skills.tip.skill.p1.K_x-4", "skills.tip.skill.p1.K_x-5",
                         "skills.tip.skill.p1.K_x-6"],
            "p1_f_tip": ["skills.tip.skill.p1.f_tip"],
            "p2_dx_d": ["skills.tip.skill.p2.dX_d-1"],
            "p2_dphi_d": ["skills.tip.skill.p2.dX_d-2"],
            "p2_ddx_d": ["skills.tip.skill.p2.ddX_d-1"],
            "p2_ddphi_d": ["skills.tip.skill.p2.ddX_d-2"],
            "p2_K_x": ["skills.tip.skill.p2.K_x-1", "skills.tip.skill.p2.K_x-2",
                       "skills.tip.skill.p2.K_x-3"],
            "p2_K_phi": ["skills.tip.skill.p2.K_x-4", "skills.tip.skill.p2.K_x-5",
                         "skills.tip.skill.p2.K_x-6"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        return x_0

    def ground_skills(self):
        print(self.setup_instructions)
        self.learn_context["skills"]["tip"]["skill"]["objects"]["Approach"] = self.objects["Approach"]
        self.learn_context["skills"]["tip"]["skill"]["objects"]["Tippable"] = self.objects["Tippable"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]

    def run_setup(self) -> bool:
        s = ServerProxy("http://collective-control-001:8000", allow_none=True)
        s.subscribe_to_event("tippable_pressed", self.robot, "12000")
        return True


class DragFactory(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "drag", [("TaxDrag", "drag", "drag")],
                         [("MoveToPoseJoint", "move", "move_joint")],
                         [("MoveToPoseJoint", "move", "move_joint")],[], [], cost_function, objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p0_dx_d": (0, 0.1),
            "p0_dphi_d": (0, 0.5),
            "p0_ddx_d": (0, 0.5),
            "p0_ddphi_d": (0, 1),
            "p0_K_x": (0, 2000),
            "p0_K_phi": (0, 200),
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p0_dx_d": ["skills.drag.skill.p0.dX_d-1"],
            "p0_dphi_d": ["skills.drag.skill.p0.dX_d-2"],
            "p0_ddx_d": ["skills.drag.skill.p0.ddX_d-1"],
            "p0_ddphi_d": ["skills.drag.skill.p0.ddX_d-2"],
            "p0_K_x": ["skills.drag.skill.p0.K_x-1", "skills.drag.skill.p0.K_x-2",
                       "skills.drag.skill.p0.K_x-3"],
            "p0_K_phi": ["skills.drag.skill.p0.K_x-4", "skills.drag.skill.p0.K_x-5",
                         "skills.drag.skill.p0.K_x-6"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        return x_0

    def ground_skills(self):
        print(self.setup_instructions)
        self.learn_context["skills"]["drag"]["skill"]["objects"]["Draggable"] = self.objects["Draggable"]
        self.learn_context["skills"]["drag"]["skill"]["objects"]["GoalPose"] = self.objects["GoalPose"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "StartPose"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "StartPose"]

    def run_setup(self) -> bool:
        for r in self.robots:
            result = call_method(r, self.mios_port, "set_grasped_object", {"object": self.objects["Draggable"]})
        return True


class TurnLeverFactory(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "turn_lever", [("TaxTurnLever", "turn_lever", "turn_lever")],
                         [("TaxTurnLever", "turn_lever", "turn_lever"),("MoveToPoseJoint", "move", "move_joint")],
                         [("TaxTurnLever", "turn_lever", "turn_lever"),("MoveToPoseJoint", "move", "move_joint")],
                         [], [],cost_function, objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p0_dx_d": (0, 0.3),
            "p0_dphi_d": (0, 1),
            "p0_ddx_d": (0, 1),
            "p0_ddphi_d": (0, 4),
            "p0_K_x": (0, 2000),
            "p0_K_y": (0, 2000),
            "p0_K_z": (0, 2000),
            "p0_K_phi": (0, 200),
            "p0_K_chi": (0, 200),
            "p0_K_psi": (0, 200),
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p0_dx_d": ["skills.turn_lever.skill.p0.dX_d-1"],
            "p0_dphi_d": ["skills.turn_lever.skill.p0.dX_d-2"],
            "p0_ddx_d": ["skills.turn_lever.skill.p0.ddX_d-1"],
            "p0_ddphi_d": ["skills.turn_lever.skill.p0.ddX_d-2"],
            "p0_K_x": ["skills.turn_lever.skill.p0.K_x-1"],
            "p0_K_y": ["skills.turn_lever.skill.p0.K_x-2"],
            "p0_K_z": ["skills.turn_lever.skill.p0.K_x-3"],
            "p0_K_phi": ["skills.turn_lever.skill.p0.K_x-4"],
            "p0_K_chi": ["skills.turn_lever.skill.p0.K_x-5"],
            "p0_K_psi": ["skills.turn_lever.skill.p0.K_x-6"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        return x_0

    def ground_skills(self):
        print(self.setup_instructions)
        self.learn_context["skills"]["turn_lever"]["skill"]["objects"]["Lever"] = self.objects["Lever"]
        self.learn_context["skills"]["turn_lever"]["skill"]["objects"]["GoalPosition"] = self.objects["GoalPosition"]
        self.setup_instructions[0]["parameters"]["skills"]["turn_lever"]["skill"]["objects"]["Lever"] = \
            self.objects["Lever"]
        self.setup_instructions[0]["parameters"]["skills"]["turn_lever"]["skill"]["objects"]["GoalPosition"] = \
            self.objects["StartPose"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "StartPose"]
        self.reset_instructions[0]["parameters"]["skills"]["turn_lever"]["skill"]["objects"]["Lever"] = \
            self.objects["Lever"]
        self.reset_instructions[0]["parameters"]["skills"]["turn_lever"]["skill"]["objects"]["GoalPosition"] =\
            self.objects["StartPose"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "StartPose"]

    def run_setup(self) -> bool:
        for r in self.robots:
            result = call_method(r, self.mios_port, "set_grasped_object", {"object": self.objects["Lever"]})
        return True


class PressButtonFactory(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "press_button", [("TaxPressButton", "press_button", "press_button")],
                         [("MoveToPoseJoint", "move", "move_joint")],
                         [("MoveToPoseJoint", "move", "move_joint")], [],[], cost_function, objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p1_dx_d": (0, 0.1),
            "p1_dphi_d": (0, 0.5),
            "p1_ddx_d": (0, 0.5),
            "p1_ddphi_d": (0, 1),
            "p1_K_x": (0, 2000),
            "p1_K_phi": (0, 200),
            "p2_K_x": (0, 2000),
            "p2_K_phi": (0, 200),
            "p2_f_push": (0, 30),
            "p3_K_x": (0, 2000),
            "p3_K_phi": (0, 200),
            "p3_f_push": (0, 30),
            "p4_dx_d": (0, 0.1),
            "p4_dphi_d": (0, 0.5),
            "p4_ddx_d": (0, 0.5),
            "p4_ddphi_d": (0, 1),
            "p4_K_x": (0, 2000),
            "p4_K_phi": (0, 200),
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p1_dx_d": ["skills.press_button.skill.p1.dX_d-1"],
            "p1_dphi_d": ["skills.press_button.skill.p1.dX_d-2"],
            "p1_ddx_d": ["skills.press_button.skill.p1.ddX_d-1"],
            "p1_ddphi_d": ["skills.press_button.skill.p1.ddX_d-2"],
            "p1_K_x": ["skills.press_button.skill.p1.K_x-1", "skills.press_button.skill.p1.K_x-2",
                       "skills.press_button.skill.p1.K_x-3"],
            "p1_K_phi": ["skills.press_button.skill.p1.K_x-4", "skills.press_button.skill.p1.K_x-5",
                         "skills.press_button.skill.p1.K_x-6"],
            "p2_K_x": ["skills.press_button.skill.p1.K_x-1", "skills.press_button.skill.p1.K_x-2",
                       "skills.press_button.skill.p1.K_x-3"],
            "p2_K_phi": ["skills.press_button.skill.p1.K_x-4", "skills.press_button.skill.p1.K_x-5",
                         "skills.press_button.skill.p1.K_x-6"],
            "p2_f_push": ["skills.press_button.skill.p1.f_push"],
            "p3_K_x": ["skills.press_button.skill.p1.K_x-1", "skills.press_button.skill.p1.K_x-2",
                       "skills.press_button.skill.p1.K_x-3"],
            "p3_K_phi": ["skills.press_button.skill.p1.K_x-4", "skills.press_button.skill.p1.K_x-5",
                         "skills.press_button.skill.p1.K_x-6"],
            "p3_f_push": ["skills.press_button.skill.p1.f_push"],
            "p4_dx_d": ["skills.press_button.skill.p2.dX_d-1"],
            "p4_dphi_d": ["skills.press_button.skill.p2.dX_d-2"],
            "p4_ddx_d": ["skills.press_button.skill.p2.ddX_d-1"],
            "p4_ddphi_d": ["skills.press_button.skill.p2.ddX_d-2"],
            "p4_K_x": ["skills.press_button.skill.p2.K_x-1", "skills.press_button.skill.p2.K_x-2",
                       "skills.press_button.skill.p2.K_x-3"],
            "p4_K_phi": ["skills.press_button.skill.p2.K_x-4", "skills.press_button.skill.p2.K_x-5",
                         "skills.press_button.skill.p2.K_x-6"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        return x_0

    def ground_skills(self):
        print(self.setup_instructions)
        self.learn_context["skills"]["press_button"]["skill"]["objects"]["Approach"] = self.objects["Approach"]
        self.learn_context["skills"]["press_button"]["skill"]["objects"]["Button"] = self.objects["Button"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]

    def run_setup(self) -> bool:
        s = ServerProxy("http://collective-control-001:8000", allow_none=True)
        s.subscribe_to_event("button_pressed", self.robot, "12000")
        return True


class BendFactory(ProblemDefinitionFactory):
    def __init__(self, robot: str, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robot, "bend", [("TaxBend", "bend", "bend")],
                         [("TaxBend", "bend", "bend"),("MoveToPoseJoint", "move", "move_joint")],
                         [("TaxBend", "bend", "bend"),("MoveToPoseJoint", "move", "move_joint")],
                         [], [], cost_function, objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p0_dx_d": (0, 0.3),
            "p0_dphi_d": (0, 1),
            "p0_ddx_d": (0, 1),
            "p0_ddphi_d": (0, 4),
            "p0_K_x": (0, 2000),
            "p0_K_y": (0, 2000),
            "p0_K_z": (0, 2000),
            "p0_K_phi": (0, 200),
            "p0_K_chi": (0, 200),
            "p0_K_psi": (0, 200),
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p0_dx_d": ["skills.bend.skill.p0.dX_d-1"],
            "p0_dphi_d": ["skills.bend.skill.p0.dX_d-2"],
            "p0_ddx_d": ["skills.bend.skill.p0.ddX_d-1"],
            "p0_ddphi_d": ["skills.bend.skill.p0.ddX_d-2"],
            "p0_K_x": ["skills.bend.skill.p0.K_x-1"],
            "p0_K_y": ["skills.bend.skill.p0.K_x-2"],
            "p0_K_z": ["skills.bend.skill.p0.K_x-3"],
            "p0_K_phi": ["skills.bend.skill.p0.K_x-4"],
            "p0_K_chi": ["skills.bend.skill.p0.K_x-5"],
            "p0_K_psi": ["skills.bend.skill.p0.K_x-6"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        return x_0

    def ground_skills(self):
        print(self.setup_instructions)
        self.learn_context["skills"]["bend"]["skill"]["objects"]["Bendable"] = self.objects["Bendable"]
        self.learn_context["skills"]["bend"]["skill"]["objects"]["GoalPose"] = self.objects["GoalPose"]
        self.setup_instructions[0]["parameters"]["skills"]["bend"]["skill"]["objects"]["Bendable"] = \
            self.objects["Bendable"]
        self.setup_instructions[0]["parameters"]["skills"]["bend"]["skill"]["objects"]["GoalPose"] = \
            self.objects["StartPose"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "StartPose"]
        self.reset_instructions[0]["parameters"]["skills"]["bend"]["skill"]["objects"]["Bendable"] = \
            self.objects["Bendable"]
        self.reset_instructions[0]["parameters"]["skills"]["bend"]["skill"]["objects"]["GoalPose"] =\
            self.objects["StartPose"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "StartPose"]

    def modify_contexts(self):
        self.setup_instructions[0]["parameters"]["skills"]["bend"]["user"]["env_X"] = [0.02, 0.02, 0.02, 0.1, 0.1, 0.1]
        self.setup_instructions[0]["parameters"]["skills"]["bend"]["skill"]["time_max"] = 20
        self.reset_instructions[0]["parameters"]["skills"]["bend"]["user"]["env_X"] = [0.02, 0.02, 0.02, 0.1, 0.1, 0.1]
        self.reset_instructions[0]["parameters"]["skills"]["bend"]["skill"]["time_max"] = 20

    def run_setup(self) -> bool:
        for r in self.robots:
            result = call_method(r, self.mios_port, "set_grasped_object", {"object": self.objects["Bendable"]})
        return True


class InsertionFactory2(ProblemDefinitionFactory):
    def __init__(self, robots: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        super().__init__(robots, "insertion2", [("Insertion2", "insertion2", "insertion2")],
                         [("MoveToPoseJoint", "move", "move_joint")],
                         [("TaxExtraction", "extraction", "extraction"),
                          ("MoveToPoseJoint", "move", "move_joint")], [],[], cost_function, objects, mios_port=mios_port)

    def get_limits(self):
        limits = {
            "p0_offset_x": (-0.005, 0.005),
            "p0_offset_y": (-0.005, 0.005),
            "p0_offset_phi": (-10, 10),
            "p0_offset_chi": (-10, 10),
            "p1_dx_d": (0, 0.1),
            "p1_dphi_d": (0, 0.5),
            "p1_ddx_d": (0, 0.5),
            "p1_ddphi_d": (0, 1),
            "p1_K_x": (0, 2000),
            "p1_K_phi": (0, 2000),
            "p2_wiggle_c_x": (0, 30),
            "p2_wiggle_c_y": (0, 30),
            "p2_wiggle_c_z": (0, 30),
            "p2_wiggle_c_phi": (0, 10),
            "p2_wiggle_c_chi": (0, 10),
            "p2_wiggle_c_psi": (0, 10), 
            "p2_wiggle_a_x": (0, 10),
            "p2_wiggle_a_y": (0, 10),
            "p2_wiggle_a_z": (0, 10),
            "p2_wiggle_a_phi": (0, 2),
            "p2_wiggle_a_chi": (0, 2),
            "p2_wiggle_a_psi": (0, 2), 
            "p2_wiggle_f_x": (0, 3),
            "p2_wiggle_f_y": (0, 3),
            "p2_wiggle_f_z": (0, 3),
            "p2_wiggle_f_phi": (0, 1),
            "p2_wiggle_f_chi": (0, 1),
            "p2_wiggle_f_psi": (0, 1),
            "p2_wiggle_phi_x": (0, 6.28),
            "p2_wiggle_phi_y": (0, 6.28),
            "p2_wiggle_phi_z": (0, 6.28),
            "p2_wiggle_phi_phi": (0, 6.28),
            "p2_wiggle_phi_chi": (0, 6.28),
            "p2_wiggle_phi_psi": (0, 6.28),
            "p2_delta_a_x": (-0.005, 0.005),
            "p2_delta_a_y": (-0.005, 0.005),
            "p2_delta_a_z": (-0.005, 0.005),
            "p2_delta_a_phi": (-10, 10),
            "p2_delta_a_chi": (-10, 10),
            "p2_delta_a_psi": (-10, 10), 
            "p2_delta_f_x": (0, 3),
            "p2_delta_f_y": (0, 3),
            "p2_delta_f_z": (0, 3),
            "p2_delta_f_phi": (0, 1),
            "p2_delta_f_chi": (0, 1),
            "p2_delta_f_psi": (0, 1),
            "p2_delta_phi_x": (0, 6.28),
            "p2_delta_phi_y": (0, 6.28),
            "p2_delta_phi_z": (0, 6.28),
            "p2_delta_phi_phi": (0, 6.28),
            "p2_delta_phi_chi": (0, 6.28),
            "p2_delta_phi_psi": (0, 6.28),
            "p2_K_x": (0, 2000),
            "p2_K_y": (0, 2000),
            "p2_K_z": (0, 2000),
            "p2_K_phi": (0, 200),
            "p2_K_chi": (0, 200),
            "p2_K_psi": (0, 200),
            "p2_t_d": (0, 5)
        }
        return limits

    def get_mapping(self):
        context_mapping = {
            "p0_offset_x": ["skills.insertion2.skill.p0.DeltaX-1"],
            "p0_offset_y": ["skills.insertion2.skill.p0.DeltaX-2"],
            "p0_offset_phi": ["skills.insertion2.skill.p0.DeltaX-4"],
            "p0_offset_chi": ["skills.insertion2.skill.p0.DeltaX-5"],
            "p1_dx_d": ["skills.insertion2.skill.p1.dX_d-1"],
            "p1_dphi_d": ["skills.insertion2.skill.p1.dX_d-2"],
            "p1_ddx_d": ["skills.insertion2.skill.p1.ddX_d-1"],
            "p1_ddphi_d": ["skills.insertion2.skill.p1.ddX_d-2"],
            "p1_K_x": ["skills.insertion2.skill.p1.K_x-1", "skills.insertion2.skill.p1.K_x-2",
                       "skills.insertion2.skill.p1.K_x-3"],
            "p1_K_phi": ["skills.insertion2.skill.p1.K_x-4", "skills.insertion2.skill.p1.K_x-5",
                         "skills.insertion2.skill.p1.K_x-6"],
            "p2_t_d": ["skills.insertion2.skill.p2.t_d"],
            "p2_wiggle_a_x": ["skills.insertion2.skill.p2.search_a-1"],
            "p2_wiggle_a_y": ["skills.insertion2.skill.p2.search_a-2"],
            "p2_wiggle_a_z": ["skills.insertion2.skill.p2.search_a-3"],
            "p2_wiggle_a_phi": ["skills.insertion2.skill.p2.search_a-4"],
            "p2_wiggle_a_chi": ["skills.insertion2.skill.p2.search_a-5"],
            "p2_wiggle_a_psi": ["skills.insertion2.skill.p2.search_a-6"],
            "p2_wiggle_f_x": ["skills.insertion2.skill.p2.search_f-1"],
            "p2_wiggle_f_y": ["skills.insertion2.skill.p2.search_f-2"],
            "p2_wiggle_f_z": ["skills.insertion2.skill.p2.search_f-3"],
            "p2_wiggle_f_phi": ["skills.insertion2.skill.p2.search_f-4"],
            "p2_wiggle_f_chi": ["skills.insertion2.skill.p2.search_f-5"],
            "p2_wiggle_f_psi": ["skills.insertion2.skill.p2.search_f-6"],
            "p2_wiggle_phi_x": ["skills.insertion2.skill.p2.search_phi-1"],
            "p2_wiggle_phi_y": ["skills.insertion2.skill.p2.search_phi-2"],
            "p2_wiggle_phi_z": ["skills.insertion2.skill.p2.search_phi-3"],
            "p2_wiggle_phi_phi": ["skills.insertion2.skill.p2.search_phi-4"],
            "p2_wiggle_phi_chi": ["skills.insertion2.skill.p2.search_phi-5"],
            "p2_wiggle_phi_psi": ["skills.insertion2.skill.p2.search_phi-6"],
            "p2_wiggle_c_x": ["skills.insertion2.skill.p2.search_c-1"],
            "p2_wiggle_c_y": ["skills.insertion2.skill.p2.search_c-2"],
            "p2_wiggle_c_z": ["skills.insertion2.skill.p2.search_c-3"],
            "p2_wiggle_c_phi": ["skills.insertion2.skill.p2.search_c-4"],
            "p2_wiggle_c_chi": ["skills.insertion2.skill.p2.search_c-5"],
            "p2_wiggle_c_psi": ["skills.insertion2.skill.p2.search_c-6"],

            "p2_delta_a_x": ["skills.insertion2.skill.p2.delta_a-1"],
            "p2_delta_a_y": ["skills.insertion2.skill.p2.delta_a-2"],
            "p2_delta_a_z": ["skills.insertion2.skill.p2.delta_a-3"],
            "p2_delta_a_phi": ["skills.insertion2.skill.p2.delta_a-4"],
            "p2_delta_a_chi": ["skills.insertion2.skill.p2.delta_a-5"],
            "p2_delta_a_psi": ["skills.insertion2.skill.p2.delta_a-6"],
            "p2_delta_f_x": ["skills.insertion2.skill.p2.delta_f-1"],
            "p2_delta_f_y": ["skills.insertion2.skill.p2.delta_f-2"],
            "p2_delta_f_z": ["skills.insertion2.skill.p2.delta_f-3"],
            "p2_delta_f_phi": ["skills.insertion2.skill.p2.delta_f-4"],
            "p2_delta_f_chi": ["skills.insertion2.skill.p2.delta_f-5"],
            "p2_delta_f_psi": ["skills.insertion2.skill.p2.delta_f-6"],
            "p2_delta_phi_x": ["skills.insertion2.skill.p2.delta_phi-1"],
            "p2_delta_phi_y": ["skills.insertion2.skill.p2.delta_phi-2"],
            "p2_delta_phi_z": ["skills.insertion2.skill.p2.delta_phi-3"],
            "p2_delta_phi_phi": ["skills.insertion2.skill.p2.delta_phi-4"],
            "p2_delta_phi_chi": ["skills.insertion2.skill.p2.delta_phi-5"],
            "p2_delta_phi_psi": ["skills.insertion2.skill.p2.delta_phi-6"],

            "p2_K_x": ["skills.insertion2.skill.p2.K_x-1"],
            "p2_K_y": ["skills.insertion2.skill.p2.K_x-2"],
            "p2_K_z": ["skills.insertion2.skill.p2.K_x-3"],
            "p2_K_phi": ["skills.insertion2.skill.p2.K_x-4"],
            "p2_K_chi": ["skills.insertion2.skill.p2.K_x-5"],
            "p2_K_psi": ["skills.insertion2.skill.p2.K_x-6"]
        }
        return context_mapping

    def get_initial_values(self):
        x_0 = dict()
        for p in self.limits:
            x_0[p] = 0.1

        x_0["p0_offset_x"] = 0.5
        x_0["p0_offset_y"] = 0.5
        x_0["p0_offset_phi"] = 0.5
        x_0["p0_offset_chi"] = 0.5
        x_0["p2_wiggle_c_x"] = 0.5
        x_0["p2_wiggle_c_y"] = 0.5
        x_0["p2_t_d"] = 4.5
        return x_0

    def ground_skills(self):
        #print(self.setup_instructions)
        self.learn_context["skills"]["insertion2"]["skill"]["objects"]["Approach"] = self.objects["Approach"]
        self.learn_context["skills"]["insertion2"]["skill"]["objects"]["Container"] = self.objects["Container"]
        self.learn_context["skills"]["insertion2"]["skill"]["objects"]["Insertable"] = self.objects["Insertable"]
        self.setup_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]
        self.reset_instructions[0]["parameters"]["skills"]["extraction"]["skill"]["objects"]["ExtractTo"] = \
        self.objects["Approach"]
        self.reset_instructions[0]["parameters"]["skills"]["extraction"]["skill"]["objects"]["Container"] = \
        self.objects["Container"]
        self.reset_instructions[0]["parameters"]["skills"]["extraction"]["skill"]["objects"]["Extractable"] = \
        self.objects["Insertable"]
        self.reset_instructions[0]["parameters"]["skills"]["move"]["skill"]["objects"]["goal_pose"] = self.objects[
            "Approach"]

    def run_setup(self) -> bool:
        for r in self.robots:
            result = call_method(r, self.mios_port, "set_grasped_object", {"object": self.objects["Insertable"]})
        return True

    def modify_contexts(self):
        self.learn_context["skills"]["insertion2"]["user"]["env_X"] = [0.003, 0.003, 0.002, 0.1, 0.1, 0.1]
        self.learn_context["skills"]["insertion2"]["skill"]["ROI_x"] = [-0.03, 0.03, -0.03, 0.03, -1, 1]
        self.learn_context["skills"]["insertion2"]["user"]["F_ext_contact"] = [12, 5]
