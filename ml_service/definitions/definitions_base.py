from problem_definition.domain import Domain
from problem_definition.problem_definition import ProblemDefinition
from problem_definition.problem_definition import CostFunction
from services.base_service import ServiceConfiguration
import abc
from abc import ABC
import os
import json


class LearnerFactory(ABC):
    def __init__(self, configuration: ServiceConfiguration):
        self.configuration = configuration

    def get_configuration(self):
        return self.configuration


class CostFunctionFactory(ABC):
    def __init__(self, skill_class: str, weights: dict, max_cost: dict, heuristic: str):
        self.cost_function = CostFunction()
        self.cost_function.optimum_skills = [skill_class]
        self.cost_function.heuristic_skills = [skill_class]
        self.cost_function.optimum_weights.update(weights)
        self.cost_function.max_cost.update(max_cost)
        self.cost_function.heuristic_expressions = heuristic

    def get_cost_function(self):
        return self.cost_function


class ProblemDefinitionFactory(ABC):

    def __init__(self, robots: list, skill_class: str, learn_skills: list, setup_skills: list, reset_skills: list, rescue_skils:list,
                 termination_skills: list, cost_function: CostFunctionFactory, objects: dict, mios_port=12000):
        project_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        )
        self.path_to_default_context = os.path.join(
            project_root, "python", "taxonomy", "default_contexts"
        )
        self.skill_class = skill_class
        self.cost_function = cost_function
        #print(robots)
        self.robots = robots
        self.objects = objects
        self.mios_port = mios_port

        self.learn_context = dict()
        self.setup_instructions = list()
        self.reset_instructions = list()
        self.rescue_instructions = list()
        self.termination_instructions = list()

        self.load_default_skill_contexts(learn_skills, setup_skills, reset_skills, rescue_skils, termination_skills)
        self.modify_contexts()
        #self.run_setup()

        self.limits = self.get_limits()
        self.mapping = self.get_mapping()
        self.x_0 = self.get_initial_values()
        self.ground_skills()
        self.domain = Domain(self.limits, self.mapping, self.x_0)

    def get_problem_definition(self, task_name: str, identity: list = None) -> ProblemDefinition:
        if identity is None:
            identity = [1]
        pd = ProblemDefinition(self.skill_class, task_name, self.domain, self.learn_context,
                               self.setup_instructions, self.termination_instructions, self.reset_instructions, self.rescue_instructions,
                               self.cost_function.get_cost_function(), identity, tags=[self.skill_class, task_name])
        return pd

    @abc.abstractmethod
    def get_limits(self) -> dict:
        raise NotImplementedError

    @abc.abstractmethod
    def get_mapping(self) -> dict:
        raise NotImplementedError

    @abc.abstractmethod
    def get_initial_values(self) -> dict:
        raise NotImplementedError

    def load_default_skill_contexts(self, learn_skills: list, setup_skills: list, reset_skills: list, rescue_skills:list,
                                    termination_skills: list):
        self.learn_context = {
            "name": "GenericTask",
            "parameters": {
                "skill_types": [],
                "skill_names": []
            },
            "skills": dict()
        }
        for s in learn_skills:
            f = open(os.path.join(self.path_to_default_context, s[2] + ".json"))
            self.learn_context["skills"][s[1]] = json.load(f)
            self.learn_context["parameters"]["skill_types"].append(s[0])
            self.learn_context["parameters"]["skill_names"].append(s[1])
            self.learn_context["skills"][s[1]]["control"]["nullspace"] = {
                                                                            "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                                            "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                                            "active": True
                                                                        }

        setup_context = {
            "name": "GenericTask",
            "parameters": {
                "skill_types": [],
                "skill_names": []
            },
            "skills": dict()
        }
        for s in setup_skills:
            f = open(os.path.join(self.path_to_default_context, s[2] + ".json"))
            setup_context["skills"][s[1]] = json.load(f)
            setup_context["parameters"]["skill_types"].append(s[0])
            setup_context["parameters"]["skill_names"].append(s[1])
            setup_context["skills"][s[1]]["control"]["nullspace"] = {
                                                                        "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                                        "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                                        "active": True
                                                                    }

        self.setup_instructions.append({"method": "start_task", "parameters": setup_context})

        reset_context = {
            "name": "GenericTask",
            "parameters": {
                "skill_types": [],
                "skill_names": []
            },
            "skills": dict()
        }
        for s in reset_skills:
            f = open(os.path.join(self.path_to_default_context, s[2] + ".json"))
            reset_context["skills"][s[1]] = json.load(f)
            reset_context["parameters"]["skill_types"].append(s[0])
            reset_context["parameters"]["skill_names"].append(s[1])
            reset_context["skills"][s[1]]["control"]["nullspace"] = {
                                                                        "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                                        "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                                        "active": True
                                                                    }

        self.reset_instructions.append({"method": "start_task", "parameters": reset_context})

        termination_context = {
            "name": "GenericTask",
            "parameters": {
                "skill_types": [],
                "skill_names": []
            },
            "skills": dict()
        }
        for s in termination_skills:
            f = open(os.path.join(self.path_to_default_context, s[2] + ".json"))
            self.termination_instructions.append(json.load(f))
            termination_context["skills"][s[1]] = json.load(f)
            termination_context["parameters"]["skill_types"].append(s[0])
            termination_context["parameters"]["skill_names"].append(s[1])
            termination_context["skills"][s[1]]["control"]["nullspace"] = {
                                                                        "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                                        "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                                        "active": True
                                                                    }
            
        rescue_context = {
            "name": "GenericTask",
            "parameters": {
                "skill_types": [],
                "skill_names": []
            },
            "skills": dict()
        }
        for s in rescue_skills:
            f = open(os.path.join(self.path_to_default_context, s[2] + ".json"))
            rescue_context["skills"][s[1]] = json.load(f)
            rescue_context["parameters"]["skill_types"].append(s[0])
            rescue_context["parameters"]["skill_names"].append(s[1])
            rescue_context["skills"][s[1]]["control"]["nullspace"] = {
                                                                        "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                                        "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                                        "active": True
                                                                    }

        self.rescue_instructions.append({"method": "start_task", "parameters": rescue_context})

        termination_context = {
            "name": "GenericTask",
            "parameters": {
                "skill_types": [],
                "skill_names": []
            },
            "skills": dict()
        }
        for s in termination_skills:
            f = open(os.path.join(self.path_to_default_context, s[2] + ".json"))
            self.termination_instructions.append(json.load(f))
            termination_context["skills"][s[1]] = json.load(f)
            termination_context["parameters"]["skill_types"].append(s[0])
            termination_context["parameters"]["skill_names"].append(s[1])
            termination_context["skills"][s[1]]["control"]["nullspace"] = {
                                                                        "K_theta": [20, 20, 15, 10, 7, 5, 2],
                                                                        "xi_theta": [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7],
                                                                        "active": True
                                                                    }

        self.termination_instructions.append({"method": "start_task", "parameters": termination_context})

    def modify_contexts(self):
        pass

    def run_setup(self):
        pass

    @abc.abstractmethod
    def ground_skills(self, objects: dict):
        raise NotImplementedError
