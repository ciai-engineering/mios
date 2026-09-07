"""Offline safety checks for the supervised physical-learning configuration."""

from definitions.cost_functions import TimeMetric
from definitions.templates import InsertionFactory
from example_learning import (
    SUPERVISED_INSERTION_APPROACH_DDX,
    SUPERVISED_INSERTION_APPROACH_DX,
    SUPERVISED_INSERTION_LIMITS,
    configure_supervised_insertion_dynamics,
    constrain_supervised_insertion_domain,
    supervised_nominal_knowledge,
)


def _insertion_problem_definition():
    return InsertionFactory(["127.0.0.1"], TimeMetric("insertion", {"time": 15}), {
        "Insertable": "samuelnew",
        "Container": "samuelnew_container",
        "Approach": "samuelnew_container_approach",
    }).get_problem_definition("samuelnew")


def test_supervised_first_trial_is_the_nominal_taught_baseline():
    problem_definition = _insertion_problem_definition()
    constrain_supervised_insertion_domain(problem_definition)

    knowledge = supervised_nominal_knowledge(problem_definition)
    parameters = knowledge["parameters"]

    assert list(parameters) == problem_definition.domain.vector_mapping
    assert parameters == problem_definition.domain.x_0
    assert knowledge["meta"]["mode"] is None
    assert knowledge["meta"]["confidence"] == 0.0
    assert parameters["p0_offset_x"] == 0.0
    assert parameters["p0_offset_y"] == 0.0
    assert parameters["p0_offset_phi"] == 0.0
    assert parameters["p0_offset_chi"] == 0.0


def test_supervised_contact_approach_is_capped_at_low_speed():
    problem_definition = _insertion_problem_definition()
    constrain_supervised_insertion_domain(problem_definition)

    assert problem_definition.domain.limits["p1_dx_d"] == (0.025, 0.030)
    assert problem_definition.domain.x_0["p1_dx_d"] == 0.0275
    assert SUPERVISED_INSERTION_LIMITS["p1_dx_d"][1] == 0.030


def test_supervised_p0_profile_stays_below_its_hard_twist_guard():
    problem_definition = _insertion_problem_definition()
    configure_supervised_insertion_dynamics(problem_definition)

    insertion = problem_definition.default_context["skills"]["insertion"]
    assert insertion["skill"]["p0"]["dX_d"] == list(SUPERVISED_INSERTION_APPROACH_DX)
    assert insertion["skill"]["p0"]["ddX_d"] == list(SUPERVISED_INSERTION_APPROACH_DDX)
    # The guard is deliberately unchanged; the command profile is lower.
    assert insertion["limits"]["cartesian_space"]["dX_max"] == [0.5, 1]
