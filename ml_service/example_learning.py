from problem_definition.problem_definition import ProblemDefinition
from services.base_service import ServiceConfiguration
from services.knowledge import Knowledge
from utils.experiment_wizard import start_experiment
from definitions.templates import InsertionFactory
from definitions.cost_functions import TimeMetric
from definitions.service_configs import SVMLearner, CMAESLearner

from xmlrpc.client import ServerProxy



def learn_task(robot:str, problem_definition: ProblemDefinition, service_config: ServiceConfiguration, tags: list,
               n_iterations: int = 10, keep_record: bool = False, knowledge = None, wait: bool = False, service_port:int = 8000):
    start_experiment(robot, [robot], problem_definition, service_config, n_iterations, knowledge=knowledge, tags=tags,
                     keep_record=keep_record, wait=wait,service_port=service_port)
    
    
def example_learning(robot:str = "localhost"):
    # tasks = {hosts: insertables}
    tasks =  {
        "localhost": "prism1"
    }
    for host, insertable in tasks.items():
        container = insertable + "_container"
        approach = container + "_approach"
        
        # configuring the learning problem (problem definition):
        # for every skill there is a definition class (eg InsertionFactory) that creates the problem_definition
        # input: list of agents (usually one robot -> IP of mios)
        #        cost function: see from definitions.cost_functions, eg.: TimeMetric (skill_class, max_time, heuristic=np.exp(var)")
        #        objects: for insertion: insertable, pose when insertable is inserted, pose when insertable is above the container
        pd = InsertionFactory([robot], TimeMetric("insertion", {"time": 5}),
                            {"Insertable": insertable, "Container": container,
                            "Approach": approach}).get_problem_definition(insertable)
        pd.variate_only_success = True  # repeat trial only when successful
        pd.n_variations = 3  # how often a trial should be repeated
        pd.host = host  # host (only for ducumentation)
        pd.optimum_thr = 0.3  # trial is taged as optimal when cost is under this threshold 
        pd.cost_function.finish_thr = 2  # learning is finished when this threshold is reached with optimal trials; if exploration mode of the learning service is True, learning will still contiue

        # Leaning Service configuration:
        # For Example: SVMLearner (https://proceedings.mlr.press/v155/voigt21a/voigt21a.pdf)
        # inputs: max trials
        #         batch size
        #         number of immigrants (old way of sharing knowledge; not used right now, keep it to 0)
        #         exploration mode: whether the learner should contiue optimizing after finding a solution
        #         batch synchronization: only used in a multi robot setup when all robots should start a batch at the same time; not need -> keep it to False
        #         request probability: new way of sharing knowledge - defines the probability for the ml_service to request knowledge from other agents instead of creating a new trial itself.
        #                              0.4 is a good probability in multi robot systems
        #         request_probability_decrease: whether the request probability should be automaticcly adapt to success rate (True) or be keept steady (False)
        sc = SVMLearner(3000,10,0,True,False, -1, True).get_configuration()
        
        # Knowledge source definition:
        # all information regarding where to find knowledge and kind of knowledge should be used
        # mode = mode  # either None, "specific", "local", "global"     (if "None", but parameters is not empty, the parameters will be used as centroid)
        # type = type  # also possible: "predicted" (use prediction), "all" (gives list of knowledges, no predicted ones),
        # scope = scope  # scope (tags of results to make this knowledge)
        # kb_location = kb_location  # location of the knowledge base
        # kb_db = kb_db  # needed if type is specific
        # kb_task_type = kb_task_type  # needed if type is specific
        # parameters = parameters #dict() with unnormalised Theta
        # confidence = confidence
        # uuid = uuid   # single uuid or list of uuids
        # prediction = prediction  # bool, wether this knowledge was predicted or not
        # prediction_error = prediction_error
        # identity = identity  # task identity
        # skill_class = skill_class  # eg. "insertion"
        # skill_instance = skill_instance  #  skill_instance from problem_definition
        # source = source  # uuid(s) of the source ml_results
        # expected_cost = expected_cost
        # time = time  # time when knowledge point was created (time.time())
        # datetime = datetime  # time.ctime()
        # tags = tags  #actual tags of the knowledge itself
        # equal_start = equal_start  # if True the svm.py will use the same first batch (from equal_tags) every time
        # equal_tags = equal_tags
        # cost_function = cost_function
        # identification_name = identification_name  # identification string, because uuid is random
        # time_range = time_range  # time window from which knowledge can be collected to create new knowledge points
        # similarity = similarity  # list of objects with request probabilities
        knowledge = Knowledge()  # nothing defined right now -> start from scratch
        knowledge = knowledge.to_dict()

        # this is a list of tags to find the entries on the mongoDB; 
        # the experiment wizard will append also some information here
        tags = ["example_learning", "tutorial"]  
        
        # helper function (experiment wizard):
        # mios IP
        # problem definition
        # service configuration
        # tags
        # knowledge source dict
        # number of iterations: how often should this experiment be repeated
        # service port: 8000
        # whether the function should return immediately or wait until learning is finished 
        learn_task(robot, pd, sc, tags, knowledge=knowledge, n_iterations=1,service_port=8000,wait=True)
        
def stop_services(robots:list = ["localhost"]):
    for r in robots:
        s = ServerProxy("http://" + r + ":8000", allow_none=True)
        try:
            s.stop_service()
        except Exception as e:
            print("Error with robot ",r)
            print(e)


if __name__ == "__main__":
    robot = "miosl.nuc3"
    example_learning(robot, "samuelnew")
    # stop_services([robot])