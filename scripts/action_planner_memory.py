
class ActionPlannerMemory:
    _instance = None
    _initialized = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, domain):
        if not self._initialized:
            ActionPlannerMemory._initialized = True
            self.domain = domain
            self.problem = None
            
    def update_domain(self, domain):
        # Lógica para parsear o domain
        self.domain = domain
        return True

    def update_problem(self, problem):
        # Lógica para parsear o problem
        self.problem = problem
        return True