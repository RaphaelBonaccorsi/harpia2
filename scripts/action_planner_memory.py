from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model.types import _UserType
from unified_planning.model import Problem, Object
from unified_planning.shortcuts import FluentExp

from unified_planning.model import FNode
from unified_planning.model.expression  import NumericConstant

class ActionPlannerMemory:
    _instance = None
    _initialized = False

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if not self._initialized:
            ActionPlannerMemory._initialized = True
    
    def init(self, domain_file, logger):
        self.logger = logger

        self.get_logger().info("Initializing ActionPlannerMemory...")
        if not self.set_domain(domain_file):
            self.get_logger().error("Failed to set domain.")
            return False
        
        return True

    def get_logger(self):
        if self.logger is None:
            raise ValueError("Logger not initialized, use the function .init()")
        return self.logger
            
    def set_domain(self, domain_file):
        self.get_logger().info(f"Setting domain '{domain_file}'")
        reader  = PDDLReader()
        try:    
            self.pddl = reader.parse_problem(domain_file)
        except Exception as e:
            self.get_logger().error(f"Error while setting domain: {e}")
            return False

        return True
    
    def add_instance(self, type_name, instance_name):
        try:
            obj_type = self.pddl.user_type(type_name)
        except KeyError:
            self.get_logger().warn(f"Type '{type_name}' not found in the domain.")
            return False
        
        # if the object already exists, return
        if self.pddl.has_object(instance_name):
            return True
        
        self.pddl.add_object(instance_name, obj_type)
        return True
        
    def get_instances(self, type_name):

        if type_name is not None:
            try:
                t: _UserType = self.pddl.user_type(type_name)
            except KeyError:
                return []

            objs = [o for o in self.pddl.all_objects if o.type == t]
            return objs
        else:
            return []
        
    def add_predicate(self, predicate_name, instance_names):
        """Adiciona um predicado ao estado inicial do problema.

        Args:
            predicate_name (str): Nome do predicado conforme definido no domínio PDDL.
            instance_names (list[str]): Lista de nomes das instâncias que compõem o predicado.

        Returns:
            bool: True se o predicado foi adicionado com sucesso, False caso contrário.
        """
        if not hasattr(self, 'pddl') or self.pddl is None:
            self.get_logger().error("Domain not loaded. Call set_domain first.")
            return False

        self.get_logger().info(f"Adding predicate '{predicate_name}' with instances {instance_names}")

        # Verifica se o predicado (fluent) existe no domínio
        fluent = next((f for f in self.pddl.fluents if f.name == predicate_name), None)
        if not fluent:
            self.get_logger().warning(f"Predicate '{predicate_name}' not found in the domain.")
            return False

        # Verifica a aridade do predicado usando a assinatura (signature)
        expected_arity = len(fluent.signature)
        if len(instance_names) != expected_arity:
            self.get_logger().warning(
                f"Predicate '{predicate_name}' expects {expected_arity} arguments, got {len(instance_names)}."
            )
            return False

        # Obtém os objetos correspondentes aos nomes das instâncias
        objects = []
        for obj_name in instance_names:
            obj = self.pddl.object(obj_name)
            if not obj:
                self.get_logger().warning(f"Object '{obj_name}' does not exist.")
                return False
            objects.append(obj)

        # Cria a expressão do predicado e adiciona ao estado inicial
        predicate_expression = FluentExp(fluent, objects)
        self.pddl.set_initial_value(predicate_expression, True)

        return True

    def remove_predicate(self, predicate_name, instance_names):
        """Remove um predicado do estado inicial do problema.

        Args:
            predicate_name (str): Nome do predicado conforme definido no domínio PDDL.
            instance_names (list[str]): Lista de nomes das instâncias que compõem o predicado.

        Returns:
            bool: True se o predicado foi removido com sucesso, False caso contrário.
        """
        if not hasattr(self, 'pddl') or self.pddl is None:
            self.get_logger().error("Domain not loaded. Call set_domain first.")
            return False

        self.get_logger().info(f"Removing predicate '{predicate_name}' with instances {instance_names}")

        # Verifica se o predicado (fluent) existe no domínio
        fluent = next((f for f in self.pddl.fluents if f.name == predicate_name), None)
        if not fluent:
            self.get_logger().warning(f"Predicate '{predicate_name}' not found in the domain.")
            return False

        # Verifica a aridade do predicado usando a assinatura (signature)
        expected_arity = len(fluent.signature)
        if len(instance_names) != expected_arity:
            self.get_logger().warning(
                f"Predicate '{predicate_name}' expects {expected_arity} arguments, got {len(instance_names)}."
            )
            return False

        # Obtém os objetos correspondentes aos nomes das instâncias
        objects = []
        for obj_name in instance_names:
            obj = self.pddl.object(obj_name)
            if not obj:
                self.get_logger().warning(f"Object '{obj_name}' does not exist.")
                return False
            objects.append(obj)

        # Cria a expressão do predicado
        predicate_expression = FluentExp(fluent, objects)

        # Remove o predicado do estado inicial (se existir)
        if predicate_expression in self.pddl.initial_values:
            del self.pddl.initial_values[predicate_expression]
            return True
        else:
            self.get_logger().warning(f"Predicate '{predicate_expression}' not found in initial state.")
            return False

    def set_function(self, function_name, instance_names, value):
        """Define o valor inicial de uma função numérica (fluent) no estado inicial.

        Args:
            function_name (str): Nome da função conforme definido no domínio PDDL.
            instance_names (list[str]): Lista de nomes das instâncias que compõem os parâmetros.
            value (float): Valor numérico a ser atribuído à função.

        Returns:
            bool: True se a função foi adicionada/atualizada com sucesso, False caso contrário.
        """
        if not hasattr(self, 'pddl') or self.pddl is None:
            self.get_logger().error("Domain not loaded. Call set_domain first.")
            return False

        self.get_logger().info(f"Setting function '{function_name}{instance_names}' = {value}")

        # Verifica se a função (fluent numérico) existe no domínio
        fluent = next((f for f in self.pddl.fluents if f.name == function_name), None)
        if not fluent:
            self.get_logger().warning(f"Function '{function_name}' not found in the domain.")
            return False

        # Verifica se a função é realmente numérica
        if not fluent.type.is_real_type() and not fluent.type.is_int_type():
            self.get_logger().warning(f"'{function_name}' is not a numeric function.")
            return False

        # Valida aridade
        if len(instance_names) != len(fluent.signature):
            self.get_logger().warning(
                f"Function '{function_name}' expects {len(fluent.signature)} arguments, got {len(instance_names)}."
            )
            return False

        # Obtém objetos
        objects = []
        for obj_name in instance_names:
            obj = self.pddl.object(obj_name)
            if not obj:
                self.get_logger().warning(f"Object '{obj_name}' does not exist.")
                return False
            objects.append(obj)

        # Cria expressão da função e define valor
        function_expression = FluentExp(fluent, objects)
        self.pddl.set_initial_value(function_expression, value)

        return True
    
    def add_goal(self, predicate_name, instance_names):
        """Define um objetivo para o problema de planejamento.

        Args:
            predicate_name (str): Nome do predicado/fluent que compõe o objetivo.
            instance_names (list[str]): Lista de instâncias do predicado.

        Returns:
            bool: True se o objetivo foi definido com sucesso, False caso contrário.
        """
        if not hasattr(self, 'pddl') or self.pddl is None:
            self.get_logger().error("Domain not loaded. Call set_domain first.")
            return False

        self.get_logger().info(f"Setting goal: {predicate_name}{instance_names}")

        # Verifica se o predicado existe
        fluent = next((f for f in self.pddl.fluents if f.name == predicate_name), None)
        if not fluent:
            self.get_logger().warning(f"Predicate '{predicate_name}' not found in domain.")
            return False

        # Valida aridade
        if len(instance_names) != len(fluent.signature):
            self.get_logger().warning(
                f"Goal expects {len(fluent.signature)} arguments, got {len(instance_names)}."
            )
            return False

        # Obtém objetos
        objects = []
        for obj_name in instance_names:
            obj = self.pddl.object(obj_name)
            if not obj:
                self.get_logger().warning(f"Object '{obj_name}' not found.")
                return False
            objects.append(obj)

        # Cria expressão do objetivo
        goal_expression = FluentExp(fluent, objects)

        # Adiciona novo objetivo
        self.pddl.add_goal(goal_expression)
        return True
    
    def clear_goals(self):
        self.pddl.clear_goals()

    def get_pddl(self):
        writer = PDDLWriter(self.pddl)
        return writer.get_domain(), writer.get_problem()