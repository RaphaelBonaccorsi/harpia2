from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.model.types import _UserType
from unified_planning.shortcuts import FluentExp
from unified_planning.model.operators import OperatorKind

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
            self.get_logger().warning(f"Type '{type_name}' not found in the domain.")
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

    def _evaluate_expression(self, expr, state):
        """Recursively evaluate expressions, including arithmetic operations."""
        # print(f"Evaluating: {expr}")
        if expr.is_constant():
            # print(expr.constant_value(), "is a constant")
            return expr.constant_value()  # Direct value for constants like numbers

        elif expr.is_fluent_exp():
            value = str(state.get(expr, None))
            if '/' in str(value):
                value = value.split('/')
                value = float(value[0]) / float(value[1])
            else:
                value = float(value)
            # print(value, "fluent")
            return value  # Lookup fluent value in the state

        elif expr.node_type in {
            OperatorKind.PLUS, OperatorKind.MINUS, 
            OperatorKind.TIMES, OperatorKind.DIV
        }:
            # print("is expression")
            evaluated_args = [self._evaluate_expression(arg, state) for arg in expr.args]
            # for arg in evaluated_args:
            #     print(arg, "evaluated arg")

            # print(expr.node_type, "operation")
            # Handle unary minus (e.g., -x)
            if expr.node_type == OperatorKind.MINUS and len(evaluated_args) == 1:
                return -evaluated_args[0]
            # Handle binary operations
            elif expr.node_type == OperatorKind.PLUS:
                return evaluated_args[0] + evaluated_args[1]
            elif expr.node_type == OperatorKind.MINUS:
                return evaluated_args[0] - evaluated_args[1]
            elif expr.node_type == OperatorKind.TIMES:
                return evaluated_args[0] * evaluated_args[1]
            elif expr.node_type == OperatorKind.DIV:
                return evaluated_args[0] / evaluated_args[1]

        else:
            raise NotImplementedError(f"Unsupported expression: {expr}")


    def _are_conditions_met(self, at_start_conditions):
        current_state = self.pddl.initial_values

        for condition in at_start_conditions:
            if hasattr(condition, 'node_type') and condition.node_type.name in {
                'GT', 'GE', 'LT', 'LE', 'EQUALS'
            }:
                left_expr, right_expr = condition.args
                # print("evaluating left side:")
                left_val = self._evaluate_expression(left_expr, current_state)
                # print("\n\nevaluating right side:")
                right_val = self._evaluate_expression(right_expr, current_state)

                # Apply the operator
                if condition.node_type.name == 'GT':
                    satisfied = left_val > right_val
                elif condition.node_type.name == 'GE':
                    satisfied = left_val >= right_val
                elif condition.node_type.name == 'LT':
                    satisfied = left_val < right_val
                elif condition.node_type.name == 'LE':
                    satisfied = left_val <= right_val
                elif condition.node_type.name == 'EQUALS':
                    satisfied = left_val == right_val
                else:
                    satisfied = False

                if not satisfied:
                    return False
            else:
                value = current_state.get(condition, False)
                value = str(value)=="true" # convert to boolean
                if not value:
                    return False

        return True


    # def substitute_parameters(self, condition, substitution_map):
    #     """Replace action parameters with concrete objects in a condition."""
    #     return condition.substitute(substitution_map)

    def _get_action_by_name(self, name):
        for action in self.pddl.actions:
            if action.name == name:
                return action
        return None


    def check_conditions_action(self, action_name, parameters, timings):

        print("checking:", action_name, " ".join(parameters))

        action = self._get_action_by_name(action_name)
        if action is None:
            raise ValueError(f"Action '{action_name}' not found in the domain.")

        # print("action:", action)

        at_start = []
        at_end = []
        over_all = []

        for k in action.conditions.keys():
            if str(k) == '[start]':
                at_start.extend(action.conditions[k])
            elif str(k) == '[end]':
                at_end.extend(action.conditions[k])
            elif str(k) == '(start, end)':
                over_all.extend(action.conditions[k])
            else:
                self.get_logger().error(f"Unknown timing: {k}")

        parameter_names = [param.name for param in action.parameters]

        if len(parameter_names) != len(parameters):
            raise ValueError(f"Number of parameters from action '{action_name}' ({len(parameter_names)}) does not match the number of provided parameters ({len(parameters)}).")

        # print("parameter_names:", parameter_names)
        # print("parameters:", parameters)

        substitution_map = {}
        for i in range(len(parameter_names)):
            substitution_map[action.parameter(parameter_names[i])] = self.pddl.object(parameters[i])
        # print("substitution_map:", substitution_map)

        if type(timings) is str:
            timings = [timings]

        conditions = []
        if "at start" in timings:
            conditions.extend(at_start)
        if "at end" in timings:
            conditions.extend(at_end)
        if "over all" in timings:
            conditions.extend(over_all)

        # Substitute parameters in conditions
        resolved_conditions = [
            cond.substitute(substitution_map)
            for cond in conditions
        ]
        print(f"resolved_conditions: {resolved_conditions}")

        # Check if resolved conditions are met
        all_met = self._are_conditions_met(resolved_conditions)
        print(f"All met: {all_met}")
        return all_met

    def apply_effects(self, action_name, parameters, timings):
        """Apply the effects of an action to the current state."""
        action = self._get_action_by_name(action_name)
        if action is None:
            raise ValueError(f"Action '{action_name}' not found in the domain.")

        if type(timings) is str:
            timings = [timings]

        temp = []
        if "at start" in timings:
            temp.append("start")
        if "at end" in timings:
            temp.append("end")
        timings = temp

        # Create substitution map for parameters
        substitution_map = {}
        for param, obj_name in zip(action.parameters, parameters):
            substitution_map[param] = self.pddl.object(obj_name)

        # Apply effects
        for timing, effects in action.effects.items():
            timing = str(timing)
            if str(timing) not in timings:
                continue
            print(f"\ncomp '{timing}'")
            for effect in effects:
                # Substitute parameters in the effect's fluent and value
                substituted_fluent = effect.fluent.substitute(substitution_map)
                substituted_value = effect.value.substitute(substitution_map)

                # Evaluate the value (if it's a complex expression)
                evaluated_value = self._evaluate_expression(substituted_value, self.pddl.initial_values)

                # Update the state
                self.pddl.set_initial_value(substituted_fluent, evaluated_value)
