"""Shared topic and command routing for one independently controlled vehicle."""
from dataclasses import dataclass
import re


@dataclass(frozen=True)
class Vehicle:
    namespace: str
    system_id: int

    def topic(self, suffix):
        return f'{self.namespace}/{suffix.lstrip("/")}'


def resolve_vehicle(instance=0, namespace='auto', system_id=0):
    """Use the workspace's SITL defaults, or explicit hardware routing.

    'auto': instance 0 -> /fmu, instance N -> /px4_N/fmu.
    '/': explicitly use bare /fmu topics, even for a nonzero instance.
    system_id 0 selects instance + 1; commands never use broadcast ID 0.
    """
    if type(instance) is not int or not 0 <= instance <= 254:
        raise ValueError('vehicle_instance must be an integer from 0 to 254')
    if type(system_id) is not int or not 0 <= system_id <= 255:
        raise ValueError('target_system_id must be 0 (auto) or an integer from 1 to 255')
    if namespace == 'auto':
        namespace = '' if instance == 0 else f'px4_{instance}'
    namespace = namespace.strip('/')
    if namespace and any(not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', token)
                         for token in namespace.split('/')):
        raise ValueError('topic_namespace must be a ROS namespace, /, or auto')
    return Vehicle('/' + namespace if namespace else '', system_id or instance + 1)


def declare_vehicle(node):
    node.declare_parameter('vehicle_instance', 0)
    node.declare_parameter('topic_namespace', 'auto')
    node.declare_parameter('target_system_id', 0)
    return resolve_vehicle(
        node.get_parameter('vehicle_instance').value,
        node.get_parameter('topic_namespace').value,
        node.get_parameter('target_system_id').value)


def validate_pair(first, second):
    if first.namespace == second.namespace:
        raise ValueError('The two vehicles must use different topic namespaces')
    if first.system_id == second.system_id:
        raise ValueError('The two vehicles must use different MAV_SYS_ID values')
