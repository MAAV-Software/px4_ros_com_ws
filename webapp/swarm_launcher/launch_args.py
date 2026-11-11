#!/usr/bin/env python3
"""Print the arguments a launch file declares, as JSON, for the phone to show and edit.

  python3 -m swarm_launcher.launch_args /path/to/share/pkg/square_launch.py

Runs as its own process because loading a launch file runs its Python, and that must
not stall or crash the agent. Needs a sourced ROS 2 environment.

Arguments declared in an included file are listed too, except where the include sets
them itself, since a value given on the command line would not reach those. Defaults
are worked out in declaration order, so one that refers to an earlier argument (such
as a path built from `LaunchConfiguration('path')`) shows its real value. A default
that can only be known once the launch runs comes back as null.
"""
import json
import os
import sys


def _walk(entities, includes):
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ResetLaunchConfigurations

    for entity in entities:
        if isinstance(entity, DeclareLaunchArgument):
            yield entity, includes
        if isinstance(entity, ResetLaunchConfigurations):
            return
        inner = includes + [entity] if isinstance(entity, IncludeLaunchDescription) else includes
        yield from _walk(entity.describe_sub_entities(), inner)
        for _, sub_entities in entity.describe_conditional_sub_entities():
            yield from _walk(sub_entities, inner)


def _set_by_include(context, name, includes):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    for include in includes:
        for key, _ in include.launch_arguments:
            try:
                # Kept as given, so a plain string here, as from {'record': 'true'}.items().
                if perform_substitutions(context, normalize_to_list_of_substitutions(key)) == name:
                    return True
            except Exception:  # A name that can't be worked out yet can't be this one.
                continue
    return False


def describe(path):
    from launch import LaunchContext
    from launch.launch_description_sources import get_launch_description_from_python_launch_file
    from launch.utilities import perform_substitutions

    description = get_launch_description_from_python_launch_file(path)
    context = LaunchContext()
    context.extend_locals({'current_launch_file_path': path,
                           'current_launch_file_directory': os.path.dirname(path)})
    declared = list(_walk(description.entities, []))
    # The file's own arguments first: on a name clash they are the ones that apply.
    declared.sort(key=lambda item: bool(item[1]))
    seen = set()
    result = []
    for action, includes in declared:
        if action.name in seen or (includes and _set_by_include(context, action.name, includes)):
            continue
        seen.add(action.name)
        default = None
        if action.default_value is not None:
            try:
                default = perform_substitutions(context, action.default_value)
                context.launch_configurations[action.name] = default
            except Exception:
                default = None
        result.append({
            'name': action.name,
            'default': default,
            'description': '' if action.description == 'no description given' else action.description,
            'choices': list(getattr(action, 'choices', None) or []),
            'included': bool(includes),
        })
    return result


def main():
    if len(sys.argv) != 2:
        sys.exit('usage: python3 -m swarm_launcher.launch_args <launch file>')
    print(json.dumps(describe(sys.argv[1])))


if __name__ == '__main__':
    main()
