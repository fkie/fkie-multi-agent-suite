# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


import launch
from launch.substitutions.substitution_failure import SubstitutionFailure
from launch.utilities import perform_substitutions
from launch_ros.substitutions.executable_in_package import ExecutableInPackage


class LaunchConfigException(Exception):
    pass


def perform_to_string(
    context: launch.LaunchContext, value: list[list] | list[launch.Substitution] | str | None, *, verbose: bool = True
) -> str | None:
    result = ""
    if isinstance(value, str):
        result = value
    elif isinstance(value, list) and len(value) > 0:
        for val in value:
            sep = " "
            if isinstance(val, list):
                item = ""
                try:
                    item = perform_substitutions(context, val)
                except (SubstitutionFailure, LookupError) as err:
                    # if executable is not found we replace it by "ros2 run" command to visualize the error in the MAS gui
                    if isinstance(val[0], ExecutableInPackage):
                        executable = perform_substitutions(context, val[0].executable)
                        package = perform_substitutions(context, val[0].package)
                        item = f"ros2 run {package} {executable}"
                    else:
                        raise err
                except Exception as err:
                    if verbose:
                        import traceback

                        print(traceback.format_exc())
                    raise LaunchConfigException(err)
                # we fix command lines with {data: xyz}
                if " " in item and "{" in item:
                    item = f"'{item}'"
                result += item + sep
            else:
                result += perform_to_string(context, val)
    elif value is not None:
        try:
            if isinstance(value, tuple):
                for tuple_item in value:
                    if isinstance(tuple_item, list):
                        result += perform_substitutions(context, tuple_item)
                    else:
                        result += perform_substitutions(context, [tuple_item])
            elif hasattr(value, "perform"):
                result = perform_substitutions(context, [value])
            else:
                result = f"{value}"
        except (SubstitutionFailure, LookupError) as err:
            if verbose:
                import traceback

                print(traceback.format_exc())
            # if executable is not found we replace it by "ros2 run" command to visualize the error in the MAS gui
            if isinstance(value, ExecutableInPackage):
                executable = perform_substitutions(context, value.executable)
                package = perform_substitutions(context, value.package)
                result = f"ros2 run {package} {executable}"
            else:
                raise err
        except Exception as err:
            if verbose:
                import traceback

                print(traceback.format_exc())
            raise LaunchConfigException(err)
    else:
        result = None
    return result


def perform_to_tuple_list(
    context: launch.LaunchContext, value: list[tuple[list[launch.Substitution], list[launch.Substitution]]] | None
) -> list[tuple[str, str]] | None:
    result = []
    if value is not None:
        for val1, val2 in value:
            result.append((perform_substitutions(context, val1), perform_substitutions(context, val2)))
    else:
        result = None
    return result
