import psutil
from typing import List
from typing import Tuple


def get_child_pid(pid: int) -> Tuple[int, str, List[int]]:
    # try to determine the process id of the node inside the screen
    found_deep = -1
    found_pid = -1
    found_name = ""
    parents2kill = []
    try:
        for process in psutil.process_iter():
            deep = -1
            found = False
            parents = process.parents()
            # search for parents with screen process id
            for p in parents:
                deep += 1
                if p.pid == pid:
                    found = True
                    break
            # use the process with most parents
            if found and deep > found_deep:
                found_deep = deep
                found_pid = process.pid
                found_name = process.name()
                parents2kill = parents
    except Exception as error:
        # fallback for psutil versions (<5.6.0) without Process.parents()
        current_pid = pid
        new_pid = current_pid
        new_name = ""
        while new_pid == current_pid:
            new_pid = -1
            # search for process which has screen id as parent
            for process in psutil.process_iter():
                parent = process.parent()
                if parent and parent.pid == current_pid:
                    new_pid = process.pid
                    new_name = process.name()
                    parents2kill.append(current_pid)
                    current_pid = process.pid
        if current_pid != pid:
            found_pid = current_pid
            found_name = new_name
    return (found_pid, found_name, parents2kill)
