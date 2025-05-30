import json


class SelfEncoder(json.JSONEncoder):
    def default(self, obj):
        result = {}
        try:
            for key, value in vars(obj).items():
                if key[0] != '_':
                    if hasattr(value, "__len__") and len(value) == 0:
                        continue
                    if value:
                        result[key] = value
        except Exception as err:
            print(f"[SelfEncoder] skipped {obj}: {err}")
        return result


class SelfAllEncoder(json.JSONEncoder):
    def default(self, obj):
        result = {}
        try:
            for key, value in vars(obj).items():
                if key[0] != '_':
                    result[key] = value
        except Exception as err:
            print(f"[SelfAllEncoder] skipped {obj}: {err}")
        return result
