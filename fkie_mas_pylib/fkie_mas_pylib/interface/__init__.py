import json


class SelfEncoder(json.JSONEncoder):
    def default(self, obj):
        result = {}
        try:
            for key, value in vars(obj).items():
                if key[0] != '_':
                    result[key] = value
        except Exception as err:
            print(f"skipped {obj}: {err}")
        return result
