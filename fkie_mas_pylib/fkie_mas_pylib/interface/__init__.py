import json


class SelfEncoder(json.JSONEncoder):
    def default(self, obj):
        result = {}
        for key, value in vars(obj).items():
            if key[0] != '_':
                result[key] = value
        return result
