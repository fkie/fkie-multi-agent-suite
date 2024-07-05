# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************

import os
import ruamel.yaml
import ruamel.yaml.compat
import threading

from fkie_mas_pylib.logging.logging import Log
from fkie_mas_pylib.defines import LOG_PATH
from fkie_mas_pylib.defines import SETTINGS_PATH


class Settings:

    def __init__(self, filename='', version=''):
        self._mutex = threading.RLock()
        self.version = version
        self.filename = filename
        if not self.filename:
            self.filename = f"{SETTINGS_PATH}mas-daemon.yaml"
        cfg_path = os.path.dirname(self.filename)
        if not os.path.isdir(cfg_path):
            os.makedirs(cfg_path)
        self._reload_callbacks = []
        self._cfg = None
        self.reload()

    def default(self):
        '''
        Creates a new default configuration.
        Value supports follow tags: {:value, :min, :max, :default, :hint(str), :ro(bool), :alt([]), :need_restart(bool)}
        '''
        result = {
            'global': {
                'version': {':value': self.version, ':ro': True},
                'file': {':value': self.filename, ':ro': True},
                'use_diagnostics_agg': {':value': True, ':hint': "subscribes to '/diagnostics_agg' topic instead of '/diagnostics'"},
                'reset': {':value': False, ':hint': 'if this flag is set to True the configuration will be reseted'},
                'log_viewer': {':value': '/usr/bin/less -fKLnQrSU'},
            },
            'sysmon':
            {
                'CPU':
                {
                    'load_warn_level': {':value': 0.9, ':type': 'float', ':min': 0, ':max': 1.0, ':hint': 'warn if more than one CPU core exceeds this percentage'},
                },
                'Disk':
                {
                    'usage_warn_level': {':value': 0.95, ':type': 'float', ':min': 0, ':max': 1.0, ':hint': "warn if used space exceeds this percentage"},
                    'path': {':value': LOG_PATH, ':hint': "Directory to observe", ':path': 'dir'}
                },
                'Memory':
                {
                    'usage_warn_level': {':value': 0.95, ':type': 'float', ':min': 0, ':max': 1.0, ':hint': "warn if used memory exceeds this percentage"},
                },
                'Network':
                {
                    'load_warn_level': {':value': 0.9, ':type': 'float', ':min': 0, ':max': 1.0, ':default': 0.9, ':hint': "warn if load exceeds the percentage of the maximum speed"},
                    'speed': {':value': 6, ':default': 6, ':type': 'float', ':min': 0, ':hint': "Maximal speed in MBit"},
                    'interface': {':value': '', ':default': '', ':hint': "interface to observe", ':alt': []},
                }
            }
        }
        # :TODO: 'paths': {':type': 'path[]', ':value': {'path': {':type': 'string', ':value': ''}}}
        return result

    def param(self, param_name, default_value=None, extract_value=True):
        '''
        Returns parameter value for given param_name.

        :param str param_name: name of the parameter. Namespace is separated by '/'.
        :param default_value: returns this value if parameter was not found (Default: None)
        :param bool extract_value: Since value is a dictionary with additional informations,
            try to extract value by default on True or return all options by False (Default: True).
        '''
        result = default_value
        try:
            path = param_name.split('/')
            value = self._cfg
            for item in path:
                value = value[item]
            if isinstance(value, dict):
                if extract_value and ':value' in value:
                    result = value[':value']
                else:
                    result = value
            else:
                result = value
        except Exception as exc:
            Log.debug(f"Cant't get parameter '{param_name}': {exc}")
        return result

    def set_param(self, param_name, value, tag=':value'):
        '''
        Sets new value to a parameter. The parameter can contain namespaces separated by '/'.
        Since a value can contain different tags, you can change the tag value
        by specifying the tag parameter.

        :param: str param_name: parameter name with namespaces.
        :param: value: new value.
        :param: str tag: tag name of parameter. It should begin with ':'.
        '''
        try:
            path = os.path.dirname(param_name).split('/')
            val_tag = tag if tag else ':value'
            cfg_item = self._cfg
            changed = False
            for item in path:
                if item:
                    if item in cfg_item:
                        cfg_item = cfg_item[item]
                    else:
                        cfg_item[item] = {}
                        cfg_item = cfg_item[item]
                        changed = True
            base_name = os.path.basename(param_name)
            if base_name in cfg_item:
                if isinstance(cfg_item[base_name], dict):
                    if self._is_writable(cfg_item[base_name]):
                        changed = cfg_item[base_name][val_tag] != value
                        cfg_item[base_name][val_tag] = value
                    else:
                        raise Exception(
                            f"{param_name} is a read only parameter!")
                else:
                    changed = cfg_item[base_name] != value
                    cfg_item[base_name] = value
            else:
                # create new parameter entry
                cfg_item[base_name] = {val_tag: value}
                changed = True
            if changed:
                self.save()
        except Exception as exc:
            Log.debug("Cant't set parameter '{param_name}': {exc}")

    def reload(self):
        '''
        Load the configuration from file. If file does not exists default configuration will be used.
        After configuration is loaded all subscribers are notified.
        '''
        with self._mutex:
            try:
                self._cfg = self.default()
                with open(self.filename, 'r') as stream:
                    yaml = ruamel.yaml.YAML(typ='rt')
                    result = yaml.load(stream)
                    if result is None:
                        Log.info('reset configuration file %s' %
                                 self.filename)
                        self.save()
                    else:
                        Log.info(
                            'loaded configuration from %s' % self.filename)
                        self._cfg = self._apply_recursive(result, self._cfg)
            except (ruamel.yaml.YAMLError, IOError) as exc:
                Log.info(f"{exc}: use default configuration!")
                self._cfg = self.default()
            self._notify_reload_callbacks()

    def save(self):
        '''
        Saves current configuration to file.
        '''
        with open(self.filename, 'w') as stream:
            try:
                yaml = ruamel.yaml.YAML(typ='rt')
                yaml.dump(self._cfg, stream)
                Log.debug(f"Configuration saved to '{self.filename}'")
            except ruamel.yaml.YAMLError as exc:
                Log.warn(
                    f"Cant't save configuration to '{self.filename}': {exc}")

    def fromString(self, data: str):
        stream = ruamel.yaml.compat.StringIO(data)
        yaml = ruamel.yaml.YAML(typ='rt')
        return yaml.load(stream)

    def apply(self, data: dict):
        '''
        Applies data (string representation of YAML).
        After new data are set the configuration will be saved to file.
        All subscribers are notified.

        :param str data: YAML as string representation.
        '''
        with self._mutex:
            self._cfg = self._apply_recursive(data, self._cfg)
            do_reset = self.param('global/reset', False)
            if do_reset:
                Log.info("Reset configuration requested!")
                self._cfg = self.default()
            else:
                Log.debug("new configuration applied, save now.")
            self.save()
            self._notify_reload_callbacks()

    def _apply_recursive(self, new_data, curr_value):
        new_cfg = dict()
        for key, value in curr_value.items():
            try:
                if isinstance(value, dict):
                    if self._is_writable(value):
                        new_cfg[key] = self._apply_recursive(
                            new_data[key], value)
                elif key not in [':hint', ':default', ':ro', ':min', ':max', ':alt']:
                    if isinstance(new_data, dict):
                        new_cfg[key] = new_data[key]
                    else:
                        new_cfg[key] = new_data
                else:
                    new_cfg[key] = value
            except Exception:
                import traceback
                print("_apply_recursive error:",
                      traceback.format_exc(), "use old value:", value)
                new_cfg[key] = value
        return new_cfg

    def _is_writable(self, value):
        if ':ro' in value:
            return value[':ro']
        return True

    def add_reload_listener(self, callback, call=True):
        '''
        Adds a subscriber to change notifications. All subscribers are notified on any changes.

        :param callback: Method of type callback(Settings)
        :param call: if True the callback is called after adding. (Default: True)
        '''
        with self._mutex:
            if callback not in self._reload_callbacks:
                self._reload_callbacks.append(callback)
                if call:
                    callback(self)

    def _notify_reload_callbacks(self):
        with self._mutex:
            for callback in self._reload_callbacks:
                callback(self)
