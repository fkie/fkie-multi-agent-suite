# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************


from io import StringIO

from lxml import etree


class LaunchValidator:
    def __init__(self):
        self.xmlschema = None

    def validate(self, path: str) -> None:
        if self.xmlschema is None:
            return
        # open and read xml file
        with open(path) as xml_file:
            xml_to_check = xml_file.read()
        # parse xml
        try:
            doc = etree.parse(StringIO(xml_to_check))
            print("XML well formed, syntax ok.")

        # check for file IO error
        except OSError:
            print("Invalid File")

        # check for XML syntax errors
        except etree.XMLSyntaxError as err:
            raise Exception(err.error_log)
        # TODO: validate against schema
        try:
            self.xmlschema.assertValid(doc)
            print("XML valid, schema validation ok.")

        except etree.DocumentInvalid as err:
            raise Exception(err.error_log)
