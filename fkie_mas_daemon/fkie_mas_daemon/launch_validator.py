# The MIT License (MIT)

# Copyright (c) 2014-2024 Fraunhofer FKIE, Alexander Tiderko

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from lxml import etree
from io import StringIO

from fkie_mas_pylib import ros_pkg


class LaunchValidator(object):

    def __init__(self):
        self.launch_xsd = ros_pkg.get_share_files_path_from_package(
            'fkie_mas_daemon', 'launch.xsd')
        print('self.launch_xsd', self.launch_xsd)
        self.xmlschema = None
        # open and read schema file
        # TODO: fix xml launch schema
        # if self.launch_xsd:
        #     with open(self.launch_xsd[0], 'r') as schema_file:
        #         schema_to_check = schema_file.read()
        #     xmlschema_doc = etree.parse(StringIO(schema_to_check))
        #     self.xmlschema = etree.XMLSchema(xmlschema_doc)

    def validate(self, path: str) -> None:
        if self.xmlschema is None:
            return
        # open and read xml file
        with open(path, 'r') as xml_file:
            xml_to_check = xml_file.read()
        # parse xml
        try:
            doc = etree.parse(StringIO(xml_to_check))
            print('XML well formed, syntax ok.')

        # check for file IO error
        except IOError:
            print('Invalid File')

        # check for XML syntax errors
        except etree.XMLSyntaxError as err:
            raise Exception(err.error_log)
            # print('XML Syntax Error, see error_syntax.log')
            # with open('error_syntax.log', 'w') as error_log_file:
            #     error_log_file.write(str(err.error_log))
            # quit()
        # validate against schema
        try:
            self.xmlschema.assertValid(doc)
            print('XML valid, schema validation ok.')

        except etree.DocumentInvalid as err:
            raise Exception(err.error_log)
            # print('Schema validation error, see error_schema.log')
            # with open('error_schema.log', 'w') as error_log_file:
            #     error_log_file.write(str(err.error_log))
