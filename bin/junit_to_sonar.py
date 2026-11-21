#!/usr/bin/env python3
"""Convert PlatformIO JUnit report to SonarQube generic test execution format."""
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

src, dst, env = sys.argv[1], sys.argv[2], sys.argv[3]

files = defaultdict(list)
for suite in ET.parse(src).getroot().iter('testsuite'):
    if suite.get('name', '').split(':')[0] != env:
        continue
    for case in suite.iter('testcase'):
        files[case.get('file')].append(case)

root = ET.Element('testExecutions', version='1')
for path, cases in files.items():
    f = ET.SubElement(root, 'file', path=path)
    for case in cases:
        duration = str(int(float(case.get('time', '0')) * 1000))
        t = ET.SubElement(f, 'testCase', name=case.get('name'), duration=duration)
        status = case.get('status', '')
        if status == 'FAILED':
            ET.SubElement(t, 'failure', message='failed')
        elif status == 'ERRORED':
            ET.SubElement(t, 'error', message='error')
        elif status == 'SKIPPED':
            ET.SubElement(t, 'skipped', message='skipped')

ET.indent(root)
ET.ElementTree(root).write(dst, encoding='utf-8', xml_declaration=True)
