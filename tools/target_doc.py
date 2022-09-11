#!/usr/bin/python

# This file parses "Targets" doxygen HTML page and generates
# simplified page for PC aplication

# Written by Adam Berlinger

import sys,struct,zlib,os

try:
    from lxml import html
except ImportError:
    print 'This script requires lxml pyhton library'
    sys.exit(1)

def conv_tag(tree,tag_a,tag_b):
    elements = tree.xpath("//"+tag_a);
    for el in elements:
        el.tag = tag_b

if __name__=="__main__":
    if(len(sys.argv) != 2):
        print 'This script expects exactly one argument'
        sys.exit(1)
    f = open(sys.argv[1], 'rb')
    tree = html.parse(f)

    # fetch only content of the page
    filtered_tree = tree.xpath("//div[@class='contents']")[0]

    # add border to tables
    tables = filtered_tree.xpath("//table");
    for table in tables:
        table.attrib['border'] = "1"

    # replacing header tags
    # This needs to go from h3 to h1
    conv_tag(filtered_tree,'h3','h4')
    conv_tag(filtered_tree,'h2','h3')
    conv_tag(filtered_tree,'h1','h2')

    # add title
    title = html.Element('h1')
    title.text = "Targets"
    filtered_tree.insert(0,title)

    html_template = html.fromstring("""
<html><head>
<style type="text/css">
th {
    background-color: darkblue;
    color: white;
}
</style>
</head><body>
</body></html>
""")
    html_template.xpath('//body')[0].append(filtered_tree)

    print html.tostring(html_template)
    f.close()
