import sys
import pandas as pd
from bokeh.plotting import figure,show,output_file
from bokeh.models import SingleIntervalTicker, LinearAxis

if len(sys.argv) != 3:
    print("Usage: {} <input-csv> <output-html>".format(sys.argv[0]))
    sys.exit(1)

table = pd.read_csv(sys.argv[1],
            header=None,
            names=['node','activity','start','stop','duration'],
            parse_dates=['start','stop'])
output_file(sys.argv[2])

nodes = table.node.unique()
nodes.sort()
p = figure(title=f"Activity",
           x_axis_type="datetime",
           x_axis_label="Time",
           y_range=[str(n) for n in nodes],
           y_axis_label="Node ID",
           width=1000,height=600)

for i,node in enumerate(nodes):
    cpu = table[(table.node==node) & (table.activity=='cpu') & (table.duration >= 0)]
    gps = table[(table.node==node) & (table.activity=='gps-fix')]
    radio = table[(table.node==node) & (table.activity=='radio')]
    abnormal = table[(table.node==node) & (table.duration > 10*60)]
    p.hbar(.5+i-0.25,0.2,cpu.stop,cpu.start,color="red",legend="CPU")
    p.hbar(.5+i     ,0.2,gps.stop,gps.start,color="green",legend="GPS")
    p.hbar(.5+i+0.25,0.2,radio.stop,radio.start,color="blue",legend="Radio")
    p.hbar(.5+i+0.4,0.05,abnormal.stop,abnormal.start,color="orange")
show(p)
