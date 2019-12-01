import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from itertools import groupby
from matplotlib.legend_handler import HandlerLine2D


class SimulationResult:
    def __init__(self, line):
        self.simulation_id = line[0]
        self.cur_speed = float(line[1])
        self.ra = float(line[2])
        self.tug_speed = float(line[2])
        self.ship_speed = float(line[3])
        self.min_obstacle_distance_1 = line[4]
        self.min_obstacle_distance = line[5]
        if line[7] is not None and line[7]!='':
            if line[7] == 'True':
                self.overtake_right_1 = True
            else: 
                self.overtake_right_1 = False
        else:
            self.overtake_right_1 = None
        self.sim_duration = line[8]

        if self.overtake_right_1 is not None:
            if self.overtake_right_1:
                self.distance_at_overtake_ship1 = float(line[9])
            else:
                self.distance_at_overtake_ship1 = -float(line[9])
        else:
            self.distance_at_overtake_ship1 = None

simulations = []
import csv
with open('/home/geus/asw_workspace/src/collision_avoidance_in_singapore/script/results.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        if line_count == 0:
            print("Column names are "+str(", ".join(row)))
            line_count += 1
        else:
            simulations.append(SimulationResult(row))
            print("\t" + str(row[0])+ ", "+ str(row[1] ))
            line_count += 1
    print("Processed" + str(line_count) + " lines.")


grouped_ship_speed_simulations = [(k, [v for v in g]) for k,g in groupby(sorted(simulations,key = lambda s: s.ship_speed), key=lambda s:s.ship_speed)]

ship_speeds_per_plot = []
ship_offsets_per_plot = []
sea_speeds_per_plot = []

def colors_from_speed(curr_speed):
    if curr_speed == 0.2:
        return 0
    elif curr_speed == 0.8:
        return 1
    else:
        return 2

colorsmap = ["blue", "red", "green"]


for kv, ksims in grouped_ship_speed_simulations:
    ship_offsets=[]
    sea_speeds = []
    for r in ksims: 
        if r.distance_at_overtake_ship1  is not None:
            ship_offsets.append(r.distance_at_overtake_ship1)
            sea_speeds.append(colors_from_speed(r.cur_speed))
    ship_offsets_per_plot.append(ship_offsets)
    sea_speeds_per_plot.append(sea_speeds)
    ship_speeds_per_plot.append(kv)


text_style = dict(horizontalalignment='right', verticalalignment='center',
                  fontsize=12, fontdict={'family': 'monospace'})

marker_style = dict(linestyle='None', markersize=8 )


def format_axes(ax, i):
    ax.margins(0.2)
    #ax.set_axis_off()
    ax.invert_yaxis()
    #ax.set_yticklabels([])
    ax.get_yaxis().set_visible(False)
    ax.set_xlim([-90,90])
    ax.set_ylim([-10,10])
    ax.set_xlabel('distance (m)')

    if i == 0:
        blue_star = Line2D([1], [1], color=colorsmap [0], marker='o', linestyle='None',
                          markersize=8, label='0.2 m/s')
        red_square = Line2D([], [], color= colorsmap[1], marker='o', linestyle='None',
                          markersize=8, label='0.8 m/s')
        purple_triangle = Line2D([], [], color= colorsmap[2], marker='o', linestyle='None',
                          markersize=8, label='1.4 m/s')

        leg = ax.legend(handles=[blue_star, red_square, purple_triangle], handler_map={blue_star: HandlerLine2D(numpoints=1), red_square: HandlerLine2D(numpoints=1), purple_triangle: HandlerLine2D(numpoints=1)})
        leg.set_title('Sea current speed', prop={'size': 14, 'weight': 'heavy'})

    ax.set_xticks(np.arange(start=-80,stop = 90, step = 20))
    pad = 0
    ax.annotate( "Ship speed: \n" + str(ship_speeds_per_plot [i]) + " m/s", xy=(0, 0.5), xytext=(20 , 0 ),
                xycoords=ax.yaxis.label, textcoords='offset points',
                size='large', ha='left', va='center')


def split_list(a_list):
    i_half = len(a_list) // 2
    return (a_list[:i_half], a_list[i_half:], a_list[i_half:])

# Filter out filled markers and marker settings that do nothing.
unfilled_markers = [m for m, func in Line2D.markers.items()
                    if func != 'nothing' and m not in Line2D.filled_markers]


data = split_list(Line2D.filled_markers)

fig, axes = plt.subplots(nrows=3)
fig.subplots_adjust(left=0.1, top=0.9, hspace= 0.6)

i = 0
for ax, markers in zip(axes, data):
    ax.plot([-90, 90], [-2,-2], color='gray', linestyle='dotted', linewidth=1, markersize=12)
    #ax.text(-0.5, y, repr(marker), **text_style)
    ax.plot([0,0], [-4, 0], color = 'black', linewidth=1)
    zs =np.zeros(len(ship_offsets_per_plot[i]))
    
    offsets = ship_offsets_per_plot[i]
    colors = sea_speeds_per_plot[i]
    for j in range(len(offsets)):
        ax.plot([offsets[j]], [-2], color = colorsmap [colors[j]] ,marker='o',  **marker_style)
    
    ax.plot()
    format_axes(ax, i )
    i+=1
fig.suptitle('Tug to Ship distance', fontsize=14)

plt.show()