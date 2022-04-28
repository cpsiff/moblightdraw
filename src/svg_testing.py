import numpy as np

def parse_svg_for_paths(svg_file):
    all_coords = []
    all_colors = []
    with open(svg_file,'r') as svgfile:
        for line in svgfile:
            l = line.strip()
            if(l[0:3] == 'd="'):
                coords = l[4:-1].split(" ")
                coords = [float(x) for x in coords][:2]
                all_coords.append(coords)
            elif(l[0:12] == 'stroke="rgb('):
                color = l[12:-2].split(" ")
                color = [int(x.strip(",")) for x in color]
                all_colors.append(color)
                
    return(np.array(all_coords), np.array(all_colors))

PATH_FILE_SVG = 'src/SVGtest2.svg'

print(parse_svg_for_paths(PATH_FILE_SVG))