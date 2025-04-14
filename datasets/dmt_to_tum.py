
import argparse

def convert_dmt_to_tum(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()

    with open(output_file, 'w') as f:
        for line in lines:
            if line[0] == '#':
                continue
            data = line.split(",")
            data = [x.strip() for x in data] # Remove extra whitespace
            timestamp = data[0]
            tx = data[1]
            ty = data[2]
            tz = data[3]
            qx = data[4]
            qy = data[5]
            qz = data[6]
            qw = data[7]
            f.write(f'{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n')
            

parser = argparse.ArgumentParser(description='Convert DMT recorder cam poses to TUM format')
parser.add_argument('input_file', type=str, help='Trajectory recorded by DMT, to be converted')
parser.add_argument('output_file', type=str, help='Output file in TUM format')

args = parser.parse_args()
convert_dmt_to_tum(args.input_file, args.output_file)
