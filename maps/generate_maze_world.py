#!/usr/bin/env python3
"""Build a Gazebo box-wall model and matching occupancy map from a PGM."""
import argparse
import os
import re
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage


def read_map_metadata(path):
    text = Path(path).read_text(encoding='utf-8')
    resolution = re.search(r'(?m)^resolution:\s*([0-9.eE+-]+)', text)
    origin = re.search(
        r'(?m)^origin:\s*\[\s*([0-9.eE+-]+)\s*,\s*'
        r'([0-9.eE+-]+)\s*,\s*([0-9.eE+-]+)\s*\]', text)
    if resolution is None or origin is None:
        raise ValueError(f'{path} must define resolution and origin')
    return float(resolution.group(1)), tuple(
        float(value) for value in origin.groups())


def rectangles(mask):
    active = {}
    finished = []
    for row in range(mask.shape[0]):
        occupied_cols = np.flatnonzero(mask[row])
        if occupied_cols.size:
            split_at = np.flatnonzero(np.diff(occupied_cols) > 1) + 1
            spans = {
                (int(span[0]), int(span[-1]) + 1)
                for span in np.split(occupied_cols, split_at)
            }
        else:
            spans = set()

        for span in tuple(active):
            if span not in spans:
                finished.append((active.pop(span), row, span[0], span[1]))
        for span in spans:
            active.setdefault(span, row)

    finished.extend(
        (start, mask.shape[0], span[0], span[1])
        for span, start in active.items())
    return finished


def make_model(mask, resolution, origin, wall_height):
    height = mask.shape[0]
    blocks = []
    for index, (row0, row1, col0, col1) in enumerate(rectangles(mask)):
        size_x = (col1 - col0) * resolution
        size_y = (row1 - row0) * resolution
        center_x = origin[0] + (col0 + col1) * resolution / 2.0
        center_y = origin[1] + (
            height - (row0 + row1) / 2.0) * resolution
        center_z = wall_height / 2.0
        pose = (f'{center_x:.4f} {center_y:.4f} {center_z:.4f} '
                '0 0 0')
        size = f'{size_x:.4f} {size_y:.4f} {wall_height:.4f}'
        blocks.append(
            f'      <collision name="wall_{index}">\n'
            f'        <pose>{pose}</pose>\n'
            '        <geometry><box><size>' + size +
            '</size></box></geometry>\n'
            '        <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2>'
            '</ode></friction></surface>\n'
            '      </collision>\n'
            f'      <visual name="wall_visual_{index}">\n'
            f'        <pose>{pose}</pose>\n'
            '        <geometry><box><size>' + size +
            '</size></box></geometry>\n'
            '        <material><ambient>0.65 0.65 0.65 1</ambient>'
            '<diffuse>0.75 0.75 0.75 1</diffuse></material>\n'
            '      </visual>')

    if not blocks:
        raise ValueError('The occupancy image has no occupied wall cells')

    return (
        '<?xml version="1.0" ?>\n'
        '<sdf version="1.9">\n'
        '  <model name="maze">\n'
        '    <static>true</static>\n'
        '    <link name="walls">\n'
        + '\n'.join(blocks) + '\n'
        '    </link>\n'
        '  </model>\n'
        '</sdf>\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default='maps/maze_map_source.pgm')
    parser.add_argument('--map-yaml', default='maps/maze_map_source.yaml')
    parser.add_argument('--map-output', default='maps/maze_map.pgm')
    parser.add_argument('--map-yaml-output', default='maps/maze_map.yaml')
    parser.add_argument('--model-output', default='models/maze/model.sdf')
    parser.add_argument('--preview-output',
                        default='maps/maze_map_preview.png')
    parser.add_argument('--wall-height', type=float, default=0.84,
                        help='Height of the static maze walls in metres')
    args = parser.parse_args()

    if args.wall_height <= 0.0:
        parser.error('wall height must be positive')
    if Path(args.map).resolve() == Path(args.map_output).resolve():
        parser.error('input map and output map must be different files')

    resolution, origin = read_map_metadata(args.map_yaml)
    image = np.asarray(Image.open(args.map).convert('L')).copy()
    occupied = image < 64

    labels, _ = ndimage.label(~occupied)
    edge_labels = np.unique(np.concatenate((
        labels[0], labels[-1], labels[:, 0], labels[:, -1])))
    exterior = np.isin(labels, edge_labels[edge_labels > 0])
    image[exterior] = 127
    image[occupied] = 0

    map_output = Path(args.map_output)
    map_yaml_output = Path(args.map_yaml_output)
    yaml_text = Path(args.map_yaml).read_text(encoding='utf-8')
    image_reference = os.path.relpath(
        map_output, start=map_yaml_output.parent or Path('.'))
    yaml_text, replacements = re.subn(
        r'(?m)^image:\s*.*$', f'image: {image_reference}', yaml_text)
    if replacements != 1:
        raise ValueError(f'{args.map_yaml} must contain exactly one image entry')
    model = make_model(occupied, resolution, origin, args.wall_height)

    map_output.parent.mkdir(parents=True, exist_ok=True)
    map_yaml_output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image, mode='L').save(map_output)
    Path(args.preview_output).parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image, mode='L').save(args.preview_output)
    Path(args.model_output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.model_output).write_text(model, encoding='utf-8')
    map_yaml_output.write_text(yaml_text, encoding='utf-8')
    print(
        f'Wrote {args.model_output} with {len(rectangles(occupied))} '
        f'box walls; map is {image.shape[1]}x{image.shape[0]} at '
        f'{resolution:.3f} m/cell')


if __name__ == '__main__':
    main()
