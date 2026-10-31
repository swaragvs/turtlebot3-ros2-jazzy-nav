#!/usr/bin/env python3
"""Rasterize the GLB maze footprint into a Nav2 occupancy map.

Generate the matching box-wall Gazebo model from the resulting PGM with
``generate_maze_world.py``.
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('glb')
    parser.add_argument('--out', default='maze_map')
    parser.add_argument('--scale', type=float, default=0.35)
    parser.add_argument('--res', type=float, default=0.05)
    parser.add_argument('--pad', type=float, default=1.0)
    parser.add_argument('--wall-thickness', type=float, default=0.10,
                        help='Minimum rasterized wall thickness in metres')
    parser.add_argument('--ss', type=int, default=8,
                        help='Supersampling factor')
    parser.add_argument('--keep-exterior-free', action='store_true')
    parser.add_argument('--boundary', type=float, default=7.5,
                        help='Outer boundary wall center coordinate')
    args = parser.parse_args()
    if args.res <= 0.0 or args.wall_thickness <= 0.0 or args.ss <= 0:
        parser.error('resolution, wall thickness, and supersampling must be positive')

    try:
        import trimesh
    except ImportError:
        parser.error('GLB map generation requires the Python package trimesh')

    scene = trimesh.load(args.glb)
    mesh = scene.dump(concatenate=True)
    rotation = trimesh.transformations.rotation_matrix(
        np.pi / 2.0, [1, 0, 0])
    mesh.apply_transform(rotation)
    mesh.apply_scale(args.scale)

    vertices, faces = mesh.vertices, mesh.faces
    triangles = vertices[faces]
    top_z = vertices[:, 2].max()
    top_faces = triangles[
        (triangles[:, :, 2] > top_z - 1e-3).all(axis=1)]
    print(
        f'Footprint x[{vertices[:, 0].min():.3f},'
        f'{vertices[:, 0].max():.3f}] '
        f'y[{vertices[:, 1].min():.3f},{vertices[:, 1].max():.3f}] '
        f'h={top_z:.3f} m, top triangles={len(top_faces)}')

    x0 = vertices[:, 0].min() - args.pad
    y0 = vertices[:, 1].min() - args.pad
    x1 = vertices[:, 0].max() + args.pad
    y1 = vertices[:, 1].max() + args.pad
    width = int(np.ceil((x1 - x0) / args.res))
    height = int(np.ceil((y1 - y0) / args.res))
    scale = args.ss
    image = Image.new('L', (width * scale, height * scale), 0)
    draw = ImageDraw.Draw(image)
    for triangle in top_faces:
        points = [
            (((point[0] - x0) / args.res) * scale,
             (height - (point[1] - y0) / args.res) * scale)
            for point in triangle
        ]
        draw.polygon(points, fill=255)

    coverage = (
        np.asarray(image, dtype=np.float32)
        .reshape(height, scale, width, scale)
        .mean(axis=(1, 3)) / 255.0)
    occupied = coverage > 0.10
    padding_cells = max(
        0, int(np.ceil(
            (args.wall_thickness - args.res) / (2.0 * args.res))))
    if padding_cells:
        occupied = ndimage.binary_dilation(
            occupied,
            structure=ndimage.generate_binary_structure(2, 1),
            iterations=padding_cells)

    boundary_image = Image.new('L', (width * scale, height * scale), 0)
    boundary_draw = ImageDraw.Draw(boundary_image)

    def px(world_x):
        return ((world_x - x0) / args.res) * scale

    def py(world_y):
        return (height - (world_y - y0) / args.res) * scale

    half_boundary = args.wall_thickness / 2.0
    boundary_rectangles = (
        (x0, args.boundary - half_boundary, x1,
         args.boundary + half_boundary),
        (x0, -args.boundary - half_boundary, x1,
         -args.boundary + half_boundary),
        (args.boundary - half_boundary, y0,
         args.boundary + half_boundary, y1),
        (-args.boundary - half_boundary, y0,
         -args.boundary + half_boundary, y1),
    )
    for left, bottom, right, top in boundary_rectangles:
        boundary_draw.rectangle(
            (px(left), py(top), px(right), py(bottom)), fill=255)

    boundary_coverage = (
        np.asarray(boundary_image, dtype=np.float32)
        .reshape(height, scale, width, scale)
        .mean(axis=(1, 3)) / 255.0)
    occupied |= boundary_coverage > 0.10

    occupancy_image = np.full((height, width), 254, dtype=np.uint8)
    occupancy_image[occupied] = 0
    labels, _ = ndimage.label(~occupied)
    border_labels = np.unique(np.concatenate((
        labels[0], labels[-1], labels[:, 0], labels[:, -1])))
    border_labels = border_labels[border_labels > 0]
    exterior = np.isin(labels, border_labels)
    interior_free = (~occupied & ~exterior).sum()
    print(
        f'Exterior free cells={exterior.sum()}, '
        f'interior free cells={interior_free}')
    if not args.keep_exterior_free and interior_free > 0:
        occupancy_image[exterior] = 127

    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(occupancy_image, mode='L').save(
        f'{args.out}.pgm')
    Image.fromarray(occupancy_image, mode='L').save(
        f'{args.out}_preview.png')
    yaml_text = (
        f'image: {output.name}.pgm\n'
        'mode: trinary\n'
        f'resolution: {args.res}\n'
        f'origin: [{x0:.4f}, {y0:.4f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n')
    output.with_suffix('.yaml').write_text(yaml_text, encoding='utf-8')
    print(
        f'Map {width}x{height}px, origin=({x0:.3f},{y0:.3f}), '
        f'size {width * args.res:.2f}x{height * args.res:.2f} m')


if __name__ == '__main__':
    main()
