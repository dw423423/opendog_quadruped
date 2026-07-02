#!/usr/bin/env python

import argparse
import sys
from pathlib import Path

from marionette_emgen import em_generate
from marionette_emgen.csv_config import Config
from marionette_emgen.urdf import *


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', '-o',
                        help='Output path')
    parser.add_argument('--no-virtual-links', action='store_true',
                        help='Do not generate with virtual links')
    parser.add_argument('--mujoco', action='store_true',
                        help='Generate MuJoCo related tags to validate URDF')
    parser.add_argument('--rgba', default='#C0C0C0FF',
                        help='Main color in Hex format')
    parser.add_argument('--mesh-prefix', default='../meshes/',
                        help='Prefix prepended to the mesh paths')

    args = parser.parse_args()

    cfg = Config.load('../csv')

    em_dict = dict(
        cfg=cfg,
        tag_link=lambda **kwargs: em_tag_link(cfg, mesh_scale=(0.001, 0.001, 0.001), mesh_prefix=args.mesh_prefix, **kwargs),
        tag_joint=lambda **kwargs: em_tag_joint(cfg, **kwargs),
        _no_virtual_links=args.no_virtual_links,
        _mujoco=args.mujoco,
        _main_rgba=(
            int(args.rgba[1:3], 16)/255.,
            int(args.rgba[3:5], 16)/255.,
            int(args.rgba[5:7], 16)/255.,
            int(args.rgba[7:9], 16)/255.,
        ),
    )

    if args.output is None:
        if args.no_virtual_links:
            args.output = 'ToGo_F_v1p0_novlnk.urdf'

    em_generate(
        Path('ToGo_F_v1p0.urdf.em'),
        args.output,
        em_dict=em_dict,
    )
