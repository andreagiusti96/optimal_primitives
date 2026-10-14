import argparse
from datetime import datetime
import json
import logging
from pathlib import Path
import time

import constants
from optimal_generator import OptimalGenerator

import matplotlib.pyplot as plt
import numpy as np


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def handle_arg_parsing():
    """
    Handle the parsing of arguments.

    Returns
    -------
    argparse.Namespace
        An object containing all parsed arguments

    """
    parser = argparse.ArgumentParser(description='Generate motionprimitives '
                                                 "for Nav2's State "
                                                 'Lattice Planner')
    parser.add_argument('--config',
                        type=Path,
                        default='./config_ackermann.json',
                        help='The config file containing the '
                        'parameters to be used')
    parser.add_argument('--output',
                        type=Path,
                        default='./outputs',
                        help='The output folder')

    return parser.parse_args()

def read_config(config_path) -> dict:
    """
    Read in the user defined parameters via JSON.

    Args:
    ----
    config_path: Path
        Path to the config file

    Returns
    -------
    dict
        Dictionary containing the user defined parameters

    """
    with open(config_path) as config_file:
        config = json.load(config_file)

    # # Curvature defaults keep existing configs backward-compatible.
    # config.setdefault('curvature_enabled', True)
    # config.setdefault('curvature_mode', 'strict')
    # config.setdefault('curvature_fallback_policy', 'zero')
    # config.setdefault('curvature_min', None)
    # config.setdefault('curvature_max', None)

    # Curvature-aware primitive generation (Phase 1). None -> curvature is
    # free (current behavior). 1 -> catalog is just [0.0]. Odd N>=3 ->
    # symmetric uniform catalog in +/- 1/turning_radius. Even N is promoted
    # to N+1 inside OptimalGenerator with a warning.
    config.setdefault('num_of_curvatures', None)
    config.setdefault('curvature_weight', 100.0)
    config.setdefault('curvature_tolerance', 0.1)

    return config

def create_heading_angle_list(minimal_set_trajectories: dict) -> list:
    """
    Create a sorted list of heading angles from the minimal trajectory set.

    Args:
    ----
    minimal_set_trajectories: dict
        The minimal spanning set

    Returns
    -------
    list
        A sorted list of heading angles

    """
    heading_angles = set(minimal_set_trajectories.keys())
    return sorted(heading_angles, key=lambda x: (x < 0, x))


def _primitive_boundary_kappas(trajectory) -> tuple:
    """Return (start_kappa, end_kappa) for a trajectory.

    Uses the target curvature stored on the parameters when available,
    otherwise falls back to the actual sampled curvature at the boundary.
    """
    params = trajectory.parameters
    if params.target_start_kappa is not None:
        start_k = float(params.target_start_kappa)
    else:
        start_k = float(trajectory.path.kappas[0])
    if params.target_end_kappa is not None:
        end_k = float(params.target_end_kappa)
    else:
        end_k = float(trajectory.path.kappas[-1])
    return round(start_k, 12), round(end_k, 12)


def create_curvature_catalog(minimal_set_trajectories: dict) -> list:
    """Create a sorted list of unique start/end curvature values."""
    curvatures = set()
    for trajectories in minimal_set_trajectories.values():
        for trajectory in trajectories:
            if trajectory.path.kappas is None or len(trajectory.path.kappas) == 0:
                continue
            start_k, end_k = _primitive_boundary_kappas(trajectory)
            curvatures.add(start_k)
            curvatures.add(end_k)
    return sorted(curvatures)

def create_header(config: dict, minimal_set_trajectories: dict) -> dict:

    header_dict = {
        'version': constants.VERSION,
        'date_generated': datetime.today().strftime('%Y-%m-%d'),
        'lattice_metadata': {},
        'primitives': [],
    }

    for key, value in config.items():
        header_dict['lattice_metadata'][key] = value

    heading_angles = create_heading_angle_list(minimal_set_trajectories)
    adjusted_heading_angles = [angle + 2*np.pi if angle < 0 else angle for angle in heading_angles]

    header_dict['lattice_metadata']['heading_angles'] = adjusted_heading_angles

    curvature_catalog = create_curvature_catalog(minimal_set_trajectories)
    header_dict['lattice_metadata']['num_of_curvatures'] = len(curvature_catalog)
    header_dict['lattice_metadata']['curvatures'] = curvature_catalog

    return header_dict

def write_to_json(output_path: Path, minimal_set_trajectories: dict, config: dict):

    strict_mode = config.get('curvature_mode', 'strict') == 'strict'

    output_dict = create_header(config, minimal_set_trajectories)

    curvature_catalog = output_dict['lattice_metadata']['curvatures']
    if output_dict['lattice_metadata']['num_of_curvatures'] != len(curvature_catalog):
        if not strict_mode:
            output_dict['lattice_metadata']['num_of_curvatures'] = len(curvature_catalog)
        else:
            raise ValueError('num_of_curvatures does not match size of curvature catalog')

    curvature_lookup = {
        round(float(curvature), 12): idx
        for idx, curvature in enumerate(curvature_catalog)
    }

    trajectory_start_angles = list(minimal_set_trajectories.keys())

    heading_angle_list = create_heading_angle_list(minimal_set_trajectories)
    heading_lookup = {angle: idx for idx, angle in
                      enumerate(heading_angle_list)}
    
    idx = 0
    for start_angle in sorted(trajectory_start_angles,
                              key=lambda x: (x < 0, x)):
        
        for trajectory in sorted(
            minimal_set_trajectories[start_angle],
            key=lambda x: x.parameters.end_angle
        ):

            traj_info = {}
            traj_info['trajectory_id'] = idx
            traj_info['start_angle_index'] = heading_lookup[trajectory.parameters.start_angle]
            traj_info['end_angle_index'] = heading_lookup[trajectory.parameters.end_angle]
            traj_info['left_turn'] = bool(trajectory.parameters.left_turn)
            traj_info['trajectory_radius'] = \
                trajectory.parameters.turning_radius
            traj_info['trajectory_length'] = round(
                trajectory.parameters.total_length, 5
            )
            traj_info['arc_length'] = round(
                trajectory.parameters.arc_length,
                5
            )
            traj_info['straight_length'] = round(
                trajectory.parameters.straight_length,
                5,
            )

            start_curvature, end_curvature = _primitive_boundary_kappas(trajectory)

            if start_curvature not in curvature_lookup or end_curvature not in curvature_lookup:
                if not strict_mode:
                    continue
                raise ValueError(
                    f"Trajectory {idx} curvature not found in lattice curvature catalog"
                )

            traj_info['start_curvature'] = start_curvature
            traj_info['end_curvature'] = end_curvature
            traj_info['start_curvature_index'] = curvature_lookup[start_curvature]
            traj_info['end_curvature_index'] = curvature_lookup[end_curvature]

            traj_info['poses'] = trajectory.path.to_output_format(include_curvature=True)

            output_dict['primitives'].append(traj_info)
            idx += 1
    
    output_dict['lattice_metadata']['number_of_trajectories'] = idx
    
    with open(output_path, 'w') as output_file:
        json.dump(output_dict, output_file, indent='\t')

def save_visualizations(visualizations_folder: Path, minimal_set_trajectories: dict) -> None:
    """
    Draw the visualizations for every trajectory and save it as an image.

    Args:
    ----
    visualizations_folder: Path
        The path to the folder for where to save the images
    minimal_set_trajectories: dict
        The minimal spanning set

    """
    # Create the directory if it doesnt exist
    visualizations_folder.mkdir(exist_ok=True)

    # All trajectories in one image
    for start_angle in minimal_set_trajectories.keys():

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, 'b')

    plt.grid(True)
    plt.axis('square')
    left_x, right_x = plt.xlim()
    left_y, right_y = plt.ylim()
    total_trajectories = sum(len(trajs) for trajs in minimal_set_trajectories.values())
    plt.title(f'{total_trajectories} trajectories')

    output_path = visualizations_folder / 'primitives_all.png'
    plt.savefig(output_path)
    plt.clf()

    # Trajectories per starting angle
    for start_angle in minimal_set_trajectories.keys():

        angle_in_deg = np.rad2deg(start_angle)

        if start_angle < 0 or start_angle > np.pi / 2:
            continue

        for trajectory in minimal_set_trajectories[start_angle]:
            plt.plot(trajectory.path.xs, trajectory.path.ys, 'b')
            plt.xlim(left_x, right_x)
            plt.ylim(left_y, right_y)

        plt.grid(True)
        plt.axis('square')
        plt.title(f'{len(minimal_set_trajectories[start_angle])} trajectories from start angle = {angle_in_deg:.1f}deg')

        output_path = visualizations_folder / f'primitives_ang{angle_in_deg:.1f}.png'
        plt.savefig(output_path)
        plt.clf()

if __name__ == '__main__':

    args = handle_arg_parsing()
    config = read_config(args.config)

    print(f'Start generating primitives for model {config["motion_model"]} with parameters: \n{json.dumps(config, indent=4)}\n')

    start = time.time()
    optimal_gen = OptimalGenerator(config)
    minimal_set_trajectories = optimal_gen.run()
    
    total_trajectories = sum(len(trajs) for trajs in minimal_set_trajectories.values())
    print(f'Finished generating primitives for model {config["motion_model"]}. Total {total_trajectories} trajectories from {len(minimal_set_trajectories)} start angles. \nTook {time.time() - start:.2f} seconds')

    # Create output directories and save results
    base_outputs_dir = args.output
    base_outputs_dir.mkdir(exist_ok=True)
    output_dir = base_outputs_dir / f'{config["motion_model"]}_primitives_opt'
    output_dir.mkdir(exist_ok=True)
    output_file = output_dir / f'{config["motion_model"]}_primitives_wcurv.json'
    figures_dir = output_dir / 'figures'
    figures_dir.mkdir(exist_ok=True)
    write_to_json(output_file, minimal_set_trajectories, config)
    save_visualizations(figures_dir, minimal_set_trajectories)
    
    print(f'Output written to {output_file}.\nVisualizations saved to {figures_dir}.')