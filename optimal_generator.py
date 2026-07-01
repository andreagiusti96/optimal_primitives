from collections import defaultdict
from enum import Enum
import warnings

from helper import angle_difference, interpolate_yaws

import numpy as np

from rtree import index

from trajectory import Path, Trajectory, TrajectoryParameters

from trajectory_generator import TrajectoryGenerator

class OptimalGenerator:

    class MotionModel(Enum):
        """An Enum used for determining the motion model to use."""

        ACKERMANN = 1
        DIFF = 2
        OMNI = 3

    class Flip(Enum):
        """An Enum used for determining how a trajectory should be flipped."""

        X = 1
        Y = 2
        BOTH = 3

    def __init__(self, config: dict):
        self.trajectory_generator = TrajectoryGenerator(config)
        self.grid_resolution = config['grid_resolution']
        self.turning_radius = config['turning_radius']
        self.stopping_threshold = config['stopping_threshold']
        self.num_of_headings = config['num_of_headings']
        self.headings = \
            self._get_heading_discretization(config['num_of_headings'])

        self.max_wavefront_expansion = config['max_wavefront_expansion']
        self.verbosity = config['verbosity']

        self.motion_model = self.MotionModel[config['motion_model'].upper()]

        # Build the discrete curvature catalog (see plan: Phase 1).
        # None -> curvature is free (backward-compatible behavior).
        self.num_of_curvatures = config.get('num_of_curvatures', None)
        self.curvatures = self._get_curvature_discretization(self.num_of_curvatures)
        if self.curvatures is not None and len(self.curvatures) > 1:
            spacings = np.diff(self.curvatures)
            self.CURVATURE_THRESHOLD = 0.5 * float(np.min(spacings))
        else:
            self.CURVATURE_THRESHOLD = 0.0

        self.DISTANCE_THRESHOLD = 0.5 * self.grid_resolution
        self.ROTATION_THRESHOLD = 0.5 * (2 * np.pi / self.num_of_headings)

    def _get_curvature_discretization(self, number_of_curvatures):
        """Build the discrete curvature catalog.

        Uniform in kappa, symmetric around 0, max |kappa| = 1 / turning_radius.
        Returns None when curvature is unconstrained.
        """
        if number_of_curvatures is None or number_of_curvatures == 0:
            return None
        if number_of_curvatures == 1:
            return [0.0]
        if number_of_curvatures % 2 == 0:
            warnings.warn(
                f"num_of_curvatures={number_of_curvatures} is even; promoting to "
                f"{number_of_curvatures + 1} so that 0 is included in the catalog"
            )
            number_of_curvatures += 1
        
        kappa_max = 1.0 / self.turning_radius
        
        list_of_curvatures = np.linspace(-kappa_max, kappa_max, number_of_curvatures).tolist()
        
        return list_of_curvatures

    def _get_wave_front_points(self, pos: int) -> np.array:
        """
        Calculate the end points that lie on the wave front.

        Uses the user supplied grid resolution to calculate the
        valid end points that lie on a wave front at a discrete
        interval away from the origin.

        Args:
        pos: int
            The number of discrete intervals of grid resolution
            away from the origin to generate the wave points at

        Returns
        -------
        np.array
            An array of coordinates

        """
        positions = []

        max_point_coord = self.grid_resolution * pos

        for i in range(pos):
            varying_point_coord = self.grid_resolution * i

            # Change the y and keep x at max
            positions.append((max_point_coord, varying_point_coord))

            # Change the x and keep y at max
            positions.append((varying_point_coord, max_point_coord))

        # Append the corner
        positions.append((max_point_coord, max_point_coord))

        return np.array(positions)    

    def _get_heading_discretization(self, number_of_headings: int) -> list:
        """
        Calculate the heading discretization based on the number of headings.

        Does not uniformly generate headings but instead generates a set of
        discrete headings that is better suited for straight line trajectories.

        Args:
        number_of_headings: int
            The number of headings to discretize a 360 degree turn into

        Returns
        -------
        list
            A list of headings in radians

        """
        max_val = int((((number_of_headings + 4) / 4) - 1) / 2)

        outer_edge_x = []
        outer_edge_y = []

        # Generate points that lie on the perimeter of the surface
        # of a square with sides of length max_val
        for i in range(-max_val, max_val + 1):
            outer_edge_x.extend([i, i])
            outer_edge_y.extend([-max_val, max_val])

            if i != max_val and i != -max_val:
                outer_edge_x.extend([-max_val, max_val])
                outer_edge_y.extend([i, i])

        return sorted([np.arctan2(j, i) for i, j in zip(outer_edge_x, outer_edge_y)])
    
    def _point_to_line_distance(self, p1: np.array, p2: np.array, q: np.array) -> float:
        """
        Return the shortest distance from a point to a line segment.

        Args:
        p1: np.array(2,)
            Start point of line segment
        p2: np.array(2,)
            End point of line segment
        q: np.array(2,)
            Point to get distance away from line of

        Returns
        -------
        float
            The shortest distance between q and line segment p1p2

        """
        # Get back the l2-norm without the square root
        l2 = np.inner(p1 - p2, p1 - p2)

        if l2 == 0:
            return np.linalg.norm(p1 - q)

        # Ensure t lies in [0, 1]
        t = max(0, min(1, np.dot(q - p1, p2 - p1) / l2))
        projected_point = p1 + t * (p2 - p1)

        return np.linalg.norm(q - projected_point)
    
    def _is_minimal_trajectory(
        self, trajectory: Trajectory, prior_end_poses: index.Rtree,
        end_kappa: float = None,
    ) -> bool:
        """
        Determine whether a trajectory is a minimal trajectory.
        A trajectory is considered minimal if for every line segment in the trajectory, there does not exist any previously recorded end pose that is BOTH:
         - Spatially close: The perpendicular distance from the prior end pose to the line segment is less than DISTANCE_THRESHOLD (= 0.5 × grid_resolution)
         - Angularly similar: The angle difference between the segment's yaw and the prior end pose's yaw is less than ROTATION_THRESHOLD (= 0.5 × (2π / num_of_headings))
         - Curvature-similar (only when a curvature catalog is active): |kappa_end - prior_kappa_end| < CURVATURE_THRESHOLD.
        I.e., a trajectory is redundant if any of its poses is sufficiently close to a previously recorded terminal pose.

        Prior end poses are saved in an RTree for speedup.

        Args:
        trajectory: Trajectory
            The trajectory to check
        prior_end_poses: RTree
            An RTree holding the current minimal set of trajectories
        end_kappa: float or None
            Target end curvature of the candidate trajectory. When None,
            curvature is not considered for dedup.

        Returns
        -------
        bool
            True if the trajectory is a minimal trajectory otherwise false

        """
        # Iterate over line segments in the trajectory
        for x1, y1, x2, y2, yaw in zip(
            trajectory.path.xs[:-1],
            trajectory.path.ys[:-1],
            trajectory.path.xs[1:],
            trajectory.path.ys[1:],
            trajectory.path.yaws[:-1],
        ):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            # Create a bounding box search region
            # around the line segment
            left_bb = min(x1, x2) - self.DISTANCE_THRESHOLD
            right_bb = max(x1, x2) + self.DISTANCE_THRESHOLD
            top_bb = max(y1, y2) + self.DISTANCE_THRESHOLD
            bottom_bb = min(y1, y2) - self.DISTANCE_THRESHOLD

            # For any previous end points in the search region we
            # check the distance to that point and the angle
            # difference. If they are within threshold then this
            # trajectory can be composed from a previous trajectory
            for prior_end_pose in prior_end_poses.intersection(
                (left_bb, bottom_bb, right_bb, top_bb), objects='raw'
            ):
                if (
                    self._point_to_line_distance(p1, p2, prior_end_pose[:2])
                    < self.DISTANCE_THRESHOLD
                    and angle_difference(yaw, prior_end_pose[2])
                    < self.ROTATION_THRESHOLD
                ):
                    # If curvature dedup is active, also require the prior
                    # trajectory's end curvature to be close to this candidate's.
                    if end_kappa is None or len(prior_end_pose) < 4:
                        return False
                    if abs(end_kappa - prior_end_pose[3]) < self.CURVATURE_THRESHOLD:
                        return False

        return True
    
    def _compute_min_trajectory_length(self) -> float:
        """
        Compute the minimum trajectory length for the given parameters.

        The minimum trajectory length is defined as the length needed
        for the sharpest possible turn to move from 0 degrees to the next
        discrete heading. Since the distance between headings is not uniform
        we take the smallest possible difference.

        Returns
        -------
        float
            The minimal length of a trajectory

        """
        # Compute arc length for a turn that moves from 0 degrees to
        # the minimum heading difference
        heading_diff = [
            abs(self.headings[i + 1] - self.headings[i])
            for i in range(len(self.headings) - 1)
        ]

        return self.turning_radius * min(heading_diff)
    
    def _generate_minimal_spanning_set(self) -> dict:
        """
        Generate the minimal spanning set.

        Iteratves over all possible trajectories and keeps only those that
        are part of the minimal set.

        Returns
        -------
        dict
            A dictionary where the key is the start_angle and the value is
            a list of trajectories that begin at that angle

        """
        # Buckets are keyed by (start_heading, start_kappa). start_kappa is
        # None when the curvature catalog is disabled.
        quadrant1_end_poses = defaultdict(list)

        # Since we only compute for quadrant 1 we only need headings between
        # 0 and 90 degrees
        initial_headings = sorted(
            filter(lambda x: 0 <= x and x <= np.pi / 2, self.headings)
        )
        print(f"Selected {len(initial_headings)} initial headings in the first quadrant.")

        # Iterable over curvature targets. When the catalog is disabled we
        # do a single pass with kappa = None (soft penalty is skipped in the OCP).
        curvatures_iter = self.curvatures if self.curvatures is not None else [None]
        if self.curvatures is not None:
            print(f"Selected {len(curvatures_iter)} curvature values: "
                  f"[{', '.join(f'{k:.3f}' for k in curvatures_iter)}]")

        # Use the minimum trajectory length to find the starting wave front
        min_trajectory_length = self._compute_min_trajectory_length()
        wave_front_start_pos = int(
            np.round(min_trajectory_length / self.grid_resolution)
        )
        print(f"Minimum trajectory length: {min_trajectory_length:.2f}, starting wave front position: {wave_front_start_pos}\n")

        buckets = [(h, k) for h in initial_headings for k in curvatures_iter]

        for bucket_idx, (start_heading, start_kappa) in enumerate(buckets):
            iterations_without_trajectory = 0

            kappa_str = 'free' if start_kappa is None else f'{start_kappa:.3f}'
            print(f'Generating set {bucket_idx + 1} of {len(buckets)}: '
                  f'initial heading = {start_heading:.2f} rad, '
                  f'initial curvature = {kappa_str} rad/m')

            prior_end_poses = index.Index()

            wave_front_cur_pos = wave_front_start_pos

            # To get target headings: sort headings radially and remove those
            # that are more than 90 degrees away
            target_headings = sorted(
                self.headings, key=lambda x: (abs(x - start_heading), -x)
            )
            target_headings = list(
                filter(lambda x: abs(start_heading - x) <= np.pi / 2, target_headings)
            )

            iter_counter = 0
            while iterations_without_trajectory < self.stopping_threshold and iter_counter < self.max_wavefront_expansion:
                iter_counter += 1
                iterations_without_trajectory += 1

                tested_trajs_counter = 0
                newtrajs_counter = 0
                redundant_counter = 0
                infeasible_counter = 0

                # Generate x,y coordinates for current wave front
                positions = self._get_wave_front_points(wave_front_cur_pos)

                # for each target position in the first quadrant
                for target_point in positions:

                    # for each target heading that is within 90 degrees of the start heading
                    for target_heading in target_headings:
                        for end_kappa in curvatures_iter:
                            tested_trajs_counter += 1
                            if self.verbosity >= 2:
                                end_kappa_str = 'free' if end_kappa is None else f'{end_kappa:.2f}'
                                print(f"  > [0.00, 0.00, {start_heading:.2f}, "
                                      f"{kappa_str}] -> "
                                      f"[{target_point[0]:.2f}, {target_point[1]:.2f}, "
                                      f"{target_heading:.2f}, {end_kappa_str}]", end='')

                            # Generate optimal trajectory from start to target
                            # trajectory = self.trajectory_generator.generate_trajectory(
                            #     target_point,
                            #     start_heading,
                            #     target_heading,
                            #     0.1 * self.grid_resolution,
                            #     start_kappa,
                            #     end_kappa,
                            # )
                            
                            trajectory = self.trajectory_generator._calculate_trajectory(
                                target_point, start_heading, target_heading, start_kappa, end_kappa
                            )

                            if trajectory is not None:
                                # Check if path overlaps something in minimal
                                # spanning set
                                if self._is_minimal_trajectory(trajectory, prior_end_poses, end_kappa):

                                    # Add end pose to minimal set (payload
                                    # includes end_kappa so distinct-curvature
                                    # variants can coexist).
                                    kappa_payload = 0.0 if end_kappa is None else float(end_kappa)
                                    new_end_pose = np.array(
                                        [target_point[0], target_point[1],
                                         target_heading, kappa_payload]
                                    )

                                    quadrant1_end_poses[(start_heading, start_kappa)].append(
                                        (target_point, target_heading, end_kappa)
                                    )

                                    # Create a new bounding box in the RTree
                                    # for this trajectory
                                    left_bb = target_point[0] - self.DISTANCE_THRESHOLD
                                    right_bb = target_point[0] + self.DISTANCE_THRESHOLD
                                    bottom_bb = target_point[1] - self.DISTANCE_THRESHOLD
                                    top_bb = target_point[1] + self.DISTANCE_THRESHOLD

                                    prior_end_poses.insert(
                                        0,
                                        (left_bb, bottom_bb, right_bb, top_bb),
                                        new_end_pose,
                                    )

                                    iterations_without_trajectory = 0
                                    newtrajs_counter += 1
                                    if self.verbosity >= 2:
                                        print(f" -> inserted in position {len(prior_end_poses)}", end='\n')
                                else:
                                    redundant_counter += 1
                                    if self.verbosity >= 2:
                                        print(f" -> not minimal", end='\n')
                            else:
                                infeasible_counter += 1
                                if self.verbosity >= 2:
                                    print(f" -> infeasible", end='\n')

                total_trajectories = sum(len(trajs) for trajs in quadrant1_end_poses.values())
                if self.verbosity >= 1:
                    print(f"> Iteration {iter_counter}: wave front = {wave_front_cur_pos * self.grid_resolution:.2f}, {total_trajectories} trajectories", end='\t')
                    print(f"({tested_trajs_counter} tested: {newtrajs_counter} new, {redundant_counter} redundant, {infeasible_counter} infeasible)")


                # Move wave front out by one step
                wave_front_cur_pos += 1
                # END OF WHILE

            # after the end of while disp info
            if self.verbosity >= 1:
                print(f'Completed set {bucket_idx + 1}: '
                      f'initial heading = {start_heading:.2f} rad, '
                      f'initial curvature = {kappa_str} rad/m.')
                print(f"Terminal iteration {iter_counter}: wave front = {(wave_front_cur_pos-1) * self.grid_resolution:.2f}, {iterations_without_trajectory} iterations without new trajectories.\n")

        total_trajectories = sum(len(trajs) for trajs in quadrant1_end_poses.values())
        print(f'Completed generating minimal set for quadrant 1: {total_trajectories} trajectories from {len(quadrant1_end_poses)} (heading, curvature) buckets.\n')

        # Once we have found the minimal trajectory set for quadrant 1
        # we can leverage symmetry to create the complete minimal set
        return self._create_complete_minimal_spanning_set(quadrant1_end_poses)
    
    def _flip_angle(self, angle: float, flip_type: Flip) -> float:
        """
        Return the the appropriate flip of the angle in self.

        Args:
        angle: float
            The angle to flip
        flip_type: Flip
            Whether to flip acrpss X axis, Y axis, or both

        Returns
        -------
        float
            The angle in self.heading that is the appropriate flip

        """
        angle_idx = self.headings.index(angle)

        if flip_type == self.Flip.X:
            heading_idx = (self.num_of_headings / 2 - 1) - angle_idx - 1
        elif flip_type == self.Flip.Y:
            heading_idx = self.num_of_headings - angle_idx - 2
        elif flip_type == self.Flip.BOTH:
            heading_idx = (
                angle_idx - (self.num_of_headings / 2)
            ) % self.num_of_headings
        else:
            raise Exception(f'Unsupported flip type: {flip_type}')

        return self.headings[int(heading_idx)]
    
    def _create_complete_minimal_spanning_set(
        self, single_quadrant_minimal_set: dict
    ) -> dict:
        """
        Create the full minimal spanning set from a single quadrant set.

        Exploits the symmetry between the quadrants to create the full set.
        This is done by flipping every trajectory in the first quadrant across
        either the X-axis, Y-axis, or both axes.

        Curvature signs follow the flip: flipping across X (or Y) inverts
        the sign of both boundary curvatures; flipping across both leaves
        them unchanged.

        Args:
        single_quadrant_minimal_set: dict
            The minimal set for quadrant 1, keyed by (start_angle, start_kappa)
            with values being lists of (end_point, end_angle, end_kappa) tuples.

        Returns
        -------
        dict
            The complete minimal spanning set containing the trajectories
            in all quadrants

        """
        all_trajectories = defaultdict(list)

        total_trajectories = sum(len(trajs) for trajs in single_quadrant_minimal_set.values())
        print(f'Creating complete spanning set from {total_trajectories} trajectories in quadrant 1...')

        for (start_angle, start_kappa), entries in single_quadrant_minimal_set.items():

            for end_point, end_angle, end_kappa in entries:

                x, y = end_point

                # Flipped curvature targets. None -> None (no flip needed).
                flipped_x_start_kappa = None if start_kappa is None else -start_kappa
                flipped_y_start_kappa = flipped_x_start_kappa
                flipped_xy_start_kappa = start_kappa
                flipped_x_end_kappa = None if end_kappa is None else -end_kappa
                flipped_y_end_kappa = flipped_x_end_kappa
                flipped_xy_end_kappa = end_kappa

                # Prevent double adding trajectories that lie on axes
                # (i.e. start and end angle are either both 0 or both pi/2)
                if start_angle == 0 and end_angle == 0:
                    unflipped_start_angle = 0.0
                    flipped_x_start_angle = np.pi

                    unflipped_end_angle = 0.0
                    flipped_x_end_angle = np.pi

                    unflipped_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([x, y]),
                            unflipped_start_angle,
                            unflipped_end_angle,
                            start_kappa, end_kappa,
                        )
                    )
                    flipped_x_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([-x, -y]),
                            flipped_x_start_angle,
                            flipped_x_end_angle,
                            flipped_x_start_kappa, flipped_x_end_kappa,
                        )
                    )

                    if unflipped_trajectory is not None:
                        all_trajectories[
                            unflipped_trajectory.parameters.start_angle
                        ].append(unflipped_trajectory)

                    if flipped_x_trajectory is not None:
                        all_trajectories[
                            flipped_x_trajectory.parameters.start_angle
                        ].append(flipped_x_trajectory)

                elif abs(start_angle) == np.pi / 2 and abs(end_angle) == np.pi / 2:
                    unflipped_start_angle = np.pi / 2
                    flipped_y_start_angle = -np.pi / 2

                    unflipped_end_angle = np.pi / 2
                    flipped_y_end_angle = -np.pi / 2

                    unflipped_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([-x, y]),
                            unflipped_start_angle,
                            unflipped_end_angle,
                            start_kappa, end_kappa,
                        )
                    )

                    flipped_y_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([x, -y]),
                            flipped_y_start_angle,
                            flipped_y_end_angle,
                            flipped_y_start_kappa, flipped_y_end_kappa,
                        )
                    )

                    if unflipped_trajectory is not None:
                        all_trajectories[
                            unflipped_trajectory.parameters.start_angle
                        ].append(unflipped_trajectory)

                    if flipped_y_trajectory is not None:
                        all_trajectories[
                        flipped_y_trajectory.parameters.start_angle
                        ].append(flipped_y_trajectory)

                else:
                    unflipped_start_angle = start_angle
                    flipped_x_start_angle = self._flip_angle(start_angle, self.Flip.X)
                    flipped_y_start_angle = self._flip_angle(start_angle, self.Flip.Y)
                    flipped_xy_start_angle = self._flip_angle(
                        start_angle, self.Flip.BOTH
                    )

                    unflipped_end_angle = end_angle
                    flipped_x_end_angle = self._flip_angle(end_angle, self.Flip.X)
                    flipped_y_end_angle = self._flip_angle(end_angle, self.Flip.Y)
                    flipped_xy_end_angle = self._flip_angle(end_angle, self.Flip.BOTH)

                    unflipped_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([x, y]),
                            unflipped_start_angle,
                            unflipped_end_angle,
                            start_kappa, end_kappa,
                        )
                    )
                    flipped_x_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([-x, y]),
                            flipped_x_start_angle,
                            flipped_x_end_angle,
                            flipped_x_start_kappa, flipped_x_end_kappa,
                        )
                    )
                    flipped_y_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([x, -y]),
                            flipped_y_start_angle,
                            flipped_y_end_angle,
                            flipped_y_start_kappa, flipped_y_end_kappa,
                        )
                    )
                    flipped_xy_trajectory = (
                        self.trajectory_generator._calculate_trajectory(
                            np.array([-x, -y]),
                            flipped_xy_start_angle,
                            flipped_xy_end_angle,
                            flipped_xy_start_kappa, flipped_xy_end_kappa,
                        )
                    )

                    if unflipped_trajectory is not None:
                        all_trajectories[
                            unflipped_trajectory.parameters.start_angle
                        ].append(unflipped_trajectory)
                    if flipped_x_trajectory is not None:
                        all_trajectories[
                            flipped_x_trajectory.parameters.start_angle
                        ].append(flipped_x_trajectory)
                    if flipped_y_trajectory is not None:
                        all_trajectories[
                            flipped_y_trajectory.parameters.start_angle
                        ].append(flipped_y_trajectory)
                    if flipped_xy_trajectory is not None:
                        all_trajectories[
                            flipped_xy_trajectory.parameters.start_angle
                        ].append(flipped_xy_trajectory)

        return all_trajectories
    
    def _handle_motion_model(self, spanning_set: dict) -> dict:
        """
        Add the appropriate motions for the user supplied motion model.

        Ackerman: No additional trajectories

        Diff: In place turns to the right and left

        Omni: Diff + Sliding movements to right and left

        Args:
        spanning set: dict
            The minimal spanning set

        Returns
        -------
        dict
            The minimal spanning set with additional trajectories based
            on the motion model

        """
        if self.motion_model == self.MotionModel.ACKERMANN:
            return spanning_set

        if self.motion_model == self.MotionModel.DIFF:
            diff_spanning_set = self._add_in_place_turns(spanning_set)
            return diff_spanning_set

        elif self.motion_model == self.MotionModel.OMNI:
            omni_spanning_set = self._add_in_place_turns(spanning_set)
            omni_spanning_set = self._add_horizontal_motions(omni_spanning_set)
            return omni_spanning_set

        else:
            print('No handling implemented for Motion Model: ' +
                  f'{self.motion_model}')
            raise NotImplementedError
        
    def _add_in_place_turns(self, spanning_set: dict) -> dict:
        """
        Add in place turns to the spanning set.

        In place turns are trajectories with only a rotational component and
        only shift a single angular heading step

        Args:
        spanning_set: dict
            The minimal spanning set

        Returns
        -------
        dict
            The minimal spanning set containing additional in place turns
            for each start angle

        """
        all_angles = sorted(spanning_set.keys())

        for idx, start_angle in enumerate(all_angles):
            prev_angle_idx = idx - 1 if idx - 1 >= 0 else len(all_angles) - 1
            next_angle_idx = idx + 1 if idx + 1 < len(all_angles) else 0

            prev_angle = all_angles[prev_angle_idx]
            next_angle = all_angles[next_angle_idx]

            left_trajectory_path, left_trajectory_length = self.trajectory_generator._create_path(0.0, 0.0, next_angle, start_angle)
            # In-place turn has zero displacement; the OCP-reported k is
            # meaningless (division by ~eps). Replace with zeros.
            left_trajectory_path = Path(
                left_trajectory_path.xs, left_trajectory_path.ys,
                left_trajectory_path.yaws, np.zeros_like(left_trajectory_path.xs),
            )
            left_trajectory_parameters = TrajectoryParameters(
                turning_radius=0.0, end_point=np.array([0, 0]),
                start_angle=start_angle, end_angle=next_angle, left_turn=True,
                total_length=0.0, straight_length=0.0,
                target_start_kappa=0.0, target_end_kappa=0.0,
            )
            left_trajectory = Trajectory(left_trajectory_path, left_trajectory_parameters)


            right_trajectory_path, right_trajectory_length = self.trajectory_generator._create_path(0.0, 0.0, next_angle, start_angle)
            right_trajectory_path = Path(
                right_trajectory_path.xs, right_trajectory_path.ys,
                right_trajectory_path.yaws, np.zeros_like(right_trajectory_path.xs),
            )
            right_trajectory_parameters = TrajectoryParameters(
                turning_radius=0.0, end_point=np.array([0, 0]),
                start_angle=start_angle, end_angle=prev_angle, left_turn=False,
                total_length=0.0, straight_length=0.0,
                target_start_kappa=0.0, target_end_kappa=0.0,
            )
            right_trajectory = Trajectory(right_trajectory_path, right_trajectory_parameters)

            spanning_set[start_angle].append(left_trajectory)
            spanning_set[start_angle].append(right_trajectory)

        return spanning_set
    
    def _add_horizontal_motions(self, spanning_set: dict) -> dict:
        """
        Add horizontal sliding motions to the spanning set.

        The horizontal sliding motions are simply straight line trajectories
        at 90 degrees to every start angle in the spanning set. The yaw of these
        trajectories is the same as the start angle for which it is generated.

        Args:
        spanning_set: dict
            The minimal spanning set

        Returns
        -------
        dict
            The minimal spanning set containing additional sliding motions
            for each start angle

        """
        # Calculate the offset in the headings list that represents an
        # angle change of 90 degrees
        idx_offset = int(self.num_of_headings / 4)

        for idx, angle in enumerate(self.headings):

            try:
                # Copy the straight line trajectory for the start angle that
                # is 90 degrees to the left
                left_angle_idx = int((idx + idx_offset) % self.num_of_headings)
                left_angle = self.headings[left_angle_idx]
                left_trajectories = spanning_set[left_angle]
                left_straight_trajectory = next(
                    t for t in left_trajectories if t.parameters.end_angle == left_angle
                )
                params_l = left_straight_trajectory.parameters
                x2, y2 = params_l.end_point
                left_trajectory_path, left_trajectory_length = self.trajectory_generator._create_path(x2, y2, left_angle, left_angle)
                # Horizontal slide is a straight segment; force zero curvature.
                left_trajectory_path = Path(
                    left_trajectory_path.xs, left_trajectory_path.ys,
                    left_trajectory_path.yaws, np.zeros_like(left_trajectory_path.xs),
                )
                left_trajectory_parameters = TrajectoryParameters(
                    turning_radius=self.turning_radius, end_point=params_l.end_point,
                    start_angle=left_angle, end_angle=left_angle, left_turn=False,
                    total_length=left_trajectory_length, straight_length=left_trajectory_length,
                    target_start_kappa=0.0, target_end_kappa=0.0,
                )
                left_trajectory = Trajectory(left_trajectory_path, left_trajectory_parameters)
                spanning_set[angle].append(left_trajectory)
            except:
                left_trajectory = None

            try:
                # Copy the straight line trajectory for the start angle that
                # is 90 degrees to the right
                right_angle_idx = int((idx - idx_offset) % self.num_of_headings)
                right_angle = self.headings[right_angle_idx]
                right_trajectories = spanning_set[right_angle]
                right_straight_trajectory = next(
                    t for t in right_trajectories if t.parameters.end_angle == right_angle
                )
                params_r = right_straight_trajectory.parameters
                x2, y2 = params_r.end_point
                right_trajectory_path, right_trajectory_length = self.trajectory_generator._create_path(x2, y2, right_angle, right_angle)
                right_trajectory_path = Path(
                    right_trajectory_path.xs, right_trajectory_path.ys,
                    right_trajectory_path.yaws, np.zeros_like(right_trajectory_path.xs),
                )
                right_trajectory_parameters = TrajectoryParameters(
                    turning_radius=self.turning_radius, end_point=params_r.end_point,
                    start_angle=right_angle, end_angle=right_angle, left_turn=False,
                    total_length=right_trajectory_length, straight_length=right_trajectory_length,
                    target_start_kappa=0.0, target_end_kappa=0.0,
                )
                right_trajectory = Trajectory(right_trajectory_path, right_trajectory_parameters)
                spanning_set[angle].append(right_trajectory)
            except:
                right_trajectory = None

        return spanning_set
    
    def run(self):
        """
        Run the lattice generator.

        Returns
        -------
        dict
            The minimal spanning set including additional motions for the
            specified motion model

        """
        complete_spanning_set = self._generate_minimal_spanning_set()
        total_trajectories = sum(len(trajs) for trajs in complete_spanning_set.values())
        print(f'Completed generating complete spanning set: {total_trajectories} trajectories.\n')

        print(f'Augmenting spanning set for motion model: {self.motion_model} ...')
        extended_spanning_set = self._handle_motion_model(complete_spanning_set)
        total_trajectories = sum(len(trajs) for trajs in extended_spanning_set.values())
        print(f'Completed generating augmented spanning set: {total_trajectories} trajectories.\n')
        
        return extended_spanning_set 