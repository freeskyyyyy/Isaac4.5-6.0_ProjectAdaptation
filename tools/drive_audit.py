"""Read course USD geometry and evaluate the installed NVIDIA drive solver.

No articulation commands are applied. Invoke only inside the owning Kit process.
"""
import json


def configure_drive(stage):
    """Match SDK moment constraints to this symmetric three-wheel chassis.

    The SDK constrains summed wheel velocities and their moments. A rigid-body
    twist maps through the wheel rolling Jacobian; its diagonal products give
    the required gains (the yaw moment has the opposite sign).
    """
    import numpy as np
    from isaacsim.robot.experimental.wheeled_robots.robots import HolonomicRobotUsdSetup
    from isaacsim.robot.experimental.wheeled_robots.controllers import HolonomicController
    root = '/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint'
    setup = HolonomicRobotUsdSetup(robot_prim_path=root, com_prim_path=root+'/base_link')
    controller = HolonomicController(wheel_radius=setup.wheel_radius,
        wheel_positions=setup.wheel_positions, wheel_orientations=setup.wheel_orientations,
        mecanum_angles=setup.mecanum_angles, wheel_axis=setup.wheel_axis, up_axis=setup.up_axis)
    directions = controller.base_dir[:2].T / np.array(setup.wheel_radius)[:, None]
    moments = setup.wheel_positions[:, 0]*directions[:, 1] - setup.wheel_positions[:, 1]*directions[:, 0]
    jacobian = np.column_stack((directions, moments)) / np.array(setup.wheel_radius)[:, None]
    matrix = np.vstack((controller.base_dir[:2], np.cross(
        controller.base_dir, controller.wheel_dists_inv, axis=0)[2]))
    product = matrix @ jacobian
    diagonal = np.diag(product)
    if (set(setup.wheel_dof_names) != {'left_wheel_joint', 'right_wheel_joint', 'front_wheel_joint'}
            or not np.allclose(setup.mecanum_angles, -90.)
            or len(setup.wheel_radius) != 3 or abs(diagonal[0]-diagonal[1]) > 0.002
            or np.max(np.abs(product-np.diag(diagonal))) > 0.001
            or min(diagonal[:2]) <= 0 or diagonal[2] >= 0):
        raise RuntimeError('Course drive is not the expected symmetric chassis; inspect full Jacobian')
    prim = stage.GetPrimAtPath('/CourseGraphs/g04_HolonomicController/holonomic_controller')
    values = {'linearGain': float(np.mean(diagonal[:2])), 'angularGain': float(diagonal[2])}
    previous = {}
    for name, value in values.items():
        attr = prim.GetAttribute('inputs:'+name)
        previous[name] = attr.Get()
        attr.Set(value)
    return {'previous': previous, 'configured': values, 'constraint_product': product.tolist(),
            'rolling_jacobian': jacobian.tolist(), 'wheel_names': list(setup.wheel_dof_names)}


def audit_drive(path):
    import numpy as np
    from isaacsim.robot.experimental.wheeled_robots.robots import HolonomicRobotUsdSetup
    from isaacsim.robot.experimental.wheeled_robots.controllers import HolonomicController

    root = '/World/bobac/bobac3_serverbot/bobac3_serverbot/base_footprint'
    setup = HolonomicRobotUsdSetup(robot_prim_path=root, com_prim_path=root+'/base_link')
    report = {'scope': 'installed SDK solver on actual USD wheel geometry; no motion commands',
              'wheel_names': list(setup.wheel_dof_names),
              'wheel_radius': list(setup.wheel_radius),
              'wheel_positions': setup.wheel_positions.tolist(),
              'wheel_orientations': setup.wheel_orientations.tolist(),
              'mecanum_angles': list(setup.mecanum_angles), 'cases': []}
    for maximum in (100000.0, 2.0):
        controller = HolonomicController(
            wheel_radius=setup.wheel_radius, wheel_positions=setup.wheel_positions,
            wheel_orientations=setup.wheel_orientations, mecanum_angles=setup.mecanum_angles,
            wheel_axis=setup.wheel_axis, up_axis=setup.up_axis,
            max_linear_speed=maximum, max_angular_speed=0.25, max_wheel_speed=30.0,
            linear_gain=-1.25, angular_gain=1.0)
        matrix = np.concatenate((controller.base_dir, np.cross(
            controller.base_dir, controller.wheel_dists_inv, axis=0)), axis=0)
        directions = controller.base_dir[:2].T / np.array(setup.wheel_radius)[:, None]
        moments = setup.wheel_positions[:, 0]*directions[:, 1] - setup.wheel_positions[:, 1]*directions[:, 0]
        jacobian = np.column_stack((directions, moments))
        jacobian /= np.array(setup.wheel_radius)[:, None]
        for command in ([0., 0., 0.08], [0.1, 0., 0.], [0., 0., 0.]):
            wheels = controller.forward(np.array(command))
            prediction = (matrix @ wheels)[[0, 1, 5]]
            expected = np.array(command) * [-1.25, -1.25, 1.0]
            report['cases'].append({'max_linear_speed': maximum, 'input': command,
                'wheel_velocities': wheels.tolist(), 'predicted_command': prediction.tolist(),
                'expected_command': expected.tolist(),
                'predicted_rigid_body_twist': (np.linalg.pinv(jacobian) @ wheels).tolist(),
                'max_constraint_error': float(np.max(np.abs(prediction-expected)))})
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2)+'\n')
    print('Drive solver diagnostic written: '+str(path), flush=True)
