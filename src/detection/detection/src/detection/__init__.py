"""Detection subsystem: camera geometry, a simulation mock detector, and target localization.

The package is split so that entering the real vehicle only replaces the
detector itself:

``camera``/``localization``
    Pure geometry shared by the simulated and the real pipeline.
``mock``/``error_model``/``occlusion``
    Simulation-only: synthesize the detections a real detector would produce
    for the known ground-truth targets, with a calibrated error model.
``ros``
    Thin ROS 2 adapters for the modules above.
"""
