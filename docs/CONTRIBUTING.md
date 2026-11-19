# Development

Create a virtual environment and install `requirements.txt`, then run:

```bash
python -m unittest discover -s tests -v
python scripts/validate_experiments.py
python scripts/validate_localization_run.py results/localization/gazebo_outage
```

Keep the estimator independent of ROS and simulator truth. Changes to the motion or RSSI model should include finite-difference Jacobian checks and uncertainty checks. Do not hide failed trials or align estimated trajectories to truth after estimation.

Regenerate media with `python scripts/render_media.py`. Raw logs and timestamped Gazebo output are local artifacts; commit only compact fixtures needed for reproducibility. When changing sensor/robot configuration, regenerate `gazebo/localization_lab.world` and rerun the demo in a compatible ROS environment.
