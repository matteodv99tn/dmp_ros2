# Goals and purposes

## Data collection

The idea is to have a file tree that looks as follows:
```
/path/to/demonstrations
    demonstration_001
        raw_data
            data.csv
            data_domain.txt
            transforms.txt
        processed_data
    demonstration_002
    ...
```

### Raw data
Raw data are collected by the ``TrajectoryListener`` ROS2 node. 
It's purpose is to store all the data stream of the optitrack inside a ``data.csv`` file.
The reference frame w.r.t. which such data are defined, is stored in ``data_domain.txt``.
The file ``transforms.txt`` contains the dictionary of all transforms known, defined w.r.t. to the _world_ link.
