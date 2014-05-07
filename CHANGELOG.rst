^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package enu
^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.2 (2014-05-07)
------------------
* Define _xerbla logging function to fix regression from swiftnav.
* Contributors: Mike Purvis

1.2.1 (2014-02-10)
------------------
* Add service dep for enu_from_fix.
* Contributors: Mike Purvis

1.2.0 (2014-02-07)
------------------
* Add a new trivial node which provides just the two conversion services.
* Remove cruft from package.xml
* Contributors: Mike Purvis, Prasenjit Mukherjee

1.1.0 (2013-10-03)
------------------
* Specify minimum version for swiftnav dependency.
* Add several key configuration parameters in support of the outdoor Husky EKF.
* adding the ability to scale the gps covariance
* adding param to lock altitude output to a rosparam set number
* added provision to initialize odom orientation at a mathematically viable quaternion orientation
* Clean up and simplify code API.

1.0.3 (2013-09-07)
------------------
* Remove swiftnav stuff from enu, now that it's a separately released package.

1.0.2 (2013-09-05)
------------------
* Attempt to fix build/install issue with swiftnav headers.
* Move libswiftnav up to package path.

1.0.1 (2013-09-04)
------------------
* Initial release of enu package for Hydro, including to_fix and from_fix
  conversion nodes, and a simple code API.
