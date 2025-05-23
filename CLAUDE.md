- Calculate `T_LiDAR_Camera2i` between the Livox LiDAR and the ZED 2i camera using ChArUco board pose estimation and ICP alignment. Process involves:
  - Locating latest session folder in `./records/`
  - Detecting ChArUco pose from ZED 2i RGB images using OpenCV
  - Extracting ChArUco point clouds from both LiDAR and camera
  - Applying ICP for point cloud alignment
  - Computing final transformation: `T_LiDAR_Camera2i = T_Charuco_LiDAR @ np.linalg.inv(T_Camera2i_Charuco)`
  - Saving the 4x4 transformation matrix to `./calibration_statique/results/tf_statique.npy`