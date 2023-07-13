import numpy as np



def camera_positions(n_camera_deck, n_camera_down, n_camera_lat, lidar_angle_deg, lidar_angle_deg_deck, lidar_step_deg, 
                     cam_dist_deck, cam_dist_down, cam_dist_lat_y, cam_dist_lat_z, 
                     min_cam, deck_position, height, length, width, n_pannels, sigma):
    '''
    Method for create a list of dicts with the following keys for each LiDAR position: ['fov_deg', 'center', 'eye', 'up', 'width_px', 'height_px'].
    Camera positions are distributed along the bridge uniformly on the deck, under the ridge and in lateral positions.

    :param n_camera_deck: max and min number of lidar position on the deck.
    :param n_camera_down: max and min number of lidar position under the bridge.
    :param n_camera_lat: max and min number of lidar position at each side of the bridge.
    :param lidar_angle_deg: opening angle in degrees of the camera.
    :param lidar_angle_deg_deck: opening angle in degrees of the camera on the deck.
    :param lidar_step_deg: step in degrees.
    :param cam_dist_deck: max and min distance in Z between deck and camera.
    :param cam_dist_down: max and min distance between in Z between the botton of the bridge and camera in down positions.
    :param cam_dist_lat_y: max and min distance between in Y between the lateral face and the lateral camera position.
    :param cam_dist_lat_z: max and min distance between in Z between the lateral face and the lateral camera position.
    :param min_cam: minimun number of camera positions.
    :param deck_position: deck position in Z.
    :param height: height of the panel of the bridge.
    :param length: length of the panel of the bridge.
    :param width: width of the panel of the bridge.
    :param n_pannels: number of pannels of the bridge.
    :param sigma: sigma for normal noise generation in the camera positions.
    '''

    # Positions of the LIDAR. their are calcualted consider the point centre in (0,0,0).
    # Number of cams
    n_cameras_deck = np.random.randint(n_camera_deck[0], n_camera_deck[1]+1) if deck_position != height else 0
    n_cameras_down = np.random.randint(n_camera_down[0], n_camera_down[1]+1)
    n_cameras_lat_pos = np.random.randint(n_camera_lat[0], n_camera_lat[1]+1)
    n_cameras_lat_neg = np.random.randint(n_camera_lat[0], n_camera_lat[1]+1)

    while n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < min_cam:
        if n_camera_lat[1] > n_cameras_lat_pos:
            n_cameras_lat_pos = np.random.randint(n_cameras_lat_pos, n_camera_lat[1]+1) 
        if not n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < min_cam: break
        if n_camera_lat[1] > n_cameras_lat_neg:
            n_cameras_lat_neg = np.random.randint(n_cameras_lat_neg, n_camera_lat[1]+1) 
        if not n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < min_cam: break
        if not n_camera_down[1] > n_cameras_down:
            n_cameras_down = np.random.randint(n_cameras_down, n_camera_down[1]+1) 
        if not n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < min_cam: break
        if n_camera_deck[1] > n_cameras_deck:
            n_cameras_deck = np.random.randint(n_cameras_deck, n_camera_deck[1]+1) if deck_position != height else 0

    # DECK. 4 cameras_deck por position to do 360.
    angle = lidar_angle_deg_deck/4
    px = int(angle*lidar_step_deg)
    # Initialising
    cameras_deck = np.zeros(n_cameras_deck*4, dtype='object')
    for j in range(n_cameras_deck):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_pannels/2 + (j+1)/(n_cameras_deck+1)* length*n_pannels + np.random.normal(scale=sigma[0])
        position[1] = 0 + np.random.normal(scale=sigma[1])
        position[2] = -height/2+deck_position + np.random.random() * (cam_dist_deck[1] - cam_dist_deck[0]) + cam_dist_deck[0]

        # pointing +X and -X.
        cameras_deck[j*4] = {'fov_deg': angle, 'center':position + [1,0,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}
        cameras_deck[j*4+1] = {'fov_deg': angle, 'center':position + [-1,0,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

        # pointing +Y and -Y
        cameras_deck[j*4+2] = {'fov_deg': angle, 'center':position + [0,1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}
        cameras_deck[j*4+3] = {'fov_deg': angle, 'center':position + [0,-1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

    angle = lidar_angle_deg
    px = int(angle*lidar_step_deg)

    #DOWN
    # Initialising
    cameras_down = np.zeros(n_cameras_down, dtype='object')
    for j in range(n_cameras_down):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_pannels/2 + (j+1)/(n_cameras_down+1)* length*n_pannels + np.random.normal(scale=sigma[0])
        position[1] = 0 + np.random.normal(scale=sigma[1])
        position[2] = -height/2 + np.random.random() * (cam_dist_down[1] - cam_dist_down[0]) + cam_dist_down[0]

        # camera pointing +Z
        cameras_down[j] = {'fov_deg': angle, 'center':position + [0,0,1], 'eye':position, 'up':[1,0,0], 'width_px':px, 'height_px':px}

    #LATERAL +Y
    # Initialising
    cameras_lateral_pos = np.zeros(n_cameras_lat_pos, dtype='object')
    for j in range(n_cameras_lat_pos):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_pannels/2 + (j+1)/(n_cameras_lat_pos+1)* length*n_pannels + np.random.normal(scale=sigma[0])
        position[1] = width/2 + np.random.random() * (cam_dist_lat_y[1] - cam_dist_lat_y[0]) + cam_dist_lat_y[0]
        position[2] = np.random.random() * (cam_dist_lat_z[1] - cam_dist_lat_z[0]) + cam_dist_lat_z[0]

        # camera pointing -Y
        cameras_lateral_pos[j] = {'fov_deg': angle, 'center':position + [0,-1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

    #LATERAL -Y
    # Initialising
    cameras_lateral_neg = np.zeros(n_cameras_lat_neg, dtype='object')
    for j in range(n_cameras_lat_neg):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_pannels/2 + (j+1)/(n_cameras_lat_neg+1)* length*n_pannels + np.random.normal(scale=sigma[0])
        position[1] = -width/2 - np.random.random() * (cam_dist_lat_y[1] - cam_dist_lat_y[0]) + cam_dist_lat_y[0]
        position[2] = np.random.random() * (cam_dist_lat_z[1] - cam_dist_lat_z[0]) + cam_dist_lat_z[0]

        # camera pointing +Y
        cameras_lateral_neg[j] = {'fov_deg': angle, 'center':position + [0,1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

    cameras = np.concatenate((cameras_deck, cameras_down, cameras_lateral_pos, cameras_lateral_neg))

    return cameras