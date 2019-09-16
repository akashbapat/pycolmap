import sys
sys.path.append("..")

import numpy as np
import os.path

from pycolmap import Quaternion, SceneManager


#-------------------------------------------------------------------------------

def main(args):
    scene_manager = SceneManager(args.input_folder)
    scene_manager.load()
    assert(len(scene_manager.cameras) == 4)
 
    zed_l  = "frame_chess_zed_left_"
    zed_r  = "frame_chess_zed_right_"    
    zed_count = 0

    for image in scene_manager.images.itervalues():
        if zed_l in image.name:
            zed_count = zed_count + 1
    
    zed_gt_scale_in_meters = 0.120   
    zed_diff_vec = np.zeros( (zed_count,3))

    for image in scene_manager.images.itervalues():
        if zed_l in image.name:
           row_id = int(image.name.split(zed_l,1)[1][:-4])
           if  row_id < zed_count:
               zed_diff_vec[row_id] = zed_diff_vec[row_id] + image.C()

        if zed_r in image.name:
           row_id = int(image.name.split(zed_r,1)[1][:-4])
           if  row_id < zed_count:
               zed_diff_vec[row_id] = zed_diff_vec[row_id] - image.C()

    print(np.linalg.norm(zed_diff_vec, axis=1))
    scale = zed_gt_scale_in_meters/np.median(np.linalg.norm(zed_diff_vec, axis=1))


    print("Scaling the model by {}".format(scale))
    scene_manager.points3D[:] = scene_manager.points3D[:] * scale
    for image in scene_manager.images.itervalues():
        image.tvec = image.tvec * scale

    # Print the relative transform from camera 1 to camera 2. [R|t]
    ricoh_image_l_name = "frame_chess_left_{}.png".format(str(0).zfill(4))
    ricoh_image_r_name = "frame_chess_right_{}.png".format(str(0).zfill(4))
 
    #ricoh_image_l_name = "cubemap_left_cam_001_{}.png".format(str(0).zfill(4))
    #ricoh_image_r_name = "cubemap_right_cam_001_{}.png".format(str(0).zfill(4))


    for image in scene_manager.images.itervalues():
        if ricoh_image_l_name == image.name:
            ricoh_image_l = image

        if ricoh_image_r_name == image.name:
            ricoh_image_r = image
 
    print("Relative transformation is:")
    rel_pose = np.matmul(ricoh_image_r.Pose(), ricoh_image_l.InvPose())
    np.set_printoptions(suppress=True)
    print(rel_pose)
    print("\nRelative transformation inverse is:")
    print(np.linalg.inv(rel_pose))
 
    scene_manager.save(args.output_folder)
    
    # Save rel_pose in the ini settings format.
    pose_save_str = "\n[RicohRelativePose]"
    pose_save_str = pose_save_str + "\nextrinsics = [ " + np.array2string(rel_pose, precision=10, separator=';',suppress_small=False).replace("]", "").replace("[", "").replace("\n", "") + " ]"
    pose_save_str = pose_save_str + "\ninput_path =  "+ args.input_folder
    pose_save_str = pose_save_str + "\noutput_path =  "+ args.output_folder
    pose_save_str = pose_save_str + "\n"

    with open(os.path.join(args.output_folder,"extrinsics.ini"), "w") as text_file:
        text_file.write(pose_save_str)


#-------------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Apply a 3x4 transformation matrix to a COLMAP model and "
        "save the result as a new model. Row-major input can be piped in from "
        "a file or entered via the command line.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("input_folder")
    parser.add_argument("output_folder")

    args = parser.parse_args()

    main(args)
