from argparse import ArgumentParser

if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("--dataset", default="dataset")
    parser.add_argument("--video_id", default="monday_roll15")
    parser.add_argument("--rod_mesh_file", default="pcd/yale/struct_with_socks_new.ply")
    parser.add_argument("--top_endcap_mesh_file", default="pcd/yale/end_cap_top_new.obj")
    parser.add_argument("--bottom_endcap_mesh_file", default="pcd/yale/end_cap_bottom_new.obj")
    parser.add_argument("--start_frame", default=0, type=int)
    parser.add_argument("--max_correspondence_distances", default=[0.3, 0.15, 0.1, 0.06, 0.03], type=float, nargs="+")
    parser.add_argument("--add_dummy_points", action="store_true")
    parser.add_argument("--num_dummy_points", type=int, default=50)
    parser.add_argument("--dummy_weights", type=float, default=0.1)
    parser.add_argument("--filter_observed_pts", action="store_true")
    parser.add_argument("--add_constrained_optimization", action="store_true")
    parser.add_argument("--add_physical_constraints", action="store_true")
    parser.add_argument("--add_ground_constraints", action="store_true")
    parser.add_argument("-v", "--visualize", action="store_true")
    args = parser.parse_args()

    print(video_id)
    print(v)