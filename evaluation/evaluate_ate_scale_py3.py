# Modified by Yuan Lin
# March, 25th, 2025
# Automatically compute the optimal scale factor for monocular VO/SLAM.


import sys
import numpy as np
import argparse
import associate_py3 as associate

def align(model, data):
    """Align two trajectories using the method of Horn (closed-form)."""
    np.set_printoptions(precision=3, suppress=True)
    model_zerocentered = model - model.mean(axis=1, keepdims=True)
    data_zerocentered = data - data.mean(axis=1, keepdims=True)

    W = np.zeros((3, 3))
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:, column], data_zerocentered[:, column])
    U, d, Vh = np.linalg.svd(W.T)
    S = np.identity(3)
    if np.linalg.det(U) * np.linalg.det(Vh) < 0:
        S[2, 2] = -1
    rot = U @ S @ Vh

    rotmodel = rot @ model_zerocentered
    dots = sum(np.dot(data_zerocentered[:, i], rotmodel[:, i]) for i in range(data_zerocentered.shape[1]))
    norms = sum(np.linalg.norm(model_zerocentered[:, i])**2 for i in range(model_zerocentered.shape[1]))

    s = float(dots / norms)

    transGT = data.mean(axis=1, keepdims=True) - s * rot @ model.mean(axis=1, keepdims=True)
    trans = data.mean(axis=1, keepdims=True) - rot @ model.mean(axis=1, keepdims=True)

    model_alignedGT = s * rot @ model + transGT
    model_aligned = rot @ model + trans

    alignment_errorGT = model_alignedGT - data
    alignment_error = model_aligned - data

    trans_errorGT = np.sqrt(np.sum(np.multiply(alignment_errorGT, alignment_errorGT), axis=0))
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error, alignment_error), axis=0))

    return rot, transGT, trans_errorGT, trans, trans_error, s

def plot_traj(ax, stamps, traj, style, color, label):
    stamps.sort()
    interval = np.median([s - t for s, t in zip(stamps[1:], stamps[:-1])])
    x, y = [], []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i] - last < 2 * interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif x:
            ax.plot(x, y, style, color=color, label=label)
            label = ""
            x, y = [], []
        last = stamps[i]
    if x:
        ax.plot(x, y, style, color=color, label=label)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory.

    File formats (space-separated):
      timestamp tx ty tz qx qy qz qw

    Only the tx, ty, tz columns will be used for trajectory alignment and error computation.
    ''')
    parser.add_argument('first_file', help='Ground truth trajectory file (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='Estimated trajectory file (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', default=0.0, type=float, help='Time offset added to the timestamps of the second file (default: 0.0)')
    parser.add_argument('--scale', default=1.0, type=float, help='Scaling factor for the second trajectory (default: 1.0)')
    parser.add_argument('--max_difference', default=20000000, type=float, help='Max allowed time difference for matching entries (default: 20000000 ns)')
    parser.add_argument('--save', help='Save aligned second trajectory to disk')
    parser.add_argument('--save_associations', help='Save associated first and aligned second trajectory to disk')
    parser.add_argument('--plot', help='Plot the first and aligned second trajectory to an image (format: png/pdf)')
    parser.add_argument('--verbose', action='store_true', help='Print detailed evaluation metrics')
    parser.add_argument('--verbose2', action='store_true', help='Print metrics with and without scale correction')
    args = parser.parse_args()

    first_list = associate.read_file_list(args.first_file, False)
    second_list = associate.read_file_list(args.second_file, False)

    matches = associate.associate(first_list, second_list, args.offset, args.max_difference)
    if len(matches) < 2:
        sys.exit("Couldn't find matching timestamp pairs!")

    first_xyz = np.array([[float(value) for value in first_list[a][0:3]] for a, b in matches]).T
    second_xyz = np.array([[float(value) * args.scale for value in second_list[b][0:3]] for a, b in matches]).T
    sorted_second_list = sorted(second_list.items())
    second_xyz_full = np.array([[float(value) * args.scale for value in entry[1][0:3]] for entry in sorted_second_list]).T

    rot, transGT, trans_errorGT, trans, trans_error, scale = align(second_xyz, first_xyz)

    second_xyz_notscaled_full = rot @ second_xyz_full + trans
    first_stamps = sorted(first_list.keys())
    first_xyz_full = np.array([[float(value) for value in first_list[b][0:3]] for b in first_stamps]).T

    second_stamps = sorted(second_list.keys())
    second_xyz_full = np.array([[float(value) * args.scale for value in second_list[b][0:3]] for b in second_stamps]).T
    second_xyz_full_aligned = scale * rot @ second_xyz_full + trans

    if args.verbose:
        print(f"compared_pose_pairs {len(trans_error)} pairs")
        print(f"absolute_translational_error.rmse {np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)):.6f} m")
        print(f"absolute_translational_error.mean {np.mean(trans_error):.6f} m")
        print(f"absolute_translational_error.median {np.median(trans_error):.6f} m")
        print(f"absolute_translational_error.std {np.std(trans_error):.6f} m")
        print(f"absolute_translational_error.min {np.min(trans_error):.6f} m")
        print(f"absolute_translational_error.max {np.max(trans_error):.6f} m")
        print(f"max idx: {np.argmax(trans_error)}")
    else:
        print(f"{np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)):.6f},{scale:.6f},{np.sqrt(np.dot(trans_errorGT, trans_errorGT) / len(trans_errorGT)):.6f}")

    if args.verbose2:
        print(f"compared_pose_pairs {len(trans_error)} pairs")
        print(f"absolute_translational_error.rmse {np.sqrt(np.dot(trans_error, trans_error) / len(trans_error)):.6f} m")
        print(f"absolute_translational_errorGT.rmse {np.sqrt(np.dot(trans_errorGT, trans_errorGT) / len(trans_errorGT)):.6f} m")

    if args.save_associations:
        with open(args.save_associations, "w") as f:
            f.write("\n".join([
                f"{a} {x1} {y1} {z1} {b} {x2} {y2} {z2}"
                for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(matches, first_xyz.T, (scale * rot @ second_xyz + trans).T)
            ]))

    if args.save:
        with open(args.save, "w") as f:
            f.write("\n".join([
                f"{stamp} {' '.join(f'{d:.6f}' for d in line)}"
                for stamp, line in zip(second_stamps, second_xyz_notscaled_full.T)
            ]))

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax, first_stamps, first_xyz_full.T, '-', "black", "ground truth")
        plot_traj(ax, second_stamps, second_xyz_full_aligned.T, '-', "blue", "estimated")
        label = "difference"
        for (a, b), (x1, y1, z1), (x2, y2, z2) in zip(matches, first_xyz.T, (scale * rot @ second_xyz).T):
            ax.plot([x1, x2], [y1, y2], '-', color="red", label=label)
            label = ""
        ax.legend()
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.axis('equal')
        plt.savefig(args.plot, format="pdf")


