import itertools
import math
import numpy as np
from collections import defaultdict

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


###############################################################################
#  A.  Core Geometry & Kabsch Fitting
###############################################################################

def compute_pairwise_distances(points):
    """Return the NxN matrix of pairwise distances."""
    N = len(points)
    dist_mat = np.zeros((N, N), dtype=float)
    for i in range(N):
        for j in range(i+1, N):
            d = np.linalg.norm(points[i] - points[j])
            dist_mat[i, j] = d
            dist_mat[j, i] = d
    return dist_mat

def kabsch_fit_transform(A, B, allow_reflections=True):
    """
    Given two sets of points A, B (both Nx3), compute the best-fit
    rotation (or reflection if allowed) R and translation t such that
        R @ A_i + t ~ B_i
    in a least-squares sense.

    If allow_reflections=False, we force det(R)=+1 by flipping one axis if needed.

    Returns (R, t, mse_error).
    """
    # 1. Subtract centroids
    A_centroid = A.mean(axis=0)
    B_centroid = B.mean(axis=0)
    A_ = A - A_centroid
    B_ = B - B_centroid

    # 2. Covariance H
    H = A_.T @ B_

    # 3. SVD of H
    U, S, Vt = np.linalg.svd(H)
    R = (Vt.T @ U.T)

    # Force orientation if not allowing reflections
    if not allow_reflections:
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = (Vt.T @ U.T)

    t = B_centroid - R @ A_centroid

    # 4. MSE
    A_fit = (R @ A_.T).T + B_centroid
    residuals = A_fit - B
    mse = np.mean(np.sum(residuals**2, axis=1))

    return R, t, mse


###############################################################################
#  B.  Distance-Graph Automorphisms
###############################################################################

def build_distance_signatures(points, round_decimals=5):
    """
    For each point i, create a signature of sorted distances
    to all other points, ignoring self-dist=0.
    """
    dist_mat = compute_pairwise_distances(points)
    N = len(points)
    signatures = []
    for i in range(N):
        row = dist_mat[i, :]
        # Exclude row[i] = 0
        sorted_dists = sorted(np.round(np.delete(row, i), round_decimals))
        signatures.append(tuple(sorted_dists))
    return dist_mat, signatures

def find_all_automorphisms(points, tol=1e-3, max_solutions=2000):
    """
    Enumerate all graph automorphisms (permutations) that preserve
    pairwise distances (within tol). Returns a list of permutations
    pi (list of length N with pi[i] = j).
    """
    points = np.asarray(points)
    N = len(points)
    dist_mat, signatures = build_distance_signatures(points, round_decimals=5)

    signature_map = defaultdict(list)
    for i, sig in enumerate(signatures):
        signature_map[sig].append(i)

    pi = [-1]*N
    used = [False]*N

    class_groups = sorted(signature_map.values(), key=len)
    backtrack_order = []
    for grp in class_groups:
        backtrack_order.extend(grp)

    solutions = []

    def backtrack(idx):
        if len(solutions) >= max_solutions:
            return
        if idx == N:
            solutions.append(pi[:])
            return

        i = backtrack_order[idx]
        sig_i = signatures[i]
        candidates = signature_map[sig_i]
        for j in candidates:
            if not used[j]:
                # Check distance preservation
                can_assign = True
                for prev_idx in range(idx):
                    i2 = backtrack_order[prev_idx]
                    j2 = pi[i2]
                    if abs(dist_mat[i, i2] - dist_mat[j, j2]) > tol:
                        can_assign = False
                        break
                if can_assign:
                    pi[i] = j
                    used[j] = True
                    backtrack(idx+1)
                    used[j] = False
                    pi[i] = -1
                    if len(solutions) >= max_solutions:
                        return

    backtrack(0)
    return solutions

def quick_distance_symmetry_check(points, tol=1e-3):
    """
    Quick check if there's any non-identity permutation that preserves distances.
    If none found => unique.  If found => possibly ambiguous.
    """
    points = np.asarray(points)
    dist_mat, signatures = build_distance_signatures(points, 5)
    signature_map = defaultdict(list)
    for i, s in enumerate(signatures):
        signature_map[s].append(i)
    N = len(points)

    pi = [-1]*N
    used = [False]*N

    class_groups = sorted(signature_map.values(), key=len)
    backtrack_order = []
    for grp in class_groups:
        backtrack_order.extend(grp)

    def backtrack(idx):
        if idx == N:
            # check identity
            for ii in range(N):
                if pi[ii] != ii:
                    return True
            return False

        i = backtrack_order[idx]
        candidates = signature_map[signatures[i]]
        for j in candidates:
            if not used[j]:
                # distance check
                can_assign = True
                for prev_idx in range(idx):
                    i2 = backtrack_order[prev_idx]
                    j2 = pi[i2]
                    if abs(dist_mat[i, i2] - dist_mat[j, j2]) > tol:
                        can_assign = False
                        break
                if can_assign:
                    pi[i] = j
                    used[j] = True
                    found = backtrack(idx+1)
                    used[j] = False
                    pi[i] = -1
                    if found:
                        return True
        return False

    return backtrack(0)


###############################################################################
#  C.  Uniqueness Checks w/ Reflection Policy & MSE Threshold
###############################################################################

def check_full_set_uniqueness(marker_positions,
                              tol=1e-3,
                              allow_reflections=True,
                              max_mse=10.0):
    """
    Return True if the marker set is unique (no non-identity transform w/ MSE<max_mse),
    otherwise False.

    1) Quick distance check to see if any non-identity permutation might exist.
    2) If none, => unique.
    3) If yes, find all permutations and do Kabsch fits with reflection policy.
       If any non-identity has MSE <= max_mse, => not unique.
    """
    markers = np.asarray(marker_positions, dtype=float)
    if len(markers) < 2:
        return True  # trivially unique

    # Quick check
    possible_sym = quick_distance_symmetry_check(markers, tol=tol)
    if not possible_sym:
        return True  # no non-identity permutations => unique

    # Otherwise, gather permutations
    perms = find_all_automorphisms(markers, tol=tol, max_solutions=2000)
    N = len(markers)
    for pi in perms:
        # check identity
        if all(pi[i] == i for i in range(N)):
            continue
        B = markers[np.array(pi)]
        R, t, mse = kabsch_fit_transform(markers, B, allow_reflections)
        if mse <= max_mse:
            return False
    return True


###############################################################################
#  D.  Gathering Subset Solutions
###############################################################################

def gather_solutions_for_subset(subset_points,
                                tol=1e-3,
                                allow_reflections=True,
                                max_mse=10.0):
    """
    Returns the list of distinct transformations (R,t,mse, pi) that map
    'subset_points' onto itself with MSE <= max_mse, 
    using the chosen reflection policy.

    Identity solutions (all pi[i]=i) are excluded from the final list.
    """
    subset_points = np.asarray(subset_points)
    # 1) find permutations
    perms = find_all_automorphisms(subset_points, tol=tol, max_solutions=2000)

    transforms = []
    N = len(subset_points)

    for pi in perms:
        # identity check
        if all(pi[i] == i for i in range(N)):
            continue
        B = subset_points[np.array(pi)]
        R, t, mse = kabsch_fit_transform(subset_points, B, allow_reflections)
        if mse <= max_mse:
            transforms.append((R, t, mse, pi))

    # deduplicate
    def transform_distance(tA, tB):
        R1, trans1, _, _ = tA
        R2, trans2, _, _ = tB
        Rdiff = np.linalg.norm(R1 - R2, ord='fro')
        tdiff = np.linalg.norm(trans1 - trans2)
        return Rdiff + tdiff

    unique_transforms = []
    for cand in transforms:
        is_new = True
        for ut in unique_transforms:
            if transform_distance(cand, ut) < 1e-3:
                is_new = False
                break
        if is_new:
            unique_transforms.append(cand)

    # sort by MSE
    unique_transforms.sort(key=lambda x: x[2])
    return unique_transforms


###############################################################################
#  E.  Plotting
###############################################################################

def draw_triad(ax, origin, R=np.eye(3), length=100.0, color_axes=('r','g','b')):
    """
    Draw a triad starting at 'origin' and oriented by 'R'.
    Colors default to (red, green, blue).
    """
    origin = np.asarray(origin, dtype=float)
    x_axis = R @ np.array([1,0,0])
    y_axis = R @ np.array([0,1,0])
    z_axis = R @ np.array([0,0,1])

    x_axis = origin + length * x_axis
    y_axis = origin + length * y_axis
    z_axis = origin + length * z_axis

    ax.plot([origin[0], x_axis[0]],
            [origin[1], x_axis[1]],
            [origin[2], x_axis[2]],
            color=color_axes[0])
    ax.plot([origin[0], y_axis[0]],
            [origin[1], y_axis[1]],
            [origin[2], y_axis[2]],
            color=color_axes[1])
    ax.plot([origin[0], z_axis[0]],
            [origin[1], z_axis[1]],
            [origin[2], z_axis[2]],
            color=color_axes[2])


def plot_ambiguous_subsets_for_k(k,
                                 marker_positions,
                                 ambiguous_subsets,
                                 allow_reflections=True):
    """
    Create a single figure for all ambiguous subsets with 'k' occlusions.
    
    Instead of row=1-subset, col=solutions, we gather *all* solutions for all subsets
    into a single list so we can arrange them in a near-square grid.

    Each subplot: 
      - The removed markers in gray
      - The subset in black
      - The transformed subset in red
      - Triad of the full set's centroid
      - Triad of the subset's centroid
      - Triad of the transformed subset's centroid
    """

    markers = np.asarray(marker_positions)
    N = len(markers)
    full_centroid = markers.mean(axis=0)

    # Build a "gallery" of (subset_indices, (R, t, mse, pi)) pairs
    gallery_entries = []
    for (subset_indices, solutions) in ambiguous_subsets:
        for sol in solutions:
            gallery_entries.append((subset_indices, sol))

    if not gallery_entries:
        return  # nothing to plot

    total_plots = len(gallery_entries)
    # near-square layout
    nrows = int(math.floor(math.sqrt(total_plots)))
    ncols = int(math.ceil(total_plots / nrows))

    fig = plt.figure(figsize=(5*ncols, 4*nrows))
    fig.suptitle(f"Ambiguous Subsets for k={k} Occlusions: {total_plots} solutions total")

    for plot_idx, (subset_indices, (R, t, mse, pi)) in enumerate(gallery_entries, start=1):
        ax = fig.add_subplot(nrows, ncols, plot_idx, projection='3d')

        subset_indices = list(subset_indices)
        subset_points = markers[subset_indices]
        subset_centroid = subset_points.mean(axis=0)
        X_trans = (subset_points @ R.T) + t
        X_trans_centroid = X_trans.mean(axis=0)

        # removed markers
        removed_indices = [i for i in range(N) if i not in subset_indices]
        if removed_indices:
            removed_pts = markers[removed_indices]
            ax.scatter(removed_pts[:,0], removed_pts[:,1], removed_pts[:,2],
                       c='gray', marker='x', label='Removed')

        # plot subset
        ax.scatter(subset_points[:,0], subset_points[:,1], subset_points[:,2],
                   c='k', marker='o', label='Subset Original')

        # transformed subset
        ax.scatter(X_trans[:,0], X_trans[:,1], X_trans[:,2],
                   c='r', marker='^', label='Subset Transformed')

        # Triads
        # 1) Full set centroid triad (global frame)
        bounding_pts = [markers]
        color_global = ('r','g','b')
        # 2) Subset centroid
        color_subset = ('r','g','b')
        # 3) Transformed subset centroid
        color_trans  = ('lightcoral', 'lightgreen', 'lightblue')

        # bounding box
        bounding_arrays = [subset_points, X_trans]
        if removed_indices:
            bounding_arrays.append(markers[removed_indices])
        combined = np.vstack(bounding_arrays)
        min_all = combined.min(axis=0)
        max_all = combined.max(axis=0)
        diag_span = np.linalg.norm(max_all - min_all)
        margin = 0.1 * diag_span if diag_span>0 else 10

        # Draw the triads
        draw_triad(ax, full_centroid, np.eye(3), color_axes=color_global, length=0.1*diag_span)
        draw_triad(ax, subset_centroid, np.eye(3), color_axes=color_subset, length=0.07*diag_span)
        draw_triad(ax, X_trans_centroid, R, color_axes=color_trans, length=0.07*diag_span)

        detR = np.linalg.det(R)
        refl = "(Reflection)" if detR<0 else "(Rotation)"
        ax.set_title(f"MSE={mse:.2f}\n{refl}")

        ax.legend()
        ax.set_xlim([min_all[0]-margin, max_all[0]+margin])
        ax.set_ylim([min_all[1]-margin, max_all[1]+margin])
        ax.set_zlim([min_all[2]-margin, max_all[2]+margin])

    plt.tight_layout()


###############################################################################
#  F.  Master Analysis Function
###############################################################################

def analyze_marker_layout(marker_positions, 
                          tol=1e-3,
                          allow_reflections=True,
                          max_mse=10.0,
                          min_subset_size=3):
    """
    1) Check if full set is unique. If not, gather all solutions for the full set and plot them.
       Then exit (skip subsets).
    2) If unique, search through *all* subsets of size >= min_subset_size.
       For each k = 1..(N - min_subset_size):
         - gather all subsets of size (N-k)
         - check which are ambiguous
         - gather solutions
         - plot them in a near-square gallery figure.

    The user can see that removing an additional marker can actually
    break symmetry (leading to no ambiguous subsets for a larger k).
    This is normal if the removed marker is critical to the symmetrical arrangement.
    """

    markers = np.asarray(marker_positions, dtype=float)
    N = len(markers)

    print("Checking full set uniqueness...\n")
    is_unique_full = check_full_set_uniqueness(markers,
                                               tol=tol,
                                               allow_reflections=allow_reflections,
                                               max_mse=max_mse)

    if not is_unique_full:
        print("=> The FULL set is non-unique.")
        # Let's gather solutions for the full set & plot them in one figure.
        full_solutions = gather_solutions_for_subset(markers,
                                                     tol=tol,
                                                     allow_reflections=allow_reflections,
                                                     max_mse=max_mse)

        if not full_solutions:
            print("No valid solutions under these constraints (possibly all had MSE>max_mse).")
        else:
            # We'll produce one near-square gallery
            ambiguous_subsets = [ (range(N), full_solutions) ]
            plot_ambiguous_subsets_for_k(k=0,
                                         marker_positions=markers,
                                         ambiguous_subsets=ambiguous_subsets,
                                         allow_reflections=allow_reflections)

        print("Skipping subset search since full set is already ambiguous.")
    else:
        print("=> Full set is unique. Proceeding to search all subsets.\n")

        for k in range(1, N - min_subset_size + 1):
            subset_size = N - k
            if subset_size < min_subset_size:
                break

            print(f"Checking k={k} (subset size={subset_size}) ...")
            ambiguous_subsets = []
            # Check all subsets of size (N-k)
            for subset_indices in itertools.combinations(range(N), subset_size):
                subset_indices = list(subset_indices)
                subset_points = markers[subset_indices]

                # If it's not unique => gather solutions
                if not check_full_set_uniqueness(subset_points, 
                                                 tol=tol,
                                                 allow_reflections=allow_reflections,
                                                 max_mse=max_mse):
                    sols = gather_solutions_for_subset(subset_points,
                                                       tol=tol,
                                                       allow_reflections=allow_reflections,
                                                       max_mse=max_mse)
                    if sols:
                        ambiguous_subsets.append((subset_indices, sols))
            if ambiguous_subsets:
                print(f"  -> Found {len(ambiguous_subsets)} ambiguous subsets for k={k}. Plotting...\n")
                plot_ambiguous_subsets_for_k(k, markers, ambiguous_subsets, allow_reflections=allow_reflections)
            else:
                print(f"  -> No ambiguous subsets found for k={k}.\n")

    # Show all
    print("\nDone. Displaying all figures (blocking).")
    plt.show(block=True)


###############################################################################
# Example Usage
###############################################################################
if __name__ == "__main__":
    base_markers = [
        [250.82, 337.81, 71.0],
        [218.68, 299.51, 61.0],
        [417.96, 48.31, 71.0],
        [368.72, 39.63, 61.0],
        [167.14, -386.12, 61.0],
        [150.04, -339.14, 71.0],
        [-167.14, -386.12, 61.0],
        [-150.04, -339.14, 71.0],
        [-417.96, 48.31, 71.0],
        [-368.72, 39.63, 61.0],
        [-250.82, 337.81, 71.0],
        [-218.68, 299.51, 61.0]
    ] # The proposed location of all the base markers

    analyze_marker_layout(
        base_markers,
        tol=1e-3,
        allow_reflections=False,
        max_mse=5.0,
        min_subset_size=3
    )
