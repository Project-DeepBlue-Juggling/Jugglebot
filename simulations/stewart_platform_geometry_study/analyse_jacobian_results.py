import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from plotly.subplots import make_subplots
import plotly.graph_objects as go
import plotly.io as pio

def load_data(filename):
    """
    Load the structured array from the file.
    
    Args:
        filename (str): Path to the file.
    
    Returns:
        numpy.ndarray: Loaded structured array.
    """
    return np.load(filename, allow_pickle=True)

def extract_all_data(data):
    """
    Extract all the aggregated metrics from the structured array.
    
    Args:
        data (numpy.ndarray): Structured array with results.
    
    Returns:
        tuple: Arrays of platform radii, platform small angles, and all aggregated metrics.
    """
    # For platRad, we only need the first element of each row
    platRad = data['platRad'][:, 0]

    # For platSmallAngle, we only need the values in the first row
    platSmallAngle = data['platSmallAngle'][0, :]

    # Extract all aggregated results
    mean_condition_numbers = data['aggregated_results'][:, :, 0]
    std_condition_numbers = data['aggregated_results'][:, :, 1]
    sum_rank_deficiencies = data['aggregated_results'][:, :, 2]
    max_rank_deficiencies = data['aggregated_results'][:, :, 3]
    proportion_full_rank = data['aggregated_results'][:, :, 4]
    mean_submatrix_condition_numbers = data['aggregated_results'][:, :, 5]
    std_submatrix_condition_numbers = data['aggregated_results'][:, :, 6]
    mean_norm_scores = data['aggregated_results'][:, :, 7]
    std_norm_scores = data['aggregated_results'][:, :, 8]
    unreachable_poses = data['aggregated_results'][:, :, 9]

    return (platRad, platSmallAngle, mean_condition_numbers, std_condition_numbers, 
            sum_rank_deficiencies, max_rank_deficiencies, proportion_full_rank, 
            mean_submatrix_condition_numbers, std_submatrix_condition_numbers, 
            mean_norm_scores, std_norm_scores, unreachable_poses)

def plot_3d_surface(x_data, y_data, z_data, z_label='Z Axis', title='3D Surface Plot', log_scale = False, x_label='Platform Radius\n(mm)', y_label='Platform Small Angle\n(deg)'):
    """
    Plot a 3D surface plot for the given data.
    
    Args:
        x_data (numpy.ndarray): Array of data for the X axis. This is the platform radius.
        y_data (numpy.ndarray): Array of data for the Y axis. This is the platform small angle.
        z_data (numpy.ndarray): Array of data for the Z axis. This is the aggregated metric.
        x_label (str): Label for the X axis.
        y_label (str): Label for the Y axis.
        z_label (str): Label for the Z axis.
        title (str): Title of the plot.
    """    
    # Create meshgrid for x_data and y_data
    X, Y = np.meshgrid(x_data, y_data)

    if log_scale:
        Z = np.log10(z_data)
    else:
        Z = z_data

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Create surface plot
    surf = ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none')
    
    # Add color bar for reference
    if not log_scale:
        fig.colorbar(surf, shrink=0.5, aspect=5)

    
    # Set labels and title
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    if log_scale:
        ax.set_zlabel(f'{z_label}\n(log10)')
    else:
        ax.set_zlabel(z_label)
    ax.set_title(title)



    
    # plt.show()

def plot_slice_at_platAngle_of_interest(platRad, platSmallAngle, platAngle_of_interest, mean_condition_numbers, mean_submatrix_condition_numbers, mean_norm_scores, unreachable_poses):
    '''
    Plot the slice of the 3D surface plot at the given platform radius. If no data exists at that platform radius, find and plot the closest one.
    '''
    closest_index = np.argmin(np.abs(platSmallAngle - platAngle_of_interest))
    platAngle_of_interest = platSmallAngle[closest_index]

    fig, axs = plt.subplots(2, 2, figsize=(12, 8))

    # Set global title
    fig.suptitle(f"Platform small angle = {platAngle_of_interest} deg")

    # Plot mean_condition_numbers
    axs[0, 0].plot(platRad, mean_condition_numbers[:, closest_index])
    axs[0, 0].set_xlabel('Platform Radius (mm)')
    axs[0, 0].set_ylabel('Mean Condition Number')
    axs[0, 0].set_title(f'Mean Condition Number vs Platform Radius')

    # Plot mean_submatrix_condition_numbers
    axs[0, 1].plot(platRad, mean_submatrix_condition_numbers[:, closest_index])
    axs[0, 1].set_xlabel('Platform Radius (mm)')
    axs[0, 1].set_ylabel('Mean Submatrix Condition Number')
    axs[0, 1].set_title(f'Mean Submatrix Condition Number vs Platform Radius')

    # Plot mean_norm_scores
    axs[1, 0].plot(platRad, mean_norm_scores[:, closest_index])
    axs[1, 0].set_xlabel('Platform Radius (mm)')
    axs[1, 0].set_ylabel('Mean Norm Score')
    axs[1, 0].set_title(f'Mean Norm Score vs Platform Radius')

    # Plot unreachable_poses
    axs[1, 1].plot(platRad, unreachable_poses[:, closest_index])
    axs[1, 1].set_xlabel('Platform Radius (mm)')
    axs[1, 1].set_ylabel('Num Unreachable Poses')
    axs[1, 1].set_title(f'Num Unreachable Poses vs Platform Radius')

    plt.tight_layout()

def plot_combined_3d_surface_matplotlib(platRad, platSmallAngle, mean_condition_numbers, mean_submatrix_condition_numbers, mean_norm_scores, unreachable_poses, platAngle_of_interest):
    """
    Plot combined 3D surface plots for mean condition number, mean submatrix condition number, mean norm scores, and unreachable poses.

    Args:
        platRad (numpy.ndarray): Array of platform radii.
        platSmallAngle (numpy.ndarray): Array of platform small angles.
        mean_condition_numbers (numpy.ndarray): Array of mean condition numbers.
        mean_submatrix_condition_numbers (numpy.ndarray): Array of mean submatrix condition numbers.
        mean_norm_scores (numpy.ndarray): Array of mean norm scores.
        unreachable_poses (numpy.ndarray): Array of unreachable poses.
        platAngle_of_interest (float): Specific platform small angle of interest.
    """
    fig = plt.figure(figsize=(16, 12))

    # Create meshgrid from platRad and platSmallAngle
    X, Y = np.meshgrid(platSmallAngle, platRad)

    def add_red_line(ax, Z):
        # Find the index of platAngle_of_interest in platSmallAngle
        angle_index = (np.abs(platSmallAngle - platAngle_of_interest)).argmin()
        # Extract the corresponding Z values at platAngle_of_interest
        Z_slice = Z[:, angle_index]
        # Add opaque red line at the intersection
        ax.plot([platAngle_of_interest] * len(platRad), platRad, Z_slice, color='red', linewidth=2, zorder=5)


    # Plot mean_condition_numbers
    ax1 = fig.add_subplot(221, projection='3d')
    Z1 = np.log10(mean_condition_numbers)
    surf1 = ax1.plot_surface(X, Y, Z1, cmap='viridis', edgecolor='none')
    fig.colorbar(surf1, ax=ax1, shrink=0.5, aspect=5)
    ax1.set_ylabel('Platform Radius\n(mm)')
    ax1.set_xlabel('Platform Small Angle\n(deg)')
    ax1.set_zlabel('Mean Condition Number\n(log10)')
    ax1.set_title('Mean Condition Number\n(log10)')
    add_red_line(ax1, Z1)

    # Plot mean_submatrix_condition_numbers
    ax2 = fig.add_subplot(222, projection='3d')
    Z2 = np.log10(mean_submatrix_condition_numbers)
    surf2 = ax2.plot_surface(X, Y, Z2, cmap='viridis', edgecolor='none')
    fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=5)
    ax2.set_ylabel('Platform Radius\n(mm)')
    ax2.set_xlabel('Platform Small Angle\n(deg)')
    ax2.set_zlabel('Mean Submatrix Condition Number\n(log10)')
    ax2.set_title('Mean Submatrix Condition Number\n(log10)')
    add_red_line(ax2, Z2)

    # Plot mean_norm_scores
    ax3 = fig.add_subplot(223, projection='3d')
    Z3 = mean_norm_scores
    surf3 = ax3.plot_surface(X, Y, Z3, cmap='viridis', edgecolor='none')
    fig.colorbar(surf3, ax=ax3, shrink=0.5, aspect=5)
    ax3.set_ylabel('Platform Radius\n(mm)')
    ax3.set_xlabel('Platform Small Angle\n(deg)')
    ax3.set_zlabel('Mean Norm Score')
    ax3.set_title('Mean Norm Score')
    add_red_line(ax3, Z3)

    # Plot unreachable_poses
    ax4 = fig.add_subplot(224, projection='3d')
    Z4 = unreachable_poses
    surf4 = ax4.plot_surface(X, Y, Z4, cmap='viridis', edgecolor='none')
    fig.colorbar(surf4, ax=ax4, shrink=0.5, aspect=5)
    ax4.set_ylabel('Platform Radius\n(mm)')
    ax4.set_xlabel('Platform Small Angle\n(deg)')
    ax4.set_zlabel('Num Unreachable Poses')
    ax4.set_title('Num Unreachable Poses')
    add_red_line(ax4, Z4)

    plt.tight_layout()
    plt.show()

def plot_combined_3d_surface_plotly(platRad, platSmallAngle, mean_condition_numbers, mean_submatrix_condition_numbers, mean_norm_scores, unreachable_poses, path):
    """
    Plot combined 3D surface plots for mean condition number, mean submatrix condition number, mean norm scores, and unreachable poses using Plotly.

    Args:
        platRad (numpy.ndarray): Array of platform radii.
        platSmallAngle (numpy.ndarray): Array of platform small angles.
        mean_condition_numbers (numpy.ndarray): Array of mean condition numbers.
        mean_submatrix_condition_numbers (numpy.ndarray): Array of mean submatrix condition numbers.
        mean_norm_scores (numpy.ndarray): Array of mean norm scores.
        unreachable_poses (numpy.ndarray): Array of unreachable poses.
        file_name (str): Name of the file to save the plot.
    """
    # Create a subplot with 2 rows and 2 columns
    fig = make_subplots(rows=2, cols=2, specs=[[{'type': 'surface'}, {'type': 'surface'}], [{'type': 'surface'}, {'type': 'surface'}]])

    # Plot mean_condition_numbers
    fig.add_trace(
        go.Surface(z=np.log10(mean_condition_numbers), x=platSmallAngle, y=platRad, colorscale='Viridis', showscale=True, colorbar=dict(x=0.46, len=0.4, y=0.75)),
        row=1, col=1
    )
    fig['layout']['scene'].update(
        yaxis_title='Platform Radius (mm)',
        xaxis_title='Platform Small Angle (deg)',
        zaxis_title='Mean Condition Number <br>(log10)',
        aspectmode='cube'
    )

    # Plot mean_submatrix_condition_numbers
    fig.add_trace(
        go.Surface(z=np.log10(mean_submatrix_condition_numbers), x=platSmallAngle, y=platRad, colorscale='Viridis', showscale=True, colorbar=dict(x=0.96, len=0.4, y=0.75)),
        row=1, col=2
    )
    fig['layout']['scene2'].update(
        yaxis_title='Platform Radius (mm)',
        xaxis_title='Platform Small Angle (deg)',
        zaxis_title='Mean Submatrix Condition Number<br>(log10)',
        aspectmode='cube'
    )

    # Plot mean_norm_scores
    fig.add_trace(
        go.Surface(z=mean_norm_scores, x=platSmallAngle, y=platRad, colorscale='Viridis', showscale=True, colorbar=dict(x=0.46, len=0.4, y=0.25)),
        row=2, col=1
    )
    fig['layout']['scene3'].update(
        yaxis_title='Platform Radius (mm)',
        xaxis_title='Platform Small Angle (deg)',
        zaxis_title='Mean Norm Score',
        aspectmode='cube'
    )

    # Plot unreachable_poses
    fig.add_trace(
        go.Surface(z=unreachable_poses, x=platSmallAngle, y=platRad, colorscale='Viridis', showscale=True, colorbar=dict(x=0.96, len=0.4, y=0.25)),
        row=2, col=2
    )
    fig['layout']['scene4'].update(
        yaxis_title='Platform Radius (mm)',
        xaxis_title='Platform Small Angle (deg)',
        zaxis_title='Num Unreachable Poses',
        aspectmode='cube'
    )

    # Add subplot titles as annotations
    title_size = 20
    annotations = [
        dict(text='Mean Condition Number<br>log(10)', x=0.2, y=0.95, xref='paper', yref='paper', showarrow=False, font=dict(size=title_size), xanchor='center', yanchor='middle', align='center'),
        dict(text='Mean Submatrix Condition Number<br>log(10)', x=0.8, y=0.95, xref='paper', yref='paper', showarrow=False, font=dict(size=title_size), xanchor='center', yanchor='middle', align='center'),
        dict(text='Mean Norm Score', x=0.2, y=0.45, xref='paper', yref='paper', showarrow=False, font=dict(size=title_size), xanchor='center', yanchor='middle', align='center'),
        dict(text='Num Unreachable Poses', x=0.8, y=0.45, xref='paper', yref='paper', showarrow=False, font=dict(size=title_size), xanchor='center', yanchor='middle', align='center')
    ]

    fig.update_layout(annotations=annotations, title_text="3D Surface Plots", height=950, width=1800)
    
    # Save the figure as an HTML file
    file_name = path + '3d_surface_plots.html'
    pio.write_html(fig, file_name, auto_open=True)

if __name__ == "__main__":
    # Load the data
    path = 'path_to_aggregated_results_file'
    data = load_data(path + 'aggregated_results_even_more_points.npy')
    
     # Extract all data
    (platRad, platSmallAngle, mean_condition_numbers, std_condition_numbers, 
     sum_rank_deficiencies, max_rank_deficiencies, proportion_full_rank, 
     mean_submatrix_condition_numbers, std_submatrix_condition_numbers, 
     mean_norm_scores, std_norm_scores, unreachable_poses) = extract_all_data(data)

    # Find the index where platSmallAngle is equal to our desired value. If no such value exists, find the closest one and update platRad_of_interest
    platAngle_of_interest = 100.0

    plot_slice_at_platAngle_of_interest(platRad, platSmallAngle, platAngle_of_interest, mean_condition_numbers, mean_submatrix_condition_numbers, mean_norm_scores, unreachable_poses)

    # Plot the combined 3D surface plots
    plot_combined_3d_surface_matplotlib(platRad, platSmallAngle, mean_condition_numbers, mean_submatrix_condition_numbers, mean_norm_scores, unreachable_poses, platAngle_of_interest)

    # Plot the combined 3D surface plots using Plotly
    # plot_combined_3d_surface_plotly(platRad, platSmallAngle, mean_condition_numbers, mean_submatrix_condition_numbers, mean_norm_scores, unreachable_poses, path)

    ################## FOR PLOTTING INDIVIDUAL PLOTS ##################

    # # Plot the unreachable points data
    # plot_3d_surface(platRad, platSmallAngle, unreachable_poses, 'Num Unreachable Poses', 'Unreachable Poses vs Platform Radius and Small Angle')

    # # Plot the sum rank deficiencies data
    # plot_3d_surface(platRad, platSmallAngle, sum_rank_deficiencies, 'Sum Rank Deficiencies', 'Sum Rank Deficiencies vs Platform Radius and Small Angle')

    # # Plot the mean condition numbers data
    # plot_3d_surface(platRad, platSmallAngle, mean_condition_numbers, 'Mean Condition Number', 'Mean Condition Number vs Platform Radius and Small Angle', log_scale=True)

    # # Plot the std deviation of condition numbers data
    # # plot_3d_surface(platRad, platSmallAngle, std_condition_numbers, 'Std Condition Number', 'Std Dev of Condition Number vs Platform Radius and Small Angle', log_scale=True)

    # # Plot the submatrix condition numbers data
    # plot_3d_surface(platRad, platSmallAngle, mean_submatrix_condition_numbers, 'Mean Submatrix Condition Number', 'Mean Submatrix Condition Number vs Platform Radius and Small Angle', log_scale=True)

    # # Plot the norm scores data
    # plot_3d_surface(platRad, platSmallAngle, mean_norm_scores, 'Mean Norm Score', 'Mean Norm Score vs Platform Radius and Small Angle', log_scale=False)

    plt.show()
