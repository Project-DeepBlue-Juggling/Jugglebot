import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# JUGGLING ANIMATION
def juggling_anim():
    # Load data
    data_name = '3a_4sec'
    df = pd.read_excel('data/' + data_name + '.xlsx', skiprows=8)

    # Number of trackers
    num_trackers = 23

    # Define connections between markers
    connections = [(0, 1), (0, 2), (2, 3), (1, 3),  # Head
                   (2, 4), (3, 4),  # Neck
                   (4, 6), (4, 8), (6, 8), (5, 8), (5, 7), (6, 7),  # Chest midline
                   (8, 15), (5, 15), (6, 15), (7, 15), (4, 15), (8, 9), (5, 9), (6, 9), (7, 9), (4, 9),  # Chest bulk
                   (15, 17), (17, 18), (18, 19), (18, 20), (19, 20), (20, 21), (19, 21),  # Left arm
                   (9, 11), (11, 12), (12, 22), (12, 13), (13, 22), (13, 14), (14, 22)  # Right arm
                   ]  # Pairs of markers to be connected

    # Create a 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-450, 250])
    ax.set_ylim([50, 600])
    ax.set_zlim([900, 1800])
    ax.set_facecolor('none')
    ax.axis('off')
    ax.view_init(elev=30, azim=120)


    # Initialize scatters and labels
    scatters = []
    labels = []
    for i in range(num_trackers):
        scatters.append(
            ax.scatter([df.iloc[0, i * 3 + 1]], [df.iloc[0, i * 3 + 2]], [df.iloc[0, i * 3 + 3]], color='b', s=5,
                       marker='o'))
        # labels.append(ax.text(df.iloc[0, i * 3 + 1], df.iloc[0, i * 3 + 2], df.iloc[0, i * 3 + 3],
        #                       str(i), fontsize=10))


    def draw_line(num, markers):
        x_data = [df.iloc[num, markers[i]*3+1] for i in range(2)]
        y_data = [df.iloc[num, markers[i]*3+2] for i in range(2)]
        z_data = [df.iloc[num, markers[i]*3+3] for i in range(2)]
        return ax.plot(x_data, y_data, z_data, color='b', lw=2)[0]


    # Initialize lines
    lines = [draw_line(0, conn) for conn in connections]


    # Animation update function
    def update(num):
        for i, scatter in enumerate(scatters):
            scatter._offsets3d = ([df.iloc[num, i*3+1]], [df.iloc[num, i*3+2]], [df.iloc[num, i*3+3]])
            # labels[i].set_position((df.iloc[num, i*3+1], df.iloc[num, i*3+2]))
            # labels[i].set_3d_properties(df.iloc[num, i*3+3], 'z')

        for i, line in enumerate(lines):
            x_data = [df.iloc[num, connections[i][j]*3+1] for j in range(2)]
            y_data = [df.iloc[num, connections[i][j]*3+2] for j in range(2)]
            z_data = [df.iloc[num, connections[i][j]*3+3] for j in range(2)]
            line.set_data(x_data, y_data)
            line.set_3d_properties(z_data)

    # Create animation
    ani = animation.FuncAnimation(fig, update, frames=len(df), interval=100)

    # Save as gif
    # ani.save(data_name + '.gif', writer='imagemagick', fps=100)

    # Show plot
    plt.show()


def wrist_trajectory():
    # Plots the wrist trajectory vs. number of balls
    file_dir = "data/"
    # List of file names
    file_names = ["3a_2sec.xlsx", "4_2sec.xlsx", "5_2sec.xlsx", "6b_2sec.xlsx", "7a_2sec.xlsx"]
    axis_names = ["3", "4", "5", "6", "7"]

    # Empty lists to store ROM values
    rom_x, rom_y, rom_z = [], [], []

    for file in file_names:
        file_name = file_dir + file
        # Read excel file
        df = pd.read_excel(file_name, header=None, skiprows=8)

        # Calculate ROM in x, y, z directions
        rom_x.append(df.iloc[:, 40].max() - df.iloc[:, 40].min())
        rom_y.append(df.iloc[:, 41].max() - df.iloc[:, 41].min())
        rom_z.append(df.iloc[:, 42].max() - df.iloc[:, 42].min())

    # Convert lists to numpy arrays for plotting
    rom_x = np.array(rom_x)
    rom_y = np.array(rom_y)
    rom_z = np.array(rom_z)

    # Create figure and plot ROM values
    plt.figure()
    plt.plot(axis_names, rom_x, label='x (Left/Right)')
    plt.plot(axis_names, rom_y, label='y (Front/Back)')
    # plt.plot(axis_names, rom_z, label='z (Up/Down)')

    plt.axhline(y=300, color='r', linestyle='--')

    plt.legend()
    plt.title("Wrist range of motion along each direction")
    plt.xlabel("Number of balls")
    plt.ylabel("Range of motion (mm)")
    plt.show()


wrist_trajectory()
