# Interactive Ball Trajectory Visualization

This project visualizes the trajectory of a ball moving under gravity within a predefined 3D workspace. You can interactively adjust the ball's initial position and velocity to see how it affects the trajectory.

## How to Run the Code

### Download the Project Files

Ensure the following files are in the same directory:

- `index.html`
- `script.js`
- `convex_hull_points_big.json`

### Set Up a Local Web Server

Due to browser security restrictions, you need to serve the files via a local web server.

#### Using Python 3:

Open a terminal in the project directory and run:

```bash
python -m http.server 8000
```

Then, open your web browser and navigate to [http://localhost:8000](http://localhost:8000).

#### Using Node.js:

Install http-server globally if you haven't already:

```bash
npm install -g http-server
```

Start the server:

```bash
http-server
```

Access the app at [http://localhost:8080](http://localhost:8080).

### Open the Application

In your web browser, go to the local server URL (e.g., http://localhost:8000).

## Key Usage Points

- **Adjust Ball Position**: Click and drag the red ball to set its initial position within the workspace.
- **Modify Initial Velocity**: Use the GUI sliders in the top-right corner to adjust the ball's initial velocity components:
  - Velocity X (m/s)
  - Velocity Y (m/s)
  - Velocity Z (m/s)
- **View Trajectory**:
  - **Green Line**: Represents the trajectory inside the workspace.
  - **Red Line**: Represents the trajectory outside the workspace.
- **Duration Display**: The total time the ball stays inside the workspace is shown in milliseconds at the bottom-left corner.
- **Camera Controls**:
  - Rotate View: Left-click and drag to orbit around the scene.
  - Zoom In/Out: Use the scroll wheel.
  - Pan View: Right-click and drag.

## Notes

- All dependencies are included via CDN links in `index.html`; no additional installation is required.
- Ensure all files are correctly placed and the local server is running to avoid any loading issues.
