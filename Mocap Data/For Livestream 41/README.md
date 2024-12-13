# Motion Capture Data

This dataset contains recordings of ball throwing captured in FBX, TSV, and MP4 formats. FBX files should be easier to work with 3D animation software, TSV should work for python/programming analysis, and the MP4 videos are to show what the recordings are each showing.

Unfortunately I haven't been able to get JSON exporting to work with unlabelled data. I'll look into this and will update this data if I ever figure it out...

Below, you will find a brief description template for each file in the dataset.

Feel free to do whatever you want with this data!

All recordings are taken at 300 FPS.

## File Descriptions

### 3_ball_juggling_uncut
**Length**: 39.60 sec (11879 frames)

**Description**: The full recording of me juggling 3 balls, including some high throws (which often briefly lose tracking), a drop at around 12.8 sec, and some very wide throws. There are intermittent erroneous markers, which I believe are from my pasty-white skin...

### 3_ball_juggling_cut
**Length**: 30.34 sec (9101 frames)

**Description**: The same recording as `3_ball_juggling_uncut` but with the start and end trimmed off.

### 5_ball_juggling_uncut
**Length**: 46.66 sec (13998 frames)

**Description**: Two 5 ball runs. A bit of a fumble between 7 and 13 seconds, then two decent runs, including some high/wide throws that briefly lose tracking.

### 5_ball_juggling_cut
**Length**: 16.56 sec (4967 frames)

**Description**: A cut of the best run from `5_ball_juggling_uncut`. Some high/wide throws that lose tracking, but no drops. Occasional erroneous markers (eg. at frame 713, at 16.56 sec), but none that last long. Generally a good, clean recording of 5 ball juggling.

### 3_throws_tracking_lost_on_each
**Length**: 12.16 sec (3649 frames)

**Description**: Throwing a single ball near Jugglebot. The ball bounces each time. Only ever one unlabelled marker (the ball) visible at a time. Tracking is lost for around 10-30% of each throw, always towards the start of the flight.

### 6_throws_minimal_tracking_loss
**Length**: 20.94 sec (6282 frames)

**Description**: Throwing a single ball near jugglebot. Some landings are not tracked. Only ever one unlabelled marker (the ball) visible at a time. Tracking is very good throughout each flight (except for missing the landing for some throws)
